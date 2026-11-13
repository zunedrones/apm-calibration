"""Calibrações do ArduCopter 4.x e detecção da família de firmware, com uma placa falsa (sem serial)."""
import time
from types import SimpleNamespace as NS

from pymavlink import mavutil

from app import ardu4, ardu32
from app.vehicle import ACCELCAL_FAILED, ACCELCAL_SUCCESS, AccelCal4, CompassCal4, Vehicle

M = mavutil.mavlink


class FakeVehicle:
    def __init__(self):
        self.params = {n: 0.0 for n in AccelCal4.NAMES}
        self.param_rx = {}
        self.connected = True
        self.conn = NS(target_system=1, target_component=1)
        self.sent = []
        self.ack = M.MAV_RESULT_ACCEPTED
        self.msgs = {"SYS_STATUS": NS(onboard_control_sensors_present=M.MAV_SYS_STATUS_SENSOR_3D_MAG)}

    def require_disarmed(self):
        pass

    def command_long(self, cmd, *p):
        self.sent.append((cmd, p))

    def command_wait_ack(self, cmd, *p, timeout=0):
        self.sent.append((cmd, p))
        return self.ack

    def _send(self, fn):
        pass

    def _wait(self, pred, timeout):
        return pred()

    def fetch_params_async(self):
        pass


def pos(code):
    return NS(command=M.MAV_CMD_ACCELCAL_VEHICLE_POS, param1=float(code))


def test_accel4_segue_a_placa_e_confirma_a_posicao():
    v = FakeVehicle()
    cal = AccelCal4(v)
    cal.start()
    assert v.sent[-1][0] == M.MAV_CMD_PREFLIGHT_CALIBRATION and v.sent[-1][1][4] == 1
    for code, name in [(1, "level"), (2, "left"), (3, "right"), (4, "nose_down"), (5, "nose_up"), (6, "back")]:
        cal.on_command_long(pos(code))
        assert (cal.state, cal.position, cal.step) == ("waiting", name, code)
        cal.next()
        # o GCS devolve o mesmo código de posição
        assert v.sent[-1] == (M.MAV_CMD_ACCELCAL_VEHICLE_POS, (code,))
        assert cal.state == "measuring"
    cal.on_command_long(pos(ACCELCAL_SUCCESS))
    assert cal.state == "success"


def test_accel4_falha_e_texto_de_reserva():
    v = FakeVehicle()
    cal = AccelCal4(v)
    cal.start()
    cal.on_text("Place vehicle on its LEFT side and press any key.")  # COMMAND_LONG perdido
    assert (cal.state, cal.position) == ("waiting", "left")
    cal.next()
    assert v.sent[-1] == (M.MAV_CMD_ACCELCAL_VEHICLE_POS, (2,))
    cal.on_command_long(pos(ACCELCAL_FAILED))
    assert cal.state == "failed"


def test_accel4_ignora_comando_fora_da_calibracao():
    cal = AccelCal4(FakeVehicle())
    cal.on_command_long(pos(1))
    assert cal.state == "idle"


def progress(cid, pct, status=2):
    return NS(get_type=lambda: "MAG_CAL_PROGRESS", compass_id=cid, completion_pct=pct, cal_status=status, attempt=1,
              completion_mask=[0xFF, 0x0F] + [0] * 8)


def report(cid, status, fitness=5.2):
    return NS(get_type=lambda: "MAG_CAL_REPORT", compass_id=cid, cal_status=status, fitness=fitness, autosaved=0,
              ofs_x=-120.4, ofs_y=33.6, ofs_z=80.1, diag_x=1.01, diag_y=0.99, diag_z=1.0, scale_factor=1.0,
              new_orientation=0, orientation_confidence=3.1)


def test_compass4_sucesso_em_todas_e_gravar():
    v = FakeVehicle()
    cal = CompassCal4(v)
    cal.start()
    assert v.sent[-1][0] == M.MAV_CMD_DO_START_MAG_CAL
    cal.on_msg(progress(0, 40))
    cal.on_msg(progress(1, 10))
    st = cal.status()
    assert [p["pct"] for p in st["progress"]] == [40, 10]
    assert sum(st["progress"][0]["cells"]) == 12 and len(st["progress"][0]["cells"]) == 80
    cal.on_msg(report(0, M.MAG_CAL_SUCCESS))
    assert cal.state == "running"  # ainda falta a bússola 1
    cal.on_msg(report(1, M.MAG_CAL_SUCCESS))
    assert cal.state == "done"
    assert cal.status()["reports"][0]["offsets"] == [-120, 34, 80]
    cal.accept()
    assert v.sent[-1][0] == M.MAV_CMD_DO_ACCEPT_MAG_CAL and cal.state == "accepted"


def test_compass4_uma_falha_reprova():
    v = FakeVehicle()
    cal = CompassCal4(v)
    cal.start()
    cal.on_msg(progress(0, 90))
    cal.on_msg(report(0, M.MAG_CAL_BAD_RADIUS, fitness=40))
    assert cal.state == "failed"
    try:
        cal.accept()
        raise AssertionError("não deveria gravar")
    except RuntimeError:
        pass


def test_compass4_cancelar():
    v = FakeVehicle()
    cal = CompassCal4(v)
    cal.start()
    cal.cancel()
    assert v.sent[-1][0] == M.MAV_CMD_DO_CANCEL_MAG_CAL and cal.state == "idle"


def test_deteccao_do_firmware():
    v = Vehicle()
    assert v.profile is ardu32
    v.version = (4, 5, 7)
    v._update_profile()
    assert v.profile is ardu4 and isinstance(v.accel, AccelCal4) and isinstance(v.compass, CompassCal4)
    v.reset_state()
    v._on_text(time.time(), 6, "APM:Copter V3.2.1 (36b405fb)")
    assert v.profile is ardu32
    v.reset_state()
    v.params["FRAME_CLASS"] = 1.0
    v._update_profile()
    assert v.profile is ardu4


def test_backup_4x_ignora_ids_de_hardware():
    rx = ardu4.PARAMS_SKIP_RESTORE_RE
    assert rx.search("COMPASS_DEV_ID3") and rx.search("INS_ACC2_ID") and rx.search("COMPASS_PRIO1_ID") and rx.search("BARO1_DEVID")
    assert not rx.search("COMPASS_OFS_X") and not rx.search("RC3_MIN")
    assert not ardu32.PARAMS_SKIP_RESTORE_RE.search("COMPASS_DEV_ID")
