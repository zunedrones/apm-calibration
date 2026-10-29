"""Máquina de estados da calibração do acelerômetro com uma placa falsa (sem serial)."""
import time

from app.vehicle import AccelCal

NAMES = AccelCal.NAMES


class FakeVehicle:
    def __init__(self):
        self.params = {n: 0.0 for n in NAMES} | {"AHRS_TRIM_X": 0.0, "AHRS_TRIM_Y": 0.0}
        self.param_rx = {}
        self.last_hb = time.time()
        self.connected = True
        self.sent = []
        self.conn = type("C", (), {"target_system": 1, "target_component": 1})()

    def require_disarmed(self):
        pass

    def command_long(self, *a):
        self.sent.append(("cmd", a))
        self.last_hb = 0.0  # o 3.2 para de mandar heartbeat enquanto calibra

    def _send(self, fn):
        self.sent.append(("send", fn))

    def _wait(self, pred, timeout):
        return pred()


def wait_state(cal, states, timeout=3):
    t0 = time.time()
    while cal.state not in states and time.time() - t0 < timeout:
        time.sleep(0.02)
    return cal.state


def make():
    v = FakeVehicle()
    cal = AccelCal(v)
    cal.STEP_TIMEOUT, cal.FINAL_TIMEOUT = 0.3, 0.6
    return v, cal


def test_fluxo_normal_por_mensagens():
    v, cal = make()
    cal.start()
    for i, txt in enumerate(["level", "on its LEFT side", "on its RIGHT side", "nose DOWN", "nose UP", "on its BACK"]):
        cal.on_text(f"Place vehicle {txt} and press any key.")
        assert cal.state == "waiting" and cal.step == i + 1 and cal.position == AccelCal.ORDER[i]
        cal.next()
    cal.on_text("Calibration FAILED")
    assert cal.state == "failed" and not cal.inferred


def test_prompt_perdido_infere_proxima_posicao():
    v, cal = make()
    cal.start()
    cal.on_text("Place vehicle level and press any key.")
    cal.next()                        # o "LEFT" se perde
    assert wait_state(cal, {"waiting"}) == "waiting"
    assert cal.step == 2 and cal.position == "left"
    cal.on_text("Place vehicle on its RIGHT side and press any key.")   # volta a sincronizar pela mensagem
    assert cal.step == 3 and cal.position == "right"


def test_resultado_perdido_inferido_pelos_parametros():
    v, cal = make()
    cal.start()
    cal.on_text("Place vehicle on its BACK and press any key.")
    assert cal.step == 6
    cal.next()
    v.params["INS_ACCOFFS_X"] = 0.21                      # a placa gravou novos offsets
    v.param_rx = {n: time.time() + 1 for n in v.params}
    v.last_hb = time.time() + 1                           # heartbeat voltou
    assert wait_state(cal, {"success", "failed"}, timeout=5) == "success"
    assert cal.inferred and cal.result["INS_ACCOFFS_X"] == 0.21


def test_resultado_perdido_sem_mudanca_e_falha():
    v, cal = make()
    cal.start()
    cal.on_text("Place vehicle on its BACK and press any key.")
    cal.next()
    v.param_rx = {n: time.time() + 1 for n in v.params}
    v.last_hb = time.time() + 1
    assert wait_state(cal, {"success", "failed"}, timeout=5) == "failed"
