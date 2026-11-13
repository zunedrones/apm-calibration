"""Conexão MAVLink com a controladora e máquinas de estado das calibrações.

Duas famílias de firmware, detectadas na conexão:
- APM 2.x com ArduCopter 3.2 (`ardu32`): MAVLink 1, a placa trava durante a calibração do
  acelerômetro e a bússola é calibrada aqui no GCS;
- Pixhawk com ArduCopter 4.x (`ardu4`): MAVLink 2, acelerômetro e bússola calibrados pela
  própria placa (ACCELCAL_VEHICLE_POS, MAG_CAL_PROGRESS/REPORT).

Uma única thread lê a serial e guarda a última mensagem de cada tipo; a API e o
WebSocket só leem esse estado. Envio é protegido por lock e sempre em MAVLink 1.
"""
from __future__ import annotations

import math
import os
import re
import threading
import time
from collections import deque
from types import ModuleType
from typing import Any, Callable

import numpy as np
from pymavlink import mavutil

from . import ardu4, ardu32, magfit

M = mavutil.mavlink
SENSOR_NAMES = {k: v.name.replace("MAV_SYS_STATUS_", "") for k, v in M.enums["MAV_SYS_STATUS_SENSOR"].items() if k < (1 << 30)}
PREARM_BIT = 0x10000000  # MAV_SYS_STATUS_PREARM_CHECK (4.x): saudável = passa no PreArm
ACCELCAL_POS = {1: "level", 2: "left", 3: "right", 4: "nose_down", 5: "nose_up", 6: "back"}
ACCELCAL_SUCCESS, ACCELCAL_FAILED = 16777215, 16777216


class VehicleError(RuntimeError):
    """Erro com mensagem para o usuário."""


class Vehicle:
    def __init__(self) -> None:
        self.conn: mavutil.mavfile | None = None
        self.port: str | None = None
        self.baud = 115200
        self._send_lock = threading.Lock()
        self._state_lock = threading.Lock()
        self._param_cv = threading.Condition()
        self._thread: threading.Thread | None = None
        self._running = False
        self._listeners: list[Callable[[Any], None]] = []
        self.reset_state()
        self.accel32, self.accel4 = AccelCal(self), AccelCal4(self)
        self.compass32, self.compass4 = CompassCal(self), CompassCal4(self)
        self.rc = RcCal(self)

    @property
    def is4(self) -> bool:
        return self.profile is ardu4

    @property
    def accel(self) -> AccelCal:
        return self.accel4 if self.is4 else self.accel32

    @property
    def compass(self) -> CompassCal | CompassCal4:
        return self.compass4 if self.is4 else self.compass32

    # ------------------------------------------------------------------ estado
    def reset_state(self) -> None:
        self.msgs: dict[str, Any] = {}
        self.msg_time: dict[str, float] = {}
        self.texts: deque[tuple[float, int, str]] = deque(maxlen=300)
        self.params: dict[str, float] = {}
        self.param_types: dict[str, int] = {}
        self.param_rx: dict[str, float] = {}
        self.param_count: int | None = None
        self.param_indices: set[int] = set()
        self.params_loading = False
        self.last_hb = 0.0
        self.connected_at = 0.0
        self.firmware: str | None = None
        self.frame_text: str | None = None
        self.board_text: str | None = None
        self.last_ack: dict[int, Any] = {}
        self.profile: ModuleType = ardu32
        self.version: tuple[int, int, int] | None = None
        self.saw_v2 = False

    @property
    def connected(self) -> bool:
        if self.conn is None:
            return False
        # durante a calibração do acelerômetro o 3.2 fica bloqueado e para de mandar heartbeat
        if not self.is4 and self.accel32.state in ("starting", "waiting", "measuring"):
            return True
        return time.time() - self.last_hb < 3.5

    def armed(self) -> bool:
        hb = self.msgs.get("HEARTBEAT")
        return bool(hb and hb.base_mode & M.MAV_MODE_FLAG_SAFETY_ARMED)

    # --------------------------------------------------------------- conexão
    def connect(self, port: str, baud: int = 115200) -> None:
        self.disconnect()
        self.reset_state()
        self.port, self.baud = port, baud
        try:
            self.conn = mavutil.mavlink_connection(port, baud=baud, source_system=255, source_component=190)
        except Exception as e:  # noqa: BLE001
            self.conn = None
            raise VehicleError(f"Não consegui abrir {port}: {e}") from e
        # a APM 2.x só entende MAVLink 1; a Pixhawk aceita os dois
        send = self.conn.mav.send
        self.conn.mav.send = lambda msg, force_mavlink1=False: send(msg, force_mavlink1=True)
        self._running = True
        self._thread = threading.Thread(target=self._loop, name="mavlink-rx", daemon=True)
        self._thread.start()
        self.connected_at = time.time()
        # na APM, abrir a USB reseta o ATmega2560 (DTR): o heartbeat só vem depois do boot
        if not self._wait(lambda: self.last_hb > 0, 15):
            self.disconnect()
            raise VehicleError("Sem heartbeat. Confira a porta, o baud (115200 na USB) e se outro programa está usando a serial.")
        self._detect_profile()
        if not self.is4:
            self._wait(lambda: any("GROUND START" in t for _, _, t in self.texts), 8)
            time.sleep(0.5)
        self.request_streams(self.profile.DEFAULT_STREAM_HZ)
        self.fetch_params_async()
        if self.is4:
            self.command_long(M.MAV_CMD_RUN_PREARM_CHECKS)

    def _detect_profile(self) -> None:
        """4.x responde AUTOPILOT_VERSION e fala MAVLink 2; o 3.2 não faz nenhum dos dois de forma confiável."""
        self.command_long(M.MAV_CMD_REQUEST_MESSAGE, M.MAVLINK_MSG_ID_AUTOPILOT_VERSION)
        self.command_long(M.MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES, 1)
        self._wait(lambda: self.version is not None, 2.5)
        self._update_profile()

    def _update_profile(self) -> None:
        if self.version:
            self.profile = ardu4 if self.version[0] >= 4 else ardu32
        elif self.saw_v2 or "FRAME_CLASS" in self.params:
            self.profile = ardu4

    def disconnect(self) -> None:
        self._running = False
        if self._thread and self._thread.is_alive() and self._thread is not threading.current_thread():
            self._thread.join(timeout=2)
        if self.conn is not None:
            try:
                self.conn.close()
            except Exception:  # noqa: BLE001
                pass
        self.conn = None
        for cal in (self.accel32, self.accel4, self.compass32, self.compass4, self.rc):
            cal.reset()

    def reboot(self) -> None:
        """Reinicia a placa e reconecta. Na USB da APM fechar/abrir a serial já reseta (DTR)."""
        if self.conn is None:
            raise VehicleError("Não conectado.")
        self.require_disarmed()
        port, baud, is4 = self.port, self.baud, self.is4
        self.command_long(M.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN, 1)
        time.sleep(0.5)
        if not is4:
            self.connect(port, baud)
            return
        # a USB da Pixhawk some e volta (~3–8 s) quando ela reinicia
        self.disconnect()
        self._wait(lambda: not os.path.exists(port), 5)
        t0, err = time.time(), None
        while time.time() - t0 < 25:
            if os.path.exists(port):
                try:
                    self.connect(port, baud)
                    return
                except VehicleError as e:
                    err = e
            time.sleep(1)
        raise err or VehicleError("A placa não voltou na USB depois de reiniciar.")

    # ------------------------------------------------------------------ RX
    def _loop(self) -> None:
        while self._running and self.conn is not None:
            try:
                msg = self.conn.recv_match(blocking=True, timeout=0.5)
            except Exception:  # noqa: BLE001  (porta removida etc.)
                time.sleep(0.5)
                continue
            if msg is None:
                continue
            t = msg.get_type()
            if t == "BAD_DATA":
                continue
            if msg.get_srcSystem() == 255:  # eco de outro GCS
                continue
            now = time.time()
            if not self.saw_v2 and msg.get_msgbuf()[:1] == b"\xfd":
                self.saw_v2 = True
            with self._state_lock:
                self.msgs[t] = msg
                self.msg_time[t] = now
            if t == "HEARTBEAT" and msg.get_srcComponent() == 1:
                self.last_hb = now
            elif t == "STATUSTEXT":
                self._on_text(now, msg.severity, msg.text)
            elif t == "PARAM_VALUE":
                self._on_param(msg, now)
            elif t == "COMMAND_ACK":
                self.last_ack[msg.command] = (now, msg.result)
            elif t == "COMMAND_LONG":  # a placa 4.x pede a posição do acelerômetro assim
                self.accel4.on_command_long(msg)
            elif t == "AUTOPILOT_VERSION":
                v = msg.flight_sw_version
                self.version = ((v >> 24) & 0xFF, (v >> 16) & 0xFF, (v >> 8) & 0xFF)
            elif t == "RAW_IMU":
                self.compass32.on_raw_imu(msg)
            elif t in ("MAG_CAL_PROGRESS", "MAG_CAL_REPORT"):
                self.compass4.on_msg(msg)
            elif t in ("RC_CHANNELS_RAW", "RC_CHANNELS"):
                self.rc.on_rc(msg)

    def _on_text(self, now: float, sev: int, text: str) -> None:
        text = text.strip("\x00").strip()
        self.texts.append((now, sev, text))
        if (fw := re.search(r"(APM:Copter|ArduCopter)\s*V?(\d+)\.(\d+)\.?(\d*)", text)):
            self.firmware = text
            if self.version is None:
                self.version = (int(fw[2]), int(fw[3]), int(fw[4] or 0))
                self._update_profile()
        elif text.startswith("Frame:"):
            self.frame_text = text.split(":", 1)[1].strip()
        elif re.match(r"(fmu|Pixhawk|CubeBlack|MatekH|omnibus)", text, re.I) and self.board_text is None:
            self.board_text = text.split()[0]
        self.accel.on_text(text)

    def _on_param(self, msg: Any, now: float) -> None:
        with self._param_cv:
            name = msg.param_id
            self.params[name] = float(msg.param_value)
            self.param_types[name] = msg.param_type
            self.param_rx[name] = now
            self.param_count = msg.param_count
            if 0 <= msg.param_index < 65535:
                self.param_indices.add(msg.param_index)
            self._param_cv.notify_all()

    def _wait(self, pred: Callable[[], bool], timeout: float) -> bool:
        t0 = time.time()
        while time.time() - t0 < timeout:
            if pred():
                return True
            time.sleep(0.05)
        return pred()

    # ------------------------------------------------------------------ TX
    def _send(self, fn: Callable[[Any], None]) -> None:
        if self.conn is None:
            raise VehicleError("Não conectado.")
        with self._send_lock:
            fn(self.conn.mav)

    def command_long(self, cmd: int, *params: float) -> None:
        p = list(params) + [0.0] * (7 - len(params))
        self._send(lambda mav: mav.command_long_send(self.conn.target_system, self.conn.target_component, cmd, 0, *p))

    def command_wait_ack(self, cmd: int, *params: float, timeout: float = 4) -> int | None:
        t0 = time.time()
        self.command_long(cmd, *params)
        ok = self._wait(lambda: cmd in self.last_ack and self.last_ack[cmd][0] >= t0, timeout)
        return self.last_ack[cmd][1] if ok else None

    def request_streams(self, hz: int, stream: int = M.MAV_DATA_STREAM_ALL) -> None:
        self._send(lambda mav: mav.request_data_stream_send(self.conn.target_system, self.conn.target_component, stream, hz, 1))

    def require_disarmed(self) -> None:
        if self.armed():
            raise VehicleError("A placa está ARMADA. Desarme antes (throttle baixo + yaw para a esquerda).")

    # ---------------------------------------------------------------- params
    def fetch_params_async(self) -> None:
        threading.Thread(target=self.fetch_params, name="param-fetch", daemon=True).start()

    def fetch_params(self) -> None:
        if self.params_loading:
            return
        self.params_loading = True
        try:
            self.param_indices.clear()
            self._send(lambda mav: mav.param_request_list_send(self.conn.target_system, self.conn.target_component))
            last_n, quiet_since = -1, time.time()
            t0 = time.time()
            while time.time() - t0 < 90:
                time.sleep(0.3)
                n = len(self.param_indices)
                if self.param_count and n >= self.param_count:
                    break
                if n != last_n:
                    last_n, quiet_since = n, time.time()
                elif time.time() - quiet_since > 2 and self.param_count:
                    # pede os que faltaram, um a um, pelo índice
                    for i in sorted(set(range(self.param_count)) - self.param_indices)[:20]:
                        self._send(lambda mav, i=i: mav.param_request_read_send(self.conn.target_system, self.conn.target_component, b"", i))
                        time.sleep(0.03)
                    quiet_since = time.time()
        finally:
            self.params_loading = False
            self._update_profile()

    def set_param(self, name: str, value: float, timeout: float = 2.0, retries: int = 3) -> float:
        """Grava e confere a leitura de volta. Retorna o valor lido."""
        if self.conn is None:
            raise VehicleError("Não conectado.")
        ptype = self.param_types.get(name, M.MAV_PARAM_TYPE_REAL32)
        for _ in range(retries):
            t0 = time.time()
            self._send(lambda mav: mav.param_set_send(self.conn.target_system, self.conn.target_component, name.encode(), float(value), ptype))
            with self._param_cv:
                self._param_cv.wait_for(lambda: self.param_rx.get(name, 0) >= t0, timeout)
            if self.param_rx.get(name, 0) >= t0:
                got = self.params[name]
                if not math.isclose(got, float(value), rel_tol=1e-5, abs_tol=1e-6):
                    raise VehicleError(f"{name}: pedi {value:g}, a placa gravou {got:g} (fora da faixa do parâmetro?).")
                return got
        raise VehicleError(f"{name}: sem resposta da placa.")

    def set_params(self, values: dict[str, float]) -> list[dict]:
        out = []
        for name, value in values.items():
            try:
                out.append({"name": name, "ok": True, "value": self.set_param(name, value)})
            except VehicleError as e:
                out.append({"name": name, "ok": False, "error": str(e)})
        return out

    # -------------------------------------------------------------- snapshot
    def prearm(self) -> dict:
        """As duas famílias reenviam as falhas de PreArm a cada ~30 s enquanto desarmadas.

        O 4.x também diz em SYS_STATUS se passa no PreArm, então o resultado sai na hora.
        """
        now = time.time()
        recent = sorted({t for ts, _, t in self.texts if t.startswith("PreArm") and now - ts < 40})
        watched = now - self.connected_at if self.connected_at else 0
        out = {"messages": recent, "watched_s": round(watched), "conclusive": watched > 40, "ok": None}
        s = self.msgs.get("SYS_STATUS")
        if self.is4 and s and s.onboard_control_sensors_present & PREARM_BIT:
            out["ok"] = bool(s.onboard_control_sensors_health & PREARM_BIT)
            out["conclusive"] = True
            if out["ok"]:
                out["messages"] = []
        return out

    def run_prearm_checks(self) -> None:
        if self.is4:
            self.command_long(M.MAV_CMD_RUN_PREARM_CHECKS)

    def snapshot(self) -> dict:
        m = self.msgs
        hb = m.get("HEARTBEAT")
        snap: dict[str, Any] = {
            "connected": self.connected,
            "port": self.port,
            "baud": self.baud,
            "hb_age": round(time.time() - self.last_hb, 1) if self.last_hb else None,
            "firmware": self.firmware or (f"ArduCopter V{'.'.join(map(str, self.version))}" if self.version else None),
            "frame_text": self.frame_text,
            "board": self.profile.KEY,
            "board_label": self.profile.LABEL,
            "board_text": self.board_text,
            "params": {"received": len(self.param_indices), "total": self.param_count, "loading": self.params_loading},
            "texts": [{"t": ts, "sev": s, "text": tx} for ts, s, tx in list(self.texts)[-40:]],
            "prearm": self.prearm(),
            "accel": self.accel.status(),
            "compass": self.compass.status(),
            "rc_cal": self.rc.status(),
        }
        if hb:
            snap["armed"] = bool(hb.base_mode & M.MAV_MODE_FLAG_SAFETY_ARMED)
            snap["mode"] = self.profile.FLIGHT_MODES.get(hb.custom_mode, str(hb.custom_mode))
            snap["custom_mode"] = hb.custom_mode
            snap["system_status"] = M.enums["MAV_STATE"][hb.system_status].name.replace("MAV_STATE_", "") if hb.system_status in M.enums["MAV_STATE"] else hb.system_status
        if (s := m.get("SYS_STATUS")):
            snap["sensors"] = [
                {"name": n, "present": bool(s.onboard_control_sensors_present & b), "enabled": bool(s.onboard_control_sensors_enabled & b), "healthy": bool(s.onboard_control_sensors_health & b)}
                for b, n in sorted(SENSOR_NAMES.items()) if s.onboard_control_sensors_present & b
            ]
            snap["load_pct"] = s.load / 10
            snap["battery"] = {"voltage": s.voltage_battery / 1000, "current": s.current_battery / 100 if s.current_battery >= 0 else None, "remaining": s.battery_remaining}
            if self.is4:
                # 4.x liga o bit de saídas de motor quando a chave de segurança está liberada
                snap["safety_off"] = bool(s.onboard_control_sensors_enabled & M.MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS)
        if (h := m.get("HWSTATUS")):
            snap["vcc"] = h.Vcc / 1000
            snap["i2c_err"] = h.I2Cerr
        elif (ps := m.get("POWER_STATUS")):
            snap["vcc"] = ps.Vcc / 1000
            snap["vservo"] = ps.Vservo / 1000
        if (a := m.get("ATTITUDE")):
            snap["attitude"] = {"roll": math.degrees(a.roll), "pitch": math.degrees(a.pitch), "yaw": math.degrees(a.yaw)}
        if (r := m.get("RAW_IMU")):
            acc = (r.xacc, r.yacc, r.zacc)
            snap["imu"] = {"acc": acc, "gyro": (r.xgyro, r.ygyro, r.zgyro), "mag": (r.xmag, r.ymag, r.zmag), "acc_norm": math.sqrt(sum(v * v for v in acc))}
        if (p := m.get("SCALED_PRESSURE")):
            snap["baro"] = {"press": p.press_abs, "temp": p.temperature / 100}
        if (v := m.get("VFR_HUD")):
            snap["hud"] = {"alt": v.alt, "heading": v.heading, "throttle": v.throttle, "climb": v.climb}
        if (g := m.get("GPS_RAW_INT")):
            snap["gps"] = {"fix": g.fix_type, "sats": g.satellites_visible, "hdop": g.eph / 100 if g.eph != 65535 else None, "lat": g.lat / 1e7, "lon": g.lon / 1e7}
        rc_t = "RC_CHANNELS" if "RC_CHANNELS" in m else "RC_CHANNELS_RAW"
        if (rc := m.get(rc_t)):
            snap["rc"] = [getattr(rc, f"chan{i}_raw") for i in range(1, 9)]
            snap["rc_age"] = round(time.time() - self.msg_time[rc_t], 1)
            # 4.x sem receptor: chancount = 0 e canais em 0. APM 2.x sem receptor: CH3 ≈ 900
            snap["rc_count"] = getattr(rc, "chancount", None)
        if (so := m.get("SERVO_OUTPUT_RAW")):
            snap["servo"] = [getattr(so, f"servo{i}_raw") for i in range(1, 9)]
        if (mi := m.get("MEMINFO")):
            snap["freemem"] = mi.freemem
        return snap


# ============================================================ calibrações
class AccelCal:
    """6 posições. No 3.2 a placa manda 'Place vehicle ... and press any key' e espera um COMMAND_ACK.

    Durante a calibração o 3.2 fica bloqueado e pode descartar STATUSTEXT (buffer da serial cheio),
    então um watchdog infere a próxima posição pela ordem fixa e, no fim, o resultado pelos parâmetros.
    """

    KEYS = [("LEFT", "left"), ("RIGHT", "right"), ("DOWN", "nose_down"), ("UP", "nose_up"), ("BACK", "back"), ("level", "level")]
    ORDER = ["level", "left", "right", "nose_down", "nose_up", "back"]
    NAMES = [f"INS_ACC{t}_{a}" for t in ("OFFS", "SCAL") for a in "XYZ"]
    STEP_TIMEOUT, FINAL_TIMEOUT = 6.0, 25.0  # s

    def __init__(self, v: Vehicle) -> None:
        self.v = v
        self.reset()

    def reset(self) -> None:
        self.state = "idle"  # idle|starting|waiting|measuring|success|failed
        self.step = 0
        self.position: str | None = None
        self.firmware_prompt: str | None = None
        self.result: dict | None = None
        self.inferred = False
        self.before: dict[str, float | None] = {}
        self.log: list[str] = []
        self._ack_t = 0.0

    def start(self) -> None:
        if self.state in ("starting", "waiting", "measuring"):
            raise VehicleError("A calibração já está em andamento.")
        if not self.v.connected:
            raise VehicleError("Não conectado.")
        self.v.require_disarmed()
        self.reset()
        self.before = {n: self.v.params.get(n) for n in self.NAMES}
        self.state = "starting"
        self.v.command_long(M.MAV_CMD_PREFLIGHT_CALIBRATION, 0, 0, 0, 0, 1, 0, 0)

    def next(self) -> None:
        if self.state != "waiting":
            raise VehicleError("A placa não está esperando uma posição agora.")
        self.state = "measuring"
        self._ack_t = time.time()
        self.v._send(lambda mav: mav.command_ack_send(0, 1))
        threading.Thread(target=self._watchdog, args=(self.step, self._ack_t), daemon=True).start()

    def _watchdog(self, step: int, ack_t: float) -> None:
        # a medida leva ~1 s; sem novidade depois disso, a mensagem foi perdida
        deadline = ack_t + (self.STEP_TIMEOUT if step < 6 else self.FINAL_TIMEOUT)
        while time.time() < deadline:
            if self.state != "measuring" or self._ack_t != ack_t:
                return
            if self.v.last_hb > ack_t + 0.5:  # heartbeat voltou: a calibração terminou
                break
            time.sleep(0.2)
        if self.state != "measuring" or self._ack_t != ack_t:
            return
        if step < 6 and self.v.last_hb <= ack_t:
            self.step += 1
            self.position = self.ORDER[self.step - 1]
            self.firmware_prompt = "(mensagem da placa perdida; posição pela ordem padrão)"
            self.state = "waiting"
            return
        self._finish_from_params()

    def _finish_from_params(self) -> None:
        self.inferred = True
        self._read_result()
        changed = any(
            self.result.get(n) is not None and self.before.get(n) is not None and abs(self.result[n] - self.before[n]) > 1e-6
            for n in self.NAMES
        )
        self.state = "success" if changed else "failed"

    def on_text(self, text: str) -> None:
        if self.state not in ("starting", "waiting", "measuring"):
            return
        self.log.append(text)
        if "press any key" in text:
            self.step = min(self.step + 1, 6)
            self.position = next((k for f, k in self.KEYS if f in text), self.ORDER[self.step - 1])
            self.step = self.ORDER.index(self.position) + 1
            self.firmware_prompt = text
            self.state = "waiting"
        elif "successful" in text.lower():
            self.state = "success"
            threading.Thread(target=self._read_result, daemon=True).start()
        elif "failed" in text.lower():
            self.state = "failed"

    def _read_result(self) -> None:
        time.sleep(1.5)
        names = self.NAMES + ["AHRS_TRIM_X", "AHRS_TRIM_Y"]
        t0 = time.time()
        for n in names:
            self.v._send(lambda mav, n=n: mav.param_request_read_send(self.v.conn.target_system, self.v.conn.target_component, n.encode(), -1))
            time.sleep(0.1)
        self.v._wait(lambda: all(self.v.param_rx.get(n, 0) >= t0 for n in names), 4)
        self.result = {n: self.v.params.get(n) for n in names}

    def status(self) -> dict:
        return {"state": self.state, "step": self.step, "position": self.position, "prompt": self.firmware_prompt,
                "result": self.result, "inferred": self.inferred, "log": self.log[-12:]}


class CompassCal:
    """Calibração 'live' feita no GCS: coleta RAW_IMU girando o drone e ajusta uma esfera."""

    def __init__(self, v: Vehicle) -> None:
        self.v = v
        self.reset()

    def reset(self) -> None:
        self.state = "idle"  # idle|collecting|done
        self.samples: list[tuple[float, float, float]] = []
        self.ofs0 = np.zeros(3)
        self.result: dict | None = None
        self.started = 0.0
        self._live: dict | None = None
        self._live_t = 0.0

    def start(self) -> None:
        if not self.v.connected:
            raise VehicleError("Não conectado.")
        sensors = self.v.msgs.get("SYS_STATUS")
        if not sensors or not (sensors.onboard_control_sensors_present & M.MAV_SYS_STATUS_SENSOR_3D_MAG):
            raise VehicleError("A placa não detectou bússola. Ligue MAG_ENABLE=1, conecte o módulo e reinicie a placa.")
        self.reset()
        p = self.v.params
        self.ofs0 = np.array([p.get("COMPASS_OFS_X", 0.0), p.get("COMPASS_OFS_Y", 0.0), p.get("COMPASS_OFS_Z", 0.0)])
        self.state = "collecting"
        self.started = time.time()
        self.v.request_streams(10, M.MAV_DATA_STREAM_RAW_SENSORS)

    def on_raw_imu(self, msg: Any) -> None:
        if self.state != "collecting":
            return
        field = np.array([msg.xmag, msg.ymag, msg.zmag], dtype=np.float64)
        if not field.any():
            return
        raw = field - self.ofs0
        self.samples.append(tuple(raw.tolist()))

    def stop(self) -> dict:
        if self.state != "collecting":
            raise VehicleError("A coleta não está ativa.")
        self.v.request_streams(self.v.profile.DEFAULT_STREAM_HZ, M.MAV_DATA_STREAM_RAW_SENSORS)
        if len(self.samples) < 50:
            self.state = "idle"
            raise VehicleError(f"Poucas amostras ({len(self.samples)}). Gire o drone por pelo menos 60 s.")
        self.result = magfit.evaluate(np.array(self.samples))
        self.state = "done"
        return self.result

    def apply(self) -> list[dict]:
        if self.state != "done" or not self.result:
            raise VehicleError("Nada para aplicar.")
        ox, oy, oz = self.result["offsets"]
        return self.v.set_params({"COMPASS_OFS_X": round(ox), "COMPASS_OFS_Y": round(oy), "COMPASS_OFS_Z": round(oz)})

    def status(self) -> dict:
        st: dict[str, Any] = {"state": self.state, "n": len(self.samples), "result": self.result}
        if self.state == "collecting":
            st["elapsed"] = round(time.time() - self.started)
            # ajuste parcial a cada 1 s para a barra de cobertura
            if len(self.samples) >= 30 and time.time() - self._live_t > 1:
                try:
                    self._live = magfit.evaluate(np.array(self.samples))
                except ValueError:
                    self._live = None
                self._live_t = time.time()
            st["live"] = self._live
            pts = self.samples[-300:]
            c = np.array(self._live["offsets"]) * -1 if self._live else np.zeros(3)
            st["points"] = [[round(x - c[0]), round(y - c[1]), round(z - c[2])] for x, y, z in pts]
        return st


class RcCal:
    """Registra mínimo/máximo de cada canal enquanto o piloto mexe os sticks, depois o centro."""

    def __init__(self, v: Vehicle) -> None:
        self.v = v
        self.reset()

    def reset(self) -> None:
        self.state = "idle"  # idle|capturing|captured
        self.mins = [3000] * 8
        self.maxs = [0] * 8
        self.trims: list[int] | None = None

    def start(self) -> None:
        if not self.v.connected:
            raise VehicleError("Não conectado.")
        self.reset()
        self.state = "capturing"
        self.v.request_streams(10, M.MAV_DATA_STREAM_RC_CHANNELS)

    def on_rc(self, msg: Any) -> None:
        if self.state != "capturing":
            return
        for i in range(8):
            v = getattr(msg, f"chan{i + 1}_raw")
            if 800 <= v <= 2200:
                self.mins[i] = min(self.mins[i], v)
                self.maxs[i] = max(self.maxs[i], v)

    def finish(self) -> dict:
        """Sticks no centro e throttle embaixo: grava o trim."""
        if self.state != "capturing":
            raise VehicleError("A captura não está ativa.")
        rc = self.v.msgs.get("RC_CHANNELS") or self.v.msgs.get("RC_CHANNELS_RAW")
        if rc is None:
            raise VehicleError("Sem dados de rádio.")
        cur = [getattr(rc, f"chan{i}_raw") for i in range(1, 9)]
        self.trims = cur
        self.v.request_streams(self.v.profile.DEFAULT_STREAM_HZ, M.MAV_DATA_STREAM_RC_CHANNELS)
        self.state = "captured"
        return self.status()

    def proposal(self) -> dict[str, float]:
        if self.state != "captured" or self.trims is None:
            raise VehicleError("Faça a captura primeiro.")
        out: dict[str, float] = {}
        bad = []
        for i in range(8):
            ch = i + 1
            span = self.maxs[i] - self.mins[i]
            if span < 200:  # canal não mexido: não altera
                if ch <= 4:
                    bad.append(ch)
                continue
            trim = self.mins[i] if ch == 3 else min(max(self.trims[i], self.mins[i]), self.maxs[i])
            out.update({f"RC{ch}_MIN": self.mins[i], f"RC{ch}_MAX": self.maxs[i], f"RC{ch}_TRIM": trim})
        if bad:
            raise VehicleError(f"Canais {bad} quase não se moveram. Mova todos os sticks até o fim do curso.")
        return out

    def save(self) -> list[dict]:
        res = self.v.set_params(self.proposal())
        self.state = "idle"
        return res

    def status(self) -> dict:
        return {"state": self.state, "mins": self.mins, "maxs": self.maxs, "trims": self.trims}


# ================================================== calibrações do ArduCopter 4.x
class AccelCal4(AccelCal):
    """6 posições feitas pela placa. Ela pede cada posição com COMMAND_LONG(ACCELCAL_VEHICLE_POS)
    e o GCS confirma mandando o mesmo comando com a posição. A placa não trava (heartbeat continua).
    """

    _pos_code = 0  # código ACCELCAL_VEHICLE_POS que a placa pediu por último

    def start(self) -> None:
        if self.state in ("starting", "waiting", "measuring"):
            raise VehicleError("A calibração já está em andamento.")
        if not self.v.connected:
            raise VehicleError("Não conectado.")
        self.v.require_disarmed()
        self.reset()
        self.before = {n: self.v.params.get(n) for n in self.NAMES}
        self.state = "starting"
        self._pos_code = 0
        self.v.command_long(M.MAV_CMD_PREFLIGHT_CALIBRATION, 0, 0, 0, 0, 1, 0, 0)

    def next(self) -> None:
        if self.state != "waiting":
            raise VehicleError("A placa não está esperando uma posição agora.")
        self.state = "measuring"
        self._ack_t = time.time()
        self.v.command_long(M.MAV_CMD_ACCELCAL_VEHICLE_POS, self._pos_code)

    def _ask(self, code: int, prompt: str | None) -> None:
        self._pos_code = code
        self.position = ACCELCAL_POS[code]
        self.step = self.ORDER.index(self.position) + 1
        if prompt:
            self.firmware_prompt = prompt
        self.state = "waiting"

    def on_command_long(self, msg: Any) -> None:
        if msg.command != M.MAV_CMD_ACCELCAL_VEHICLE_POS or self.state not in ("starting", "waiting", "measuring"):
            return
        code = int(msg.param1)
        if code in ACCELCAL_POS:
            self._ask(code, self.firmware_prompt if self.state == "waiting" else None)
        elif code == ACCELCAL_SUCCESS:
            self.state = "success"
            threading.Thread(target=self._read_result, daemon=True).start()
        elif code == ACCELCAL_FAILED:
            self.state = "failed"

    def on_text(self, text: str) -> None:
        if self.state not in ("starting", "waiting", "measuring"):
            return
        self.log.append(text)
        if text.startswith("Place vehicle"):
            # o texto chega junto com o COMMAND_LONG; serve de reserva se o comando se perder
            pos = next((k for f, k in self.KEYS if f in text), None)
            code = next((c for c, k in ACCELCAL_POS.items() if k == pos), 0)
            if code:
                self._ask(code, text)
        elif "successful" in text.lower():
            self.state = "success"
            threading.Thread(target=self._read_result, daemon=True).start()
        elif "failed" in text.lower():
            self.state = "failed"


class CompassCal4:
    """Calibração feita pela placa (MAV_CMD_DO_START_MAG_CAL). Ajusta elipsoide e detecta a
    orientação de cada bússola; o GCS só mostra o progresso e confirma o resultado.
    """

    def __init__(self, v: Vehicle) -> None:
        self.v = v
        self.reset()

    def reset(self) -> None:
        self.state = "idle"  # idle|running|done|failed|accepted
        self.progress: dict[int, dict] = {}
        self.reports: dict[int, dict] = {}
        self.started = 0.0

    def start(self) -> None:
        if not self.v.connected:
            raise VehicleError("Não conectado.")
        self.v.require_disarmed()
        s = self.v.msgs.get("SYS_STATUS")
        if not s or not (s.onboard_control_sensors_present & M.MAV_SYS_STATUS_SENSOR_3D_MAG):
            raise VehicleError("A placa não detectou bússola. Ligue COMPASS_ENABLE=1, conecte o módulo e reinicie a placa.")
        self.reset()
        # p1 máscara (0 = todas), p2 tentar de novo, p3 gravar sozinho, p4 atraso, p5 reiniciar sozinho
        res = self.v.command_wait_ack(M.MAV_CMD_DO_START_MAG_CAL, 0, 0, 0, 0, 0, timeout=3)
        if res not in (None, M.MAV_RESULT_ACCEPTED):
            removed = any("removed" in t and t.startswith("Mag:") for _, _, t in self.v.texts)
            raise VehicleError("A placa recusou a calibração: uma bússola da lista de prioridade não está ligada. "
                               "Use \"Usar as bússolas presentes e reiniciar\" e tente de novo." if removed
                               else f"A placa recusou a calibração (resultado {res}).")
        self.state = "running"
        self.started = time.time()

    def on_msg(self, msg: Any) -> None:
        if self.state not in ("running", "done", "failed"):
            return
        cid = msg.compass_id
        if msg.get_type() == "MAG_CAL_PROGRESS":
            if self.state == "running":
                mask = bytes(msg.completion_mask) if not isinstance(msg.completion_mask, (bytes, bytearray)) else msg.completion_mask
                self.progress[cid] = {"id": cid, "pct": msg.completion_pct, "status": msg.cal_status, "attempt": msg.attempt,
                                      "cells": [bool(b >> i & 1) for b in mask for i in range(8)]}
            return
        st = msg.cal_status
        self.reports[cid] = {
            "id": cid, "status": st, "ok": st == M.MAG_CAL_SUCCESS, "fitness": round(msg.fitness, 1),
            "offsets": [round(msg.ofs_x), round(msg.ofs_y), round(msg.ofs_z)],
            "diag": [round(msg.diag_x, 3), round(msg.diag_y, 3), round(msg.diag_z, 3)],
            "scale": round(getattr(msg, "scale_factor", 0) or 0, 3), "autosaved": bool(msg.autosaved),
            "orientation": getattr(msg, "new_orientation", None), "orientation_confidence": round(getattr(msg, "orientation_confidence", 0) or 0, 2),
            "message": M.enums["MAG_CAL_STATUS"][st].name.replace("MAG_CAL_", "") if st in M.enums["MAG_CAL_STATUS"] else str(st),
        }
        if set(self.progress) <= set(self.reports):
            self.state = "done" if all(r["ok"] for r in self.reports.values()) else "failed"

    def accept(self) -> dict:
        if self.state != "done":
            raise VehicleError("Nada para gravar: a calibração não terminou com sucesso.")
        res = self.v.command_wait_ack(M.MAV_CMD_DO_ACCEPT_MAG_CAL, 0, timeout=4)
        if res not in (None, M.MAV_RESULT_ACCEPTED):
            raise VehicleError(f"A placa recusou gravar (resultado {res}).")
        self.state = "accepted"
        self.v.fetch_params_async()
        return self.status()

    def cancel(self) -> None:
        if self.state == "running" and self.v.conn is not None:
            self.v.command_long(M.MAV_CMD_DO_CANCEL_MAG_CAL, 0)
        self.reset()

    def status(self) -> dict:
        st: dict[str, Any] = {"state": self.state, "progress": sorted(self.progress.values(), key=lambda p: p["id"]),
                              "reports": sorted(self.reports.values(), key=lambda r: r["id"])}
        if self.state == "running":
            st["elapsed"] = round(time.time() - self.started)
        return st
