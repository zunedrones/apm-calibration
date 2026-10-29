"""Conexão MAVLink com a APM 2.x (ArduCopter 3.2) e máquinas de estado das calibrações.

Uma única thread lê a serial e guarda a última mensagem de cada tipo; a API e o
WebSocket só leem esse estado. Envio é protegido por lock.
"""
from __future__ import annotations

import math
import re
import threading
import time
from collections import deque
from typing import Any, Callable

import numpy as np
from pymavlink import mavutil

from . import ardu32, magfit

M = mavutil.mavlink
SENSOR_NAMES = {k: v.name.replace("MAV_SYS_STATUS_", "") for k, v in M.enums["MAV_SYS_STATUS_SENSOR"].items() if k < (1 << 30)}


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
        self.accel = AccelCal(self)
        self.compass = CompassCal(self)
        self.rc = RcCal(self)

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
        self.last_ack: dict[int, Any] = {}

    @property
    def connected(self) -> bool:
        if self.conn is None:
            return False
        # durante a calibração do acelerômetro o 3.2 fica bloqueado e para de mandar heartbeat
        if self.accel.state in ("starting", "waiting", "measuring"):
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
        self._running = True
        self._thread = threading.Thread(target=self._loop, name="mavlink-rx", daemon=True)
        self._thread.start()
        self.connected_at = time.time()
        # abrir a USB reseta o ATmega2560 (DTR); espera heartbeat e o fim do boot
        if not self._wait(lambda: self.last_hb > 0, 15):
            self.disconnect()
            raise VehicleError("Sem heartbeat. Confira a porta, o baud (115200 na USB) e se outro programa está usando a serial.")
        self._wait(lambda: any("GROUND START" in t for _, _, t in self.texts), 8)
        time.sleep(0.5)
        self.request_streams(ardu32.DEFAULT_STREAM_HZ)
        self.fetch_params_async()

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
        self.accel.reset()
        self.compass.reset()
        self.rc.reset()

    def reboot(self) -> None:
        """Reinicia a placa. Na USB fechar/abrir a serial já reseta (DTR); o comando cobre o caso da telemetria."""
        if self.conn is None:
            raise VehicleError("Não conectado.")
        self.require_disarmed()
        port, baud = self.port, self.baud
        self.command_long(M.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN, 1)
        time.sleep(0.5)
        self.connect(port, baud)

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
            now = time.time()
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
            elif t == "RAW_IMU":
                self.compass.on_raw_imu(msg)
            elif t == "RC_CHANNELS_RAW":
                self.rc.on_rc(msg)

    def _on_text(self, now: float, sev: int, text: str) -> None:
        text = text.strip("\x00").strip()
        self.texts.append((now, sev, text))
        if re.search(r"(APM:Copter|ArduCopter)\s*V?\d", text):
            self.firmware = text
        elif text.startswith("Frame:"):
            self.frame_text = text.split(":", 1)[1].strip()
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
        """O 3.2 reenvia as falhas de PreArm periodicamente (~30 s) enquanto desarmado."""
        now = time.time()
        recent = sorted({t for ts, _, t in self.texts if t.startswith("PreArm") and now - ts < 40})
        watched = now - self.connected_at if self.connected_at else 0
        return {"messages": recent, "watched_s": round(watched), "conclusive": watched > 40}

    def snapshot(self) -> dict:
        m = self.msgs
        hb = m.get("HEARTBEAT")
        snap: dict[str, Any] = {
            "connected": self.connected,
            "port": self.port,
            "baud": self.baud,
            "hb_age": round(time.time() - self.last_hb, 1) if self.last_hb else None,
            "firmware": self.firmware,
            "frame_text": self.frame_text,
            "params": {"received": len(self.param_indices), "total": self.param_count, "loading": self.params_loading},
            "texts": [{"t": ts, "sev": s, "text": tx} for ts, s, tx in list(self.texts)[-40:]],
            "prearm": self.prearm(),
            "accel": self.accel.status(),
            "compass": self.compass.status(),
            "rc_cal": self.rc.status(),
        }
        if hb:
            snap["armed"] = bool(hb.base_mode & M.MAV_MODE_FLAG_SAFETY_ARMED)
            snap["mode"] = ardu32.FLIGHT_MODES.get(hb.custom_mode, str(hb.custom_mode))
            snap["custom_mode"] = hb.custom_mode
            snap["system_status"] = M.enums["MAV_STATE"][hb.system_status].name.replace("MAV_STATE_", "") if hb.system_status in M.enums["MAV_STATE"] else hb.system_status
        if (s := m.get("SYS_STATUS")):
            snap["sensors"] = [
                {"name": n, "present": bool(s.onboard_control_sensors_present & b), "enabled": bool(s.onboard_control_sensors_enabled & b), "healthy": bool(s.onboard_control_sensors_health & b)}
                for b, n in sorted(SENSOR_NAMES.items()) if s.onboard_control_sensors_present & b
            ]
            snap["load_pct"] = s.load / 10
            snap["battery"] = {"voltage": s.voltage_battery / 1000, "current": s.current_battery / 100 if s.current_battery >= 0 else None, "remaining": s.battery_remaining}
        if (h := m.get("HWSTATUS")):
            snap["vcc"] = h.Vcc / 1000
            snap["i2c_err"] = h.I2Cerr
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
        if (rc := m.get("RC_CHANNELS_RAW")):
            snap["rc"] = [getattr(rc, f"chan{i}_raw") for i in range(1, 9)]
            snap["rc_age"] = round(time.time() - self.msg_time["RC_CHANNELS_RAW"], 1)
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
        self.v.request_streams(ardu32.DEFAULT_STREAM_HZ, M.MAV_DATA_STREAM_RAW_SENSORS)
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
        rc = self.v.msgs.get("RC_CHANNELS_RAW")
        if rc is None:
            raise VehicleError("Sem dados de rádio.")
        cur = [getattr(rc, f"chan{i}_raw") for i in range(1, 9)]
        self.trims = cur
        self.v.request_streams(ardu32.DEFAULT_STREAM_HZ, M.MAV_DATA_STREAM_RC_CHANNELS)
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
