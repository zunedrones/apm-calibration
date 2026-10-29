"""API HTTP + WebSocket da interface de calibração."""
from __future__ import annotations

import asyncio
import os
import time
from contextlib import asynccontextmanager
from pathlib import Path
from typing import Any

from fastapi import FastAPI, HTTPException, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse, PlainTextResponse
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel
from pymavlink import mavutil
from serial.tools import list_ports

from . import ardu32
from .vehicle import Vehicle, VehicleError

M = mavutil.mavlink
STATIC = Path(__file__).parent / "static"

veh = Vehicle()


@asynccontextmanager
async def lifespan(_: FastAPI):
    port = os.environ.get("APM_PORT")
    if port and os.environ.get("APM_AUTOCONNECT", "0") == "1" and os.path.exists(port):
        try:
            await asyncio.to_thread(veh.connect, port, int(os.environ.get("APM_BAUD", "115200")))
        except VehicleError:
            pass
    yield
    veh.disconnect()


app = FastAPI(title="Zune Drones — Calibração APM 2.x", lifespan=lifespan)


def guard(fn, *a, **kw) -> Any:
    try:
        return fn(*a, **kw)
    except VehicleError as e:
        raise HTTPException(400, str(e)) from e


# ---------------------------------------------------------------- conexão
class ConnectReq(BaseModel):
    port: str
    baud: int = 115200


@app.get("/api/ports")
def ports() -> list[dict]:
    out = []
    for p in list_ports.comports():
        if not (p.device.startswith(("/dev/ttyACM", "/dev/ttyUSB")) or p.vid):
            continue
        out.append({"device": p.device, "description": p.description, "vid": p.vid, "pid": p.pid,
                    "apm": p.vid == 0x2341 and p.pid == 0x0010})
    return sorted(out, key=lambda d: (not d["apm"], d["device"]))


@app.post("/api/connect")
def connect(req: ConnectReq) -> dict:
    guard(veh.connect, req.port, req.baud)
    return veh.snapshot()


@app.post("/api/disconnect")
def disconnect() -> dict:
    veh.disconnect()
    return {"ok": True}


@app.post("/api/reboot")
def reboot() -> dict:
    guard(veh.reboot)
    return veh.snapshot()


@app.get("/api/meta")
def meta() -> dict:
    return {
        "modes": ardu32.FLIGHT_MODES, "gps_modes": sorted(ardu32.GPS_MODES), "fltmode_pwm": ardu32.FLTMODE_PWM,
        "frames": ardu32.FRAMES, "motor_layout": ardu32.MOTOR_TEST_LAYOUT, "presets": ardu32.PRESETS,
        "arming_bits": ardu32.ARMING_CHECK_BITS, "power_module": ardu32.APM2_POWER_MODULE,
    }


# ------------------------------------------------------------- parâmetros
class ParamsReq(BaseModel):
    params: dict[str, float]


class ImportReq(BaseModel):
    text: str


@app.get("/api/params")
def params() -> dict:
    return {"params": veh.params, "total": veh.param_count, "loading": veh.params_loading}


@app.post("/api/params/refresh")
def params_refresh() -> dict:
    guard(veh.fetch_params_async)
    return {"ok": True}


@app.post("/api/params/set")
def params_set(req: ParamsReq) -> list[dict]:
    if not veh.connected:
        raise HTTPException(400, "Não conectado.")
    for name in req.params:
        if name not in veh.params:
            raise HTTPException(400, f"Parâmetro desconhecido nesta placa: {name}")
    return veh.set_params(req.params)


@app.get("/api/params/export")
def params_export() -> PlainTextResponse:
    if not veh.params:
        raise HTTPException(400, "Nenhum parâmetro carregado.")
    body = "".join(f"{k},{v:.7g}\n" for k, v in sorted(veh.params.items()))
    name = time.strftime("apm2-params-%Y%m%d-%H%M%S.param")
    return PlainTextResponse(body, headers={"Content-Disposition": f'attachment; filename="{name}"'})


def parse_param_file(text: str) -> dict[str, float]:
    """Aceita o formato do Mission Planner/APM Planner: 'NOME,VALOR' (ou espaço/tab)."""
    out: dict[str, float] = {}
    for line in text.splitlines():
        line = line.split("#", 1)[0].strip()
        if not line:
            continue
        parts = line.replace("\t", ",").replace(" ", ",").split(",")
        parts = [p for p in parts if p]
        if len(parts) < 2:
            continue
        try:
            out[parts[0].strip()] = float(parts[1])
        except ValueError:
            continue
    return out


@app.post("/api/params/diff")
def params_diff(req: ImportReq) -> dict:
    new = parse_param_file(req.text)
    if not new:
        raise HTTPException(400, "Arquivo sem parâmetros válidos.")
    changes, unknown, skipped = [], [], []
    for k, v in sorted(new.items()):
        if k in ardu32.PARAMS_SKIP_RESTORE or k.startswith(ardu32.PARAMS_SKIP_RESTORE_PREFIX):
            skipped.append(k)
        elif k not in veh.params:
            unknown.append(k)
        elif abs(veh.params[k] - v) > max(1e-6, 1e-5 * abs(v)):
            changes.append({"name": k, "current": veh.params[k], "new": v})
    return {"changes": changes, "unknown": unknown, "skipped": skipped, "total": len(new)}


# ----------------------------------------------------------------- presets
@app.post("/api/preset/{name}")
def preset(name: str) -> dict:
    if name not in ardu32.PRESETS:
        raise HTTPException(404, "Preset desconhecido.")
    values = ardu32.PRESETS[name]
    changed = {k for k, v in values.items() if veh.params.get(k) != v}
    res = veh.set_params(values)
    return {"results": res, "needs_reboot": bool(changed & ardu32.PRESET_NEEDS_REBOOT)}


# ------------------------------------------------------------- calibrações
@app.post("/api/accel/{action}")
def accel(action: str) -> dict:
    acts = {"start": veh.accel.start, "next": veh.accel.next, "reset": veh.accel.reset}
    if action not in acts:
        raise HTTPException(404)
    guard(acts[action])
    return veh.accel.status()


@app.post("/api/compass/{action}")
def compass(action: str) -> Any:
    acts = {"start": veh.compass.start, "stop": veh.compass.stop, "apply": veh.compass.apply, "reset": veh.compass.reset}
    if action not in acts:
        raise HTTPException(404)
    out = guard(acts[action])
    return out if out is not None else veh.compass.status()


@app.post("/api/rc/{action}")
def rc(action: str) -> Any:
    acts = {"start": veh.rc.start, "finish": veh.rc.finish, "proposal": veh.rc.proposal, "save": veh.rc.save, "reset": veh.rc.reset}
    if action not in acts:
        raise HTTPException(404)
    out = guard(acts[action])
    return out if out is not None else veh.rc.status()


@app.post("/api/calibrate/{what}")
def calibrate(what: str) -> dict:
    # MAV_CMD_PREFLIGHT_CALIBRATION: p1 giro, p3 baro, p5=2 nivelamento (AHRS_TRIM)
    params = {"gyro": (1, 0, 0, 0, 0), "baro": (0, 0, 1, 0, 0), "level": (0, 0, 0, 0, 2)}
    if what not in params:
        raise HTTPException(404)
    guard(veh.require_disarmed)
    result = guard(veh.command_wait_ack, M.MAV_CMD_PREFLIGHT_CALIBRATION, *params[what], timeout=8)
    return {"result": result, "ok": result == M.MAV_RESULT_ACCEPTED}


class MotorTestReq(BaseModel):
    motor: int          # 1..4 na ordem de teste (A..D)
    throttle: float = 7  # %
    duration: float = 2  # s
    props_removed: bool = False


@app.post("/api/motor_test")
def motor_test(req: MotorTestReq) -> dict:
    if not req.props_removed:
        raise HTTPException(400, "Confirme que as hélices foram removidas.")
    if not 1 <= req.motor <= 8 or not 0 < req.throttle <= 30 or not 0 < req.duration <= 5:
        raise HTTPException(400, "Valores fora da faixa segura (throttle até 30 %, até 5 s).")
    guard(veh.require_disarmed)
    # MAV_CMD_DO_MOTOR_TEST: p1 sequência, p2 tipo (0 = %), p3 valor, p4 timeout
    result = guard(veh.command_wait_ack, M.MAV_CMD_DO_MOTOR_TEST, req.motor, 0, req.throttle, req.duration, timeout=3)
    msgs = [t for ts, _, t in veh.texts if t.startswith("Motor Test") and time.time() - ts < 4]
    return {"result": result, "ok": result == M.MAV_RESULT_ACCEPTED, "messages": msgs}


# --------------------------------------------------------------- telemetria
@app.get("/api/status")
def status() -> dict:
    return veh.snapshot()


@app.websocket("/ws")
async def ws(sock: WebSocket) -> None:
    await sock.accept()
    try:
        while True:
            await sock.send_json(await asyncio.to_thread(veh.snapshot))
            await asyncio.sleep(0.2)
    except (WebSocketDisconnect, RuntimeError):
        pass


@app.get("/")
def index() -> FileResponse:
    return FileResponse(STATIC / "index.html")


app.mount("/static", StaticFiles(directory=STATIC), name="static")
