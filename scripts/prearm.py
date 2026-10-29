"""Escuta STATUSTEXT (PreArm) e salva backup dos parametros."""
import json, sys, time
from pymavlink import mavutil
m = mavutil.mavlink_connection("/dev/ttyACM0", baud=115200)
m.wait_heartbeat(timeout=15)
m.mav.request_data_stream_send(m.target_system, m.target_component, mavutil.mavlink.MAV_DATA_STREAM_ALL, 4, 1)  # 4 Hz = valor original de SR0_* (o 3.2 grava a taxa pedida)
t0 = time.time(); textos = []
while time.time() - t0 < 45:
    msg = m.recv_match(type=["STATUSTEXT", "SYS_STATUS"], blocking=True, timeout=1)
    if msg and msg.get_type() == "STATUSTEXT":
        print(f"[{time.time()-t0:5.1f}s] {msg.text}"); textos.append(msg.text)
print("PreArm pendentes:", sorted({t for t in textos if t.startswith("PreArm")}) or "nenhum")
if len(sys.argv) > 1:
    m.mav.param_request_list_send(m.target_system, m.target_component)
    params, total, t0 = {}, None, time.time()
    while time.time() - t0 < 60 and (total is None or len(params) < total):
        p = m.recv_match(type="PARAM_VALUE", blocking=True, timeout=3)
        if p: params[p.param_id] = p.param_value; total = p.param_count
    print(f"params: {len(params)}/{total}")
    json.dump(params, open(f"/work/backup/{sys.argv[1]}.json", "w"), indent=1, sort_keys=True)
    open(f"/work/backup/{sys.argv[1]}.param", "w").write("".join(f"{k},{v:.6g}\n" for k, v in sorted(params.items())))
