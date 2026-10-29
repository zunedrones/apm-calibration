"""Grava parametros e confere a leitura de volta. Uso: set_params.py NOME=VALOR ..."""
import sys, time
from pymavlink import mavutil
M = mavutil.mavlink
m = mavutil.mavlink_connection("/dev/ttyACM0", baud=115200)
m.wait_heartbeat(timeout=15)
time.sleep(4)  # boot apos reset da serial
ok = True
for arg in sys.argv[1:]:
    nome, valor = arg.split("="); valor = float(valor)
    m.mav.param_set_send(m.target_system, m.target_component, nome.encode(), valor, M.MAV_PARAM_TYPE_REAL32)
    t0 = time.time(); p = None
    while time.time() - t0 < 5:
        p = m.recv_match(type="PARAM_VALUE", blocking=True, timeout=1)
        if p and p.param_id == nome: break
        p = None
    lido = p.param_value if p else None
    print(f"{nome}: pedido={valor:g} lido={lido}")
    ok &= lido is not None and abs(lido - valor) < 1e-6
print("OK" if ok else "FALHOU"); sys.exit(0 if ok else 1)
