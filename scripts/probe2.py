import time, statistics as st
from pymavlink import mavutil
M = mavutil.mavlink
m = mavutil.mavlink_connection("/dev/ttyACM0", baud=115200)
t0=time.time()
texts=[]
hb = m.wait_heartbeat(timeout=10)
# versao: no Copter 3.x o MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES / PREFLIGHT_STORAGE nao manda string; usamos 'set REQUEST' via param fetch
m.mav.command_long_send(m.target_system, m.target_component, M.MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES, 0, 1,0,0,0,0,0,0)
m.mav.request_data_stream_send(m.target_system, m.target_component, M.MAV_DATA_STREAM_ALL, 4, 1)  # 4 Hz = valor original de SR0_* (o 3.2 grava a taxa pedida)
acc=[];gyr=[];mag=[];sysst=None
while time.time()-t0<25:
    msg=m.recv_match(blocking=True,timeout=1)
    if not msg: continue
    t=msg.get_type()
    if t=="STATUSTEXT": texts.append(round(time.time()-t0,1)); print(f"[{time.time()-t0:5.1f}s] {msg.text}")
    elif t=="RAW_IMU": acc.append((msg.xacc,msg.yacc,msg.zacc)); gyr.append((msg.xgyro,msg.ygyro,msg.zgyro)); mag.append((msg.xmag,msg.ymag,msg.zmag))
    elif t=="SYS_STATUS": sysst=msg
    elif t in ("AUTOPILOT_VERSION","COMMAND_ACK"): print(msg.to_dict())
def stats(name, arr, unit):
    for i,ax in enumerate("xyz"):
        v=[a[i] for a in arr]; print(f"  {name}.{ax}: media={st.mean(v):8.1f} std={st.pstdev(v):6.2f} {unit}")
print(f"amostras IMU: {len(acc)}")
stats("acc",acc,"mG"); stats("gyro",gyr,"mrad/s"); stats("mag",mag,"mGauss")
names={v.name:k for k,v in M.enums['MAV_SYS_STATUS_SENSOR'].items()}
print("Sensores (presente/habilitado/saudavel):")
for k,v in sorted(M.enums['MAV_SYS_STATUS_SENSOR'].items()):
    if k>=1<<30 or not (sysst.onboard_control_sensors_present & k): continue
    print(f"  {v.name.replace('MAV_SYS_STATUS_',''):32s} P={1} E={int(bool(sysst.onboard_control_sensors_enabled&k))} H={int(bool(sysst.onboard_control_sensors_health&k))}")
