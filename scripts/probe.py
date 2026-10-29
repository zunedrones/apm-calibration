import sys, time
from pymavlink import mavutil
for baud in (115200, 57600):
    print(f"== tentando {baud} baud")
    m = mavutil.mavlink_connection("/dev/ttyACM0", baud=baud)
    hb = m.wait_heartbeat(timeout=8)
    if hb:
        break
    m.close()
else:
    print("SEM HEARTBEAT"); sys.exit(1)
print("HEARTBEAT:", hb.to_dict())
print("autopilot type:", mavutil.mavlink.enums['MAV_AUTOPILOT'][hb.autopilot].name,
      "| vehicle:", mavutil.mavlink.enums['MAV_TYPE'][hb.type].name,
      "| mode:", m.flightmode, "| armed:", bool(hb.base_mode & 128))
m.mav.request_data_stream_send(m.target_system, m.target_component, mavutil.mavlink.MAV_DATA_STREAM_ALL, 4, 1)
seen = {}
t0 = time.time()
while time.time() - t0 < 12:
    msg = m.recv_match(blocking=True, timeout=1)
    if not msg: continue
    t = msg.get_type()
    if t == "STATUSTEXT":
        print("STATUSTEXT:", msg.text)
    if t not in seen:
        seen[t] = msg
for t in ["SYS_STATUS","RAW_IMU","SCALED_PRESSURE","ATTITUDE","GPS_RAW_INT","HWSTATUS","VFR_HUD","RC_CHANNELS_RAW","SERVO_OUTPUT_RAW","SENSOR_OFFSETS","MEMINFO","POWER_STATUS"]:
    if t in seen: print(t, seen[t].to_dict())
print("tipos recebidos:", sorted(seen))
# versao do firmware
m.mav.param_request_read_send(m.target_system, m.target_component, b"SYSID_SW_MREV", -1)
m.mav.param_request_list_send(m.target_system, m.target_component)
params = {}; t0=time.time(); total=None
while time.time()-t0 < 60:
    p = m.recv_match(type=["PARAM_VALUE","STATUSTEXT"], blocking=True, timeout=3)
    if p is None:
        if total and len(params)>=total: break
        continue
    if p.get_type()=="STATUSTEXT": print("STATUSTEXT:", p.text); continue
    params[p.param_id]=p.param_value; total=p.param_count
    if len(params)>=total: break
print(f"params: {len(params)}/{total}")
import json; json.dump(params, open("/work/params.json","w"), indent=1, sort_keys=True)
for k in ["SYSID_SW_MREV","SYSID_SW_TYPE","FRAME","INS_GYROFFS_X","INS_ACCOFFS_X","INS_ACCSCAL_X","COMPASS_OFS_X","COMPASS_OFS_Y","COMPASS_OFS_Z","COMPASS_USE","RC1_MIN","RC1_MAX","RC3_MIN","RC3_MAX","FLTMODE1","ARMING_CHECK","BATT_MONITOR","SERIAL1_BAUD","AHRS_ORIENTATION"]:
    if k in params: print(f"  {k} = {params[k]}")
