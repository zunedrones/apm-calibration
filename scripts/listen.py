"""Escuta passiva (nao pede streams, nao altera SR0_*): conta mensagens por tipo."""
import time
from collections import Counter
from pymavlink import mavutil
m = mavutil.mavlink_connection("/dev/ttyACM0", baud=115200)
m.wait_heartbeat(timeout=15)
time.sleep(4)
c = Counter(); t0 = time.time()
while time.time() - t0 < 10:
    x = m.recv_match(blocking=True, timeout=1)
    if x: c[x.get_type()] += 1
print({k: f"{v/10:.1f} Hz" for k, v in c.most_common()})
