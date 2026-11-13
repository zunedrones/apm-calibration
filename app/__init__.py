import os

# Dialeto MAVLink 2 para receber da Pixhawk (ArduCopter 4.x). Ele também lê MAVLink 1
# (APM 2.x); o envio é sempre forçado em MAVLink 1, que as duas placas entendem.
# Precisa vir antes de qualquer import do pymavlink.
os.environ.setdefault("MAVLINK20", "1")
