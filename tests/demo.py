from old_src.connection import my_mavlink_connection 
from pymavlink import mavutil
import time, json


m = my_mavlink_connection('udp:0.0.0.0:14550')

m.wait_heartbeat()

MSG = mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE   # example
hz  = 10
interval_us = int(1e6 / hz)

m.mav.command_long_send(
    m.target_system, m.target_component,
    mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
    0,
    MSG,            # param1: message ID
    interval_us,    # param2: interval in microseconds (negative to stop)
    0,0,0,0,0
)

count = 0
while count < 10:
    # your normal reads; the hook runs under the hood on every message parsed
    msg = m.recv_match()

    if msg is None:
        print("None continue\n")
        continue
    if msg.get_type() == 'BAD_DATA':
        print("Bad data continue\n")
        continue
    mtype = msg.get_type()
        # Pretty compact printout without relying on external helpers
    try:
        as_dict = msg.to_dict()
    except Exception:
        as_dict = {k: getattr(msg, k) for k in dir(msg) if not k.startswith('_')}
    print(f"{mtype}: {as_dict}")
    
    count += 1
    
m.close()

print(f"Total messages are: {count}")
