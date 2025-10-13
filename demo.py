from src.wrapper import my_mavlink_connection 
from pymavlink import mavutil
from src.my_util import info_for
import time, json


m = my_mavlink_connection('udp:0.0.0.0:14550')

m.wait_heartbeat()

# MSG = mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE   # example
# hz  = 10
# interval_us = int(1e6 / hz)

# m.mav.command_long_send(
#     m.target_system, m.target_component,
#     mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
#     0,
#     MSG,            # param1: message ID
#     interval_us,    # param2: interval in microseconds (negative to stop)
#     0,0,0,0,0
# )

t_end = time.time() + 1
count = 0
while time.time() < t_end:
    # your normal reads; the hook runs under the hood on every message parsed
    msg = m.recv_msg()

    if msg is None or msg.get_type() == 'BAD_DATA' :
        continue
    
    info = info_for(msg)
    print(info)
    count += 1
    
m.close()

print(f"Total messages are: {count}")
