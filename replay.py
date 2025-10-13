from src.wrapper import my_mavlink_connection 
from src.my_util import info_for

mlog = my_mavlink_connection('parsed_log.tlog', tlog_path="replay_log.logg")
# out = open("parsed_log.tlog.txt", "w")

heartbeat = mlog.wait_heartbeat(timeout=5)
if not heartbeat:
    print("WTF")

count = 0
while True:
    msg = mlog.recv_msg()
    if not msg:
        break
    if msg.get_type() == 'BAD_DATA':
        print("BAD_DATA")
    
    info = info_for(msg)
    print(info)
    count += 1

print(f"Total messages are: {count}")

mlog.close()