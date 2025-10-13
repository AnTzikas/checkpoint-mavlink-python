from src.wrapper import my_mavlink_connection 

conn = my_mavlink_connection('udp:127.0.0.1:14550')
# conn.wait_heartbeat()


print("Wait for heartbeat!")
conn.wait_heartbeat()

print("Got heartbeat!")

msg = conn.recv_match(type="HEARTBEAT", blocking=True, timeout=5) 

print("got Message")
if msg is not None:
    print(f"Developer recv_match got: {msg.get_type()}")

print("Exit")