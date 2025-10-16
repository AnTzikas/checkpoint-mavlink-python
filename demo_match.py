from src.connection import my_mavlink_connection
from pymavlink import mavutil

def main():
    conn = my_mavlink_connection('udp:127.0.0.1:14550')

    conn.wait_heartbeat()
    
    # Choose the 3–4 kinds you want to capture
    # wanted_types = ['HEARTBEAT', 'ATTITUDE', 'GPS_RAW_INT', 'SYS_STATUS']
    wanted_types = sorted(n.replace('MAVLINK_MSG_ID_', '')
                        for n in dir(mavutil.mavlink)
                        if n.startswith('MAVLINK_MSG_ID_'))
    
    try:
        count = 0
        # Keep pulling only the wanted types until we've seen each at least once
        while count < 15:
            # Single call that filters to just the types we care about
            count += 1
            print(f"Call recv_match time no: {count}")
            msg = conn.recv_match(type=wanted_types, blocking=True)

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
                

    finally:
        conn.close()

if __name__ == "__main__":
    main()
