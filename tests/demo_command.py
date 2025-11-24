import time
from pymavlink import mavutil
from old_src.connection import my_mavlink_connection

def _result_name(code: int) -> str:
    try:
        return mavutil.mavlink.enums['MAV_RESULT'][code].name
    except Exception:
        return str(code)

import time
from pymavlink import mavutil

# def send_cmd(conn, command, *, p1=0, p2=0, p3=0, p4=0, p5=0, p6=0, p7=0, confirmation=0, timeout=2.0):
#     # send
#     conn.mav.command_long_send(
#         conn.target_system, conn.target_component,
#         command, confirmation,
#         float(p1), float(p2), float(p3), float(p4),
#         float(p5), float(p6), float(p7),
#     )

#     # wait for the matching ACK (no condition lambda)
#     deadline = time.time() + timeout
#     while True:
#         remaining = deadline - time.time()
#         if remaining <= 0:
#             raise TimeoutError(f"COMMAND_ACK timeout for cmd={command}")

#         ack = conn.recv_match(type='COMMAND_ACK', blocking=True, timeout=remaining)
#         if ack is None:
#             continue
#         if getattr(ack, 'command', None) == command:
#             print("ACK:", ack.to_dict())
#             return ack

def send_cmd(conn, command, *, p1=0, p2=0, p3=0, p4=0, p5=0, p6=0, p7=0, confirmation=0, timeout=2.0):
    # Print what we're about to send
    print(f"[CMD] Sending COMMAND_LONG: cmd={command} ({mavutil.mavlink.enums['MAV_CMD'][command].name if command in mavutil.mavlink.enums['MAV_CMD'] else 'UNKNOWN'})")
    print(f"      tsys={conn.target_system} tcmp={conn.target_component} conf={confirmation}")
    print(f"      params: {p1}, {p2}, {p3}, {p4}, {p5}, {p6}, {p7}")

    # send
    conn.mav.command_long_send(
        conn.target_system, conn.target_component,
        command, confirmation,
        float(p1), float(p2), float(p3), float(p4),
        float(p5), float(p6), float(p7),
    )

    # wait for the matching ACK
    deadline = time.time() + timeout
    while True:
        remaining = deadline - time.time()
        if remaining <= 0:
            print(f"[ACK] Timeout waiting for COMMAND_ACK cmd={command}")
            raise TimeoutError(f"COMMAND_ACK timeout for cmd={command}")

        ack = conn.recv_match(type='COMMAND_ACK', blocking=True, timeout=remaining)
        if ack is None:
            print("[ACK] recv_match returned None; continuing...")
            continue

        # Print every ack we see (sometimes you get unrelated acks)
        try:
            ack_dict = ack.to_dict()
        except Exception:
            ack_dict = {"command": getattr(ack, "command", None), "result": getattr(ack, "result", None)}
        print(f"[ACK] Seen ACK: {ack_dict}")

        if getattr(ack, 'command', None) == command:
            res = getattr(ack, 'result', None)
            print(f"[ACK] Matched cmd={command} result={_result_name(res)} ({res})")
            return ack
        
def main():
    # conn = mavutil.mavlink_connection('udp:127.0.0.1:14550')
    conn = my_mavlink_connection('udp:127.0.0.1:14550')
    print("[SYS] Waiting for heartbeat...")
    conn.wait_heartbeat()
    print(f"[SYS] Got heartbeat from sys={conn.target_system} comp={conn.target_component}")

    # 1) Set message intervals (µs). e.g., ATTITUDE@10Hz, GPS_RAW_INT@5Hz
    MSG_ATTITUDE    = mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE
    MSG_GPS_RAW_INT = mavutil.mavlink.MAVLINK_MSG_ID_GPS_RAW_INT

    send_cmd(conn, mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, p1=MSG_ATTITUDE,    p2=100_000)  # 10 Hz
    send_cmd(conn, mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, p1=MSG_GPS_RAW_INT, p2=200_000)  # 5 Hz

    # 2) Request AUTOPILOT_VERSION and read it
    print("[REQ] Requesting AUTOPILOT_VERSION...")
    send_cmd(conn, mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE, p1=mavutil.mavlink.MAVLINK_MSG_ID_AUTOPILOT_VERSION)

    print("[RECV] Waiting for AUTOPILOT_VERSION...")
    apv = conn.recv_match(type="AUTOPILOT_VERSION", blocking=True, timeout=2.0)
    if apv:
        try:
            print("[RECV] AUTOPILOT_VERSION:", apv.to_dict())
        except Exception:
            print("[RECV] AUTOPILOT_VERSION (raw):", apv)
    else:
        print("[RECV] AUTOPILOT_VERSION: None (timeout)")

    # 3) (optional) Arm / Disarm
    print("[CMD] Arming...")
    send_cmd(conn, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, p1=1)
    print("[CMD] Disarming...")
    send_cmd(conn, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, p1=0)

    # (Optional) brief stream to show your rate change worked
    print("[STREAM] Reading a few ATTITUDE/GPS_RAW_INT messages...")
    
    count = 0
    while count < 200:
        m = conn.recv_match(type=['ATTITUDE','GPS_RAW_INT'], blocking=True, timeout=0.5)
        if m:
            try:
                print(f"[MSG] {m.get_type()}: {m.to_dict()}")
            except Exception:
                print(f"[MSG] {m.get_type()}: <unprintable>")
        count += 1

if __name__ == "__main__":
    main()
