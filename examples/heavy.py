# actions_heavy.py
import os, time, math, hashlib
from pymavlink import mavutil

CONN = "udpin:0.0.0.0:14550"
TAKEOFF_ALT = 15

def wait_hb(m, t=15):
    print("[*] Waiting HEARTBEAT…")
    if not m.recv_match(type="HEARTBEAT", blocking=True, timeout=t):
        raise RuntimeError("No HEARTBEAT")
    print(f"    sys={m.target_system} comp={m.target_component}")

def set_mode(m, name, t=10):
    print(f"[*] Mode -> {name}")
    m.set_mode(name)
    t0 = time.time()
    while time.time()-t0 < t:
        m.recv_match(type="HEARTBEAT", blocking=True, timeout=1)
        if getattr(m, "flightmode", None) == name: return
    raise RuntimeError(f"Failed to set mode {name}")

def arm_and_wait(m):
    print("[*] Arming…")
    m.mav.command_long_send(m.target_system, m.target_component,
                            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                            0, 1, 0, 0,0,0,0,0)
    m.motors_armed_wait()
    print("    ARMED")

def takeoff(m, alt):
    print(f"[*] Takeoff to {alt} m")
    m.mav.command_long_send(m.target_system, m.target_component,
                            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
                            0, 0,0,0,0, 0,0, float(alt))
    t0=time.time()
    while True:
        g = m.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=1.5)
        if not g: continue
        alt_rel = g.relative_alt/1000.0
        if alt_rel >= alt-0.8 and time.time()-t0>2:
            print("    reached")
            return

def haversine(lat1, lon1, lat2, lon2):
    R=6371000.0
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dphi = math.radians(lat2-lat1)
    dl   = math.radians(lon2-lon1)
    a = math.sin(dphi/2)**2 + math.cos(p1)*math.cos(p2)*math.sin(dl/2)**2
    return 2*R*math.asin(math.sqrt(a))

def offset(lat, lon, dx_m, dy_m):
    dlat = dy_m/111320.0
    dlon = dx_m/(40075000.0*math.cos(math.radians(lat))/360.0)
    return lat + dlat, lon + dlon

def boot_clock(master):
    g = master.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=0.5)
    if not g: return lambda: 0
    base, wall = g.time_boot_ms, time.time()
    return lambda: base + int((time.time()-wall)*1000)

def goto_global_int(m, lat, lon, alt, timeout=120):
    print(f"[*] Goto {lat:.7f},{lon:.7f} a={alt}")
    now_boot = boot_clock(m)
    ignore_v = (mavutil.mavlink.POSITION_TARGET_TYPEMASK_VX_IGNORE
                | mavutil.mavlink.POSITION_TARGET_TYPEMASK_VY_IGNORE
                | mavutil.mavlink.POSITION_TARGET_TYPEMASK_VZ_IGNORE
                | mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE)
    ignore_a = (mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE
                | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE
                | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE)
    mask = ignore_v | ignore_a
    deadline = time.time()+timeout
    last = 0
    while time.time() < deadline:
        if time.time()-last > 0.5:
            m.mav.set_position_target_global_int_send(
                now_boot(), m.target_system, m.target_component,
                mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, mask,
                int(lat*1e7), int(lon*1e7), float(alt),
                0,0,0, 0,0,0, 0,0
            )
            last = time.time()
        g = m.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=0.5)
        if g:
            clat, clon = g.lat/1e7, g.lon/1e7
            calt = g.relative_alt/1000.0
            d = haversine(clat,clon,lat,lon)
            print(f"    d={d:5.1f}m alt={calt:4.1f}m   ", end="\r")
            if d < 2.5 and abs(calt-alt) < 1.5:
                print("\n    reached")
                return
    print("\n    timeout")

# --------- EXTRA ACTIONS ---------
def change_speed(m, mps=5.0):
    print(f"[*] DO_CHANGE_SPEED -> {mps} m/s")
    # param1 speed type=1 (ground speed), param2 speed (m/s)
    m.mav.command_long_send(m.target_system, m.target_component,
                            mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED,
                            0, 1, float(mps), -1, 0, 0,0,0)

def condition_yaw(m, heading_deg=90, rate_deg_s=20, clockwise=True, absolute=True):
    print(f"[*] CONDITION_YAW -> hdg={heading_deg} rate={rate_deg_s}")
    m.mav.command_long_send(m.target_system, m.target_component,
                            mavutil.mavlink.MAV_CMD_CONDITION_YAW,
                            0, float(heading_deg), float(rate_deg_s),
                            1 if clockwise else -1,
                            1 if absolute else 0, 0,0,0)

def start_capture(m, interval_s=2.0, count=0, comp=0):
    # interval=0 -> single shot; count=0 -> continuous until STOP
    print(f"[*] IMAGE_START_CAPTURE interval={interval_s}s count={count or '∞'}")
    m.mav.command_long_send(m.target_system, comp,
                            mavutil.mavlink.MAV_CMD_IMAGE_START_CAPTURE,
                            0, float(interval_s), float(count), 0,0,0,0,0)

def stop_capture(m, comp=0):
    print("[*] IMAGE_STOP_CAPTURE")
    m.mav.command_long_send(m.target_system, comp,
                            mavutil.mavlink.MAV_CMD_IMAGE_STOP_CAPTURE,
                            0, 0,0,0,0,0,0,0)

def set_msg_interval(m, msg_id, hz):
    interval_us = 0 if hz<=0 else int(1e6/hz)
    print(f"[*] SET_MESSAGE_INTERVAL id={msg_id} -> {hz} Hz")
    m.mav.command_long_send(m.target_system, m.target_component,
                            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                            0, float(msg_id), float(interval_us), 0,0,0,0,0)

def mavftp_put_get(m, size_mb=5, remote="/fs/microsd/test/test.bin", local="./roundtrip.bin"):
    try:
        from pymavlink.tools.mavftp import MAVFtp
        os.makedirs(os.path.dirname(local) or ".", exist_ok=True)
        os.makedirs(os.path.dirname(remote) or "/", exist_ok=True)
        # make data
        data = os.urandom(size_mb*1024*1024)
        md = hashlib.sha256(data).hexdigest()
        ftp = MAVFtp(m, timeout=10)
        print(f"[*] MAVFTP PUT {size_mb}MB -> {remote}")
        ftp.put(remote, data)
        print("[*] MAVFTP GET <- file")
        back = ftp.get(remote)
        md2 = hashlib.sha256(back).hexdigest()
        ok = (md == md2 and len(back)==len(data))
        with open(local, "wb") as f: f.write(back)
        print(f"    ok={ok} saved={len(back)} bytes -> {local}")
    except Exception as e:
        print(f"[ftp] error/skip: {e}")

# --------- MAIN TEST SEQUENCE ---------
def main():
    m = mavutil.mavlink_connection(CONN, source_system=255, source_component=5)
    wait_hb(m)
    set_mode(m, "GUIDED")
    arm_and_wait(m)
    takeoff(m, TAKEOFF_ALT)

    # boost GLOBAL_POSITION_INT to 10 Hz to stress inbound
    set_msg_interval(m, mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT, 10)

    # get current position
    g = m.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=2)
    home_lat, home_lon = g.lat/1e7, g.lon/1e7

    # change speed to 8 m/s
    change_speed(m, 8.0)

    # start continuous capture (every 2s), will stop later
    start_capture(m, interval_s=2.0, count=0)

    # fly to A (60 m east)
    A_lat, A_lon = offset(home_lat, home_lon, dx_m=60, dy_m=0)
    goto_global_int(m, A_lat, A_lon, TAKEOFF_ALT)

    # yaw to face north slowly
    condition_yaw(m, heading_deg=0, rate_deg_s=15, clockwise=True, absolute=True)

    # optional: FTP roundtrip 5MB file
    mavftp_put_get(m, size_mb=5, remote="/fs/microsd/test/test.bin", local="./roundtrip.bin")

    # fly to B (then north 80 m)
    B_lat, B_lon = offset(home_lat, home_lon, dx_m=60, dy_m=80)
    goto_global_int(m, B_lat, B_lon, TAKEOFF_ALT)

    # stop capture
    stop_capture(m)

    # slow down to 3 m/s
    change_speed(m, 3.0)

    # return and land
    print("[*] RTL")
    set_mode(m, "RTL")
    print("[*] Done")

if __name__ == "__main__":
    main()
