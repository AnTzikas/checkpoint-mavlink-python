# simple_flight.py
import math, time, sys
from pymavlink import mavutil

CONN = "udpin:0.0.0.0:14550"   # matches your sim_vehicle --out target
TAKEOFF_ALT = 10               # meters AGL

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
        if getattr(m, "flightmode", None) == name:
            print(f"    now {name}")
            return
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
    # wait using GLOBAL_POSITION_INT (relative_alt in mm)
    t0=time.time()
    while True:
        g = m.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=1.5)
        if not g: continue
        alt_rel = g.relative_alt/1000.0
        if alt_rel >= alt-0.8 and time.time()-t0>2:
            print("    reached")
            return

def now_boot_ms(m):
    # best-effort time base from latest GLOBAL_POSITION_INT
    g = m.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=0.5)
    if not g: return 0
    base = g.time_boot_ms
    wall = time.time()
    return lambda: base + int((time.time()-wall)*1000)

def goto_latlon_alt(m, lat, lon, alt, timeout=60):
    print(f"[*] Goto lat={lat:.7f} lon={lon:.7f} alt={alt} (rel)")
    _now = now_boot_ms(m)

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
                _now(),
                m.target_system, m.target_component,
                mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                mask,
                int(lat*1e7), int(lon*1e7), float(alt),
                0,0,0, 0,0,0, 0, 0
            )
            last = time.time()

        g = m.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=0.5)
        if not g: continue
        cur_lat, cur_lon = g.lat/1e7, g.lon/1e7
        cur_alt = g.relative_alt/1000.0
        d = haversine_m(cur_lat, cur_lon, lat, lon)
        print(f"    d={d:5.1f}m alt={cur_alt:4.1f}m   ", end="\r")
        if d < 2.5 and abs(cur_alt-alt) < 1.5:
            print("\n    reached")
            return
    print("\n    timeout (continuing)")

def haversine_m(lat1, lon1, lat2, lon2):
    R=6371000.0
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dphi = math.radians(lat2-lat1)
    dl   = math.radians(lon2-lon1)
    a = math.sin(dphi/2)**2 + math.cos(p1)*math.cos(p2)*math.sin(dl/2)**2
    return 2*R*math.asin(math.sqrt(a))

def offset_deg(lat, lon, dx_m, dy_m):
    # +dx east, +dy north (flat approx good for small distances)
    dlat = dy_m/111320.0
    dlon = dx_m/(40075000.0*math.cos(math.radians(lat))/360.0)
    return lat + dlat, lon + dlon

def main():
    m = mavutil.mavlink_connection(CONN, source_system=255, source_component=5)
    wait_hb(m)
    set_mode(m, "GUIDED")
    arm_and_wait(m)
    takeoff(m, TAKEOFF_ALT)

    # current position (from last GLOBAL_POSITION_INT)
    g = m.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=2)
    if not g:
        print("No GLOBAL_POSITION_INT; exiting"); sys.exit(1)
    home_lat, home_lon = g.lat/1e7, g.lon/1e7

    # A: 50m east, 0m north @ takeoff alt
    A_lat, A_lon = offset_deg(home_lat, home_lon, dx_m=50, dy_m=0)
    goto_latlon_alt(m, A_lat, A_lon, TAKEOFF_ALT)

    # B: 0m east, 60m north @ takeoff alt
    B_lat, B_lon = offset_deg(home_lat, home_lon, dx_m=0, dy_m=60)
    goto_latlon_alt(m, B_lat, B_lon, TAKEOFF_ALT)

    # Return and land
    print("[*] RTL")
    set_mode(m, "RTL")
    print("[*] Done")

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n[!] Interrupted")
