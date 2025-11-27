import time
import math
from pymavlink import mavutil
from src.mavlink_wrapper import mavlink_connection_wrapper

# --- MISSION CONFIGURATION ---
# THESIS NOTE: Larger radius and slower speed = longer flight legs.
# This ensures we have a large window (20+ seconds) to trigger a crash 
# mid-flight and test the wrapper's "State Injection" replay.
TARGET_ALTITUDE = 15     # Meters
PATROL_RADIUS = 80       # Meters (Large square)
SPEED_MPS = 1            # Meters/Second (Slow enough to observe)
LOITER_TIME = 5.0        # Seconds to hover at corners
TOTAL_LAPS = 5         
CONNECTION_STR = 'udpin:localhost:14551'

# ------------------------------------------------------------------------------
# HELPER: DETERMINISTIC LOITER
# ------------------------------------------------------------------------------
def loiter_deterministic(connection, duration_sec):
    """
    Waits for 'duration_sec' using VEHICLE time (time_boot_ms).
    
    [Thesis Verification]: 
    In Live Mode, this takes real seconds.
    In Replay Mode, the wrapper feeds logs instantly, so this function 
    returns immediately, proving the 'Fast Catch-up' capability.
    """
    print(f"   [Action] Scanning area for {duration_sec}s...")
    
    # Get Start Time from the Wrapper (Input Boundary)
    start_msg = connection.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
    if not start_msg: return
    start_ms = start_msg.time_boot_ms
    
    duration_ms = duration_sec * 1000
    
    while True:
        # Get Current Time from the Wrapper
        msg = connection.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
        if not msg: continue
        
        if (msg.time_boot_ms - start_ms) >= duration_ms:
            break

# ------------------------------------------------------------------------------
# HELPER: MOVEMENT
# ------------------------------------------------------------------------------
def wait_for_arrival(connection, lat, lon):
    """ Blocks until drone is within 1m of target. """
    while True:
        msg = connection.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
        if msg:
            current_lat = msg.lat / 1e7
            current_lon = msg.lon / 1e7
            
            d_lat = (lat - current_lat) * 1.113195e5
            d_lon = (lon - current_lon) * 1.113195e5
            dist = math.sqrt(d_lat**2 + d_lon**2)
            
            if dist < 1.0: return

def get_offset_location(base_lat, base_lon, d_north, d_east):
    """ Calculates new lat/lon given meter offsets. """
    earth_radius = 6378137.0
    dLat = d_north / earth_radius
    dLon = d_east / (earth_radius * math.cos(math.pi * base_lat / 180))
    return (base_lat + (dLat * 180/math.pi), base_lon + (dLon * 180/math.pi))

# ------------------------------------------------------------------------------
# MAIN MISSION
# ------------------------------------------------------------------------------
def run_patrol():
    print("--- SYSTEM STARTUP ---")
    
    # 1. CONNECT
    master = mavlink_connection_wrapper(CONNECTION_STR)
    master.wait_heartbeat()
    print("   -> Heartbeat Locked.")

    # 2. CONFIGURE TELEMETRY
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
        33, 200000, 0, 0, 0, 0, 0 # 33=GLOBAL_POSITION_INT, 5Hz
    )

    # 3. SETUP & TAKEOFF (Side Effects suppressed during replay)
    master.mav.set_mode_send(
        master.target_system, 
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 
        master.mode_mapping()['GUIDED']
    )
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 1, 0, 0, 0, 0, 0, 0
    )
    print("   -> Taking Off...")
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0, 0, 0, 0, 0, 0, 0, TARGET_ALTITUDE
    )
    
    # Wait for altitude
    while True:
        msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
        if msg.relative_alt / 1000.0 > (TARGET_ALTITUDE * 0.95): break

    # 4. SET SPEED (CRITICAL STEP ADDED HERE)
    print(f"   -> Setting Speed to {SPEED_MPS} m/s...")
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED,
        0,
        1,              # Speed Type (1=Ground Speed)
        SPEED_MPS,      # Speed (-1 means no change, we set it explicitly)
        -1, 0, 0, 0, 0  # Unused parameters
    )

    # 5. GENERATE PATH
    pos = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
    home_lat, home_lon = pos.lat / 1e7, pos.lon / 1e7
    
    # waypoints = [
    #     get_offset_location(home_lat, home_lon, PATROL_RADIUS, 0),              # North
    #     get_offset_location(home_lat, home_lon, PATROL_RADIUS, PATROL_RADIUS),  # North-East
    #     get_offset_location(home_lat, home_lon, 0, PATROL_RADIUS),              # East
    #     (home_lat, home_lon)                                                    # Home
    # ]
    waypoints = [
        get_offset_location(home_lat, home_lon, PATROL_RADIUS/2, 0),               # 1. North-Mid
        get_offset_location(home_lat, home_lon, PATROL_RADIUS, 0),                 # 2. North (Corner)
        get_offset_location(home_lat, home_lon, PATROL_RADIUS, PATROL_RADIUS/2),   # 3. North-East-Mid
        get_offset_location(home_lat, home_lon, PATROL_RADIUS, PATROL_RADIUS),     # 4. North-East (Corner)
        get_offset_location(home_lat, home_lon, PATROL_RADIUS/2, PATROL_RADIUS),   # 5. East-Mid
        get_offset_location(home_lat, home_lon, 0, PATROL_RADIUS),                 # 6. East (Corner)
        get_offset_location(home_lat, home_lon, 0, PATROL_RADIUS/2),               # 7. Home-Mid
        (home_lat, home_lon)                                                       # 8. Home
    ]
    # Calculate and Print Flight Window for the Experiment
    est_flight_time = PATROL_RADIUS / SPEED_MPS
    print(f"\n[EXPERIMENT INFO]")
    print(f"   Leg Distance: {PATROL_RADIUS}m")
    print(f"   Flight Speed: {SPEED_MPS} m/s")
    print(f"   CRASH WINDOW: You have approx {est_flight_time:.1f} SECONDS per leg to kill the process.")

    # 6. EXECUTE LOOPS
    print(f"\n--- STARTING PATROL ({TOTAL_LAPS} Laps) ---")
    
    for lap in range(TOTAL_LAPS):
        print(f"\n=== LAP {lap+1} START ===")
        
        for i, (wp_lat, wp_lon) in enumerate(waypoints):
            print(f"   -> Moving to Waypoint {i+1}...")
            
            # A. COMMAND (Intent)
            master.mav.command_int_send(
                master.target_system, master.target_component,
                mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                mavutil.mavlink.MAV_CMD_DO_REPOSITION,
                0, 0, -1, 0, 0, 0, 
                int(wp_lat * 1e7), int(wp_lon * 1e7), TARGET_ALTITUDE
            )
            
            # B. WAIT (Input)
            wait_for_arrival(master, wp_lat, wp_lon)
            
            # C. SCAN (Time)
            loiter_deterministic(master, LOITER_TIME)
            # print("CRASH HERE")
            # time.sleep(5)
            
        print(f"   [STATUS] Lap {lap+1} Complete. Patrol Count updated.")
        
        # Write external state to prove persistence
        with open("patrol_log.txt", "a") as f:
            f.write(f"COMPLETED_LAP:{lap+1}\n")

    print("\n--- MISSION COMPLETE: RTL ---")
    master.mav.command_long_send(
        master.target_system, master.target_component, 
        mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 
        0, 0, 0, 0, 0, 0, 0, 0
    )

if __name__ == '__main__':
    run_patrol()