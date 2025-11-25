import time
import math
import csv
from datetime import datetime
from pymavlink import mavutil
from src.mavlink_wrapper import mavlink_connection_wrapper

# --- CONFIGURATION ---
CONNECTION_STRING = 'udpin:localhost:14551'
TARGET_ALTITUDE = 10
GRID_SIZE = 20
ACCEPTANCE_RADIUS = 1.0

# ==============================================================================
#  SECTION A: USER DEFINED TASKS (Add your calculations here!)
# ==============================================================================

# Task 1: Check Battery
def task_measure_battery(connection, log_data):
    # Get message but don't block forever, use the wrapper's non-blocking or short timeout
    msg = connection.recv_match(type='SYS_STATUS', blocking=True, timeout=1)
    if msg:
        voltage = msg.voltage_battery / 1000.0
        log_data['battery_volts'] = voltage
        # CAPTURE DETERMINISTIC TIME
        # time_boot_ms is strictly from the log during replay
        log_data['vehicle_time_ms'] = msg.time_boot_ms 
    else:
        log_data['battery_volts'] = 0
# Task 2: Check Orientation (Roll/Pitch)
def task_measure_attitude(connection, log_data):
    msg = connection.recv_match(type='ATTITUDE', blocking=True, timeout=1)
    if msg:
        # Convert radians to degrees
        roll = math.degrees(msg.roll)
        pitch = math.degrees(msg.pitch)
        print(f"   [Task] Attitude - Roll: {roll:.1f}°, Pitch: {pitch:.1f}°")
        log_data['roll'] = roll
        log_data['pitch'] = pitch

# Task 3: Simulate a Heavy Sensor Calculation
def task_heavy_calculation(connection, log_data):
    print("   [Task] Processing spectral data...", end='', flush=True)
    time.sleep(1.0) # Simulate processing time
    result = 42 * 1.5 # Dummy math
    print(" Done.")
    log_data['sensor_value'] = result

# --- REGISTER YOUR TASKS HERE ---
# Just add the function name to this list to enable it
MISSION_TASKS = [
    task_measure_battery,
    task_measure_attitude,
    task_heavy_calculation
]

# ==============================================================================
#  SECTION B: CORE MISSION INFRASTRUCTURE (Do not change unless necessary)
# ==============================================================================

def execute_tasks(connection, waypoint_index):
    """ Runs all functions in MISSION_TASKS and saves to CSV """
    print(f">> Starting tasks for Waypoint {waypoint_index}...")
    
    # Dictionary to store results from all tasks
    log_data = {
        'timestamp': datetime.now().strftime("%H:%M:%S"),
        'waypoint_id': waypoint_index
    }

    # Run every function in the list
    for task_func in MISSION_TASKS:
        task_func(connection, log_data)

    # Save to CSV
    file_exists = False
    try:
        with open('mission_log.csv', 'r') as f: file_exists = True
    except FileNotFoundError: pass

    with open('mission_log.csv', 'a', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=log_data.keys())
        if not file_exists:
            writer.writeheader()
        writer.writerow(log_data)
    
    print(">> Tasks Complete. Data saved to mission_log.csv")

def get_distance_metres(loc1, loc2):
    lat1, lon1 = loc1.lat / 1e7, loc1.lon / 1e7
    lat2, lon2 = loc2.lat / 1e7, loc2.lon / 1e7
    dlat, dlong = lat2 - lat1, lon2 - lon1
    return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5

def wait_for_arrival(connection, target_lat, target_lon):
    print(f" -> Moving to Target...", end='\r')
    while True:
        msg = connection.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=1)
        if msg:
            class Loc: pass
            current, target = Loc(), Loc()
            current.lat, current.lon = msg.lat, msg.lon
            target.lat, target.lon = target_lat, target_lon
            
            dist = get_distance_metres(current, target)
            if dist < ACCEPTANCE_RADIUS:
                print(f"\n -> Arrived! (Dist: {dist:.2f}m)          ")
                return

def run_mission():
    print("--- CONNECTING ---")
    master = mavlink_connection_wrapper(CONNECTION_STRING)
    master.wait_heartbeat()
    
    print("Requesting Data Stream...")
    # master.mav.request_data_stream_send(
    #     master.target_system, master.target_component,
    #     mavutil.mavlink.MAV_DATA_STREAM_ALL, 4, 1
    # )
    # ---------------------------------------------------------
    # REPLACEMENT: Request Data using command_long_send (Primitive)
    # ---------------------------------------------------------
    print("Requesting GLOBAL_POSITION_INT via Command Long...")
    
    # We use command_long_send, which is explicitly in your table
    master.mav.command_long_send(
        master.target_system,            # Target System
        master.target_component,         # Target Component
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, # Command 511
        0,                               # Confirmation
        33,                              # Param 1: Message ID (33 = GLOBAL_POSITION_INT)
        250000,                          # Param 2: Interval in microseconds (250000us = 4Hz)
        0, 0, 0, 0, 0                    # Param 3-7: Unused
    )

    print("Switching to GUIDED & Arming...")
    mode_id = master.mode_mapping()['GUIDED']
    master.mav.set_mode_send(master.target_system, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, mode_id)
    
    master.mav.command_long_send(master.target_system, master.target_component, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)
    master.motors_armed_wait()
    
    master.mav.command_long_send(master.target_system, master.target_component, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, TARGET_ALTITUDE)
    
    # Wait for altitude
    while True:
        msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
        if (msg.relative_alt / 1000.0) >= TARGET_ALTITUDE * 0.95: break

    # Grid Gen
    pos_msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
    home_lat, home_lon = pos_msg.lat, pos_msg.lon
    lat_off, lon_off = int((GRID_SIZE/111111.0)*1e7), int((GRID_SIZE/(111111.0*math.cos(math.radians(home_lat/1e7))))*1e7)
    
    waypoints = [
        (home_lat + lat_off, home_lon),
        (home_lat + lat_off, home_lon + lon_off),
        (home_lat,           home_lon + lon_off),
        (home_lat,           home_lon)
    ]

    print("--- MISSION START ---")

    for i, (wp_lat, wp_lon) in enumerate(waypoints):
        print(f"\n--- Waypoint {i+1} ---")
        master.mav.command_int_send(
            master.target_system, master.target_component,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            mavutil.mavlink.MAV_CMD_DO_REPOSITION,
            0, 0, -1, 0, 0, 0, wp_lat, wp_lon, TARGET_ALTITUDE
        )
        
        wait_for_arrival(master, wp_lat, wp_lon)

        # *** HERE IS THE MAGIC ***
        # We just call one function, and it handles all your calculations
        execute_tasks(master, i+1)

    print("\nRTL.")
    master.mav.command_long_send(master.target_system, master.target_component, mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0, 0, 0, 0, 0, 0, 0, 0)

if __name__ == '__main__':
    run_mission()