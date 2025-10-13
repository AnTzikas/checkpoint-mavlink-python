import time
from pymavlink import mavutil

master = mavutil.mavlink_connection("udp:127.0.0.1:14550",source_system=255,source_component=5)

master.wait_heartbeat()
print("Got heartbeat")

def send_command(command,confirmation,param1=0,param2=0,param3=0,param4=0,param5=0,param6=0,param7=0):
    """
    Send a COMMAND_LONG message to (sys,comp) = (1,1)
    """
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        command,
        confirmation,
        param1,
        param2,
        param3,
        param4,
        param5,
        param6,
        param7
        )  

# Set mode to GUIDED
master.set_mode("GUIDED")

# Check that mode change was successful
while master.flightmode != "GUIDED":
    print("Waiting for flightmode change")
    master.wait_heartbeat()

# Arm the vehicle
master.arducopter_arm()
master.motors_armed_wait()

# Command takeoff to 20m
send_command(mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,0,param7=20)

# Check we sucessfully sent takeoff
msg = master.recv_match(type="COMMAND_ACK",blocking=True)
print(msg)
if msg.result != mavutil.mavlink.MAV_RESULT_ACCEPTED:
    print("Error sending takeoff command")
    exit()

# Wait for us to get to somewhere near 20m
while abs(20 - master.location(relative_alt=True).alt) > 0.5: # Alt in m
    pass

print("Takeoff complete")

# Get current system time
system_time = master.recv_match(type="SYSTEM_TIME",blocking=True)
time_pair = (time.time(), system_time.time_boot_ms)

# Setup the bitfields to tell the vehicle to ignore velocity and accelerations
ignore_velocity = (mavutil.mavlink.POSITION_TARGET_TYPEMASK_VX_IGNORE
    | mavutil.mavlink.POSITION_TARGET_TYPEMASK_VY_IGNORE
    | mavutil.mavlink.POSITION_TARGET_TYPEMASK_VZ_IGNORE
    | mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
    )

ignore_accel = (mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE
    | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE
    | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE
    )

# Send a position target
master.mav.set_position_target_global_int_send(
    time_pair[1]+int(round((time.time()-time_pair[0])*1000)),
    1,
    1,
    mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
    (ignore_velocity | ignore_accel),
    int(51.4232303*(10**7)), # Lat (degE7)
    int(-2.6710604*(10**7)), # Long (degE7)
    100, # Altitude
    0,0,0, # Velocities
    0,0,0, # Accels
    0, # Yaw
    0 # Yaw rate
    )

print("Set intial position")
time.sleep(0.5)
exit()

def set_position(lat,lng,alt):
    master.mav.set_position_target_global_int_send(
        time_pair[1]+int(round((time.time()-time_pair[0])*1000)),
        1,
        1,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
        (ignore_velocity | ignore_accel),
        int(lat*(10**7)), # Lat (degE7)
        int(lng*(10**7)), # Long (degE7)
        alt, # Altitude
        0,0,0, # Velocities
        0,0,0, # Accels
        0, # Yaw
        0 # Yaw rate
        )

# set_position(51.425,-2.671,50)