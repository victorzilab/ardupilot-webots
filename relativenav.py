from pymavlink import mavutil
import time
import math

# Connect to the drone
master = mavutil.mavlink_connection('udpin:0.0.0.0:14550')  # Modify as per your connection
master.wait_heartbeat()
print("Connected to the drone!")

# Function to send the SET_POSITION_TARGET_LOCAL_NED command
def send_position_target(master, x, y, z=0, yaw=0, yaw_rate=0):
    # Sending position command using SET_POSITION_TARGET_LOCAL_NED
    master.mav.set_position_target_local_ned_send(
        0,  # Time boot_ms (use 0 for no specific time)
        master.target_system,  # Target system
        master.target_component,  # Target component
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,  # Coordinate frame (NED)
        0b0000111111111000,  # Type mask (indicates that we're setting position and yaw)
        x,  # X Position (meters in North direction)
        y,  # Y Position (meters in East direction)
        z,  # Z Position (meters in Down direction, negative for descending)
        0,  # X velocity (m/s, not used)
        0,  # Y velocity (m/s, not used)
        0,  # Z velocity (m/s, not used)
        0,  # X acceleration (not used)
        0,  # Y acceleration (not used)
        0,  # Z acceleration (not used)
        yaw,  # Yaw angle (radians)
        yaw_rate  # Yaw rate (degrees per second)
    )

    print(f"Moving to position: x={x}, y={y}, z={z}, yaw={math.degrees(yaw)}°")

# Function to calculate the yaw angle based on the target position
def calculate_yaw(current_x, current_y, target_x, target_y):
    # Calculate the difference in position
    delta_x = target_x - current_x
    delta_y = target_y - current_y

    # Calculate the yaw angle using atan2 (result in radians)
    yaw = math.atan2(delta_y, delta_x)
    return yaw

# Function to move the drone by a distance in X and Y from the current position
def move_drone(master, distance_x, distance_y):
    # Get the current local position
    msg = master.recv_match(type='LOCAL_POSITION_NED', blocking=True)
    current_x = msg.x
    current_y = msg.y
    current_z = msg.z

    # Calculate the new position
    new_x = current_x + distance_x
    new_y = current_y + distance_y
    new_z = current_z  # Keep the same altitude

    # Calculate the desired yaw angle
    yaw = calculate_yaw(current_x, current_y, new_x, new_y)

    # Send the position command with the desired yaw angle
    send_position_target(master, new_x, new_y, new_z, yaw=yaw, yaw_rate=0)

# Function to arm the drone and set to GUIDED mode
def arm_and_set_guided(master):
    # Set mode to GUIDED
    mode = 'GUIDED'
    mode_id = master.mode_mapping()[mode]
    master.mav.set_mode_send(
        master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id)

    # Arm the drone
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        1, 0, 0, 0, 0, 0, 0)

    print("Armed and set to GUIDED mode")

# Arm and set to GUIDED mode
arm_and_set_guided(master)

# Wait for the drone to be ready
time.sleep(2)

# Ask the user for distances to travel
try:
    while True:
        # Get user input for distances
        distance_x = float(input("Enter distance to travel in the North direction (meters): "))
        distance_y = float(input("Enter distance to travel in the East direction (meters): "))

        # Move the drone
        move_drone(master, distance_x, distance_y)

        # Wait for the drone to reach the target position
        time.sleep(5)  # Adjust this delay as needed

        # Ask if the user wants to continue
        continue_input = input("Do you want to move again? (yes/no): ").strip().lower()
        if continue_input != 'yes':
            print("Exiting...")
            break

except KeyboardInterrupt:
    print("Program interrupted by user.")
