from pymavlink import mavutil
import time
import threading

# Connect to SITL (change port if needed)
connection_string = "udp:127.0.0.1:14550"
master = mavutil.mavlink_connection(connection_string)
master.wait_heartbeat()
print("Connected to drone!")

# Set the mode to "GUIDED" before arming the drone
def set_guided_mode():
    # Request to change mode to GUIDED (mode 4)
    master.mav.set_mode_send(
        master.target_system, 
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 
        4  # GUIDED mode
    )
    print("Set to GUIDED mode!")

# Arm the drone
def arm_drone():
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
        1, 0, 0, 0, 0, 0, 0)
    print("Drone armed!")
    time.sleep(2)  # Add a delay to allow time for acknowledgment

# Takeoff to a specific altitude (in meters)
def takeoff(target_altitude):
    print(f"Taking off to {target_altitude} meters!")
    master.mav.command_long_send(
        master.target_system,  # Target system
        master.target_component,  # Target component
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,  # Command for takeoff
        0,  # Confirmation
        0,  # Param1 (unused, set to 0)
        0,  # Param2 (unused, set to 0)
        0,  # Param3 (unused, set to 0)
        0,  # Param4 (unused, set to 0)
        0,  # Param5 (unused, set to 0)
        0,  # Param6 (unused, set to 0)
        target_altitude,  # Target altitude in meters
    )

# Function to monitor altitude and check if we reached the target
def monitor_altitude(target_altitude):
    while True:
        message = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
        if message:
            altitude = message.relative_alt / 1000.0  # Convert from millimeters to meters
            print(f"Current altitude: {altitude} meters")
            
            if altitude >= target_altitude:
                print(f"Target altitude of {target_altitude} meters reached!")
                break
        time.sleep(1)

# Function to listen for acknowledgment messages (non-blocking)
def listen_for_acknowledgments():
    while True:
        message = master.recv_match(type='COMMAND_ACK', blocking=False)
        if message:
            print(f"Received ACK: {message}")
            if message.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                print("Command acknowledged successfully!")
            elif message.result != mavutil.mavlink.MAV_RESULT_IN_PROGRESS:
                print(f"Command failed with result: {message.result}")
        time.sleep(0.1)

# Main function to control drone
def control_drone():
    # Start a separate thread for listening to acknowledgments
    ack_thread = threading.Thread(target=listen_for_acknowledgments)
    ack_thread.daemon = True
    ack_thread.start()

    # Perform drone operations
    set_guided_mode()  # Set GUIDED mode
    time.sleep(2)  # Wait a little while to ensure mode change is processed
    arm_drone()  # Arm the drone
    time.sleep(2)  # Wait a little while to ensure arm command is processed
    takeoff(10)  # Issue takeoff command to 10 meters
    monitor_altitude(10)  # Monitor altitude to ensure the drone reaches 10 meters

if __name__ == "__main__":
    control_drone()
