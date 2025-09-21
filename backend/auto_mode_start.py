#!/usr/bin/env python3

"""
auto_mode_start.py - A Pymavlink script to start a pre-loaded mission.

This script connects to a running MAVLink instance (like MAVProxy or SITL),
waits for the vehicle to be ready, switches to AUTO mode, arms the vehicle,
and then starts the mission.
"""

import sys
import time
from pymavlink import mavutil

# --- Configuration ---
# The MAVLink connection string. 
# Use 'udp:127.0.0.1:14550' for a local MAVProxy or SITL instance.
CONNECTION_STRING = 'udp:127.0.0.1:14550'

def connect_to_vehicle():
    """
    Connects to the vehicle using the specified CONNECTION_STRING.
    Waits for the first heartbeat message to confirm the connection.
    
    Returns:
        The master connection object if successful, None otherwise.
    """
    print(f"Connecting to vehicle on: {CONNECTION_STRING}")
    try:
        master = mavutil.mavlink_connection(CONNECTION_STRING)
        master.wait_heartbeat()
        print(f"Heartbeat from system (system {master.target_system} component {master.target_component})")
        print("Connection successful.")
        return master
    except Exception as e:
        print(f"Failed to connect: {e}")
        return None

def wait_for_prearm_checks(master):
    """
    Waits until the vehicle's pre-arm checks pass.
    
    Args:
        master: The Pymavlink connection object.
        
    Returns:
        True if pre-arm checks pass, False on timeout or failure.
    """
    print("Waiting for vehicle to pass pre-arm checks...")
    start_time = time.time()
    timeout = 60  # 60-second timeout

    while time.time() - start_time < timeout:
        # We listen for SYS_STATUS messages
        msg = master.recv_match(type='SYS_STATUS', blocking=True, timeout=1)
        if msg:
            # The bitmask for pre-arm checks is in onboard_control_sensors_health
            # A value of 1 means all checks have passed.
            # See MAV_SYS_STATUS_FLAGS enum for details.
            if msg.onboard_control_sensors_health & mavutil.mavlink.MAV_SYS_STATUS_PREARM_CHECK == 1:
                print("Pre-arm checks passed!")
                return True
        print("Waiting for pre-arm checks to complete...")
        time.sleep(1)
        
    print("Error: Timed out waiting for pre-arm checks to pass.")
    return False

def set_mode(master, mode_name):
    """
    Sets the flight mode of the vehicle.
    
    Args:
        master: The Pymavlink connection object.
        mode_name: The name of the mode to set (e.g., 'AUTO', 'GUIDED').
    
    Returns:
        True if mode was set successfully, False otherwise.
    """
    print(f"Setting mode to {mode_name}...")
    
    # Check if the desired mode is available
    if mode_name not in master.mode_mapping():
        print(f"Error: Mode '{mode_name}' not available.")
        return False

    # Get the mode ID
    mode_id = master.mode_mapping()[mode_name]

    # Set the mode
    master.mav.set_mode_send(
        master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id
    )

    # Wait for the mode change to be confirmed
    start_time = time.time()
    while time.time() - start_time < 5: # 5 second timeout
        msg = master.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
        if msg and msg.custom_mode == mode_id:
            print(f"Mode successfully changed to {mode_name}.")
            return True
            
    print(f"Error: Failed to set mode to {mode_name}.")
    return False

def arm_vehicle(master):
    """
    Arms the vehicle's motors.
    
    Args:
        master: The Pymavlink connection object.
    """
    print("Arming vehicle...")
    try:
        master.mav.command_long_send(
            master.target_system,
            master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,  # Confirmation
            1,  # Param 1: 1 to arm, 0 to disarm
            0, 0, 0, 0, 0, 0  # Unused params
        )
        # Wait for arming confirmation
        master.motors_armed_wait()
        print("Vehicle ARMED successfully.")
        return True
    except Exception as e:
        print(f"Failed to arm vehicle: {e}")
        return False
        
def start_mission(master):
    """
    Commands the vehicle to start its pre-loaded mission.
    
    Args:
        master: The Pymavlink connection object.
    """
    print("Sending MISSION_START command...")
    try:
        master.mav.command_long_send(
            master.target_system,
            master.target_component,
            mavutil.mavlink.MAV_CMD_MISSION_START,
            0, # Confirmation
            0, # First waypoint
            0, # Last waypoint
            0, 0, 0, 0, 0 # Unused params
        )
        print("Mission start command sent.")
        return True
    except Exception as e:
        print(f"Failed to send MISSION_START command: {e}")
        return False

def main():
    """
    Main function to execute the mission start sequence.
    """
    master = connect_to_vehicle()
    if not master:
        sys.exit(1) # Exit if connection fails

    # Wait for a good GPS fix and health before proceeding
    if not wait_for_prearm_checks(master):
        sys.exit(1)

    # Set mode to AUTO
    if not set_mode(master, 'AUTO'):
        sys.exit(1)
        
    # Arm the vehicle
    if not arm_vehicle(master):
        sys.exit(1)

    # Start the mission
    if not start_mission(master):
        sys.exit(1)
        
    print("\nMission has been initiated. Monitor vehicle progress in your GCS.")
    print("This script will now exit.")

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nScript interrupted by user. Exiting.")
        sys.exit(0)