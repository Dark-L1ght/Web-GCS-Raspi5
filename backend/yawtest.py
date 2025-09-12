# ==============================================================================
# Simple Drone Yaw Control Test Script
# ==============================================================================
import time
import math
from pymavlink import mavutil

# --- Drone & Mission Configuration ---
CONNECTION_STRING = 'udp:127.0.0.1:14550'
BAUD_RATE = 921600
TAKEOFF_ALTITUDE = 1.2  # meters (a bit higher for safety)
ARMING_RETRIES = 3
ARMING_RETRY_DELAY = 3

# --- Yaw Test Configuration ---
YAW_RATE = 0.5  # Radians per second (approx. 30 degrees/sec). Positive is Clockwise.
YAW_DURATION = 3.0  # Seconds to apply the yaw rate for.
HOVER_DURATION = 2.0  # Seconds to hover between movements.

# --- MAVLink & ArduPilot Constants ---
GUIDED_MODE = 4
# This bitmask tells the flight controller to ONLY listen to the yaw rate command.
# It ignores position, velocity, and acceleration fields.
# Binary: 0b0000101111111111
YAW_CONTROL_BITMASK = 2047 


def arm_and_takeoff(master, altitude):
    """
    Arms the drone and takes off to a specified altitude.
    Returns True on successful takeoff, False on failure.
    """
    print("Setting mode to GUIDED...")
    master.mav.set_mode_send(
        master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        GUIDED_MODE
    )
    for attempt in range(1, ARMING_RETRIES + 1):
        print(f"Arming motors (Attempt {attempt}/{ARMING_RETRIES})...")
        master.mav.command_long_send(
            master.target_system, master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
            1, 0, 0, 0, 0, 0, 0)
        try:
            master.motors_armed_wait()
            print("Motors successfully armed!")
            break
        except Exception as e:
            if attempt == ARMING_RETRIES:
                print(f"Could not arm motors after {ARMING_RETRIES} attempts: {e}")
                return False
            time.sleep(ARMING_RETRY_DELAY)

    print(f"Taking off to {altitude} meters...")
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, altitude
    )
    while True:
        msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=10)
        if not msg:
            print("No altitude data received. Takeoff may have failed.")
            return False
        current_altitude = msg.relative_alt / 1000.0
        print(f"\rCurrent altitude: {current_altitude:.2f}m", end="")
        if current_altitude >= altitude * 0.95:
            print("\nTarget altitude reached.")
            return True
        time.sleep(0.1)


def land_normally(master):
    """Commands the drone to perform a standard, non-precision landing."""
    print("Executing normal landing...")
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_LAND, 0, 0, 0, 0, 0, 0, 0, 0
    )
    master.motors_disarmed_wait()
    print("Landed and disarmed.")


def test_yaw(master):
    """
    Executes a sequence of yaw movements to test control.
    """
    print(f"\n--- STARTING YAW TEST ---")

    # --- 1. Yaw Right (Clockwise) ---
    print(f"Yawing RIGHT at {YAW_RATE:.2f} rad/s for {YAW_DURATION} seconds...")
    start_time = time.time()
    while time.time() - start_time < YAW_DURATION:
        master.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(
            0,  # time_boot_ms
            master.target_system, master.target_component,
            mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED, # Coordinate frame
            YAW_CONTROL_BITMASK, # Bitmask to use only yaw rate
            0, 0, 0,  # x, y, z positions (ignored)
            0, 0, 0,  # vx, vy, vz velocities (ignored)
            0, 0, 0,  # afx, afy, afz accelerations (ignored)
            0, YAW_RATE # yaw, yaw_rate (yaw is ignored, yaw_rate is used)
        ))
        time.sleep(0.1)
    
    # --- 2. Hover ---
    print(f"Hovering for {HOVER_DURATION} seconds...")
    master.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(
        0, master.target_system, master.target_component,
        mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED, YAW_CONTROL_BITMASK,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 # Command zero yaw rate to stop turning
    ))
    time.sleep(HOVER_DURATION)

    # --- 3. Yaw Left (Counter-Clockwise) ---
    print(f"Yawing LEFT at {YAW_RATE:.2f} rad/s for {YAW_DURATION} seconds...")
    start_time = time.time()
    while time.time() - start_time < YAW_DURATION:
        master.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(
            0, master.target_system, master.target_component,
            mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED, YAW_CONTROL_BITMASK,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -YAW_RATE # Negative yaw_rate for left turn
        ))
        time.sleep(0.1)

    # --- 4. Hover ---
    print(f"Hovering for {HOVER_DURATION} seconds...")
    master.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(
        0, master.target_system, master.target_component,
        mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED, YAW_CONTROL_BITMASK,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 # Command zero yaw rate
    ))
    time.sleep(HOVER_DURATION)
    
    print("--- YAW TEST COMPLETE ---")


def main():
    """Main function to connect to the drone and run the yaw test."""
    
    master = mavutil.mavlink_connection(CONNECTION_STRING, baud=BAUD_RATE)
    master.wait_heartbeat()
    print(f"Heartbeat from system (system {master.target_system} component {master.target_component})")

    try:
        if not arm_and_takeoff(master, TAKEOFF_ALTITUDE):
            print("Aborting mission due to takeoff failure.")
            return

        test_yaw(master)

        land_normally(master)

        print("\n✅ Yaw Test Mission Finished Successfully! ✅")

    except KeyboardInterrupt:
        print("\nKeyboard interrupt received. Landing immediately...")
        land_normally(master)
    except Exception as e:
        print(f"\nMISSION FAILED: {e}")
        print("Attempting to land immediately...")
        land_normally(master)
    finally:
        print("Resources cleaned up.")

if __name__ == "__main__":
    main()
