import socket
import json
import time
import math
from pymavlink import mavutil
import sys

# --- Drone & Mission Configuration ---
CONNECTION_STRING = 'udp:127.0.0.1:14550'
BAUD_RATE = 921600
TAKEOFF_ALTITUDE = 2.0  # meters
ARMING_RETRIES = 3      # Number of times to attempt arming
ARMING_RETRY_DELAY = 3  # Seconds to wait between arming attempts
WAYPOINT_RADIUS = 0.5   # meters
TRACKING_SPEED = 0.5    # m/s
FWD_GAIN = 1.0
ALT_GAIN = 0.5

LANDING_APPROACH_ALT = 0.75 # meters, altitude to trigger final LAND command
LANDING_TIMEOUT = 15 # seconds to search before aborting landing

CENTERING_TIMEOUT = 20 # seconds to search before aborting centering
CENTERING_ALTITUDE = 0.7 # meters, altitude to hold when centering
CENTERING_HOLD_DURATION = 3.0 # Seconds to hold position once centered

TARGET_LOST_HOVER_DURATION = 3.0  # Seconds to hover before starting active search
REACQUIRE_ASCEND_ALTITUDE = 1.5   # Meters to climb above current altitude to search
REACQUIRE_ASCEND_SPEED = 0.3      # m/s for the search ascent

GAIN_MAX_ALT = 2.0  # Altitude (m) at which the gain is 1.0 (full speed)
GAIN_MIN_ALT = 0.5  # Altitude (m) at which the gain is at its minimum
MAX_HORIZONTAL_GAIN = 0.8 # The gain at or above GAIN_MAX_ALT
MIN_HORIZONTAL_GAIN = 0.2 # The minimum gain at or below GAIN_MIN_ALT

# --- UDP Network Configuration ---
UDP_RECEIVE_IP = "127.0.0.2"
UDP_RECEIVE_PORT = 5005
CONTROL_SERVER_IP = "127.0.0.2"
CONTROL_SERVER_PORT = 5006

# --- MAVLink & ArduPilot Constants ---
GUIDED_MODE = 4
VELOCITY_CONTROL_BITMASK = 0b0000111111000111
POSITION_CONTROL_BITMASK = 0b110111111000

# --- Mission Waypoints ---
# IMPORTANT: Update these with your actual GPS coordinates
WAYPOINTS = [
    (-7.8332912, 110.3842767, 2.5), # Waypoint 1 (Precision Land on Logistics - ID 0)
    (-7.8333110, 110.3842802, 2.5), # Waypoint 2 (Precision Land on Logistics - ID 0)
    (-7.8333401, 110.3842762, 2.5), # Waypoint 3 (Center on Barrel - ID 1)
    (-7.8333536, 110.3842784, 2.5)  # Waypoint 4 (Final Normal Land)
]

# --- Global Socket ---
data_sock = None

# --- Core Functions ---
def send_control_command(command):
    """Sends a 'pause' or 'resume' command to the detection script via TCP."""
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.connect((CONTROL_SERVER_IP, CONTROL_SERVER_PORT))
            s.sendall(command.encode('utf-8'))
            print(f"Sent control command: '{command}' to detection script.")
    except ConnectionRefusedError:
        print(f"Error: Connection refused. Is the detection script running on port {CONTROL_SERVER_PORT}?")
    except Exception as e:
        print(f"Error sending control command: {e}")

def arm_and_takeoff(master, altitude):
    """
    Arms the drone and takes off to a specified altitude.
    Includes a retry mechanism for arming.
    Returns True on successful takeoff, False on failure.
    """
    print("Setting mode to GUIDED...")
    master.mav.set_mode_send(
        master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        GUIDED_MODE
    )
    
    # --- Arming Retry Loop ---
    for attempt in range(1, ARMING_RETRIES + 1):
        print(f"Arming motors (Attempt {attempt}/{ARMING_RETRIES})...")
        try:
            master.mav.command_long_send(
                master.target_system, master.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
                1, 0, 0, 0, 0, 0, 0)
            
            # Wait for the drone to confirm it's armed. 
            # This will timeout and raise an exception on failure.
            master.motors_armed_wait() 
            
            print("Motors successfully armed!")
            break # Exit the loop on success
            
        except Exception as e:
            print(f"Arming failed on attempt {attempt}: {e}")
            if attempt == ARMING_RETRIES:
                print("Could not arm motors after all attempts. Aborting takeoff.")
                return False # Indicate failure
            print(f"Retrying in {ARMING_RETRY_DELAY} seconds...")
            time.sleep(ARMING_RETRY_DELAY)

    # --- Takeoff Logic (only runs if arming was successful) ---
    print(f"Taking off to {altitude} meters...")
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, altitude
    )
    
    # Wait for takeoff to complete
    while True:
        msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=10)
        if not msg:
            print("No GLOBAL_POSITION_INT message received for 10s. Aborting takeoff wait.")
            return False

        current_altitude = msg.relative_alt / 1000.0
        print(f"Current altitude: {current_altitude:.2f}m")
        if current_altitude >= altitude * 0.90:
            print("Target altitude reached.")
            return True # Indicate successful takeoff
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

def navigate_to_waypoint(master, lat, lon, alt):
    """Commands the drone to fly to a specific GPS waypoint and waits for arrival."""
    print(f"Navigating to waypoint: ({lat}, {lon}) at {alt}m")
    master.mav.set_position_target_global_int_send(
        0, master.target_system, master.target_component,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
        POSITION_CONTROL_BITMASK, int(lat * 1e7), int(lon * 1e7), alt,
        0, 0, 0, 0, 0, 0, 0, 0
    )
    while True:
        msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
        if not msg: continue
        current_lat, current_lon = msg.lat / 1e7, msg.lon / 1e7
        dlat = math.radians(lat - current_lat)
        dlon = math.radians(lon - current_lon)
        a = math.sin(dlat/2)**2 + math.cos(math.radians(current_lat)) * math.cos(math.radians(lat)) * math.sin(dlon/2)**2
        distance = 6371000 * 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
        print(f"Distance to target: {distance:.1f}m")
        if distance <= WAYPOINT_RADIUS:
            print("Waypoint reached!")
            break
        time.sleep(0.1)
    time.sleep(2)

def calculate_velocities(x_center, y_center, frame_w, frame_h):
    """Calculates horizontal velocities to track the target."""

    # --- FIX FOR MIRRORED VIDEO ---
    # The Hailo GStreamer pipeline flips the video horizontally. To compensate,
    # we "un-mirror" the x_center coordinate before calculating the drone's movement.
    corrected_x_center = frame_w - x_center
    # --- END FIX ---

    # Use the corrected coordinate for the rest of the calculation.
    # The y-axis (up/down) is not affected.
    x_offset = (corrected_x_center - frame_w / 2) / (frame_w / 2)
    y_offset = (y_center - frame_h / 2) / (frame_h / 2)
    
    right_vel = TRACKING_SPEED * x_offset if abs(x_offset) > 0.1 else 0
    forward_vel = -TRACKING_SPEED * y_offset * FWD_GAIN if abs(y_offset) > 0.1 else 0
    
    return forward_vel, right_vel

def flush_socket_buffer(sock):
    """Clears any old data from the UDP socket buffer."""
    print("Flushing UDP socket buffer...")
    while True:
        try:
            sock.recvfrom(1024)
        except socket.timeout:
            print("Buffer flushed.")
            break

def center_above_target(master, sock, target_class_id):
    """
    Centers the drone above a target and holds the position for a specified duration,
    while actively correcting for drift.
    """
    flush_socket_buffer(sock)
    send_control_command('resume')
    print(f"Centering above target (ID: {target_class_id}) at {CENTERING_ALTITUDE}m...")

    overall_start_time = time.time()
    last_detection_time = time.time()

    centered_start_time = None # This will track when we start the hold

    while True:
        # Overall timeout for the maneuver
        if time.time() - overall_start_time > CENTERING_TIMEOUT:
            print("\nCentering timeout reached. Aborting.")
            master.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(
                0, master.target_system, master.target_component, mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
                VELOCITY_CONTROL_BITMASK, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0))
            return False

        # --- Continuous Correction Loop ---
        fwd_vel, right_vel, down_vel = 0, 0, 0
        is_target_visible = False
        detection_data = None

        try:
            data, _ = sock.recvfrom(1024)
            detection_data = json.loads(data.decode())
            if detection_data.get("state") == "TRACKING" and detection_data.get("class_id") == target_class_id:
                is_target_visible = True
                last_detection_time = time.time()
                x, y = detection_data["x_center"], detection_data["y_center"]
                w, h = detection_data["frame_width"], detection_data["frame_height"]
                fwd_vel, right_vel = calculate_velocities(x, y, w, h)
        except (socket.timeout, json.JSONDecodeError, KeyError):
            is_target_visible = False

        alt_msg = master.recv_match(type='RANGEFINDER', blocking=False, timeout=0.05)
        current_alt = alt_msg.distance if alt_msg else CENTERING_ALTITUDE
        horizontal_gain = get_dynamic_gain(current_alt)
        fwd_vel *= horizontal_gain
        right_vel *= horizontal_gain

        time_since_lost = time.time() - last_detection_time
        if not is_target_visible and time_since_lost > TARGET_LOST_HOVER_DURATION:
            down_vel = REACQUIRE_ASCEND_SPEED
        else:
            alt_error = CENTERING_ALTITUDE - current_alt
            down_vel = -ALT_GAIN * alt_error

        master.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(
            0, master.target_system, master.target_component, mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
            VELOCITY_CONTROL_BITMASK, 0, 0, 0, fwd_vel, right_vel, down_vel, 0, 0, 0, 0, 0))
        # --- End Continuous Correction Loop ---

        # --- Simplified State Machine for Centering and Holding ---
        if is_target_visible:
            center_error_ratio = abs(detection_data["frame_width"]/2 - detection_data["x_center"]) / detection_data["frame_width"]

            if center_error_ratio < 0.1 and abs(CENTERING_ALTITUDE - current_alt) < 0.15:
                # Condition: We are centered
                if centered_start_time is None:
                    centered_start_time = time.time()
                    print("\nTarget centered. Holding position...")

                # Check if the hold duration has been met
                if time.time() - centered_start_time > CENTERING_HOLD_DURATION:
                    print(f"\nHeld position for {CENTERING_HOLD_DURATION} seconds. Proceeding with mission.")
                    return True # Success

                # Update user on hold progress
                time_left = CENTERING_HOLD_DURATION - (time.time() - centered_start_time)
                sys.stdout.write(f"\rHolding position... {time_left:.1f}s remaining.")
                sys.stdout.flush()
            else:
                # Condition: We are visible but have drifted off-center
                if centered_start_time is not None:
                    print("\nDrifted off-center. Re-centering...")
                centered_start_time = None # Reset the hold timer
        else:
            # Condition: Target is not visible
            sys.stdout.write(f"\rSearching for target... Time since last seen: {time_since_lost:.1f}s")
            sys.stdout.flush()
            centered_start_time = None # Reset hold timer if we lose the target

        time.sleep(0.05)

def get_dynamic_gain(current_alt):
    """Calculates a dynamic gain scaling factor based on altitude."""
    if current_alt >= GAIN_MAX_ALT: return MAX_HORIZONTAL_GAIN
    if current_alt <= GAIN_MIN_ALT: return MIN_HORIZONTAL_GAIN
    gain = MIN_HORIZONTAL_GAIN + (MAX_HORIZONTAL_GAIN - MIN_HORIZONTAL_GAIN) * \
           ((current_alt - GAIN_MIN_ALT) / (GAIN_MAX_ALT - GAIN_MIN_ALT))
    return gain

def execute_precision_landing(master, sock, target_class_id):
    """Manages precision landing on a target with reacquisition logic."""
    flush_socket_buffer(sock)
    send_control_command('resume')
    print(f"Starting precision landing sequence on target (ID: {target_class_id})...")
    
    search_start_time = time.time()
    last_detection_time = time.time() # Initialize the timer once

    while True:
        if time.time() - search_start_time > LANDING_TIMEOUT:
            print("Landing timeout reached. Aborting and hovering.")
            master.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(
                0, master.target_system, master.target_component, mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
                VELOCITY_CONTROL_BITMASK, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0))
            # I've added a 'return False' here to make it consistent with the mission planner suggestion
            return False

        try:
            alt_msg = master.recv_match(type='RANGEFINDER', blocking=False, timeout=0.05)
            current_altitude = alt_msg.distance if alt_msg else GAIN_MAX_ALT 

            data, _ = sock.recvfrom(1024)
            detection = json.loads(data.decode())
            
            # Check for a valid target FIRST
            #if detection.get("state") != "TRACKING" or detection.get("class_id") != target_class_id:
            #    raise socket.timeout()

            # --- FIX: Only update timers AFTER a successful and valid detection ---
            last_detection_time = time.time()
            search_start_time = time.time()
            # --- END FIX ---

            x, y, area = detection["x_center"], detection["y_center"], detection["area"]
            w, h = detection["frame_width"], detection["frame_height"]
            
            # ... (rest of the function is identical) ...
            fwd_vel, right_vel = calculate_velocities(x, y, w, h)
            horizontal_gain = get_dynamic_gain(current_altitude)
            fwd_vel *= horizontal_gain
            right_vel *= horizontal_gain
            
            target_area = 0.2 * (w * h)
            area_error = 1.0 - (area / target_area) if target_area > 0 else 0
            down_vel = TRACKING_SPEED * area_error * ALT_GAIN if abs(area_error) > 0.2 else 0
            master.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(
                0, master.target_system, master.target_component, mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
                VELOCITY_CONTROL_BITMASK, 0, 0, 0, fwd_vel, right_vel, down_vel, 0, 0, 0, 0, 0))
            center_error_ratio = abs(x - w / 2) / w
            print(f"LANDING (ID {target_class_id}): Alt: {current_altitude:.2f}m, Gain: {horizontal_gain:.2f}, Err: {center_error_ratio:.2%}")

            if current_altitude < LANDING_APPROACH_ALT and center_error_ratio < 0.15:
                print("Target centered at low altitude. Switching to LAND mode.")
                land_normally(master)
                time.sleep(2) # Pause after landing
                # I've added a 'return True' here for consistency
                return True

        except (socket.timeout, json.JSONDecodeError, KeyError):
            time_since_lost = time.time() - last_detection_time
            print(f"Searching for target ID {target_class_id}... Time since last seen: {time_since_lost:.1f}s")
            
            vz = 0 
            if time_since_lost < TARGET_LOST_HOVER_DURATION:
                print("-> Phase 1: Hovering briefly.")
            else:
                print(f"-> Phase 2: Ascending to search at {REACQUIRE_ASCEND_SPEED} m/s.")
                vz = -REACQUIRE_ASCEND_SPEED

            master.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(
                0, master.target_system, master.target_component, mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
                VELOCITY_CONTROL_BITMASK, 0, 0, 0, 0, 0, vz, 0, 0, 0, 0, 0))
            
def main():
    """Main function to connect to the drone and run the new mission."""

    global WAYPOINTS
    if len(sys.argv) > 1:
        try:
            # sys.argv[1] contains the JSON string of waypoints from gcs_server.py
            print("Received waypoints from GCS command.")
            waypoints_from_gcs = json.loads(sys.argv[1])
            # The JSON will be a list of lists/dicts. Convert to a list of tuples.
            WAYPOINTS = [ (wp['lat'], wp['lon'], wp['alt']) for wp in waypoints_from_gcs ]
            print(f"Successfully updated mission waypoints: {WAYPOINTS}")
        except (json.JSONDecodeError, IndexError, KeyError) as e:
            print(f"ERROR: Could not parse waypoints from GCS: {e}. Using default waypoints.")
    else:
        print("No waypoints received from GCS. Using default hardcoded waypoints.")

    global data_sock
    data_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    data_sock.bind((UDP_RECEIVE_IP, UDP_RECEIVE_PORT))
    data_sock.settimeout(0.5)

    master = mavutil.mavlink_connection(CONNECTION_STRING, baud=BAUD_RATE)
    master.wait_heartbeat()
    print(f"Heartbeat from system (system {master.target_system} component {master.target_component})")

    #Requesting Rangefinder data stream
    master.mav.command_long_send(master.target_system, master.target_component, mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0, mavutil.mavlink.MAVLINK_MSG_ID_RANGEFINDER, 1000000, 0, 0, 0, 0, 0)

    try:
        send_control_command('pause')
        if not arm_and_takeoff(master, TAKEOFF_ALTITUDE):
            raise Exception("Failed to takeoff for Mission 1. Aborting mission.")

        # --- Mission 1: Fly to WP1 and Precision Land on Target 0 ---
        print("\n--- MISSION 1: Precision Land at Waypoint 1 (Target ID 0) ---")
        navigate_to_waypoint(master, WAYPOINTS[0][0], WAYPOINTS[0][1], WAYPOINTS[0][2])
        execute_precision_landing(master, sock=data_sock, target_class_id=0)
        
        # --- Mission 2: Takeoff, fly to WP3 and Center on Target 1 ---
        print("\n--- MISSION 2: Center over Target at Waypoint 3 (Target ID 1) ---")
        send_control_command('pause')
        if not arm_and_takeoff(master, TAKEOFF_ALTITUDE):
            raise Exception("Failed to takeoff for Mission 3. Aborting mission.")
        navigate_to_waypoint(master, WAYPOINTS[2][0], WAYPOINTS[2][1], WAYPOINTS[2][2])
        center_above_target(master, sock=data_sock, target_class_id=1)

        # --- Mission 3: Fly to WP2 and Precision Land on Target 0 ---
        print("\n--- MISSION 3: Precision Land at Waypoint 2 (Target ID 0) ---")
        send_control_command('pause')
        navigate_to_waypoint(master, WAYPOINTS[1][0], WAYPOINTS[1][1], WAYPOINTS[1][2])
        execute_precision_landing(master, sock=data_sock, target_class_id=0)

        # --- Mission 4: Takeoff, fly to WP3 and Center on Target 1 Again ---
        print("\n--- MISSION 4: Center over Target at Waypoint 3 Again (Target ID 1) ---")
        send_control_command('pause')
        if not arm_and_takeoff(master, TAKEOFF_ALTITUDE):
            raise Exception("Failed to takeoff for Mission 5. Aborting mission.")
        navigate_to_waypoint(master, WAYPOINTS[2][0], WAYPOINTS[2][1], WAYPOINTS[2][2])
        center_above_target(master, sock=data_sock, target_class_id=1)

        # --- Mission 5: Fly to WP4 and Land ---
        print("\n--- MISSION 5: Final Landing at Waypoint 4 ---")
        send_control_command('pause')
        navigate_to_waypoint(master, WAYPOINTS[3][0], WAYPOINTS[3][1], WAYPOINTS[3][2])
        land_normally(master)

        print("\nMission finished successfully!")

    except KeyboardInterrupt:
        print("Keyboard interrupt received. Landing immediately...")
        land_normally(master)
    finally:
        if data_sock: data_sock.close()
        print("Resources cleaned up.")

if __name__ == "__main__":
    main()
