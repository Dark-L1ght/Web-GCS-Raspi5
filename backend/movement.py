import socket
import json
import time
import math
from pymavlink import mavutil
import sys
import servo_control

# --- Drone & Mission Configuration ---
CONNECTION_STRING = 'udp:127.0.0.1:14550'
BAUD_RATE = 921600
TAKEOFF_ALTITUDE = 1.1  # meters
ARMING_RETRIES = 3      # Number of times to attempt arming
ARMING_RETRY_DELAY = 3  # Seconds to wait between arming attempts
WAYPOINT_RADIUS = 0.5   # meters
TRACKING_SPEED = 0.5    # m/s
CENTERING_SPEED = 0.25  # m/s, horizontal speed for fine-tuning position
FWD_GAIN = 0.6
ALT_GAIN = 0.3

# CAMERA CENTER OFFSET
VERTICAL_CENTER_RATIO = 0.10

LANDING_APPROACH_ALT = 0.5 # meters, altitude to trigger final LAND command
LANDING_TIMEOUT = 15 # seconds to search before aborting landing
FORCED_LAND_ALTITUDE = 0.30 # NEW: Altitude to force landing regardless of target visibility

CENTERING_TIMEOUT = 20 # seconds to search before aborting centering
CENTERING_ALTITUDE = 0.75 # meters, altitude to hold when centering

TARGET_LOST_HOVER_DURATION = 3.0  # Seconds to hover before starting active search
REACQUIRE_ASCEND_SPEED = 0.3      # m/s for the search ascent

GAIN_MAX_ALT = 1.1  # Altitude (m) at which the gain is 1.0 (full speed)
GAIN_MIN_ALT = 0.4  # Altitude (m) at which the gain is at its minimum
MAX_HORIZONTAL_GAIN = 0.4 # The gain at or above GAIN_MAX_ALT
MIN_HORIZONTAL_GAIN = 0.1 # The minimum gain at or below GAIN_MIN_ALT

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
WAYPOINTS = [
    (-7.8332912, 110.3842767, 2.5), # Waypoint 1 (Precision Land on Logistics - ID 0)
    (-7.8333110, 110.3842802, 2.5), # Waypoint 2 (Precision Land on Logistics - ID 0 / Corridor Elbow )
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
    print("Setting mode to GUIDED...")
    master.mav.set_mode_send(master.target_system, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, GUIDED_MODE)
    for attempt in range(1, ARMING_RETRIES + 1):
        print(f"Arming motors (Attempt {attempt}/{ARMING_RETRIES})...")
        master.mav.command_long_send(
            master.target_system, master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)
        try:
            master.motors_armed_wait()
            print("Motors successfully armed!")
            break
        except Exception:
            if attempt == ARMING_RETRIES:
                print("Could not arm motors. Aborting takeoff.")
                return False
            time.sleep(ARMING_RETRY_DELAY)

    print(f"Taking off to {altitude} meters...")
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, altitude)
    while True:
        msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=5)
        if not msg:
            print("No altitude data received. Takeoff may have failed.")
            return False
        current_altitude = msg.relative_alt / 1000.0
        print(f"\rCurrent altitude: {current_altitude:.2f}m", end="")
        if current_altitude >= altitude * 0.80:
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
    try:
        master.motors_disarmed_wait()
        print("Landed and disarmed.")
    except Exception as e:
        print(f"Warning: Did not receive disarm confirmation, but landing was commanded. {e}")


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

def calculate_velocities(x_center, y_center, frame_w, frame_h, speed):
    """Calculates forward and right velocities based on target position and a given speed."""
    corrected_x_center = frame_w - x_center
    x_offset = (corrected_x_center - frame_w / 2) / (frame_w / 2)

    target_y = frame_h * VERTICAL_CENTER_RATIO
    y_offset = (y_center - target_y) / (frame_h / 2)
    
    right_vel = speed * x_offset if abs(x_offset) > 0.1 else 0
    forward_vel = -speed * y_offset * FWD_GAIN if abs(y_offset) > 0.1 else 0
    return forward_vel, right_vel

def flush_socket_buffer(sock):
    """Clears any old data from the UDP socket buffer."""
    while True:
        try:
            sock.recvfrom(1024)
        except socket.timeout:
            break

def center_above_target(master, sock, target_class_id, target_alt):
    flush_socket_buffer(sock)
    send_control_command('resume')
    print(f"Centering above target (ID: {target_class_id}) at {target_alt}m...")
    
    start_time = time.time()
    
    while time.time() - start_time < CENTERING_TIMEOUT:
        alt_msg = master.recv_match(type='DISTANCE_SENSOR', blocking=False, timeout=0.05)
        current_alt = -1 # Default value if no sensor data
        if alt_msg and alt_msg.orientation == 25:
            current_alt = alt_msg.current_distance / 100.0
            
        fwd_vel, right_vel, down_vel = 0, 0, 0
        
        try:
            data, _ = sock.recvfrom(1024)
            detection = json.loads(data.decode())
            if detection.get("state") == "TRACKING" and detection.get("class_id") == target_class_id:
                x, y, w, h = detection["x_center"], detection["y_center"], detection["frame_width"], detection["frame_height"]
                fwd_vel, right_vel = calculate_velocities(x, y, w, h, CENTERING_SPEED)

                center_error_ratio = abs(x - w / 2) / (w/2)
                sys.stdout.write(f"\rCentering... Err: {center_error_ratio:.2%}, Current Alt: {current_alt:.2f}m")
                sys.stdout.flush()

                if center_error_ratio < 0.05:
                    print("\nTarget centered. Stabilizing for drop...")
                    master.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(
                        0, master.target_system, master.target_component, mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
                        VELOCITY_CONTROL_BITMASK, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0))
                    
                    time.sleep(1)
                    
                    print("Drone stable. Opening gripper to drop package.")
                    servo_control.open_gripper()
                    time.sleep(1.0)
                    send_control_command('pause')
                    return True
        except (socket.timeout, json.JSONDecodeError, KeyError):
            print("\rSearching for drop-off target...", end="")

        master.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(
            0, master.target_system, master.target_component, mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
            VELOCITY_CONTROL_BITMASK, 0, 0, 0, fwd_vel, right_vel, down_vel, 0, 0, 0, 0, 0))
        time.sleep(0.05)

    print("\nCentering timeout reached.")
    send_control_command('pause')
    return False

def get_dynamic_gain(current_alt):
    """Calculates a dynamic gain scaling factor based on altitude."""
    if current_alt is None: return MIN_HORIZONTAL_GAIN
    if current_alt >= GAIN_MAX_ALT: return MAX_HORIZONTAL_GAIN
    if current_alt <= GAIN_MIN_ALT: return MIN_HORIZONTAL_GAIN
    
    gain_range = MAX_HORIZONTAL_GAIN - MIN_HORIZONTAL_GAIN
    alt_range = GAIN_MAX_ALT - GAIN_MIN_ALT
    gain = MIN_HORIZONTAL_GAIN + gain_range * ((current_alt - GAIN_MIN_ALT) / alt_range)
    return gain

# UPDATED: Added a low-altitude safety check to force landing.
def execute_precision_landing(master, sock):
    flush_socket_buffer(sock)
    send_control_command('resume')
    print(f"Starting precision landing sequence (Accepting IDs: [0, 1])...")
    
    search_start_time = time.time()
    last_detection_time = time.time()
    last_known_alt = TAKEOFF_ALTITUDE

    while time.time() - search_start_time < LANDING_TIMEOUT:
        # --- Get current altitude first on every loop iteration ---
        alt_msg = master.recv_match(type='DISTANCE_SENSOR', blocking=False, timeout=0.05)
        if alt_msg and alt_msg.orientation == 25:
            last_known_alt = alt_msg.current_distance / 100.0
        current_altitude = last_known_alt
        
        # --- NEW: Safety check for forced landing ---
        if current_altitude < FORCED_LAND_ALTITUDE:
            print(f"\nAltitude is below {FORCED_LAND_ALTITUDE}m. Forcing immediate landing.")
            land_normally(master)
            # Perform post-landing actions
            time.sleep(1)
            print("Landed. Closing gripper to pick up package.")
            time.sleep(1)
            servo_control.close_gripper()
            return True # Exit the function

        try:
            data, _ = sock.recvfrom(1024)
            detection = json.loads(data.decode())
            
            detected_id = detection.get("class_id")
            if detection.get("state") != "TRACKING" or detected_id not in [0, 1]:
                raise socket.timeout

            last_detection_time = time.time()
            x, y = detection["x_center"], detection["y_center"]
            w, h = detection["frame_width"], detection["frame_height"]
            
            fwd_vel, right_vel = calculate_velocities(x, y, w, h, TRACKING_SPEED)
            horizontal_gain = get_dynamic_gain(current_altitude)
            fwd_vel *= horizontal_gain
            right_vel *= horizontal_gain
            
            down_vel = 0.1 # Slow, constant descent when target is visible
            master.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(
                0, master.target_system, master.target_component, mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
                VELOCITY_CONTROL_BITMASK, 0, 0, 0, fwd_vel, right_vel, down_vel, 0, 0, 0, 0, 0))
            
            center_error_ratio = abs(x - w / 2) / (w/2)
            print(f"\rLANDING (ID {detected_id}): Alt: {current_altitude:.2f}m, Gain: {horizontal_gain:.2f}, Err: {center_error_ratio:.2%}", end="")

            if current_altitude < LANDING_APPROACH_ALT and center_error_ratio < 0.15:
                print("\nTarget centered at low altitude. Switching to LAND mode.")
                land_normally(master)
                time.sleep(1)
                print("Landed. Closing gripper to pick up package.")
                time.sleep(1)
                servo_control.close_gripper()
                return True

        except (socket.timeout, json.JSONDecodeError, KeyError):
            time_since_lost = time.time() - last_detection_time
            print(f"\rSearching for landing target... Time since last seen: {time_since_lost:.1f}s", end="")
            vz = 0 
            if time_since_lost > TARGET_LOST_HOVER_DURATION:
                vz = -REACQUIRE_ASCEND_SPEED
            master.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(
                0, master.target_system, master.target_component, mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
                VELOCITY_CONTROL_BITMASK, 0, 0, 0, 0, 0, vz, 0, 0, 0, 0, 0))

    print("\nLanding timeout reached. Aborting and hovering.")
    master.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(
        0, master.target_system, master.target_component, mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
        VELOCITY_CONTROL_BITMASK, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0))
    return False
                                    
def main():
    """Main function to connect to the drone and run the new mission."""

    global WAYPOINTS
    if len(sys.argv) > 1:
        try:
            print("Received waypoints from GCS command.")
            waypoints_from_gcs = json.loads(sys.argv[1])
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

    print("Requesting DISTANCE_SENSOR stream at 10Hz...")
    master.mav.command_long_send(
        master.target_system, 
        master.target_component, 
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
        mavutil.mavlink.MAVLINK_MSG_ID_DISTANCE_SENSOR,
        100000,  # 10Hz
        0, 0, 0, 0, 0
    )
    try:
        servo_control.setup()
        servo_control.open_gripper()
        send_control_command('pause')
        if not arm_and_takeoff(master, TAKEOFF_ALTITUDE):
            raise Exception("Failed to takeoff for Mission 1. Aborting mission.")

        # --- Mission 1: Fly to WP1 and Precision Land on Target 0 ---
        print("\n--- MISSION 1: Precision Land at Waypoint 1 (Target ID 0) ---")
        navigate_to_waypoint(master, WAYPOINTS[0][0], WAYPOINTS[0][1], WAYPOINTS[0][2])
        execute_precision_landing(master, sock=data_sock)
        
        # --- GOTO Corridor Elbow WP2---
        send_control_command('pause')
        if not arm_and_takeoff(master, TAKEOFF_ALTITUDE):
            raise Exception("Failed to takeoff for Elbow. Aborting mission.")
        navigate_to_waypoint(master, WAYPOINTS[1][0], WAYPOINTS[1][1], WAYPOINTS[1][2])

        # --- Mission 2: Takeoff, fly to WP3 and Center on Target 1 ---
        print("\n--- MISSION 2: Center over Target at Waypoint 3 Barrel (Target ID 1) ---")
        navigate_to_waypoint(master, WAYPOINTS[2][0], WAYPOINTS[2][1], WAYPOINTS[2][2])
        center_above_target(master, sock=data_sock, target_class_id=1, target_alt=WAYPOINTS[2][2])

        # --- Mission 3: Fly to WP2 and Precision Land on Target 0 ---
        print("\n--- MISSION 3: Precision Land at Waypoint 2 (Target ID 0) ---")
        send_control_command('pause')
        navigate_to_waypoint(master, WAYPOINTS[1][0], WAYPOINTS[1][1], WAYPOINTS[1][2])
        execute_precision_landing(master, sock=data_sock)

        # --- Mission 4: Takeoff, fly to WP3 and Center on Target ID 1 Again ---
        print("\n--- MISSION 4: Center over Target at Waypoint 3 Barrel Again (Target ID 1) ---")
        send_control_command('pause')
        if not arm_and_takeoff(master, TAKEOFF_ALTITUDE):
            raise Exception("Failed to takeoff for Mission 5. Aborting mission.")
        navigate_to_waypoint(master, WAYPOINTS[2][0], WAYPOINTS[2][1], WAYPOINTS[2][2])
        center_above_target(master, sock=data_sock, target_class_id=1, target_alt=WAYPOINTS[2][2])

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
        servo_control.cleanup()
        if data_sock: data_sock.close()
        print("Resources cleaned up.")

if __name__ == "__main__":
    main()