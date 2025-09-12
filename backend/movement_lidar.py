# ==============================================================================
# Drone Mission Controller with Lidar Corridor Navigation & Vision Landing
# ==============================================================================
import socket
import json
import time
import math
import sys
import threading
import serial
from pymavlink import mavutil
import servo_control  # Assuming you have this file for servo control

# --- Drone & Mission Configuration ---
CONNECTION_STRING = 'udp:127.0.0.1:14550'
BAUD_RATE = 921600
TAKEOFF_ALTITUDE = 1.1  # meters
ARMING_RETRIES = 3
ARMING_RETRY_DELAY = 3

# --- Vision & Landing Configuration ---
WAYPOINT_RADIUS = 0.5   # meters
TRACKING_SPEED = 0.5    # m/s
FWD_GAIN = 0.6
ALT_GAIN = 0.2
VERTICAL_CENTER_RATIO = 0.25
LANDING_APPROACH_ALT = 0.4
LANDING_TIMEOUT = 15
CENTERING_TIMEOUT = 20
CENTERING_ALTITUDE = 0.75
TARGET_LOST_HOVER_DURATION = 3.0
REACQUIRE_ASCEND_SPEED = 0.3
GAIN_MAX_ALT = 1.1
GAIN_MIN_ALT = 0.4
MAX_HORIZONTAL_GAIN = 0.4
MIN_HORIZONTAL_GAIN = 0.1

# --- Lidar & Corridor Navigation Configuration ---
# IMPORTANT: Update these port names and mappings for your setup
PORT_MAPPING = {
    '/dev/ttyAMA0': 'right',
    '/dev/ttyAMA1': 'left',
    '/dev/ttyAMA2': 'back',
    '/dev/ttyAMA3': 'front',
}
CORRIDOR_FWD_SPEED = 0.4 # m/s

# --- UDP Network Configuration ---
UDP_RECEIVE_IP = "127.0.0.2"
UDP_RECEIVE_PORT = 5005
CONTROL_SERVER_IP = "127.0.0.2"
CONTROL_SERVER_PORT = 5006

# --- MAVLink & ArduPilot Constants ---
GUIDED_MODE = 4
VELOCITY_CONTROL_BITMASK = 0b0000111111000111
VELOCITY_CONTROL_YAW_RATE_BITMASK = 0b0000111111000011
POSITION_CONTROL_BITMASK = 0b110111111000

# --- Global Socket ---
data_sock = None
has_turned_corner = False

# ==============================================================================
# Lidar Manager Class (Unchanged)
# ==============================================================================
class LidarManager:
    """Manages threaded reading of multiple TFmini lidars connected to the Pi."""
    def __init__(self, port_mapping):
        self.port_mapping = port_mapping
        self.lidar_data = {}
        self.data_lock = threading.Lock()
        self.stop_event = threading.Event()
        self.threads = []

        for port, name in self.port_mapping.items():
            with self.data_lock:
                self.lidar_data[name] = None

    def _lidar_thread_worker(self, port_name, sensor_name):
        """Worker thread that reads from a single serial port."""
        while not self.stop_event.is_set():
            ser = None
            try:
                ser = serial.Serial(port_name, 115200, timeout=0.5)
                while not self.stop_event.is_set():
                    if ser.in_waiting >= 9:
                        if ser.read(1) == b'\x59' and ser.read(1) == b'\x59':
                            frame = ser.read(7)
                            checksum = (0x59 + 0x59 + sum(frame[:6])) & 0xFF
                            if checksum == frame[6]:
                                distance = frame[0] + (frame[1] << 8)
                                with self.data_lock:
                                    self.lidar_data[sensor_name] = distance / 100.0 # Convert cm to meters
            except serial.SerialException:
                time.sleep(2)
            finally:
                if ser and ser.is_open:
                    ser.close()

    def start(self):
        """Starts the lidar reading threads."""
        print("Starting LidarManager threads...")
        for port, name in self.port_mapping.items():
            thread = threading.Thread(target=self._lidar_thread_worker, args=(port, name))
            thread.daemon = True
            self.threads.append(thread)
            thread.start()
        print("All lidar threads started.")
        time.sleep(1)

    def stop(self):
        """Stops all lidar reading threads."""
        print("Stopping LidarManager threads...")
        self.stop_event.set()

    def get_distances(self):
        """Returns a thread-safe copy of the latest lidar data."""
        with self.data_lock:
            return self.lidar_data.copy()

# ==============================================================================
# Core Drone Control Functions (Unchanged)
# ==============================================================================
def send_control_command(command):
    """Sends a 'pause' or 'resume' command to the detection script via TCP."""
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.connect((CONTROL_SERVER_IP, CONTROL_SERVER_PORT))
            s.sendall(command.encode('utf-8'))
    except Exception as e:
        print(f"Error sending control command: {e}")

def arm_and_takeoff(master, altitude):
    """Arms the drone and takes off to a specified altitude. Returns True on success."""
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
        mavutil.mavlink.MAV_CMD_NAV_LAND, 0, 0, 0, 0, 0, 0, 0, 0)
    master.motors_disarmed_wait()
    print("Landed and disarmed.")

def flush_socket_buffer(sock):
    """Clears any old data from the UDP socket buffer."""
    while True:
        try:
            sock.recvfrom(1024)
        except socket.timeout:
            break

# ==============================================================================
# Vision-Guided Functions
# ==============================================================================
def calculate_velocities(x_center, y_center, frame_w, frame_h):
    """Calculates horizontal velocities to track the target."""
    corrected_x_center = frame_w - x_center # Correction for mirrored video
    x_offset = (corrected_x_center - frame_w / 2) / (frame_w / 2)
    target_y = frame_h * VERTICAL_CENTER_RATIO
    y_offset = (y_center - target_y) / (frame_h / 2)
    right_vel = TRACKING_SPEED * x_offset if abs(x_offset) > 0.1 else 0
    forward_vel = -TRACKING_SPEED * y_offset * FWD_GAIN if abs(y_offset) > 0.1 else 0
    return forward_vel, right_vel

def get_dynamic_gain(current_alt):
    """Calculates a dynamic gain scaling factor based on altitude."""
    if current_alt >= GAIN_MAX_ALT: return MAX_HORIZONTAL_GAIN
    if current_alt <= GAIN_MIN_ALT: return MIN_HORIZONTAL_GAIN
    gain = MIN_HORIZONTAL_GAIN + (MAX_HORIZONTAL_GAIN - MIN_HORIZONTAL_GAIN) * \
           ((current_alt - GAIN_MIN_ALT) / (GAIN_MAX_ALT - GAIN_MIN_ALT))
    return gain

# ==============================================================================
# --- NEW: IMPROVED PRECISION LANDING FUNCTION ---
# ==============================================================================
def execute_precision_landing(master, sock, target_class_id):
    """Manages precision landing on a target with reacquisition logic."""
    flush_socket_buffer(sock)
    send_control_command('resume')
    print(f"Starting precision landing sequence on target (ID: {target_class_id})...")
    
    search_start_time = time.time()
    last_detection_time = time.time()
    last_known_alt = TAKEOFF_ALTITUDE

    while True:
        if time.time() - search_start_time > LANDING_TIMEOUT:
            print("Landing timeout reached. Aborting and hovering.")
            master.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(
                0, master.target_system, master.target_component, mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
                VELOCITY_CONTROL_BITMASK, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0))
            return False

        try:
            alt_msg = master.recv_match(type='DISTANCE_SENSOR', blocking=False, timeout=0.05)
            if alt_msg:
                last_known_alt = alt_msg.current_distance / 100.0
            current_altitude = last_known_alt

            data, _ = sock.recvfrom(1024)
            detection = json.loads(data.decode())
            
            # Check for a valid target FIRST
            #if detection.get("state") != "TRACKING" or detection.get("class_id") != target_class_id:
            #    raise socket.timeout()

            last_detection_time = time.time()
            search_start_time = time.time()

            x, y, area = detection["x_center"], detection["y_center"], detection["area"]
            w, h = detection["frame_width"], detection["frame_height"]
            
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
                time.sleep(1)
                print("Landed. Closing gripper to pick up package.")
                time.sleep(1)
                servo_control.close_gripper()
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

def center_above_target(master, sock, target_class_id):
    """Centers the drone above a target and opens the gripper."""
    flush_socket_buffer(sock)
    send_control_command('resume')
    print(f"Centering above target (ID: {target_class_id}) at {CENTERING_ALTITUDE}m...")
    
    start_time = time.time()
    last_known_alt = CENTERING_ALTITUDE

    while time.time() - start_time < CENTERING_TIMEOUT:
        # --- NEW: Read altitude from the downward-facing lidar via MAVLink ---
        alt_msg = master.recv_match(type='DISTANCE_SENSOR', blocking=False, timeout=0.05)
        if alt_msg and alt_msg.orientation == 25:
            current_alt = alt_msg.current_distance / 100.0
            last_known_alt = current_alt
        else:
            current_alt = last_known_alt
        # --- END NEW ---

        fwd_vel, right_vel, down_vel = 0, 0, 0
        target_visible = False
        
        try:
            data, _ = sock.recvfrom(1024)
            detection = json.loads(data.decode())
            if detection.get("state") == "TRACKING" and detection.get("class_id") == target_class_id:
                target_visible = True
                x, y, w, h = detection["x_center"], detection["y_center"], detection["frame_width"], detection["frame_height"]
                fwd_vel, right_vel = calculate_velocities(x, y, w, h)
                
                alt_error = CENTERING_ALTITUDE - current_alt
                down_vel = -ALT_GAIN * alt_error
                
                center_error_ratio = abs(w/2 - (w-x)) / w
                if center_error_ratio < 0.05 and abs(alt_error) < 0.10:
                    print("\nTarget centered. Opening gripper to drop package.")
                    master.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(
                        0, master.target_system, master.target_component, mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
                        VELOCITY_CONTROL_BITMASK, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0))
                    servo_control.open_gripper()
                    time.sleep(1.0)
                    send_control_command('pause')
                    return True
        except (socket.timeout, json.JSONDecodeError, KeyError):
            pass

        master.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(
            0, master.target_system, master.target_component, mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
            VELOCITY_CONTROL_BITMASK, 0, 0, 0, fwd_vel, right_vel, down_vel, 0, 0, 0, 0, 0))
        time.sleep(0.05)

    print("Centering timeout reached.")
    send_control_command('pause')
    return False

# ==============================================================================
# Lidar-Guided Corridor Navigation (Unchanged)
# ==============================================================================
def follow_corridor(master, lidar_manager, fwd_speed, stop_condition_func):
    """
    Follows a corridor until a stop condition is met, handling turns automatically.
    - fwd_speed: Positive to move forward, negative to move backward.
    - stop_condition_func: A lambda function that takes a distances dictionary and returns True to stop.
    """
    global has_turned_corner
    print(f"--- Entering follow_corridor (Speed: {fwd_speed} m/s) ---")
    
    CENTERING_GAIN, TURN_THRESHOLD, WALL_GONE_THRESHOLD, TURN_YAW_RATE, TIMEOUT = 1.0, 0.8, 3.0, 0.6, 90
    start_time = time.time()
    is_turning = False

    # --- FIX: WAIT FOR INITIAL SENSOR DATA ---
    print("Waiting for initial lidar readings...")
    while True:
        distances = lidar_manager.get_distances()
        # Check if essential lidars have reported a value (are not None)
        if all(distances.get(sensor) is not None for sensor in ['front', 'left', 'right']):
            print("Initial lidar readings received.")
            break
        if time.time() - start_time > 5: # 5 second timeout for getting data
            print("Error: Timed out waiting for initial lidar data.")
            return False
        time.sleep(0.1)

    while True:
        if time.time() - start_time > TIMEOUT:
            print("Corridor navigation timed out!")
            return False

        distances = lidar_manager.get_distances()

        if any(distances.get(sensor) is None for sensor in ['front', 'left', 'right']):
            print("\rWarning: Lost lidar data, hovering...", end="")
            master.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(
                0, master.target_system, master.target_component, mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
                VELOCITY_CONTROL_BITMASK, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0))
            time.sleep(0.1)
            continue

        if stop_condition_func(distances):
            print("\nStop condition met. Exiting follow_corridor.")
            master.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(
                0, master.target_system, master.target_component, mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
                VELOCITY_CONTROL_BITMASK, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0))
            return True

        front_dist = distances.get('front')
        left_dist = distances.get('left')
        right_dist = distances.get('right')

        if fwd_speed > 0 and front_dist < TURN_THRESHOLD and not is_turning:
            turn_direction = None
            if left_dist > WALL_GONE_THRESHOLD: turn_direction, yaw_rate = "LEFT", TURN_YAW_RATE
            elif right_dist > WALL_GONE_THRESHOLD: turn_direction, yaw_rate = "RIGHT", -TURN_YAW_RATE
            
            if turn_direction:
                print(f"\nCorner detected! Executing {turn_direction} turn.")
                is_turning = True
                turn_duration = 1.6
                turn_start = time.time()
                while time.time() - turn_start < turn_duration:
                    master.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(
                        0, master.target_system, master.target_component, mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
                        VELOCITY_CONTROL_YAW_RATE_BITMASK, 0, 0, 0, 0, 0, 0, 0, 0, 0, yaw_rate, 0))
                    time.sleep(0.1)
                print("Turn complete. Resuming forward motion.")
                has_turned_corner = True
                is_turning = False
                continue

        if left_dist > 5 or right_dist > 5: centering_error = 0
        else: centering_error = left_dist - right_dist
        
        right_vel = -CENTERING_GAIN * centering_error if fwd_speed > 0 else CENTERING_GAIN * centering_error
        
        sys.stdout.write(f"\rFollowing... Fwd: {fwd_speed:.2f}, Right Vel: {right_vel:.2f}, Front: {front_dist:.2f}m")
        sys.stdout.flush()

        master.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(
            0, master.target_system, master.target_component, mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
            VELOCITY_CONTROL_BITMASK, 0, 0, 0, fwd_speed, right_vel, 0, 0, 0, 0, 0, 0))
        
        time.sleep(0.05)

# ==============================================================================
# Main Mission Execution (Unchanged)
# ==============================================================================
def main():
    global data_sock, lidar_manager
    lidar_manager = LidarManager(PORT_MAPPING)
    lidar_manager.start()

    data_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    data_sock.bind((UDP_RECEIVE_IP, UDP_RECEIVE_PORT))
    data_sock.settimeout(0.5)

    master = mavutil.mavlink_connection(CONNECTION_STRING, baud=BAUD_RATE)
    master.wait_heartbeat()
    print(f"Heartbeat from system (system {master.target_system} component {master.target_component})")

    try:
        servo_control.setup()
        servo_control.open_gripper()
        send_control_command('pause')
        
        if not arm_and_takeoff(master, TAKEOFF_ALTITUDE): raise Exception("Failed initial takeoff")
        
        # 1. Approach first logistic, stop when front lidar is <= 5m
        print("\n--- 1. Approaching first logistic ---")
        stop_approach = lambda dists: dists.get('front', 999) <= 4.7
        if not follow_corridor(master, lidar_manager, CORRIDOR_FWD_SPEED, stop_approach): raise Exception("Failed to approach first logistic")

        # 2. Precision land to pick up the logistic
        print("\n--- 2. Landing to pick up first logistic ---")
        if not execute_precision_landing(master, data_sock, 0): raise Exception("Precision land for first logistic failed")
        
        # 3. Takeoff and navigate corridor until back lidar is >= 6m
        print("\n--- 3. Taking off and navigating to drop-off point ---")
        if not arm_and_takeoff(master, TAKEOFF_ALTITUDE): raise Exception("Failed takeoff after first pickup")
        stop_at_dropoff = lambda dists: dists.get('back', 0) >= 6.0

        has_turned_corner = False

        stop_at_dropoff = lambda dists: has_turned_corner and dists.get('back', 0) >= 6.0
        if not follow_corridor(master, lidar_manager, CORRIDOR_FWD_SPEED, stop_at_dropoff): raise Exception("Failed to navigate corridor to drop-off point")


        # 4. Center and drop the first logistic on the barrel
        print("\n--- 4. Centering to drop first logistic ---")
        if not center_above_target(master, data_sock, 1): raise Exception("Centering for first drop-off failed")
        
        # 5. Go BACKWARD to the corner
        print("\n--- 5. Returning to corner for second logistic ---")
        stop_at_corner = lambda dists: dists.get('back', 0) >= 1.0
        if not follow_corridor(master, lidar_manager, -CORRIDOR_FWD_SPEED, stop_at_corner): raise Exception("Failed to return to corner")
        
        # 6. Land and pick up the second logistic
        print("\n--- 6. Landing to pick up second logistic ---")
        if not execute_precision_landing(master, data_sock, 0): raise Exception("Precision land for second logistic failed")
            
        # 7. Takeoff and go forward to the drop-off point again
        print("\n--- 7. Navigating to drop-off point again ---")
        if not arm_and_takeoff(master, TAKEOFF_ALTITUDE): raise Exception("Failed takeoff after second pickup")
        if not follow_corridor(master, lidar_manager, CORRIDOR_FWD_SPEED, stop_at_dropoff): raise Exception("Failed to navigate to drop-off for second time")
            
        # 8. Center and drop the second logistic
        print("\n--- 8. Centering to drop second logistic ---")
        if not center_above_target(master, data_sock, 1): raise Exception("Centering for second drop-off failed")

        # 9. Move a bit forward and land normally
        print("\n--- 9. Moving to final landing spot ---")
        stop_final = lambda dists: dists.get('front', 999) <= 2.0
        follow_corridor(master, lidar_manager, 0.3, stop_final)
        print("Final landing.")
        land_normally(master)

        print("\nMISSION FINISHED SUCCESSFULLY!")

    except KeyboardInterrupt:
        print("\nKeyboard interrupt received. Landing immediately...")
        land_normally(master)
    except Exception as e:
        print(f"\nMISSION FAILED: {e}")
        print("Attempting to land immediately...")
        land_normally(master)
    finally:
        lidar_manager.stop()
        servo_control.cleanup()
        if data_sock: data_sock.close()
        print("Resources cleaned up.")

if __name__ == "__main__":
    main()