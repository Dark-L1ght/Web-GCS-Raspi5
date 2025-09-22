import socket
import json
import time
import sys
import threading
import serial
import math
from pymavlink import mavutil
import servo_control  # servo_control.py

# ==============================================================================
# --- Configuration Constants ---
# ==============================================================================

# --- Drone & Mission Configuration ---
CONNECTION_STRING = 'udp:127.0.0.1:14550'
BAUD_RATE = 921600
TAKEOFF_ALTITUDE = 1.1  # meters
ARMING_RETRIES = 3
ARMING_RETRY_DELAY = 3
GUIDED_MODE = 4

# --- Vision & Landing Configuration ---
TRACKING_SPEED = 0.5    # Base speed for horizontal correction during landing
CENTERING_SPEED = 0.25      # m/s, horizontal speed for fine-tuning position over a target.
FWD_GAIN = 0.8              # Proportional gain for forward velocity based on vertical target offset.
ALT_GAIN = 0.15             # Proportional gain for vertical velocity.
VERTICAL_CENTER_RATIO = 0.25# The vertical point in the camera frame to aim for (0.0=top, 1.0=bottom).
LANDING_APPROACH_ALT = 0.45 # Altitude at which we commit to landing.
LANDING_TIMEOUT = 25        # Max time for the entire landing sequence.
CENTERING_TIMEOUT = 20      # Max time to center over a drop-off target.
CENTERING_ALTITUDE = 1.0    # Target altitude for centering/dropping.
TARGET_LOST_HOVER_DURATION = 3.0 # Time to wait before ascending to reacquire a lost target.
REACQUIRE_ASCEND_SPEED = 0.3     # Speed at which to ascend to reacquire target.

# --- Dynamic Gain for Vision Control ---
GAIN_MAX_ALT = 1.1          # Altitude at which horizontal gain is maximum.
GAIN_MIN_ALT = 0.4          # Altitude at which horizontal gain is minimum.
MAX_HORIZONTAL_GAIN = 0.5
MIN_HORIZONTAL_GAIN = 0.2

# --- Lidar & Corridor Navigation Configuration ---
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
# MAVLINK BTIMASKS
# For velocity control without yaw rate (vision-guided)
# BITS:  (1=ignore)  YAW_RATE, YAW,      POS,       VEL,       ACCEL
# BINARY:             1,        1,   111,111,   000,       111
VELOCITY_CONTROL_BITMASK =        0b001111000111
# For velocity control with yaw rate (lidar-guided)
# BINARY:             0,        1,   111,111,   000,       111
VELOCITY_CONTROL_YAW_RATE_BITMASK = 0b001111000111 & ~0b100000000000

# --- Global State Variables ---
data_sock = None
has_turned_corner = False

# ==============================================================================
# --- Lidar Manager Class ---
# ==============================================================================
class LidarManager:
    """Manages threaded reading of multiple TFmini lidars connected to the Pi."""
    def __init__(self, port_mapping):
        self.port_mapping = port_mapping
        self.lidar_data = {name: None for name in port_mapping.values()}
        self.data_lock = threading.Lock()
        self.stop_event = threading.Event()
        self.threads = []

    def _lidar_thread_worker(self, port_name, sensor_name):
        """Worker thread that reads from a single serial port."""
        while not self.stop_event.is_set():
            ser = None
            try:
                ser = serial.Serial(port_name, 115200, timeout=0.5)
                while not self.stop_event.is_set():
                    if ser.in_waiting >= 9 and ser.read(1) == b'\x59' and ser.read(1) == b'\x59':
                        frame = ser.read(7)
                        checksum = (0x59 + 0x59 + sum(frame[:6])) & 0xFF
                        if checksum == frame[6]:
                            distance = frame[0] + (frame[1] << 8)
                            with self.data_lock:
                                self.lidar_data[sensor_name] = distance / 100.0 # Convert cm to meters
            except serial.SerialException:
                print(f"Warning: Lidar on {port_name} not available. Retrying...")
                time.sleep(2)
            finally:
                if ser and ser.is_open:
                    ser.close()

    def start(self):
        print("Starting LidarManager threads...")
        for port, name in self.port_mapping.items():
            thread = threading.Thread(target=self._lidar_thread_worker, args=(port, name))
            thread.daemon = True
            self.threads.append(thread)
            thread.start()
        print("All lidar threads started.")
        time.sleep(1)

    # UPDATED: Replaced with test version for a graceful shutdown.
    def stop(self):
        print("Stopping LidarManager threads...")
        self.stop_event.set()
        for thread in self.threads:
            thread.join() # Wait for threads to finish cleanly

    def get_distances(self):
        with self.data_lock:
            return self.lidar_data.copy()

# ==============================================================================
# --- Core Drone Control & Utility Functions ---
# ==============================================================================
def send_control_command(command):
    """Sends a command (e.g., 'resume' or 'pause') to the vision processing server."""
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.connect((CONTROL_SERVER_IP, CONTROL_SERVER_PORT))
            s.sendall(command.encode('utf-8'))
    except Exception as e:
        print(f"Error sending control command '{command}': {e}")

def arm_and_takeoff(master, altitude):
    """Arms the drone and takes off to a specified altitude."""
    print("Setting mode to GUIDED...")
    master.mav.set_mode_send(master.target_system, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, GUIDED_MODE)
    for attempt in range(1, ARMING_RETRIES + 1):
        print(f"Arming motors (Attempt {attempt}/{ARMING_RETRIES})...")
        master.mav.command_long_send(
            master.target_system, master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)
        try:
            master.motors_armed_wait(timeout=5)
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
        if current_altitude >= altitude * 0.90:
            print("\nTarget altitude reached.")
            return True
        time.sleep(0.1)

def land_normally(master):
    """Commands the drone to land using the standard LAND mode."""
    print("Executing normal landing...")
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_LAND, 0, 0, 0, 0, 0, 0, 0, 0)
    try:
        master.motors_disarmed_wait(timeout=30)
        print("Landed and disarmed.")
    except Exception as e:
        print(f"Warning: Did not receive disarm confirmation. {e}")

def flush_socket_buffer(sock):
    while True:
        try:
            sock.recvfrom(1024)
        except socket.timeout:
            break

# ==============================================================================
# --- Vision-Guided Functions ---
# ==============================================================================

# UPDATED: Replaced with test version to handle mirrored camera video feeds.
def calculate_velocities(x_center, y_center, frame_w, frame_h, speed):
    """Calculates forward and right velocities based on target position and a given speed."""
    # This logic corrects for a horizontally flipped video stream
    corrected_x_center = frame_w - x_center
    x_offset = (corrected_x_center - frame_w / 2) / (frame_w / 2)
    
    target_y = frame_h * VERTICAL_CENTER_RATIO
    y_offset = (y_center - target_y) / (frame_h / 2)
    
    right_vel = speed * x_offset if abs(x_offset) > 0.1 else 0
    # The negative sign correctly maps "target is below center" to "move forward"
    forward_vel = -speed * y_offset * FWD_GAIN if abs(y_offset) > 0.1 else 0
    return forward_vel, right_vel

def get_dynamic_gain(current_alt):
    if current_alt is None: return MIN_HORIZONTAL_GAIN
    if current_alt >= GAIN_MAX_ALT: return MAX_HORIZONTAL_GAIN
    if current_alt <= GAIN_MIN_ALT: return MIN_HORIZONTAL_GAIN
    gain_range = MAX_HORIZONTAL_GAIN - MIN_HORIZONTAL_GAIN
    alt_range = GAIN_MAX_ALT - GAIN_MIN_ALT
    gain = MIN_HORIZONTAL_GAIN + gain_range * ((current_alt - GAIN_MIN_ALT) / alt_range)
    return gain

# UPDATED: Replaced with test version for consistency and simpler logic.
def execute_precision_landing(master, sock):
    flush_socket_buffer(sock)
    send_control_command('resume')
    print(f"Starting precision landing sequence (Accepting IDs: [0, 1])...")
    
    # CORRECTED: Set the overall timeout start time ONCE.
    search_start_time = time.time()
    last_detection_time = time.time()
    last_known_alt = TAKEOFF_ALTITUDE

    while time.time() - search_start_time < LANDING_TIMEOUT:
        try:
            alt_msg = master.recv_match(type='DISTANCE_SENSOR', blocking=False, timeout=0.05)
            if alt_msg and alt_msg.orientation == 25: # Downward facing
                last_known_alt = alt_msg.current_distance / 100.0
            current_altitude = last_known_alt

            data, _ = sock.recvfrom(1024)
            detection = json.loads(data.decode())
            
            detected_id = detection.get("class_id")
            if detection.get("state") != "TRACKING" or detected_id not in [0, 1]:
                raise socket.timeout # Treat non-target detections as "not found"

            last_detection_time = time.time()

            x, y = detection["x_center"], detection["y_center"]
            w, h = detection["frame_width"], detection["frame_height"]
            
            # Use TRACKING_SPEED for descent, let dynamic gain control horizontal
            fwd_vel, right_vel = calculate_velocities(x, y, w, h, TRACKING_SPEED)
            horizontal_gain = get_dynamic_gain(current_altitude)
            fwd_vel *= horizontal_gain
            right_vel *= horizontal_gain
            
            # Simple proportional descent towards the target
            down_vel = 0.2 # Slow, constant descent when target is visible
            master.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(
                0, master.target_system, master.target_component, mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
                VELOCITY_CONTROL_BITMASK, 0, 0, 0, fwd_vel, right_vel, down_vel, 0, 0, 0, 0, 0))
            
            center_error_ratio = abs(x - w / 2) / (w/2)
            print(f"\rLANDING (ID {detected_id}): Alt: {current_altitude:.2f}m, Gain: {horizontal_gain:.2f}, Err: {center_error_ratio:.2%}", end="")

            if current_altitude < LANDING_APPROACH_ALT and center_error_ratio < 0.175:
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
                vz = -REACQUIRE_ASCEND_SPEED # Ascend to find target again

            master.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(
                0, master.target_system, master.target_component, mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
                VELOCITY_CONTROL_BITMASK, 0, 0, 0, 0, 0, vz, 0, 0, 0, 0, 0))

    print("\nLanding timeout reached. Aborting and hovering.")
    master.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(
        0, master.target_system, master.target_component, mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
        VELOCITY_CONTROL_BITMASK, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0))
    return False

# UPDATED: Replaced with test version for clearer, more robust logic.
def center_above_target(master, sock, target_class_id):
    flush_socket_buffer(sock)
    send_control_command('resume')
    print(f"Centering above target (ID: {target_class_id}) at {CENTERING_ALTITUDE}m...")
    
    start_time = time.time()
    last_known_alt = CENTERING_ALTITUDE

    while time.time() - start_time < CENTERING_TIMEOUT:
        alt_msg = master.recv_match(type='DISTANCE_SENSOR', blocking=False, timeout=0.05)
        if alt_msg and alt_msg.orientation == 25:
            current_alt = alt_msg.current_distance / 100.0
            last_known_alt = current_alt
        else:
            current_alt = last_known_alt

        fwd_vel, right_vel = 0, 0

        try:
            data, _ = sock.recvfrom(1024)
            detection = json.loads(data.decode())
            if detection.get("state") == "TRACKING" and detection.get("class_id") == target_class_id:
                x, y, w, h = detection["x_center"], detection["y_center"], detection["frame_width"], detection["frame_height"]
                fwd_vel, right_vel = calculate_velocities(x, y, w, h, CENTERING_SPEED)

                center_error_ratio = abs(x - w / 2) / (w/2)
                alt_error = CENTERING_ALTITUDE - current_alt
                sys.stdout.write(f"\rCentering... Err: {center_error_ratio:.2%}, Alt Err: {alt_error:+.2f}m")
                sys.stdout.flush()

                if center_error_ratio < 0.1 and abs(alt_error) < 0.1:
                    print("\nTarget centered. Stabilizing...")
                    master.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(
                        0, master.target_system, master.target_component, mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
                        VELOCITY_CONTROL_BITMASK, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0))
                    
                    time.sleep(1.0)
                    
                    print("Opening gripper.")
                    servo_control.open_gripper()
                    time.sleep(1.0)
                    send_control_command('pause')
                    return True
        except (socket.timeout, json.JSONDecodeError, KeyError):
            print("\rSearching for drop-off target...", end="")
        # Altitude control is handled outside the try/except for continuous adjustment
        alt_error = CENTERING_ALTITUDE - current_alt
        down_vel = -ALT_GAIN * alt_error

        master.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(
            0, master.target_system, master.target_component, mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
            VELOCITY_CONTROL_BITMASK, 0, 0, 0, fwd_vel, right_vel, down_vel, 0, 0, 0, 0, 0))
        time.sleep(0.05)

    print("\nCentering timeout reached.")
    send_control_command('pause')
    return False

# ==============================================================================
# --- Lidar-Guided Corridor Navigation ---
# ==============================================================================

def get_current_heading(master):
    msg = master.recv_match(type='ATTITUDE', blocking=True, timeout=1)
    return msg.yaw if msg else None

def get_angle_difference(angle1_rad, angle2_rad):
    diff = angle2_rad - angle1_rad
    while diff <= -math.pi: diff += 2 * math.pi
    while diff > math.pi: diff -= 2 * math.pi
    return diff

# UPDATED: Replaced with safer, more robust test version.
def turn_drone_by_heading(master, angle_degrees, yaw_rate_rads=math.radians(30), tolerance_rad=0.1):
    print(f"\n--- Executing {angle_degrees}° turn ---")
    
    start_heading_rad = get_current_heading(master)
    if start_heading_rad is None:
        print("Error: Could not get initial heading.")
        return False

    target_heading_rad = start_heading_rad + math.radians(angle_degrees)
    target_heading_rad = (target_heading_rad + math.pi) % (2 * math.pi) - math.pi

    print(f"Start: {math.degrees(start_heading_rad):.1f}°, Target: {math.degrees(target_heading_rad):.1f}°")

    turn_yaw_rate = math.copysign(yaw_rate_rads, angle_degrees)
    start_time = time.time()
    TURN_TIMEOUT = 10

    while time.time() - start_time < TURN_TIMEOUT:
        current_heading_rad = get_current_heading(master)
        if current_heading_rad is None:
            print("Warning: Lost heading data, hovering.")
            master.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(
                0, master.target_system, master.target_component, mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
                VELOCITY_CONTROL_YAW_RATE_BITMASK, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0))
            time.sleep(0.1)
            continue

        remaining_angle_rad = get_angle_difference(current_heading_rad, target_heading_rad)
        
        sys.stdout.write(f"\rTurning... Remaining: {math.degrees(remaining_angle_rad):.1f}°")
        sys.stdout.flush()

        if abs(remaining_angle_rad) < tolerance_rad:
            print("\nTurn complete.")
            master.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(
                0, master.target_system, master.target_component, mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
                VELOCITY_CONTROL_YAW_RATE_BITMASK, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0))
            time.sleep(0.5)
            return True

        master.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(
            0, master.target_system, master.target_component, mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
            VELOCITY_CONTROL_YAW_RATE_BITMASK, 0, 0, 0, 0, 0, 0, 0, 0, 0, turn_yaw_rate, 0))
        
        time.sleep(0.05)

    print("\nError: Turn timed out.")
    return False

# UPDATED: Replaced with test version for consistency.
def follow_corridor(master, lidar_manager, fwd_speed, stop_condition_func):
    global has_turned_corner
    print(f"--- Entering follow_corridor (Speed: {fwd_speed} m/s) ---")
    
    CENTERING_GAIN, TURN_THRESHOLD, WALL_GONE_THRESHOLD, TIMEOUT = 1.0, 1.2, 3.0, 90
    start_time = time.time()
    is_turning = False

    wait_start_time = time.time()
    while True:
        distances = lidar_manager.get_distances()
        check_sensors = ['left', 'right']
        check_sensors.append('front' if fwd_speed >= 0 else 'back')
        if all(distances.get(s) is not None for s in check_sensors):
            break
        if time.time() - wait_start_time > 5:
            print("Error: Timed out waiting for initial lidar data.")
            return False
        time.sleep(0.1)

    while time.time() - start_time < TIMEOUT:
        distances = lidar_manager.get_distances()
        if any(d is None for d in [distances.get('front'), distances.get('left'), distances.get('right')]):
            master.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(
                0, master.target_system, master.target_component, mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
                VELOCITY_CONTROL_YAW_RATE_BITMASK, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0))
            time.sleep(0.1)
            continue

        if stop_condition_func(distances):
            print("\nStop condition met. Exiting corridor.")
            master.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(
                0, master.target_system, master.target_component, mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
                VELOCITY_CONTROL_YAW_RATE_BITMASK, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0))
            return True

        front_dist, left_dist, right_dist = distances['front'], distances['left'], distances['right']
        
        if fwd_speed > 0 and front_dist < TURN_THRESHOLD and not is_turning:
            turn_direction = None
            if left_dist > WALL_GONE_THRESHOLD: turn_direction = "LEFT"
            elif right_dist > WALL_GONE_THRESHOLD: turn_direction = "RIGHT"
            
            if turn_direction:
                print(f"\nObstacle ahead. Turning {turn_direction}.")
                master.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(
                    0, master.target_system, master.target_component, mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
                    VELOCITY_CONTROL_YAW_RATE_BITMASK, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0))
                time.sleep(1.0)
                is_turning = True
                turn_angle_deg = -90.0 if turn_direction == "LEFT" else 90.0
                if turn_drone_by_heading(master, turn_angle_deg):
                    has_turned_corner = True
                else:
                    return False
                is_turning = False
                continue
        
        centering_error = left_dist - right_dist if not (left_dist > 5 or right_dist > 5) else 0
        right_vel = -CENTERING_GAIN * centering_error
        
        sys.stdout.write(f"\rFollowing... Fwd: {fwd_speed:.2f}, Right Vel: {right_vel:.2f}, Front: {front_dist:.2f}m")
        sys.stdout.flush()

        master.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(
            0, master.target_system, master.target_component, mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
            VELOCITY_CONTROL_YAW_RATE_BITMASK, 0, 0, 0, fwd_speed, right_vel, 0, 0, 0, 0, 0, 0))
        time.sleep(0.05)
    
    print("\nCorridor navigation timed out!")
    return False

# UPDATED: Replaced with safer test version that fixes altitude control bug.
def fly_straight(master, lidar_manager, fwd_speed, stop_condition_func, target_altitude=None):
    print(f"--- Flying straight (Speed: {fwd_speed} m/s) ---")
    TIMEOUT = 30
    start_time = time.time()
    last_known_alt = -1 # Use -1 to indicate we need the first reading

    check_sensor = 'front' if fwd_speed >= 0 else 'back'
    wait_start_time = time.time()
    while True:
        # Get initial altitude before moving
        if last_known_alt == -1:
            alt_msg = master.recv_match(type='DISTANCE_SENSOR', blocking=True, timeout=1)
            if alt_msg and alt_msg.orientation == 25:
                last_known_alt = alt_msg.current_distance / 100.0
        
        dist = lidar_manager.get_distances().get(check_sensor)
        if dist is not None and last_known_alt != -1:
            print(f"Initial readings received: {check_sensor.capitalize()} {dist:.2f}m, Alt {last_known_alt:.2f}m")
            break
        if time.time() - wait_start_time > 5:
            print("Error: Timed out waiting for initial sensor data.")
            return False
        time.sleep(0.1)

    while time.time() - start_time < TIMEOUT:
        distances = lidar_manager.get_distances()
        
        # Altitude control logic runs continuously
        down_vel = 0
        if target_altitude is not None:
            alt_msg = master.recv_match(type='DISTANCE_SENSOR', blocking=False, timeout=0.01)
            current_alt = last_known_alt
            if alt_msg and alt_msg.orientation == 25:
                current_alt = alt_msg.current_distance / 100.0
                last_known_alt = current_alt
            alt_error = target_altitude - current_alt
            if abs(alt_error) > 0.05: 
                down_vel = -ALT_GAIN * alt_error
        
        current_dist = distances.get(check_sensor)
        
        if current_dist is None:
            print(f"\rWarning: Lost {check_sensor} lidar, hovering...", end="")
            # BUG FIX: Maintain altitude control even when hovering
            master.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(
                0, master.target_system, master.target_component, mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
                VELOCITY_CONTROL_YAW_RATE_BITMASK, 0, 0, 0, 0, 0, down_vel, 0, 0, 0, 0, 0))
            time.sleep(0.1)
            continue
        
        if stop_condition_func(distances):
            print("\nStop condition met.")
            master.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(
                0, master.target_system, master.target_component, mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
                VELOCITY_CONTROL_YAW_RATE_BITMASK, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0))
            return True

        master.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(
            0, master.target_system, master.target_component, mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
            VELOCITY_CONTROL_YAW_RATE_BITMASK, 0, 0, 0, fwd_speed, 0, down_vel, 0, 0, 0, 0, 0))
        
        sys.stdout.write(f"\rFlying straight... {check_sensor.capitalize()}: {current_dist:.2f}m, Alt: {last_known_alt:.2f}m")
        sys.stdout.flush()
        time.sleep(0.05)

    print("\nfly_straight timed out!")
    return False

# ==============================================================================
# Main Mission Execution
# ==============================================================================
def main():
    global data_sock, lidar_manager, has_turned_corner
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
        
        if not arm_and_takeoff(master, TAKEOFF_ALTITUDE): raise Exception("Initial takeoff failed")
        
        # 1. Approach first logistic
        print("\n--- 1. Approaching first logistic ---")
        first_logistic = lambda dists: dists.get('front', 999) <= 4.7
        if not follow_corridor(master, lidar_manager, CORRIDOR_FWD_SPEED, first_logistic): raise Exception("Failed to approach first logistic")

        # 2. Land and pick up logistic
        print("\n--- 2. Landing to pick up first logistic ---")
        if not execute_precision_landing(master, data_sock, 0): raise Exception("Precision land for first logistic failed")
        
        # 3. Navigate to drop-off point
        print("\n--- 3. Taking off and navigating to drop-off point ---")
        if not arm_and_takeoff(master, TAKEOFF_ALTITUDE): raise Exception("Failed takeoff after first pickup")
        
        print("--- Navigating to corridor exit ---")
        has_turned_corner = False
        stop_at_exit = lambda dists: has_turned_corner and (dists.get('back', 0) >= 3.8)
        if not follow_corridor(master, lidar_manager, CORRIDOR_FWD_SPEED, stop_at_exit): 
            raise Exception("Failed to navigate corridor exit")
        
        print("\n--- Corridor exited. Flying straight to drop-off point. ---")
        stop_at_dropoff = lambda dists: dists.get('front', 99) <= 4.0
        if not fly_straight(master, lidar_manager, CORRIDOR_FWD_SPEED, stop_at_dropoff):
            raise Exception("Failed to fly to drop-off point")

        # 4. Center and drop
        print("\n--- 4. Centering for drop-off ---")
        if not center_above_target(master, data_sock, 1): raise Exception("Centering for drop-off failed")
        
        # 5. Return to corner
        print("\n--- 5. Returning for second logistic ---")
        stop_at_entrance = lambda dists: dists.get('left', 999) < 2.0 and dists.get('right', 999) < 2.0
        if not fly_straight(master, lidar_manager, -CORRIDOR_FWD_SPEED, stop_at_entrance): raise Exception("Return to corridor failed")
        
        print("\n--- Navigating back to corner ---")
        stop_at_corner = lambda dists: dists.get('back', 999) <= 2.5
        if not follow_corridor(master, lidar_manager, -CORRIDOR_FWD_SPEED, stop_at_corner): raise Exception("Return to corner failed")
        
        print("\n--- Aligning for second pickup ---")
        stop_near_pickup = lambda dists: dists.get('front', 999) <= 1.0 
        if not fly_straight(master, lidar_manager, CORRIDOR_FWD_SPEED, stop_near_pickup): raise Exception("Final alignment for pickup failed")
        
        # 6. Land and pick up second logistic
        print("\n--- 6. Landing for second logistic ---")
        if not execute_precision_landing(master, data_sock, 0): raise Exception("Second precision land failed")
        
        # 7. Navigate to drop-off again
        print("\n--- 7. Taking off for second delivery ---")
        if not arm_and_takeoff(master, TAKEOFF_ALTITUDE): raise Exception("Takeoff after second pickup failed")

        get_out_from_corner = lambda dists: dists.get('back', 0) >= 2.5
        if not fly_straight(master, lidar_manager, CORRIDOR_FWD_SPEED, get_out_from_corner): raise Exception("Failed to exit corner")

        print("\n--- Navigating corridor exit (second trip) ---")
        if not follow_corridor(master, lidar_manager, CORRIDOR_FWD_SPEED, stop_at_exit): raise Exception("Second corridor navigation failed")
        
        print("\n--- Flying to drop-off point (second trip) ---")
        if not fly_straight(master, lidar_manager, CORRIDOR_FWD_SPEED, stop_at_dropoff, target_altitude=0.8): raise Exception("Second flight to drop-off failed")
        
        # 8. Center and drop second logistic
        print("\n--- 8. Centering for second drop-off ---")
        if not center_above_target(master, data_sock, 1): raise Exception("Second centering for drop-off failed")

        # 9. Move to final landing spot
        print("\n--- 9. Moving to final landing spot ---")
        stop_final = lambda dists: dists.get('front', 999) <= 2.0
        # BUG FIX: Use fly_straight for the final approach, not follow_corridor
        if not fly_straight(master, lidar_manager, 0.3, stop_final): raise Exception("Final approach failed")
        
        print("Final landing.")
        land_normally(master)
        print("\nMISSION COMPLETE!")

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