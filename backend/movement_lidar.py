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

# --- Dynamic Gain for Vision Control (adjusts horizontal speed based on altitude) ---
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
                    # TFmini-S frame format: 0x59 0x59 Dist_L Dist_H ... Checksum
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
        time.sleep(1) # Give threads time to initialize

    def stop(self):
        print("Stopping LidarManager threads...")
        self.stop_event.set()
        for thread in self.threads:
            thread.join()

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
    # Wait until the drone reaches target altitude
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
        print(f"Warning: Did not receive disarm confirmation, but landing was commanded. {e}")

def flush_socket_buffer(sock):
    """Clears any old data from the UDP socket buffer."""
    while True:
        try:
            sock.recvfrom(1024)
        except socket.timeout:
            break

# ==============================================================================
# --- Vision-Guided Functions ---
# ==============================================================================
def calculate_velocities(x_center, y_center, frame_w, frame_h, speed):
    """Calculates forward and right velocities based on target position and a given speed."""
    # Correct for camera inversion if necessary. Assuming standard coordinates (0,0 top-left).
    # A target to the right of center (x_center > w/2) should result in a positive right_vel.
    # A target below the desired vertical point should result in a positive forward_vel.
    x_offset = (x_center - frame_w / 2) / (frame_w / 2)
    target_y = frame_h * VERTICAL_CENTER_RATIO
    y_offset = (y_center - target_y) / (frame_h / 2)

    # Deadband to prevent oscillation from minor noise
    right_vel = speed * x_offset if abs(x_offset) > 0.1 else 0
    forward_vel = speed * y_offset * FWD_GAIN if abs(y_offset) > 0.1 else 0
    return forward_vel, right_vel

def get_dynamic_gain(current_alt):
    """Calculates a horizontal speed gain based on altitude for smoother control."""
    if current_alt is None: return MIN_HORIZONTAL_GAIN
    if current_alt >= GAIN_MAX_ALT: return MAX_HORIZONTAL_GAIN
    if current_alt <= GAIN_MIN_ALT: return MIN_HORIZONTAL_GAIN

    gain_range = MAX_HORIZONTAL_GAIN - MIN_HORIZONTAL_GAIN
    alt_range = GAIN_MAX_ALT - GAIN_MIN_ALT
    gain = MIN_HORIZONTAL_GAIN + gain_range * ((current_alt - GAIN_MIN_ALT) / alt_range)
    return gain

def execute_precision_landing(master, sock, target_class_id):
    """Executes a vision-guided landing sequence over a specified target ID."""
    flush_socket_buffer(sock)
    send_control_command('resume')
    print(f"Starting precision landing sequence (Accepting ID: {target_class_id})...")

    # BUG FIX: Set the overall timeout start time ONCE.
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

            # Ensure we are tracking the correct target for this landing
            if detection.get("state") != "TRACKING" or detection.get("class_id") != target_class_id:
                raise socket.timeout() # Treat non-target detections as "not found"

            last_detection_time = time.time()

            x, y, w, h = detection["x_center"], detection["y_center"], detection["frame_width"], detection["frame_height"]

            # Use a base speed and apply dynamic gain for horizontal movements
            fwd_vel, right_vel = calculate_velocities(x, y, w, h, 1.0) # Base speed of 1.0
            horizontal_gain = get_dynamic_gain(current_altitude)
            fwd_vel *= horizontal_gain
            right_vel *= horizontal_gain

            # Use a slow, constant descent when the target is visible
            down_vel = 0.2
            master.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(
                0, master.target_system, master.target_component, mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
                VELOCITY_CONTROL_BITMASK, 0, 0, 0, fwd_vel, right_vel, down_vel, 0, 0, 0, 0, 0))

            center_error_ratio = abs(x - w / 2) / (w/2)
            print(f"\rLANDING (ID {target_class_id}): Alt: {current_altitude:.2f}m, Gain: {horizontal_gain:.2f}, Err: {center_error_ratio:.2%}", end="")

            # Check for landing conditions
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
            print(f"\rSearching for landing target (ID {target_class_id})... Time since last seen: {time_since_lost:.1f}s", end="")

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

def center_above_target(master, sock, target_class_id):
    """Centers the drone over a target at a specific altitude, then opens the gripper."""
    flush_socket_buffer(sock)
    send_control_command('resume')
    print(f"Centering above target (ID: {target_class_id}) at {CENTERING_ALTITUDE}m...")

    start_time = time.time()
    last_known_alt = CENTERING_ALTITUDE

    while time.time() - start_time < CENTERING_TIMEOUT:
        alt_msg = master.recv_match(type='DISTANCE_SENSOR', blocking=False, timeout=0.05)
        current_alt = last_known_alt
        if alt_msg and alt_msg.orientation == 25:
            current_alt = alt_msg.current_distance / 100.0
            last_known_alt = current_alt
        
        fwd_vel, right_vel = 0, 0
        try:
            data, _ = sock.recvfrom(1024)
            detection = json.loads(data.decode())
            if detection.get("state") == "TRACKING" and detection.get("class_id") == target_class_id:
                x, y, w, h = detection["x_center"], detection["y_center"], detection["frame_width"], detection["frame_height"]
                # Use the slower CENTERING_SPEED for fine adjustments
                fwd_vel, right_vel = calculate_velocities(x, y, w, h, CENTERING_SPEED)

                center_error_ratio = abs(x - w / 2) / (w/2)
                alt_error = CENTERING_ALTITUDE - current_alt
                sys.stdout.write(f"\rCentering... Err: {center_error_ratio:.2%}, Alt Err: {alt_error:+.2f}m")
                sys.stdout.flush()

                if center_error_ratio < 0.08 and abs(alt_error) < 0.10:
                    print("\nTarget centered. Stabilizing for drop...")
                    # Command a hover to stop all movement
                    master.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(
                        0, master.target_system, master.target_component, mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
                        VELOCITY_CONTROL_BITMASK, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0))
                    time.sleep(1.0) # Wait for inertia to dissipate
                    
                    print("Drone stable. Opening gripper to drop package.")
                    servo_control.open_gripper()
                    time.sleep(1.0) # Wait for package to drop clear
                    send_control_command('pause')
                    return True
            else:
                 raise socket.timeout() # Not the right target, treat as not found
        except (socket.timeout, json.JSONDecodeError, KeyError):
            # LOGIC IMPROVEMENT: If target is lost, actively command a hover.
            fwd_vel, right_vel = 0, 0
            print("\rSearching for drop-off target...", end="")

        alt_error = CENTERING_ALTITUDE - current_alt
        down_vel = -ALT_GAIN * alt_error if abs(alt_error) > 0.05 else 0

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
    """Gets the current heading of the drone in radians from the ATTITUDE message."""
    msg = master.recv_match(type='ATTITUDE', blocking=True, timeout=1)
    if msg:
        return msg.yaw
    return None

def get_angle_difference(angle1_rad, angle2_rad):
    """Calculates the shortest difference between two angles in radians."""
    diff = angle2_rad - angle1_rad
    while diff <= -math.pi: diff += 2 * math.pi
    while diff > math.pi: diff -= 2 * math.pi
    return diff

def turn_drone_by_heading(master, angle_degrees, turn_speed_rads=math.radians(30), tolerance_rad=0.1):
    """Turns the drone by a specific angle based on its IMU heading."""
    start_heading_rad = get_current_heading(master)
    if start_heading_rad is None:
        print("Error: Could not get initial heading for turn.")
        return False

    target_heading_rad = start_heading_rad + math.radians(angle_degrees)
    turn_yaw_rate = math.copysign(turn_speed_rads, angle_degrees)
    
    print(f"Executing turn: Start {math.degrees(start_heading_rad):.1f}°, Target {math.degrees(target_heading_rad):.1f}°")
    
    start_time = time.time()
    TURN_TIMEOUT = 10
    
    while time.time() - start_time < TURN_TIMEOUT:
        current_heading_rad = get_current_heading(master)
        if current_heading_rad is None:
            time.sleep(0.1)
            continue

        remaining_angle = get_angle_difference(current_heading_rad, target_heading_rad)
        if abs(remaining_angle) < tolerance_rad:
            print("\nTurn complete.")
            # Send a final command to stop rotation
            master.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(
                0, master.target_system, master.target_component, mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
                VELOCITY_CONTROL_YAW_RATE_BITMASK, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0))
            time.sleep(0.5)
            return True

        master.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(
            0, master.target_system, master.target_component, mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
            VELOCITY_CONTROL_YAW_RATE_BITMASK, 0, 0, 0, 0, 0, 0, 0, 0, 0, turn_yaw_rate, 0))
        
        sys.stdout.write(f"\rTurning... Remaining: {math.degrees(remaining_angle):.1f}°")
        sys.stdout.flush()
        time.sleep(0.05)
        
    print("\nTurn timed out.")
    return False

def follow_corridor(master, lidar_manager, fwd_speed, stop_condition_func):
    """Follows a corridor until a stop condition is met, handling turns automatically."""
    global has_turned_corner
    print(f"--- Entering follow_corridor (Speed: {fwd_speed} m/s) ---")
    
    CENTERING_GAIN, TURN_THRESHOLD, WALL_GONE_THRESHOLD, TIMEOUT = 1.0, 1.2, 3.0, 90
    start_time = time.time()
    is_turning = False

    # Wait for valid initial sensor readings before moving
    while True:
        distances = lidar_manager.get_distances()
        if all(distances.get(s) is not None for s in ['front', 'left', 'right']): break
        if time.time() - start_time > 5:
            print("Error: Timed out waiting for initial lidar data.")
            return False
        time.sleep(0.1)

    while time.time() - start_time < TIMEOUT:
        distances = lidar_manager.get_distances()
        if any(d is None for d in [distances.get('front'), distances.get('left'), distances.get('right')]):
            print("\rWarning: Lost critical lidar data, hovering...", end="")
            master.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(
                0, master.target_system, master.target_component, mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
                VELOCITY_CONTROL_YAW_RATE_BITMASK, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0))
            time.sleep(0.1)
            continue

        if stop_condition_func(distances):
            print("\nStop condition met. Exiting follow_corridor.")
            master.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(
                0, master.target_system, master.target_component, mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
                VELOCITY_CONTROL_YAW_RATE_BITMASK, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0))
            return True

        front_dist, left_dist, right_dist = distances['front'], distances['left'], distances['right']

        # --- BUG FIX & ROBUSTNESS IMPROVEMENT: Heading-based turn logic ---
        if fwd_speed > 0 and front_dist < TURN_THRESHOLD and not is_turning:
            turn_direction = None
            if left_dist > WALL_GONE_THRESHOLD: turn_direction = "LEFT"
            elif right_dist > WALL_GONE_THRESHOLD: turn_direction = "RIGHT"
            
            if turn_direction:
                print(f"\nObstacle detected. Aligning before {turn_direction} turn.")
                master.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(
                    0, master.target_system, master.target_component, mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
                    VELOCITY_CONTROL_YAW_RATE_BITMASK, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0))
                time.sleep(1.0)

                is_turning = True
                turn_angle_deg = -90.0 if turn_direction == "LEFT" else 90.0
                if turn_drone_by_heading(master, turn_angle_deg):
                    has_turned_corner = True
                else:
                    print("Turn failed! Aborting corridor follow.")
                    return False
                is_turning = False
                continue

        # Wall Following Logic
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

def fly_straight(master, lidar_manager, fwd_speed, stop_condition_func, target_altitude=None):
    """Flies the drone straight using lidar for guidance, with optional altitude control."""
    print(f"--- Entering fly_straight (Speed: {fwd_speed} m/s, Target Alt: {target_altitude or 'N/A'}) ---")
    TIMEOUT = 30
    start_time = time.time()
    last_known_alt = TAKEOFF_ALTITUDE

    # BUG FIX: Correctly check 'front' when moving forward, 'back' when reversing.
    check_sensor = 'front' if fwd_speed >= 0 else 'back'
    print(f"Waiting for initial reading from '{check_sensor}' lidar...")
    wait_start_time = time.time()
    while True:
        dist = lidar_manager.get_distances().get(check_sensor)
        if dist is not None and dist < 50.0:
            print(f"Initial lidar reading received: {dist:.2f}m")
            break
        if time.time() - wait_start_time > 5:
            print("Error: Timed out waiting for initial lidar data.")
            return False
        time.sleep(0.1)

    while time.time() - start_time < TIMEOUT:
        distances = lidar_manager.get_distances()
        
        # Determine current distance based on direction
        current_dist = distances.get('front') if fwd_speed >= 0 else distances.get('back')

        # Safety hover on lost lidar data
        if current_dist is None or current_dist > 50.0:
            print(f"\rWarning: Lost {check_sensor} lidar, hovering for safety...", end="")
            master.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(
                0, master.target_system, master.target_component, mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
                VELOCITY_CONTROL_YAW_RATE_BITMASK, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0))
            time.sleep(0.1)
            continue
        
        if stop_condition_func(distances):
            print("\nStop condition met. Exiting fly_straight.")
            master.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(
                0, master.target_system, master.target_component, mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
                VELOCITY_CONTROL_YAW_RATE_BITMASK, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0))
            return True

        # Altitude control logic
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

        master.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(
            0, master.target_system, master.target_component, mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
            VELOCITY_CONTROL_YAW_RATE_BITMASK, 0, 0, 0, fwd_speed, 0, down_vel, 0, 0, 0, 0, 0))
        
        sys.stdout.write(f"\rFlying straight... {check_sensor.capitalize()}: {current_dist:.2f}m, Alt: {last_known_alt:.2f}m")
        sys.stdout.flush()
        time.sleep(0.05)

    print("\nfly_straight timed out!")
    return False

# ==============================================================================
# Main Mission Execution (Corrected Logic)
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
        
        if not arm_and_takeoff(master, TAKEOFF_ALTITUDE): raise Exception("Failed initial takeoff")
        
        # 1. Approach first logistic
        print("\n--- 1. Approaching first logistic ---")
        stop_approach = lambda dists: dists.get('front', 999) <= 4.7
        if not follow_corridor(master, lidar_manager, CORRIDOR_FWD_SPEED, stop_approach): raise Exception("Failed to approach first logistic")

        # 2. Land and pick up logistic
        print("\n--- 2. Landing to pick up first logistic ---")
        if not execute_precision_landing(master, data_sock, 0): raise Exception("Precision land for first logistic failed")
        
        # 3. Navigate to drop-off point
        print("\n--- 3. Taking off and navigating to drop-off point ---")
        if not arm_and_takeoff(master, TAKEOFF_ALTITUDE): raise Exception("Failed takeoff after first pickup")
        
        
        print("--- Navigating to corridor exit ---")
        has_turned_corner = False
        stop_at_exit = lambda dists: has_turned_corner and (3.8 <= dists.get('back', 0) < 12.0)
        if not follow_corridor(master, lidar_manager, CORRIDOR_FWD_SPEED, stop_at_exit): 
            raise Exception("Failed to navigate to corrdior exit")
        
        print("\n--- Corridor exited. Flying straight to drop-off point. ---")
        stop_at_dropoff = lambda dists: 6.5 <= dists.get('back', 0) < 12.0
        if not fly_straight(master, lidar_manager, CORRIDOR_FWD_SPEED, stop_at_dropoff, target_altitude=0.8):
            raise Exception("Failed to fly to drop-off point in open area")

        # 4. Center and drop logistic
        print("\n--- 4. Centering to drop first logistic ---")
        if not center_above_target(master, data_sock, 1): raise Exception("Centering for first drop-off failed")
        
        # 5. Return to corner for second logistic
        print("\n--- 5. Returning to corner for second logistic ---")
        print("--- Flying straight to re-enter corridor ---")
        stop_at_entrance = lambda dists: dists.get('left', 999) < 2.0 and dists.get('right', 999) < 2.0
        if not fly_straight(master, lidar_manager, -CORRIDOR_FWD_SPEED, stop_at_entrance):
            raise Exception("Failed to re-enter corridor")
        
        print("\n--- Corridor entered. Navigating backward to corner ---")
        stop_at_corner = lambda dists: dists.get('back', 999) <= 2.5
        if not follow_corridor(master, lidar_manager, -CORRIDOR_FWD_SPEED, stop_at_corner): 
            raise Exception("Failed to return to before corner")
        
        # Corrected this logic block
        print("\n--- Aligning for second pickup ---")
        stop_near_pickup = lambda dists: dists.get('front', 999) <= 1.0 
        if not fly_straight(master, lidar_manager, CORRIDOR_FWD_SPEED, stop_near_pickup):
            raise Exception("Failed to fly to corner for second pickup")
        
        # 6. Land and pick up second logistic
        print("\n--- 6. Landing to pick up second logistic ---")
        if not execute_precision_landing(master, data_sock, 0): raise Exception("Precision land for second logistic failed")
            
        # 7. Navigate to drop-off point again
        print("\n--- 7. Navigating to drop-off point again ---")
        if not arm_and_takeoff(master, TAKEOFF_ALTITUDE): raise Exception("Failed takeoff after second pickup")

        get_out_from_corner = lambda dists: dists.get('back', 0) >= 2.5
        if not fly_straight(master, lidar_manager, CORRIDOR_FWD_SPEED, get_out_from_corner):
            raise Exception("Failed to fly out from corner area")

        print("\n--- Navigating to corridor exit (second trip) ---")
        # Re-use stop_at_exit from before
        if not follow_corridor(master, lidar_manager, CORRIDOR_FWD_SPEED, stop_at_exit): 
            raise Exception("Failed to navigate to corridor exit (second trip)")
            
        print("\n--- Corridor exited. Flying straight to drop-off point (second trip) ---")
        # Re-use stop_at_dropoff from before
        if not fly_straight(master, lidar_manager, CORRIDOR_FWD_SPEED, stop_at_dropoff, target_altitude=0.8):
            raise Exception("Failed to fly to drop-off point (second trip)")
            
        # 8. Center and drop second logistic
        print("\n--- 8. Centering to drop second logistic ---")
        if not center_above_target(master, data_sock, 1): raise Exception("Centering for second drop-off failed")

        # 9. Move to final landing spot
        print("\n--- 9. Moving to final landing spot ---")
        stop_final = lambda dists: dists.get('front', 999) <= 2.0
        if not follow_corridor(master, lidar_manager, 0.3, stop_final): raise Exception("Failed final approach")
        
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