import socket
import json
import time
import sys
import threading
import serial
from pymavlink import mavutil
import servo_control  # servo_control.py

# --- Drone & Mission Configuration ---
CONNECTION_STRING = 'udp:127.0.0.1:14550'
BAUD_RATE = 921600
TAKEOFF_ALTITUDE = 1.1  # meters
ARMING_RETRIES = 3
ARMING_RETRY_DELAY = 3

# --- Vision & Landing Configuration ---
WAYPOINT_RADIUS = 0.5   # meters
TRACKING_SPEED = 0.5    # m/s
CENTERING_SPEED = 0.25
FWD_GAIN = 0.8
ALT_GAIN = 0.15
VERTICAL_CENTER_RATIO = 0.25
LANDING_APPROACH_ALT = 0.45
LANDING_TIMEOUT = 15
CENTERING_TIMEOUT = 20
CENTERING_ALTITUDE = 1
TARGET_LOST_HOVER_DURATION = 3.0
REACQUIRE_ASCEND_SPEED = 0.3
GAIN_MAX_ALT = 1.1
GAIN_MIN_ALT = 0.4
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
# Lidar Manager Class
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
        print("Starting LidarManager threads...")
        for port, name in self.port_mapping.items():
            thread = threading.Thread(target=self._lidar_thread_worker, args=(port, name))
            thread.daemon = True
            self.threads.append(thread)
            thread.start()
        print("All lidar threads started.")
        time.sleep(1)

    def stop(self):
        print("Stopping LidarManager threads...")
        self.stop_event.set()

    def get_distances(self):
        with self.data_lock:
            return self.lidar_data.copy()

# ==============================================================================
# Core Drone Control Functions
# ==============================================================================
def send_control_command(command):
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.connect((CONTROL_SERVER_IP, CONTROL_SERVER_PORT))
            s.sendall(command.encode('utf-8'))
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
        if current_altitude >= altitude * 0.80: # Loosen tolerance slightly
            print("\nTarget altitude reached.")
            return True
        time.sleep(0.1)

def land_normally(master):
    print("Executing normal landing...")
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_LAND, 0, 0, 0, 0, 0, 0, 0, 0)
    try:
        master.motors_disarmed_wait()
        print("Landed and disarmed.")
    except Exception as e:
        print(f"Warning: Did not receive disarm confirmation, but landing command was sent. {e}")


def flush_socket_buffer(sock):
    while True:
        try:
            sock.recvfrom(1024)
        except socket.timeout:
            break

def change_altitude(master, target_alt, timeout=10):
    """
    Changes the drone's altitude while hovering in place.
    """
    print(f"\n--- Changing altitude to {target_alt}m ---")
    start_time = time.time()
    last_known_alt = -1 # Initialize to an invalid value

    while time.time() - start_time < timeout:
        # Get current altitude
        alt_msg = master.recv_match(type='DISTANCE_SENSOR', blocking=False, timeout=0.05)
        if alt_msg and alt_msg.orientation == 25: # Downward facing sensor
            current_alt = alt_msg.current_distance / 100.0
            last_known_alt = current_alt
        else:
            # If no new message, use the last known good value
            if last_known_alt == -1:
                 # If we have never gotten a reading, hover and wait
                 print("\rWaiting for initial altitude reading...", end="")
                 master.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(
                    0, master.target_system, master.target_component, mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
                    VELOCITY_CONTROL_YAW_RATE_BITMASK, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0))
                 time.sleep(0.1)
                 continue
            current_alt = last_known_alt

        alt_error = target_alt - current_alt
        sys.stdout.write(f"\rChanging altitude... Current: {current_alt:.2f}m, Target: {target_alt:.2f}m, Error: {alt_error:.2f}m")
        sys.stdout.flush()

        # Check if we've reached the target altitude
        if abs(alt_error) < 0.10: # 10cm tolerance
            print("\nTarget altitude reached.")
            # Send a final hover command to stop vertical movement
            master.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(
                0, master.target_system, master.target_component, mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
                VELOCITY_CONTROL_YAW_RATE_BITMASK, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0))
            time.sleep(0.5) # Let it stabilize
            return True

        # Calculate vertical speed
        down_vel = -ALT_GAIN * alt_error

        # Send hover command with vertical velocity
        master.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(
            0, master.target_system, master.target_component, mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
            VELOCITY_CONTROL_YAW_RATE_BITMASK, 0, 0, 0, 0, 0, down_vel, 0, 0, 0, 0, 0))

        time.sleep(0.05)

    print("\nAltitude change timed out.")
    return False


def flush_socket_buffer(sock):
    while True:
        try:
            sock.recvfrom(1024)
        except socket.timeout:
            break
        
# ==============================================================================
# Vision-Guided Functions
# ==============================================================================
def calculate_velocities(x_center, y_center, frame_w, frame_h, speed): # <-- Added 'speed' parameter
    """Calculates forward and right velocities based on target position and a given speed."""
    corrected_x_center = frame_w - x_center
    x_offset = (corrected_x_center - frame_w / 2) / (frame_w / 2)
    target_y = frame_h * VERTICAL_CENTER_RATIO
    y_offset = (y_center - target_y) / (frame_h / 2)
    # Use the passed 'speed' parameter instead of the global constant
    right_vel = speed * x_offset if abs(x_offset) > 0.1 else 0
    forward_vel = -speed * y_offset * FWD_GAIN if abs(y_offset) > 0.1 else 0
    return forward_vel, right_vel

def get_dynamic_gain(current_alt):
    if current_alt is None: return MIN_HORIZONTAL_GAIN
    if current_alt >= GAIN_MAX_ALT: return MAX_HORIZONTAL_GAIN
    if current_alt <= GAIN_MIN_ALT: return MIN_HORIZONTAL_GAIN
    gain = MIN_HORIZONTAL_GAIN + (MAX_HORIZONTAL_GAIN - MIN_HORIZONTAL_GAIN) * \
           ((current_alt - GAIN_MIN_ALT) / (GAIN_MAX_ALT - GAIN_MIN_ALT))
    return gain

def execute_precision_landing(master, sock, target_class_id):
    flush_socket_buffer(sock)
    send_control_command('resume')
    print(f"Starting precision landing sequence (Accepting IDs: [0, 1])...")
    
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
            if alt_msg and alt_msg.orientation == 25: # Downward facing
                last_known_alt = alt_msg.current_distance / 100.0
            current_altitude = last_known_alt

            data, _ = sock.recvfrom(1024)
            detection = json.loads(data.decode())
            
            # Accept class ID 0 or 1 for landing
            detected_id = detection.get("class_id")
            if detection.get("state") != "TRACKING" or detected_id not in [0, 1]:
                raise socket.timeout()

            last_detection_time = time.time()
            search_start_time = time.time() # Reset timeout on successful detection

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
            center_error_ratio = abs(x - w / 2) / (w/2)
            # Show the actual detected ID in the status message
            print(f"\rLANDING (Detected {detected_id}): Alt: {current_altitude:.2f}m, Gain: {horizontal_gain:.2f}, Err: {center_error_ratio:.2%}", end="")


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
            print(f"\rSearching for landing target (IDs [0, 1])... Time since last seen: {time_since_lost:.1f}s", end="")
            
            vz = 0 
            if time_since_lost > TARGET_LOST_HOVER_DURATION:
                vz = -REACQUIRE_ASCEND_SPEED

            master.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(
                0, master.target_system, master.target_component, mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
                VELOCITY_CONTROL_BITMASK, 0, 0, 0, 0, 0, vz, 0, 0, 0, 0, 0))

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

        fwd_vel, right_vel, down_vel = 0, 0, 0
        
        try:
            data, _ = sock.recvfrom(1024)
            detection = json.loads(data.decode())
            if detection.get("state") == "TRACKING" and detection.get("class_id") == target_class_id:
                x, y, w, h = detection["x_center"], detection["y_center"], detection["frame_width"], detection["frame_height"]
                # MODIFIED: Use the new, slower centering speed for finer adjustments
                fwd_vel, right_vel = calculate_velocities(x, y, w, h, CENTERING_SPEED)

                alt_error = CENTERING_ALTITUDE - current_alt
                down_vel = -ALT_GAIN * alt_error
                
                center_error_ratio = abs(w/2 - (w-x)) / w
                sys.stdout.write(f"\rCentering... Err: {center_error_ratio:.2%}, Alt Err: {alt_error:+.2f}m, Fwd: {fwd_vel:.2f}, Right: {right_vel:.2f}")
                sys.stdout.flush()

                # --- MODIFIED EXIT LOGIC ---
                if center_error_ratio < 0.05 and abs(alt_error) < 0.10:
                    print("\nTarget centered. Halting movement to stabilize...")
                    # 1. Command the drone to hover to stop all movement
                    master.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(
                        0, master.target_system, master.target_component, mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
                        VELOCITY_CONTROL_BITMASK, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0))
                    
                    # 2. Wait for 1 second to allow inertia to dissipate
                    time.sleep(1.0) 
                    
                    # 3. Now that it's stable, open the gripper
                    print("Drone stable. Opening gripper to drop package.")
                    servo_control.open_gripper()
                    time.sleep(1.0) # Wait for package to drop clear
                    send_control_command('pause')
                    return True
        except (socket.timeout, json.JSONDecodeError, KeyError):
            # If target is lost, command a hover
            fwd_vel, right_vel, down_vel = 0, 0, 0
            print("\rSearching for drop-off target...", end="")

        master.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(
            0, master.target_system, master.target_component, mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
            VELOCITY_CONTROL_BITMASK, 0, 0, 0, fwd_vel, right_vel, down_vel, 0, 0, 0, 0, 0))
        time.sleep(0.05)

    print("\nCentering timeout reached.")
    send_control_command('pause')
    return False

# ==============================================================================
# Lidar-Guided Corridor Navigation
# ==============================================================================
def follow_corridor(master, lidar_manager, fwd_speed, stop_condition_func):
    """Follows a corridor until a stop condition is met, handling turns automatically."""
    global has_turned_corner
    print(f"--- Entering follow_corridor (Speed: {fwd_speed} m/s) ---")
    
    CENTERING_GAIN, TURN_THRESHOLD, WALL_GONE_THRESHOLD, TURN_YAW_RATE, TIMEOUT = 1.0, 1.0, 3.0, 0.15, 90
    start_time = time.time()
    is_turning = False

    print("Waiting for initial lidar readings...")
    wait_start_time = time.time()
    while True:
        distances = lidar_manager.get_distances()
        if all(distances.get(sensor) is not None for sensor in ['front', 'left', 'right']):
            print("Initial lidar readings received.")
            break
        if time.time() - wait_start_time > 5:
            print("Error: Timed out waiting for initial lidar data.")
            return False
        time.sleep(0.1)

    while time.time() - start_time < TIMEOUT:
        distances = lidar_manager.get_distances()
        if any(distances.get(sensor) is None for sensor in ['front', 'left', 'right']):
            print("\rWarning: Lost lidar data, hovering...", end="")
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

        front_dist, left_dist, right_dist = distances.get('front'), distances.get('left'), distances.get('right')

        if fwd_speed > 0 and front_dist < TURN_THRESHOLD and not is_turning:
            turn_direction = None
            if left_dist > WALL_GONE_THRESHOLD: 
                turn_direction, yaw_rate = "LEFT", -TURN_YAW_RATE 
            elif right_dist > WALL_GONE_THRESHOLD: 
                turn_direction, yaw_rate = "RIGHT", TURN_YAW_RATE
            
            if turn_direction:
                print(f"\nAligning before {turn_direction} turn.")
                master.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(
                    0, master.target_system, master.target_component, mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
                    VELOCITY_CONTROL_YAW_RATE_BITMASK, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0))
                time.sleep(1.0) 

                print(f"Executing {turn_direction} turn.")
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
            VELOCITY_CONTROL_YAW_RATE_BITMASK,
            0, 0, 0, fwd_speed, right_vel, 0, 0, 0, 0, 0, 0))
        
        time.sleep(0.05)
    
    print("\nCorridor navigation timed out!")
    return False

def fly_straight(master, lidar_manager, fwd_speed, stop_condition_func, target_altitude=None): # <-- ADD target_altitude
    """
    Flies the drone straight, with optional altitude control.
    """
    print(f"--- Entering fly_straight (Speed: {fwd_speed} m/s, Target Alt: {target_altitude or 'N/A'}) ---")
    TIMEOUT = 30
    start_time = time.time()
    last_known_alt = TAKEOFF_ALTITUDE # <-- ADD: Initialize with a safe default

    # --- INITIAL READING LOGIC (No change needed here) ---
    print("Waiting for a valid initial lidar reading...")
    wait_start_time = time.time()
    initial_reading_ok = False
    while not initial_reading_ok:
        distances = lidar_manager.get_distances()
        # Use a relevant sensor based on direction of travel
        check_sensor = 'back' if fwd_speed > 0 else 'front' 
        dist = distances.get(check_sensor)
        if dist is not None and dist < 50.0:
            print(f"Initial lidar reading received: {dist:.2f}m")
            initial_reading_ok = True
            break
        if time.time() - wait_start_time > 5:
            print("Error: Timed out waiting for a valid initial lidar data.")
            return False
        time.sleep(0.1)

    # --- MODIFIED MAIN MOVEMENT LOOP ---
    while time.time() - start_time < TIMEOUT:
        distances = lidar_manager.get_distances()
        front_dist = distances.get('front')

        # --- ALTITUDE CONTROL LOGIC ---
        down_vel = 0 # Default to no vertical movement
        if target_altitude is not None:
            alt_msg = master.recv_match(type='DISTANCE_SENSOR', blocking=False, timeout=0.01)
            if alt_msg and alt_msg.orientation == 25: # Downward facing sensor
                current_alt = alt_msg.current_distance / 100.0
                last_known_alt = current_alt
            else:
                current_alt = last_known_alt # Use last known if no new message
            
            alt_error = target_altitude - current_alt
            # Only apply significant velocity if error is more than 5cm
            if abs(alt_error) > 0.05: 
                down_vel = -ALT_GAIN * alt_error
        
        # --- LIDAR DATA VALIDATION ---
        if front_dist is None or front_dist > 50.0:
            print("\rWarning: Lost or invalid lidar data, continuing with last command...", end="")
            # We still send a command to maintain altitude if needed
            master.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(
                0, master.target_system, master.target_component, mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
                VELOCITY_CONTROL_YAW_RATE_BITMASK, 0, 0, 0, fwd_speed, 0, down_vel, 0, 0, 0, 0, 0))
            time.sleep(0.1)
            continue
        
        # --- STOP CONDITION CHECK ---
        if stop_condition_func(distances):
            print("\nStop condition met. Exiting fly_straight.")
            master.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(
                0, master.target_system, master.target_component, mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
                VELOCITY_CONTROL_YAW_RATE_BITMASK, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0))
            return True

        # --- SEND MOVEMENT COMMAND ---
        master.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(
            0, master.target_system, master.target_component, mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
            VELOCITY_CONTROL_YAW_RATE_BITMASK,
            0, 0, 0, fwd_speed, 0, down_vel, 0, 0, 0, 0, 0)) # <-- Use calculated down_vel
        
        sys.stdout.write(f"\rFlying straight... Front Lidar: {front_dist:.2f}m, Alt: {last_known_alt:.2f}m")
        sys.stdout.flush()
        time.sleep(0.05)

    print("\nfly_straight timed out!")
    return False

# ==============================================================================
# Main Test Execution
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
        send_control_command('pause')
        
        servo_control.close_gripper()
        print("Gripper closed. Starting navigation.")
        # 1. Takeoff and simulate pickup
        print("\n--- 1. Taking off for navigation test ---")
        if not arm_and_takeoff(master, TAKEOFF_ALTITUDE): raise Exception("Failed initial takeoff")
    
        # 2. Navigate to drop-off point
        print("\n--- 2. Navigating to drop-off point ---")
        print("--- Navigating to corridor exit ---")
        has_turned_corner = False
        stop_at_exit = lambda dists: has_turned_corner and (3.8 <= dists.get('back', 0) < 12.0)
        if not follow_corridor(master, lidar_manager, CORRIDOR_FWD_SPEED, stop_at_exit): 
            raise Exception("Failed to navigate to corrdior exit")
        
        # NEW STEP: Change altitude before moving forward
        if not change_altitude(master, 0.8):
            raise Exception("Failed to lower altitude after exiting corridor")
        
        print("\n--- Corridor exited. Flying straight to drop-off point. ---")
        stop_at_dropoff = lambda dists: dists.get('front', 0) <= 4
        if not fly_straight(master, lidar_manager, CORRIDOR_FWD_SPEED, stop_at_dropoff):
            raise Exception("Failed to fly to drop-off point in open area")

        # 3. Center and drop logistic
        print("\n--- 3. Centering to drop logistic ---")
        if not center_above_target(master, data_sock, 1): raise Exception("Centering for drop-off failed")
        
        # 4. Move to final landing spot
        print("\n--- 4. Moving to final landing spot ---")
        stop_final = lambda dists: dists.get('front', 999) <= 2.0
        if not follow_corridor(master, lidar_manager, 0.3, stop_final): raise Exception("Failed final approach")
        
        print("Final landing.")
        land_normally(master)

        print("\nNAVIGATION TEST FINISHED SUCCESSFULLY!")

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