import socket
import json
import time
import math
from pymavlink import mavutil
import sys
import threading
import serial
import servo_control # servo_control.py
import winch_control

# ==============================================================================
# --- Configuration Constants ---
# ==============================================================================

# --- Drone & Mission Configuration ---
CONNECTION_STRING = 'udp:127.0.0.1:14550'
BAUD_RATE = 921600
TAKEOFF_ALTITUDE = 1.1  # meters
ARMING_RETRIES = 3      # Number of times to attempt arming
ARMING_RETRY_DELAY = 3  # Seconds to wait between arming attempts
WAYPOINT_RADIUS = 0.5   # meters
TRACKING_SPEED = 0.4    # m/s
CENTERING_SPEED = 0.250   # m/s, horizontal speed for fine-tuning position
FWD_GAIN = 0.6
BLIND_FORWARD_SPEED = 0.2

# --- Vision & Landing Configuration ---
LANDING_VERTICAL_RATIO = 0.5
CENTERING_VERTICAL_RATIO = 0.5
CENTERING_SUCCESS_THRESHOLD = 0.1
LANDING_D_GAIN = 0.15
CENTERING_D_GAIN = 0.2
LANDING_APPROACH_ALT = 0.45
LANDING_TIMEOUT = 30
FORCED_LAND_ALTITUDE = 0.4
MAX_DOWN_VEL = 0.10  # m/s, max descent speed when perfectly centered
MIN_DOWN_VEL = 0.03  # m/s, min descent speed when error is high
CENTERING_ERROR_THRESHOLD = 0.50 # Normalized error (50%) at which descent speed hits minimum.
WINCH_LOWER_DURATION = 3.0      # Time to lower magnet onto the logistic
WINCH_LIFT_DURATION = 1.0       # Time to lift logistic slightly off the ground
WINCH_PULL_UP_DURATION = 4.0    # Time to pull magnet through the hole for release
CENTERING_TIMEOUT = 15
CENTERING_TARGET_AREA_RATIO = 0.05
CENTERING_CONFIRMATION_DURATION = 0.5
FINAL_VERIFICATION_TIMEOUT = 1.5  # Seconds to wait for a final confirming frame
FINAL_VERIFICATION_THRESHOLD = 0.15 # A tighter centering tolerance (15%) for the final check
TARGET_LOST_HOVER_DURATION = 2.5
REACQUIRE_ASCEND_SPEED = 0.3

# --- Window Approach Configuration ---
ALTITUDE_CHECK_TOLERANCE = 0.3 # meters. Fails if GPS alt is off by more than this.

# --- Lidar & Corridor Navigation Configuration ---
# NEW: Arduino serial port configuration
ARDUINO_LIDAR_PORT = '/dev/ttyUSB0'
ARDUINO_LIDAR_BAUDRATE = 115200
CORRIDOR_FWD_SPEED = 0.4 # m/s
SIDEWAYS_SPEED = 0.35    # m/s, speed for strafing left/right

# --- Dynamic Gain for Vision Control ---
GAIN_MAX_ALT = 1
GAIN_MIN_ALT = 0.4
MAX_HORIZONTAL_GAIN = 0.8
MIN_HORIZONTAL_GAIN = 0.4

# --- UDP Network Configuration ---
UDP_RECEIVE_IP = "127.0.0.2"
UDP_RECEIVE_PORT = 5005
CONTROL_SERVER_IP = "127.0.0.2"
CONTROL_SERVER_PORT = 5006

# --- MAVLink & ArduPilot Constants ---
GUIDED_MODE = 4
VELOCITY_CONTROL_BITMASK = 0b0000111111000111
VELOCITY_CONTROL_YAW_RATE_BITMASK = 0b001111000111 & ~0b100000000000
POSITION_CONTROL_BITMASK = 0b110111111000

# --- Mission Waypoints (for GPS phase) ---
WAYPOINTS = [
    (-7.8332912, 110.3842767, 1.1), # 0: Logistic 1 (Handled by LiDAR)
    (-7.8333110, 110.3842802, 1.1), # 1: Logistic 2 (Handled by LiDAR)
    (-7.8333401, 110.3842762, 1.1), # 2: Barrel Drop (Handled by LiDAR)
    (-7.8333536, 110.3842784, 1.6), # 3: Before Exit
    (-7.8333700, 110.3842700, 1.6), # 4: After Exit
    (-7.8333800, 110.3842600, 1.1), # 5: Outdoor Drop 1 (Placeholder coords)
    (-7.8333900, 110.3842500, 1.1), # 6: Outdoor Drop 2 (Placeholder coords)
    (-7.8334000, 110.3842400, 1.1), # 7: Final Land (Placeholder coords)
]

# --- Global Socket ---
data_sock = None
lidar_manager = None

# ==============================================================================
# --- Arduino Lidar Reader Class ---
# ==============================================================================
class ArduinoLidarReader:
    """
    Manages reading and parsing LiDAR data from an Arduino over a single
    serial port. Expects data in 'front,left,right' format.
    """
    def __init__(self, port, baudrate):
        self.port = port
        self.baudrate = baudrate
        self.lidar_data = {'front': None, 'left': None, 'right': None}
        self.data_lock = threading.Lock()
        self.stop_event = threading.Event()
        self.thread = None
        print(f"ArduinoLidarReader initialized for port {self.port} at {self.baudrate} baud.")

    def _read_thread(self):
        """Worker thread that connects to, reads from, and parses Arduino data."""
        while not self.stop_event.is_set():
            ser = None
            try:
                ser = serial.Serial(self.port, self.baudrate, timeout=1)
                print(f"Successfully connected to Arduino on {self.port}")
                while not self.stop_event.is_set():
                    if ser.in_waiting > 0:
                        try:
                            line = ser.readline().decode('utf-8').strip()
                            parts = line.split(',')
                            if len(parts) == 3:
                                # Arduino sends distance in cm, convert to meters
                                front_dist = int(parts[0]) / 100.0
                                left_dist = int(parts[1]) / 100.0
                                right_dist = int(parts[2]) / 100.0
                                with self.data_lock:
                                    self.lidar_data['front'] = front_dist
                                    self.lidar_data['left'] = left_dist
                                    self.lidar_data['right'] = right_dist
                        except (ValueError, IndexError, UnicodeDecodeError):
                            # Ignore corrupted or incomplete lines
                            print(f"Warning: Could not parse LiDAR data line: '{line}'")
                            pass
            except serial.SerialException:
                print(f"Error: Could not connect to Arduino on {self.port}. Retrying in 5 seconds...")
                time.sleep(5)
            finally:
                if ser and ser.is_open:
                    ser.close()

    def start(self):
        """Starts the reader thread."""
        print("Starting ArduinoLidarReader thread...")
        if self.thread is None:
            self.thread = threading.Thread(target=self._read_thread)
            self.thread.daemon = True
            self.thread.start()
            print("ArduinoLidarReader thread started.")
            time.sleep(2) # Give a moment for the connection to establish

    def stop(self):
        """Stops the reader thread."""
        if self.thread and self.thread.is_alive():
            print("Stopping ArduinoLidarReader thread...")
            self.stop_event.set()
            self.thread.join()
            print("ArduinoLidarReader thread stopped.")

    def get_distances(self):
        """Returns a copy of the latest LiDAR data."""
        with self.data_lock:
            return self.lidar_data.copy()

# ==============================================================================
# --- Core Drone Control & Utility Functions ---
# ==============================================================================
class VelocityController:
    """A Proportional-Derivative (PD) controller to calculate smooth drone velocities."""
    def __init__(self, p_gain, d_gain):
        self.p_gain = p_gain
        self.d_gain = d_gain
        self.prev_x_error = 0.0
        self.prev_y_error = 0.0
        self.current_x_error = 0.0
        self.current_y_error = 0.0
        print(f"PD VelocityController initialized with P={self.p_gain}, D={self.d_gain}")

    def calculate_pd_velocities(self, x_center, y_center, frame_w, frame_h, vertical_ratio):
        corrected_x_center = frame_w - x_center
        self.current_x_error = (corrected_x_center - frame_w / 2) / (frame_w / 2)
        target_y = frame_h * vertical_ratio
        self.current_y_error = (y_center - target_y) / (frame_h / 2)
        x_derivative = self.current_x_error - self.prev_x_error
        y_derivative = self.current_y_error - self.prev_y_error
        self.prev_x_error = self.current_x_error
        self.prev_y_error = self.current_y_error
        x_output = (self.p_gain * self.current_x_error) + (self.d_gain * x_derivative)
        y_output = (self.p_gain * self.current_y_error) + (self.d_gain * y_derivative)
        right_vel = x_output
        forward_vel = -y_output * FWD_GAIN
        if abs(right_vel) < 0.02: right_vel = 0
        if abs(forward_vel) < 0.02: forward_vel = 0
        return forward_vel, right_vel

def send_control_command(command):
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
        if current_altitude >= altitude * 0.75:
            print("\nTarget altitude reached.")
            return True
        time.sleep(0.1)

def land_normally(master):
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
        print(f"\rDistance to target: {distance:.1f}m", end="")
        if distance <= WAYPOINT_RADIUS:
            print("\nWaypoint reached!")
            break
        time.sleep(0.1)
    time.sleep(2)

def flush_socket_buffer(sock):
    print("Flushing UDP socket buffer...")
    packets_cleared = 0
    start_time = time.time()
    while True:
        try:
            sock.recvfrom(1024)
            packets_cleared += 1
            sys.stdout.write(f"\rCleared {packets_cleared} old packets...")
            sys.stdout.flush()
        except socket.timeout:
            break
        except Exception as e:
            print(f"\nAn unexpected error occurred while flushing buffer: {e}")
            break
    if packets_cleared > 0:
        duration = time.time() - start_time
        print(f"\nFinished flushing. Cleared a total of {packets_cleared} packets in {duration:.2f} seconds.")
    else:
        print("Buffer was already clear.")

def verify_final_position(sock, target_class_ids, frame_w, frame_h):
    print("Performing final position verification...")
    verification_start_time = time.time()
    if not isinstance(target_class_ids, list):
        target_class_ids = [target_class_ids]
    flush_socket_buffer(sock)
    while time.time() - verification_start_time < FINAL_VERIFICATION_TIMEOUT:
        try:
            data, _ = sock.recvfrom(1024)
            detection = json.loads(data.decode())
            if detection.get("class_id") not in target_class_ids:
                continue
            x = detection["x_center"]
            final_error_ratio = abs(x - frame_w / 2) / (frame_w / 2)
            if final_error_ratio < FINAL_VERIFICATION_THRESHOLD:
                print(f"Verification PASSED. Final error: {final_error_ratio:.1%}")
                return True
            else:
                print(f"Verification FAILED. Target drifted. Final error: {final_error_ratio:.1%}")
                return False
        except (socket.timeout, json.JSONDecodeError, KeyError):
            pass
    print("Verification FAILED. Timed out waiting for a confirming frame.")
    return False

def center_above_target(master, sock, target_alt, drop_type='dual'):
    flush_socket_buffer(sock)
    send_control_command(f"set_ratio:{CENTERING_VERTICAL_RATIO}")
    time.sleep(0.1)
    send_control_command('resume')
    print(f"Centering for '{drop_type}' drop (Accepting IDs: [0, 1]) at {target_alt:.2f}m...")

    controller = VelocityController(p_gain=CENTERING_SPEED, d_gain=CENTERING_D_GAIN)
    start_time = time.time()
    last_detection_time = time.time()
    last_known_alt = target_alt
    centered_confirmation_start = None

    while time.time() - start_time < CENTERING_TIMEOUT:
        alt_msg = master.recv_match(type='DISTANCE_SENSOR', blocking=False, timeout=0.05)
        if alt_msg and alt_msg.orientation == 25:
            last_known_alt = alt_msg.current_distance / 100.0
        current_altitude = last_known_alt

        fwd_vel, right_vel, down_vel = 0, 0, 0

        try:
            data, _ = sock.recvfrom(1024)
            detection = json.loads(data.decode())
            detected_id = detection.get("class_id")
            if detection.get("state") != "TRACKING" or detected_id not in [0, 1]:
                raise socket.timeout

            last_detection_time = time.time()
            x, y, w, h = detection["x_center"], detection["y_center"], detection["frame_width"], detection["frame_height"]
            fwd_vel, right_vel = controller.calculate_pd_velocities(x, y, w, h, CENTERING_VERTICAL_RATIO)
            horizontal_gain = get_dynamic_gain(current_altitude)
            fwd_vel *= horizontal_gain
            right_vel *= horizontal_gain
            horizontal_error_ratio = abs(x - w / 2) / (w / 2)
            is_horizontally_centered = horizontal_error_ratio < CENTERING_SUCCESS_THRESHOLD
            target_y = h * CENTERING_VERTICAL_RATIO
            vertical_error_ratio = abs(y - target_y) / (h / 2)
            is_vertically_centered = vertical_error_ratio < CENTERING_SUCCESS_THRESHOLD
            target_pixel_area = w * h * CENTERING_TARGET_AREA_RATIO
            is_close_enough = detection["area"] >= target_pixel_area
            area_progress_ratio = min(detection["area"] / target_pixel_area, 1.0)
            print(f"\rCENTERING (ID {detected_id}): Alt: {current_altitude:.2f}m, H-Err: {horizontal_error_ratio:.1%}, V-Err: {vertical_error_ratio:.1%}, Area: {area_progress_ratio:.1%}", end="")

            if is_horizontally_centered and is_vertically_centered and is_close_enough:
                if centered_confirmation_start is None:
                    centered_confirmation_start = time.time()
                if time.time() - centered_confirmation_start > CENTERING_CONFIRMATION_DURATION:
                    print("\nTarget fully centered. Stabilizing...")
                    master.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(0, master.target_system, master.target_component, mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED, VELOCITY_CONTROL_BITMASK, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0))
                    time.sleep(1)
                    send_control_command("pause")
                    if verify_final_position(sock, [0, 1], w, h):
                        print(f"Drone stable. Commencing '{drop_type}' drop.")
                        if drop_type == 'dual':
                            servo_control.open_gripper()
                            print("Package 2 (gripper) dropped.")
                            winch_control.raise_winch(WINCH_PULL_UP_DURATION)
                            print("Package 1 (winch) dropped.")
                        elif drop_type == 'winch':
                             winch_control.raise_winch(WINCH_PULL_UP_DURATION)
                             print("Package 1 (winch) dropped.")
                        elif drop_type == 'gripper':
                             servo_control.open_gripper()
                             print("Package 2 (gripper) dropped.")
                        print("Drop sequence complete.")
                        send_control_command('pause')
                        return True
                    else:
                        print("Resuming centering...")
                        send_control_command('resume')
                        centered_confirmation_start = None
        except (socket.timeout, json.JSONDecodeError, KeyError):
            time_since_lost = time.time() - last_detection_time
            fwd_vel, right_vel = 0, 0
            if time_since_lost < TARGET_LOST_HOVER_DURATION:
                print(f"\rTarget lost. Hovering... ({time_since_lost:.1f}s)", end="")
            else:
                print(f"\rSearching for drop-off target...", end="")
        master.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(0, master.target_system, master.target_component, mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED, VELOCITY_CONTROL_BITMASK, 0, 0, 0, fwd_vel, right_vel, down_vel, 0, 0, 0, 0, 0))
        time.sleep(0.05)
    print("\nCentering timeout reached.")
    send_control_command('pause')
    master.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(0, master.target_system, master.target_component, mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED, VELOCITY_CONTROL_BITMASK, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0))
    return False

def get_dynamic_gain(current_alt):
    if current_alt is None: return MIN_HORIZONTAL_GAIN
    if current_alt >= GAIN_MAX_ALT: return MAX_HORIZONTAL_GAIN
    if current_alt <= GAIN_MIN_ALT: return MIN_HORIZONTAL_GAIN
    gain_range = MAX_HORIZONTAL_GAIN - MIN_HORIZONTAL_GAIN
    alt_range = GAIN_MAX_ALT - GAIN_MIN_ALT
    gain = MIN_HORIZONTAL_GAIN + gain_range * ((current_alt - GAIN_MIN_ALT) / alt_range)
    return gain

def execute_precision_landing(master, sock, pickup_type):
    print("Opening lower aperture to prepare for landing over the package...")
    servo_control.open_gripper()
    time.sleep(1.0)
    flush_socket_buffer(sock)
    send_control_command(f"set_ratio:{LANDING_VERTICAL_RATIO}")
    time.sleep(0.1)
    send_control_command('resume')
    print(f"Starting precision landing for '{pickup_type}' pickup (Accepting IDs: [0, 1])...")
    pd_controller = VelocityController(p_gain=TRACKING_SPEED, d_gain=LANDING_D_GAIN)
    search_start_time = time.time()
    last_detection_time = time.time()
    last_known_alt = TAKEOFF_ALTITUDE
    def perform_pickup_action():
        print("Landed. Closing lower aperture to secure/center package.")
        servo_control.close_gripper()
        time.sleep(1.5)
        if pickup_type == 'winch':
            print("Activating winch to attach package.")
            winch_control.lower_winch(WINCH_LOWER_DURATION)
            time.sleep(1.0)
            winch_control.raise_winch(WINCH_LIFT_DURATION)
            print("Winch pickup complete.")
        elif pickup_type == 'gripper':
            print("Package secured by aperture.")
            time.sleep(1.0)
        return True
    while time.time() - search_start_time < LANDING_TIMEOUT:
        alt_msg = master.recv_match(type='DISTANCE_SENSOR', blocking=False, timeout=0.05)
        if alt_msg and alt_msg.orientation == 25:
            last_known_alt = alt_msg.current_distance / 100.0
        current_altitude = last_known_alt
        if current_altitude < FORCED_LAND_ALTITUDE:
            print(f"\nAltitude is below {FORCED_LAND_ALTITUDE}m. Forcing immediate landing.")
            land_normally(master)
            return perform_pickup_action()
        try:
            data, _ = sock.recvfrom(1024)
            detection = json.loads(data.decode())
            detected_id = detection.get("class_id")
            if detection.get("state") != "TRACKING" or detected_id not in [0, 1]:
                raise socket.timeout
            last_detection_time = time.time()
            x, y, w, h = detection["x_center"], detection["y_center"], detection["frame_width"], detection["frame_height"]
            fwd_vel, right_vel = pd_controller.calculate_pd_velocities(x, y, w, h, LANDING_VERTICAL_RATIO)
            horizontal_gain = get_dynamic_gain(current_altitude)
            fwd_vel *= horizontal_gain
            right_vel *= horizontal_gain
            center_error_ratio = abs(x - w / 2) / (w / 2)
            error_scale = min(center_error_ratio / CENTERING_ERROR_THRESHOLD, 1.0)
            vel_range = MAX_DOWN_VEL - MIN_DOWN_VEL
            dynamic_down_vel = MAX_DOWN_VEL - (error_scale * vel_range)
            master.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(0, master.target_system, master.target_component, mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED, VELOCITY_CONTROL_BITMASK, 0, 0, 0, fwd_vel, right_vel, dynamic_down_vel, 0, 0, 0, 0, 0))
            print(f"\rLANDING (ID {detected_id}): Alt: {current_altitude:.2f}m, Gain: {horizontal_gain:.2f}, Err: {center_error_ratio:.2%}, Vz: {dynamic_down_vel:.2f} m/s", end="")
            if current_altitude < LANDING_APPROACH_ALT and center_error_ratio < 0.05:
                print("\nTarget centered at low altitude. Switching to LAND mode.")
                land_normally(master)
                return perform_pickup_action()
        except (socket.timeout, json.JSONDecodeError, KeyError):
            time_since_lost = time.time() - last_detection_time
            vx, vy, vz = 0, 0, 0
            if time_since_lost < TARGET_LOST_HOVER_DURATION:
                print(f"\rTarget lost. Hovering... (Time since last seen: {time_since_lost:.1f}s)", end="")
            else:
                vz = -REACQUIRE_ASCEND_SPEED
                vx = BLIND_FORWARD_SPEED
                print(f"\rSearching... Ascending and moving forward.", end="")
            master.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(0, master.target_system, master.target_component, mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED, VELOCITY_CONTROL_BITMASK, 0, 0, 0, vx, vy, vz, 0, 0, 0, 0, 0))
    print("\nLanding timeout reached. Aborting and hovering.")
    master.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(0, master.target_system, master.target_component, mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED, VELOCITY_CONTROL_BITMASK, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0))
    return False

def handle_window_approach(master, target_altitude):
    print("\n--- WINDOW APPROACH ---")
    print("Switching EKF to Source Set 2 (GPS for altitude)...")
    master.mav.command_long_send(master.target_system, master.target_component, mavutil.mavlink.MAV_CMD_DO_AUX_FUNCTION, 0, 90, 1, 0, 0, 0, 0, 0)
    time.sleep(1)
    print("Verifying altitude from new source...")
    msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=3)
    if not msg:
        print("Error: Did not receive altitude data after EKF switch.")
        return False
    current_gps_alt = msg.relative_alt / 1000.0
    altitude_error = abs(current_gps_alt - target_altitude)
    print(f"Target Alt: {target_altitude:.2f}m | GPS Alt: {current_gps_alt:.2f}m | Error: {altitude_error:.2f}m")
    if altitude_error > ALTITUDE_CHECK_TOLERANCE:
        print(f"Altitude check FAILED. Error exceeds tolerance.")
        return False
    else:
        print("Altitude check PASSED.")
        return True

# ==============================================================================
# --- LiDAR Navigation Functions ---
# ==============================================================================
def strafe_until_distance(master, lidar_manager, direction, target_distance):
    """
    Strafes left or right while maintaining distance from a forward wall,
    stopping when the side LiDAR reaches a target distance.
    """
    print(f"--- Strafing {direction} until side distance is > {target_distance}m ---")
    TIMEOUT = 20
    WALL_FOLLOW_DIST = 1.5  # Maintain 1.5m from the wall in front
    CENTERING_GAIN = 0.8
    start_time = time.time()
    sideways_vel = SIDEWAYS_SPEED if direction == 'right' else -SIDEWAYS_SPEED
    
    while time.time() - start_time < TIMEOUT:
        distances = lidar_manager.get_distances()
        front_dist = distances.get('front')
        side_dist = distances.get(direction)

        if front_dist is None or side_dist is None:
            print("\rWaiting for LiDAR data...", end="")
            master.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(0, master.target_system, master.target_component, mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED, VELOCITY_CONTROL_BITMASK, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0))
            time.sleep(0.1)
            continue
        
        if side_dist >= target_distance:
            print(f"\nReached target side distance of {side_dist:.2f}m. Stopping strafe.")
            master.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(0, master.target_system, master.target_component, mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED, VELOCITY_CONTROL_BITMASK, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0))
            return True

        # P-controller to maintain distance from the front wall
        error = WALL_FOLLOW_DIST - front_dist
        fwd_vel = -CENTERING_GAIN * error
        
        sys.stdout.write(f"\rStrafing {direction}... Side Dist: {side_dist:.2f}m, Fwd Vel: {fwd_vel:.2f}m/s")
        sys.stdout.flush()
        
        master.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(
            0, master.target_system, master.target_component, mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
            VELOCITY_CONTROL_BITMASK, 0, 0, 0, fwd_vel, sideways_vel, 0, 0, 0, 0, 0, 0))
        time.sleep(0.05)

    print("\nTimeout reached while strafing to target distance.")
    return False


def get_current_heading(master):
    msg = master.recv_match(type='ATTITUDE', blocking=True, timeout=1)
    return msg.yaw if msg else None

def get_angle_difference(angle1_rad, angle2_rad):
    diff = angle2_rad - angle1_rad
    while diff <= -math.pi: diff += 2 * math.pi
    while diff > math.pi: diff -= 2 * math.pi
    return diff

def turn_drone_by_heading(master, angle_degrees, yaw_rate_rads=math.radians(30), tolerance_rad=0.1):
    print(f"\n--- Executing {angle_degrees}° turn ---")
    start_heading_rad = get_current_heading(master)
    if start_heading_rad is None:
        print("Error: Could not get initial heading.")
        return False
    target_heading_rad = start_heading_rad + math.radians(angle_degrees)
    target_heading_rad = (target_heading_rad + math.pi) % (2 * math.pi) - math.pi
    turn_yaw_rate = math.copysign(yaw_rate_rads, angle_degrees)
    start_time = time.time()
    TURN_TIMEOUT = 15
    while time.time() - start_time < TURN_TIMEOUT:
        current_heading_rad = get_current_heading(master)
        if current_heading_rad is None:
            master.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(0, master.target_system, master.target_component, mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED, VELOCITY_CONTROL_YAW_RATE_BITMASK, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0))
            continue
        remaining_angle_rad = get_angle_difference(current_heading_rad, target_heading_rad)
        sys.stdout.write(f"\rTurning... Remaining: {math.degrees(remaining_angle_rad):.1f}°")
        sys.stdout.flush()
        if abs(remaining_angle_rad) < tolerance_rad:
            print("\nTurn complete.")
            master.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(0, master.target_system, master.target_component, mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED, VELOCITY_CONTROL_YAW_RATE_BITMASK, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0))
            time.sleep(0.5)
            return True
        master.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(0, master.target_system, master.target_component, mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED, VELOCITY_CONTROL_YAW_RATE_BITMASK, 0, 0, 0, 0, 0, 0, 0, 0, 0, turn_yaw_rate, 0))
        time.sleep(0.05)
    print("\nError: Turn timed out.")
    return False

def navigate_sideways_to_target(master, lidar_manager, sock, direction, target_class_id):
    print(f"--- Navigating sideways ({direction}) to find target ID {target_class_id} ---")
    flush_socket_buffer(sock)
    send_control_command('resume')
    TIMEOUT = 20
    WALL_FOLLOW_DIST = 1.0
    CENTERING_GAIN = 0.8
    start_time = time.time()
    sideways_vel = SIDEWAYS_SPEED if direction == 'right' else -SIDEWAYS_SPEED
    while time.time() - start_time < TIMEOUT:
        try:
            data, _ = sock.recvfrom(1024)
            detection = json.loads(data.decode())
            if detection.get("state") == "TRACKING" and detection.get("class_id") == target_class_id:
                print(f"\nTarget ID {target_class_id} found! Stopping sideways movement.")
                master.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(0, master.target_system, master.target_component, mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED, VELOCITY_CONTROL_BITMASK, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0))
                return True
        except (socket.timeout, json.JSONDecodeError, KeyError):
            pass
        distances = lidar_manager.get_distances()
        front_dist = distances.get('front')
        if front_dist is None:
            master.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(0, master.target_system, master.target_component, mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED, VELOCITY_CONTROL_BITMASK, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0))
            continue
        error = WALL_FOLLOW_DIST - front_dist
        fwd_vel = -CENTERING_GAIN * error
        sys.stdout.write(f"\rStrafing {direction}... Wall Dist: {front_dist:.2f}m, Fwd Vel: {fwd_vel:.2f} m/s")
        sys.stdout.flush()
        master.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(0, master.target_system, master.target_component, mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED, VELOCITY_CONTROL_BITMASK, 0, 0, 0, fwd_vel, sideways_vel, 0, 0, 0, 0, 0, 0))
        time.sleep(0.05)
    print("\nTimeout reached while searching for target sideways.")
    return False

def follow_corridor(master, lidar_manager, fwd_speed, stop_condition_func):
    print(f"--- Entering follow_corridor (Speed: {fwd_speed} m/s) ---")
    CENTERING_GAIN, TIMEOUT = 1.0, 90
    start_time = time.time()
    wait_start_time = time.time()
    while True:
        distances = lidar_manager.get_distances()
        if all(distances.get(s) is not None for s in ['left', 'right', 'front']):
            break
        if time.time() - wait_start_time > 5:
            print("Error: Timed out waiting for initial lidar data.")
            return False
        time.sleep(0.1)
    while time.time() - start_time < TIMEOUT:
        distances = lidar_manager.get_distances()
        if any(d is None for d in [distances.get('front'), distances.get('left'), distances.get('right')]):
            master.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(0, master.target_system, master.target_component, mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED, VELOCITY_CONTROL_YAW_RATE_BITMASK, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0))
            continue
        if stop_condition_func(distances):
            print("\nStop condition met. Exiting corridor follow.")
            master.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(0, master.target_system, master.target_component, mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED, VELOCITY_CONTROL_YAW_RATE_BITMASK, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0))
            return True
        left_dist, right_dist = distances['left'], distances['right']
        centering_error = left_dist - right_dist if not (left_dist > 5 or right_dist > 5) else 0
        right_vel = -CENTERING_GAIN * centering_error
        sys.stdout.write(f"\rFollowing... Fwd: {fwd_speed:.2f}, Right Vel: {right_vel:.2f}, Front: {distances['front']:.2f}m")
        sys.stdout.flush()
        master.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(0, master.target_system, master.target_component, mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED, VELOCITY_CONTROL_YAW_RATE_BITMASK, 0, 0, 0, fwd_speed, right_vel, 0, 0, 0, 0, 0, 0))
        time.sleep(0.05)
    print("\nCorridor navigation timed out!")
    return False

def fly_straight(master, lidar_manager, fwd_speed, stop_condition_func, target_altitude=None):
    print(f"--- Flying straight (Speed: {fwd_speed} m/s) ---")
    TIMEOUT = 30
    start_time = time.time()
    last_known_alt = -1
    check_sensor = 'front'
    wait_start_time = time.time()
    while True:
        if last_known_alt == -1:
            alt_msg = master.recv_match(type='DISTANCE_SENSOR', blocking=True, timeout=1)
            if alt_msg and alt_msg.orientation == 25:
                last_known_alt = alt_msg.current_distance / 100.0
        dist = lidar_manager.get_distances().get(check_sensor)
        if dist is not None and last_known_alt != -1:
            break
        if time.time() - wait_start_time > 5:
            print("Error: Timed out waiting for initial sensor data.")
            return False
        time.sleep(0.1)
    while time.time() - start_time < TIMEOUT:
        distances = lidar_manager.get_distances()
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
            master.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(0, master.target_system, master.target_component, mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED, VELOCITY_CONTROL_YAW_RATE_BITMASK, 0, 0, 0, 0, 0, down_vel, 0, 0, 0, 0, 0))
            continue
        if stop_condition_func(distances):
            print("\nStop condition met.")
            master.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(0, master.target_system, master.target_component, mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED, VELOCITY_CONTROL_YAW_RATE_BITMASK, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0))
            return True
        master.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(0, master.target_system, master.target_component, mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED, VELOCITY_CONTROL_YAW_RATE_BITMASK, 0, 0, 0, fwd_speed, 0, down_vel, 0, 0, 0, 0, 0))
        sys.stdout.write(f"\rFlying straight... {check_sensor.capitalize()}: {current_dist:.2f}m, Alt: {last_known_alt:.2f}m")
        sys.stdout.flush()
        time.sleep(0.05)
    print("\nfly_straight timed out!")
    return False

# ==============================================================================
# --- Main Mission Execution ---
# ==============================================================================
def main():
    """Main function to run the hybrid LiDAR and GPS mission."""
    global WAYPOINTS, data_sock, lidar_manager
    if len(sys.argv) > 1:
        try:
            print("Received waypoints from GCS command.")
            waypoints_from_gcs = json.loads(sys.argv[1])
            if len(waypoints_from_gcs) == 8:
                WAYPOINTS = [ (wp['lat'], wp['lon'], wp['alt']) for wp in waypoints_from_gcs ]
                print(f"Successfully updated mission with 8 waypoints.")
            else:
                print(f"ERROR: Expected 8 waypoints, got {len(waypoints_from_gcs)}. Using defaults.")
        except Exception as e:
            print(f"ERROR: Could not parse waypoints from GCS: {e}. Using defaults.")
    else:
        print("No waypoints received from GCS. Using default hardcoded waypoints.")

    data_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    data_sock.bind((UDP_RECEIVE_IP, UDP_RECEIVE_PORT))
    data_sock.settimeout(0.5)
    
    lidar_manager = ArduinoLidarReader(port=ARDUINO_LIDAR_PORT, baudrate=ARDUINO_LIDAR_BAUDRATE)
    lidar_manager.start()

    master = mavutil.mavlink_connection(CONNECTION_STRING, baud=BAUD_RATE)
    master.wait_heartbeat()
    print(f"Heartbeat from system (system {master.target_system} component {master.target_component})")

    master.mav.command_long_send(master.target_system, master.target_component, mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0, mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT, 100000, 0, 0, 0, 0, 0)

    try:
        servo_control.setup()
        winch_control.setup()
        servo_control.close_gripper()
        send_control_command('pause')

        if not arm_and_takeoff(master, TAKEOFF_ALTITUDE):
            raise Exception("Failed initial takeoff.")

        # --- PHASE 1: INDOOR LIDAR MISSION (Pickup 1 & 2, Drop at Barrel) ---
        print("\n--- PHASE 1: Acquiring Logistic 1 (LiDAR) ---")
        stop_at_logistic_1 = lambda dists: dists.get('front', 999) <= 4.7
        if not follow_corridor(master, lidar_manager, CORRIDOR_FWD_SPEED, stop_at_logistic_1):
            raise Exception("Failed to reach Logistic 1.")
        
        print("\nReached Logistic 1 location. Starting precision landing.")
        if not execute_precision_landing(master, sock=data_sock, pickup_type='winch'):
            raise Exception("Failed to land at Logistic 1.")
        
        if not arm_and_takeoff(master, TAKEOFF_ALTITUDE):
            raise Exception("Failed takeoff after Logistic 1.")

        print("\n--- PHASE 2: Acquiring Logistic 2 (LiDAR) ---")
        stop_at_corner = lambda dists: dists.get('front', 999) <= 1.5
        if not follow_corridor(master, lidar_manager, CORRIDOR_FWD_SPEED, stop_at_corner):
             raise Exception("Failed to reach the corner for Logistic 2.")
        
        print("\nReached corner for Logistic 2. Starting precision landing.")
        if not execute_precision_landing(master, sock=data_sock, pickup_type='gripper'):
             raise Exception("Failed to land at Logistic 2.")
        
        if not arm_and_takeoff(master, TAKEOFF_ALTITUDE):
             raise Exception("Failed takeoff after Logistic 2.")
        servo_control.close_gripper()

        print("\n--- PHASE 3: Dropping at Barrel (LiDAR) ---")
        time.sleep(1.0) # Pause to ensure LiDAR readings are stable
        distances = lidar_manager.get_distances()
        opening_direction = 'left' if distances.get('left', 0) > distances.get('right', 0) else 'right'
        print(f"Corner opening detected to the {opening_direction}.")

        if not strafe_until_distance(master, lidar_manager, opening_direction, 4.0):
            raise Exception("Failed to strafe to barrel drop-off point.")

        if not center_above_target(master, sock=data_sock, target_alt=TAKEOFF_ALTITUDE, drop_type='dual'):
            print("WARNING: Failed to center on barrel, but continuing mission.")

        # --- MISSION SWITCHOVER TO GPS ---
        print("\n--- INDOOR MISSION COMPLETE. SWITCHING TO GPS NAVIGATION. ---")

        # --- PHASE 4: Navigating Exit ---
        print("\n--- PHASE 4: Navigating Exit (GPS) ---")
        navigate_to_waypoint(master, WAYPOINTS[3][0], WAYPOINTS[3][1], WAYPOINTS[3][2])
        if not handle_window_approach(master, WAYPOINTS[3][2]):
            raise Exception("Altitude check failed at window. Aborting.")
        navigate_to_waypoint(master, WAYPOINTS[4][0], WAYPOINTS[4][1], WAYPOINTS[4][2])
        print("\nSwitching EKF back to Source Set 1 (Rangefinder for altitude)...")
        master.mav.command_long_send(master.target_system, master.target_component, mavutil.mavlink.MAV_CMD_DO_AUX_FUNCTION, 0, 90, 0, 0, 0, 0, 0, 0)
        time.sleep(1)

        # --- PHASE 5 & 6: Outdoor Drops ---
        print("\n--- PHASE 5: Outdoor Drop 1 (GPS) ---")
        navigate_to_waypoint(master, WAYPOINTS[5][0], WAYPOINTS[5][1], WAYPOINTS[5][2])
        # This function needs to be defined in servo_control.py
        # servo_control.drop_package_outdoor_1()
        print("Placeholder for Outdoor Drop 1.")

        print("\n--- PHASE 6: Outdoor Drop 2 (GPS) ---")
        navigate_to_waypoint(master, WAYPOINTS[6][0], WAYPOINTS[6][1], WAYPOINTS[6][2])
        # This function needs to be defined in servo_control.py
        # servo_control.drop_package_outdoor_2()
        print("Placeholder for Outdoor Drop 2.")


        # --- FINAL PHASE: Landing ---
        print("\n--- FINAL PHASE: Landing (GPS) ---")
        navigate_to_waypoint(master, WAYPOINTS[7][0], WAYPOINTS[7][1], WAYPOINTS[7][2])
        land_normally(master)

        print("\nFull mission finished successfully!")

    except Exception as e:
        print(f"\nMISSION FAILED: {e}")
        print("Attempting to land at current position...")
        land_normally(master)
    except KeyboardInterrupt:
        print("\nKeyboard interrupt received. Landing immediately...")
        land_normally(master)
        servo_control.open_gripper()
    finally:
        if lidar_manager: lidar_manager.stop()
        servo_control.cleanup()
        winch_control.cleanup()
        if data_sock: data_sock.close()
        print("Resources cleaned up.")

if __name__ == "__main__":
    main()

