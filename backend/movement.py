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
CENTERING_SPEED = 0.5  # m/s, horizontal speed for fine-tuning position
FWD_GAIN = 0.6
BLIND_FORWARD_SPEED = 0.2

# CAMERA CENTER OFFSET
LANDING_VERTICAL_RATIO = 0.10
CENTERING_VERTICAL_RATIO = 0.25

LANDING_D_GAIN = 0.1
CENTERING_D_GAIN = 0.1

LANDING_APPROACH_ALT = 0.5
LANDING_TIMEOUT = 30
FORCED_LAND_ALTITUDE = 0.5
MAX_DOWN_VEL = 0.20  # m/s, max descent speed when perfectly centered
MIN_DOWN_VEL = 0.05  # m/s, min descent speed when error is high
CENTERING_ERROR_THRESHOLD = 0.50 # Normalized error (50%) at which descent speed hits minimum.

CENTERING_TIMEOUT = 20
CENTERING_TARGET_AREA_RATIO = 0.05
CENTERING_CONFIRMATION_DURATION = 1

TARGET_LOST_HOVER_DURATION = 2.5
REACQUIRE_ASCEND_SPEED = 0.3

# --- NEW: Configuration for Window Approach ---
ALTITUDE_CHECK_TOLERANCE = 0.3 # meters. Fails if GPS alt is off by more than this.

GAIN_MAX_ALT = 1
GAIN_MIN_ALT = 0.4
MAX_HORIZONTAL_GAIN = 0.6
MIN_HORIZONTAL_GAIN = 0.2

# --- UDP Network Configuration ---
UDP_RECEIVE_IP = "127.0.0.2"
UDP_RECEIVE_PORT = 5005
CONTROL_SERVER_IP = "127.0.0.2"
CONTROL_SERVER_PORT = 5006

# --- MAVLink & ArduPilot Constants ---
GUIDED_MODE = 4
VELOCITY_CONTROL_BITMASK = 0b0000111111000111
POSITION_CONTROL_BITMASK = 0b110111111000

# --- UPDATED: Mission Waypoints ---
WAYPOINTS = [
    (-7.8332912, 110.3842767, 1.1), # Waypoint 0 (Precision Land 1)
    (-7.8333110, 110.3842802, 1.1), # Waypoint 1 (Corridor Elbow)
    (-7.8333401, 110.3842762, 1.1), # Waypoint 2 (Center on Barrel)
    (-7.8333536, 110.3842784, 1.6), # Waypoint 3 (Before Window)
    (-7.8333700, 110.3842700, 1.6), # Waypoint 4 (After Window)
    (-7.8333900, 110.3842700, 1.6), # Waypoint 5 (Final Land Original Position)
]

# --- Global Socket ---
data_sock = None

# --- Core Functions ---
class VelocityController:
    """A Proportional-Derivative (PD) controller to calculate smooth drone velocities."""
    def __init__(self, p_gain, d_gain):
        self.p_gain = p_gain  # The Proportional gain (responds to current error)
        self.d_gain = d_gain  # The Derivative gain (responds to rate of change of error)
        
        # Initialize state variables to store the error from the previous cycle
        self.prev_x_error = 0.0
        self.prev_y_error = 0.0
        print(f"PD VelocityController initialized with P={self.p_gain}, D={self.d_gain}")

    def calculate_pd_velocities(self, x_center, y_center, frame_w, frame_h, vertical_ratio):
        """Calculates velocities using PD logic to reduce overshoot."""
        
        # --- 1. Calculate Current Proportional Error ---
        # Normalized error from -1.0 to 1.0 for both axes
        corrected_x_center = frame_w - x_center
        current_x_error = (corrected_x_center - frame_w / 2) / (frame_w / 2)
        
        target_y = frame_h * vertical_ratio
        current_y_error = (y_center - target_y) / (frame_h / 2)

        # --- 2. Calculate Derivative of Error (The Damping Term) ---
        # This measures how fast the error is changing.
        x_derivative = current_x_error - self.prev_x_error
        y_derivative = current_y_error - self.prev_y_error
        
        # --- 3. Update state for the next cycle ---
        # It's important to do this *after* calculating the derivative.
        self.prev_x_error = current_x_error
        self.prev_y_error = current_y_error

        # --- 4. Calculate the combined PD Control Output ---
        # The final output is the sum of the P and D responses.
        x_output = (self.p_gain * current_x_error) + (self.d_gain * x_derivative)
        y_output = (self.p_gain * current_y_error) + (self.d_gain * y_derivative)

        # --- 5. Convert to drone's frame of reference and apply gains ---
        # Note the sign changes to match MAVLink NED frame conventions
        right_vel = x_output
        forward_vel = -y_output * FWD_GAIN
        
        # Apply a deadband to prevent small jitters when near the target
        if abs(right_vel) < 0.02: right_vel = 0
        if abs(forward_vel) < 0.02: forward_vel = 0
            
        return forward_vel, right_vel

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
        if current_altitude >= altitude * 0.8:
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

def calculate_velocities(x_center, y_center, frame_w, frame_h, speed, vertical_ratio):
    """Calculates forward and right velocities based on target position and a given speed."""
    corrected_x_center = frame_w - x_center
    x_offset = (corrected_x_center - frame_w / 2) / (frame_w / 2)

    target_y = frame_h * vertical_ratio
    y_offset = (y_center - target_y) / (frame_h / 2)
    
    right_vel = speed * x_offset if abs(x_offset) > 0.1 else 0
    forward_vel = -speed * y_offset * FWD_GAIN if abs(y_offset) > 0.1 else 0
    return forward_vel, right_vel

def flush_socket_buffer(sock):
    """
    Clears any old data from the UDP socket buffer with real-time feedback.
    """
    print("Flushing UDP socket buffer...")
    packets_cleared = 0
    start_time = time.time()
    while True:
        try:
            # Receive data, but we don't need to do anything with it.
            sock.recvfrom(1024) 
            packets_cleared += 1
            # Update the status on a single line using a carriage return
            sys.stdout.write(f"\rCleared {packets_cleared} old packets...")
            sys.stdout.flush()
        except socket.timeout:
            # This is the expected exit condition: no more data to read.
            break
        except Exception as e:
            print(f"\nAn unexpected error occurred while flushing buffer: {e}")
            break

    # Move to the next line after the loop finishes
    if packets_cleared > 0:
        duration = time.time() - start_time
        print(f"\nFinished flushing. Cleared a total of {packets_cleared} packets in {duration:.2f} seconds.")
    else:
        print("Buffer was already clear.")

# In movement.py

def center_above_target(master, sock, target_class_id, target_alt):
    flush_socket_buffer(sock)
    send_control_command(f"set_ratio:{CENTERING_VERTICAL_RATIO}")
    time.sleep(0.1) 
    send_control_command('resume')
    print(f"Centering above target (ID: {target_class_id}) at {target_alt}m...")
    pd_controller = VelocityController(p_gain=CENTERING_SPEED, d_gain=CENTERING_D_GAIN)

    start_time = time.time()
    
    # --- NEW: State variable for confirmation logic ---
    # This will store the timestamp when the drone first enters the confirmation phase.
    stable_candidate_since = None 
    
    while time.time() - start_time < CENTERING_TIMEOUT:
        alt_msg = master.recv_match(type='DISTANCE_SENSOR', blocking=False, timeout=0.05)
        current_alt = -1
        if alt_msg and alt_msg.orientation == 25:
            current_alt = alt_msg.current_distance / 100.0
            
        # Default to hover unless a target is found and we need to move
        fwd_vel, right_vel, down_vel = 0, 0, 0
        
        try:
            data, _ = sock.recvfrom(1024)
            detection = json.loads(data.decode())
            if detection.get("state") == "TRACKING" and detection.get("class_id") == target_class_id:
                x, y, w, h = detection["x_center"], detection["y_center"], detection["frame_width"], detection["frame_height"]
                current_area = detection["area"]
                
                target_pixel_area = w * h * CENTERING_TARGET_AREA_RATIO
                
                # --- Check if conditions for dropping are met ---
                is_horizontally_centered = abs(x - w / 2) / (w/2) < 0.05
                is_close_enough = current_area >= target_pixel_area

                if is_horizontally_centered and is_close_enough:
                    # --- START of Confirmation Logic ---
                    
                    # If this is the first time we are centered, record the time.
                    if stable_candidate_since is None:
                        stable_candidate_since = time.time()
                    
                    # Calculate how long we have been stable.
                    stable_duration = time.time() - stable_candidate_since
                    
                    # Check if we have been stable long enough.
                    if stable_duration >= CENTERING_CONFIRMATION_DURATION:
                        print(f"\nPosition confirmed for {stable_duration:.1f}s. Stabilizing for drop...")
                        # Command a final hover to ensure stability
                        master.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(
                            0, master.target_system, master.target_component, mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
                            VELOCITY_CONTROL_BITMASK, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0))
                        
                        time.sleep(1.0) # Final stabilization wait
                        
                        print("Drone stable. Opening gripper to drop package.")
                        servo_control.open_gripper()
                        time.sleep(1.0)
                        send_control_command('pause')
                        return True
                    else:
                        # We are centered, but not for long enough. Command a hover and wait.
                        # The fwd_vel and right_vel are already 0, so we just update the display.
                        progress = (stable_duration / CENTERING_CONFIRMATION_DURATION)
                        sys.stdout.write(
                            f"\rConfirming position... {progress:.1%} ({stable_duration:.1f}s / {CENTERING_CONFIRMATION_DURATION:.1f}s)"
                        )
                        sys.stdout.flush()
                        # --- END of Confirmation Logic ---
                else:
                    # --- We are not centered, so reset the confirmation timer ---
                    stable_candidate_since = None 
                    
                    # Calculate velocities to get back to the center
                    fwd_vel, right_vel = pd_controller.calculate_pd_velocities(x, y, w, h, CENTERING_VERTICAL_RATIO)

                    # Update status display for normal centering
                    center_error_ratio = abs(x - w / 2) / (w/2)
                    area_progress_ratio = current_area / target_pixel_area
                    sys.stdout.write(
                        f"\rCentering... Pos Err: {center_error_ratio:.1%}, "
                        f"Area Prog: {min(area_progress_ratio, 1.0):.1%}, "
                        f"Alt: {current_alt:.2f}m"
                    )
                    sys.stdout.flush()
            else:
                # Target is not the correct one, reset confirmation timer
                stable_candidate_since = None
                print("\rSearching for drop-off target (wrong ID detected)...", end="")

        except (socket.timeout, json.JSONDecodeError, KeyError):
            # --- Target lost, so reset the confirmation timer ---
            stable_candidate_since = None
            print("\rSearching for drop-off target (no detection)...", end="")

        # Send the calculated velocity command (either for centering or for hovering during confirmation)
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

def execute_precision_landing(master, sock):
    flush_socket_buffer(sock)
    send_control_command(f"set_ratio:{LANDING_VERTICAL_RATIO}")
    time.sleep(0.1)
    send_control_command('resume')
    print(f"Starting precision landing sequence (Accepting IDs: [0, 1])...")
    pd_controller = VelocityController(p_gain=TRACKING_SPEED, d_gain=LANDING_D_GAIN)
    search_start_time = time.time()
    last_detection_time = time.time()
    last_known_alt = TAKEOFF_ALTITUDE

    while time.time() - search_start_time < LANDING_TIMEOUT:
        alt_msg = master.recv_match(type='DISTANCE_SENSOR', blocking=False, timeout=0.05)
        if alt_msg and alt_msg.orientation == 25:
            last_known_alt = alt_msg.current_distance / 100.0
        current_altitude = last_known_alt
        
        if current_altitude < FORCED_LAND_ALTITUDE:
            print(f"\nAltitude is below {FORCED_LAND_ALTITUDE}m. Forcing immediate landing.")
            land_normally(master)
            time.sleep(1)
            print("Landed. Closing gripper to pick up package.")
            time.sleep(1)
            servo_control.close_gripper()
            return True

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
            
            # Calculate the normalized horizontal error (0.0 = center, 1.0 = edge of frame)
            # We use the absolute error in x, as it's typically the most important axis for horizontal alignment.
            center_error_ratio = abs(x - w / 2) / (w / 2)

            # Calculate the scaling factor for the velocity.
            # Clamp the error ratio so it doesn't go above our threshold, preventing negative velocities.
            error_scale = min(center_error_ratio / CENTERING_ERROR_THRESHOLD, 1.0) 

            # Linearly interpolate the downward velocity.
            # If error_scale is 0 (centered), down_vel = MAX_DOWN_VEL.
            # If error_scale is 1 (at/beyond threshold), down_vel = MIN_DOWN_VEL.
            vel_range = MAX_DOWN_VEL - MIN_DOWN_VEL
            dynamic_down_vel = MAX_DOWN_VEL - (error_scale * vel_range)
            master.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(
                0, master.target_system, master.target_component, mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
                VELOCITY_CONTROL_BITMASK, 0, 0, 0, fwd_vel, right_vel, dynamic_down_vel, 0, 0, 0, 0, 0))
            
            # Updated print statement to show the dynamic velocity
            print(f"\rLANDING (ID {detected_id}): Alt: {current_altitude:.2f}m, Gain: {horizontal_gain:.2f}, Err: {center_error_ratio:.2%}, Vz: {dynamic_down_vel:.2f} m/s", end="")

            if current_altitude < LANDING_APPROACH_ALT and center_error_ratio < 0.10:
                print("\nTarget centered at low altitude. Switching to LAND mode.")
                land_normally(master)
                time.sleep(1)
                print("Landed. Closing gripper to pick up package.")
                time.sleep(1)
                servo_control.close_gripper()
                return True

        except (socket.timeout, json.JSONDecodeError, KeyError):
            time_since_lost = time.time() - last_detection_time
            vx, vy, vz = 0, 0, 0
            
            if time_since_lost < TARGET_LOST_HOVER_DURATION:
                print(f"\rTarget lost. Hovering... (Time since last seen: {time_since_lost:.1f}s)", end="")
            else:
                vz = -REACQUIRE_ASCEND_SPEED
                vx = BLIND_FORWARD_SPEED
                print(f"\rSearching... Ascending and moving forward. (Time since last seen: {time_since_lost:.1f}s)", end="")
            
            master.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(
                0, master.target_system, master.target_component, mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
                VELOCITY_CONTROL_BITMASK, 0, 0, 0, vx, vy, vz, 0, 0, 0, 0, 0))

    print("\nLanding timeout reached. Aborting and hovering.")
    master.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(
        0, master.target_system, master.target_component, mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
        VELOCITY_CONTROL_BITMASK, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0))
    return False

def handle_window_approach(master, target_altitude):
    """
    Switches to EKF source set 2 (GPS for altitude) and verifies the reading.
    Returns True if altitude is within tolerance, False otherwise.
    """
    print("\n--- WINDOW APPROACH ---")
    print("Switching EKF to Source Set 2 (GPS for altitude)...")
    
    # Send command to switch EKF source set
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_DO_AUX_FUNCTION, 
        0, # confirmation
        90, # param1: Auxiliary Function code,same as RCx_OPTIONS, 90 for EKF Source Set Switch
        1, # param2: 0:Source Set 1, 1:Source Set 2, 2:Source Set 3
        0, # param3: Unused
        0, # param4: Unused
        0, # param5: Unused
        0, # param6: Unused
        0  # param7: Unused
    )
    time.sleep(1)
    
    print("Verifying altitude from new source...")
    msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=3)
    
    if not msg:
        print("Error: Did not receive altitude data after EKF switch.")
        return False
        
    current_gps_alt = msg.relative_alt / 1000.0
    altitude_error = abs(current_gps_alt - target_altitude)
    
    print(f"Target Altitude: {target_altitude:.2f}m | GPS Altitude: {current_gps_alt:.2f}m | Error: {altitude_error:.2f}m")
    
    if altitude_error > ALTITUDE_CHECK_TOLERANCE:
        print(f"Altitude check FAILED. Error ({altitude_error:.2f}m) exceeds tolerance ({ALTITUDE_CHECK_TOLERANCE:.2f}m).")
        return False
    else:
        print("Altitude check PASSED.")
        return True

                                    
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
        print("\n--- MISSION 5: Before Window 4 ---")
        send_control_command('pause')
        navigate_to_waypoint(master, WAYPOINTS[3][0], WAYPOINTS[3][1], 1.6)

        # Perform EKF switch and altitude check
        if not handle_window_approach(master, 1.6):
            print("\nAltitude check failed. Mission aborted. Landing immediately.")
            land_normally(master)
            return # End the mission
            
        print("\n--- Navigating to 'After Window' at Waypoint 5 ---")
        navigate_to_waypoint(master, WAYPOINTS[4][0], WAYPOINTS[4][1], 1.6)

        print("\nSwitching EKF back to Source Set 1 (Rangefinder for altitude)...")
        master.mav.command_long_send(
            master.target_system, master.target_component,
            mavutil.mavlink.MAV_CMD_DO_AUX_FUNCTION,
            0,  # confirmation
            90, # param1: Auxiliary Function code for EKF Source Set Switch
            0,  # param2: 0 for Source Set 1
            0, 0, 0, 0, 0
        )
        time.sleep(1) # Allow time for the switch to process
        print("EKF source successfully set to 1.")
        
        # --- Mission 6: Final Landing ---
        print("\n--- MISSION 6: Final Landing at Waypoint 6---")
        navigate_to_waypoint(master, WAYPOINTS[5][0], WAYPOINTS[5][1], 1.6)
        land_normally(master)

        print("\nMission finished successfully!")

    except KeyboardInterrupt:
        print("Keyboard interrupt received. Landing immediately...")
        land_normally(master)
        servo_control.open_gripper()

    finally:
        servo_control.cleanup()
        if data_sock: data_sock.close()
        print("Resources cleaned up.")

if __name__ == "__main__":
    main()