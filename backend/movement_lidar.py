import socket
import json
import time
import math
from pymavlink import mavutil
import sys

# Try to import the serial library, fail gracefully if not installed
try:
    import serial
except ImportError:
    print("Error: 'pyserial' library not found.")
    print("Please install it using: pip install pyserial")
    sys.exit(1)

# --- Drone & General Configuration ---
CONNECTION_STRING = 'udp:127.0.0.1:14550'
BAUD_RATE = 921600
TAKEOFF_ALTITUDE = 1.5  # meters
ARMING_RETRIES = 3
ARMING_RETRY_DELAY = 3

# --- LiDAR Navigation Configuration ---
FORWARD_SPEED = 0.4        # m/s
TURN_DIRECTION = 'left'
FRONT_WALL_THRESHOLD = 0.7 # meters (Stop distance for corners)
DROP_ZONE_DISTANCE_FROM_CORNER = 7.5 # meters (Stop when back LiDAR reaches this)

# --- PID Controller Gains ---
WALL_FOLLOW_KP, WALL_FOLLOW_KI, WALL_FOLLOW_KD = 0.6, 0.02, 0.15
CENTERING_KP, CENTERING_KI, CENTERING_KD = 0.7, 0.03, 0.2

# --- LiDAR UART Configuration ---
# IMPORTANT: Use the lidar_test.py script to find the correct port for each direction,
# then update this dictionary with your findings.
LIDAR_PORTS = {
    'front': '/dev/ttyAMA0', # Example port, change as needed
    'back':  '/dev/ttyAMA1', # Example port, change as needed
    'left':  '/dev/ttyAMA2', # Example port, change as needed
    'right': '/dev/ttyAMA3'  # Example port, change as needed
}
LIDAR_BAUDRATE = 115200

# --- Mission Task Configuration ---
LOGISTIC_1_STOP_DISTANCE_FRONT = 3.7
DROP_ZONE_ID = 1

# --- Vision System Configuration ---
UDP_RECEIVE_IP, UDP_RECEIVE_PORT = "127.0.0.2", 5005
CONTROL_SERVER_IP, CONTROL_SERVER_PORT = "127.0.0.2", 5006
TRACKING_SPEED, FWD_GAIN, ALT_GAIN = 0.5, 1.0, 0.5
LANDING_APPROACH_ALT, CENTERING_ALTITUDE = 0.75, 0.75
CENTERING_TIMEOUT, LANDING_TIMEOUT = 20, 20
TARGET_LOST_HOVER_DURATION, REACQUIRE_ASCEND_SPEED = 3.0, 0.3
GAIN_MAX_ALT, GAIN_MIN_ALT = 2.0, 0.5
MAX_HORIZONTAL_GAIN, MIN_HORIZONTAL_GAIN = 0.8, 0.2

# --- MAVLink & ArduPilot Constants ---
GUIDED_MODE = 4
VELOCITY_CONTROL_BITMASK = 0b0000111111000111

# --- Global Socket ---
data_sock = None

# ############################################################################
# ## Hardware Interface Classes (LiDAR & PID)
# ############################################################################

class TFminiUART:
    """Handles communication with a single TFmini LiDAR sensor over a UART serial port."""
    def __init__(self, port, baudrate):
        self.port = port
        self.baudrate = baudrate
        self.ser = None
        try:
            self.ser = serial.Serial(port, baudrate, timeout=0.1)
        except serial.SerialException as e:
            print(f"Error opening serial port {port}: {e}")

    def readDistance(self):
        """Reads a 9-byte data packet from the TFmini and returns the distance in cm."""
        if not self.ser or not self.ser.is_open:
            return None
        
        # Continuously read from the serial port to find a valid packet
        while True:
            # The packet starts with two 0x59 bytes (the header)
            header1 = self.ser.read(1)
            if header1 == b'\x59':
                header2 = self.ser.read(1)
                if header2 == b'\x59':
                    # Header found, read the remaining 7 bytes
                    packet = self.ser.read(7)
                    if len(packet) == 7:
                        # Verify the checksum (sum of first 8 bytes modulo 256)
                        checksum = sum(b'\x59\x59' + packet[:6]) & 0xFF
                        if checksum == packet[6]:
                            # Packet is valid, extract the distance
                            # Distance is in the 3rd and 4th bytes (little-endian)
                            dist_cm = packet[0] + (packet[1] << 8)
                            return dist_cm
        return None # Should not be reached in normal operation

class LidarManager:
    """Manages all TFminiUART sensors."""
    def __init__(self, ports_dict, baudrate):
        self.sensors = {direction: TFminiUART(port, baudrate) for direction, port in ports_dict.items()}

    def get_all_distances(self):
        """Returns a dictionary of all sensor distances in meters."""
        # Read from each sensor, defaulting to a large number on failure
        distances = {d: (s.readDistance() or 9900) / 100.0 for d, s in self.sensors.items()}
        return distances

class PIDController:
    """A simple PID controller."""
    def __init__(self, Kp, Ki, Kd, setpoint):
        self.Kp, self.Ki, self.Kd, self.setpoint = Kp, Ki, Kd, setpoint
        self._integral, self._prev_error, self.last_time = 0, 0, time.time()

    def update(self, current_value):
        dt = time.time() - self.last_time
        if dt <= 0: return 0
        error = self.setpoint - current_value
        self._integral += error * dt
        derivative = (error - self._prev_error) / dt
        output = (self.Kp * error) + (self.Ki * self._integral) + (self.Kd * derivative)
        self._prev_error, self.last_time = error, time.time()
        return output

# ############################################################################
# ## Core Drone Control Functions
# ############################################################################

def send_control_command(command):
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.connect((CONTROL_SERVER_IP, CONTROL_SERVER_PORT))
            s.sendall(command.encode('utf-8'))
    except Exception as e: print(f"Control command error: {e}")

def arm_and_takeoff(master, altitude):
    print("Setting mode to GUIDED...")
    master.mav.set_mode_send(master.target_system, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, GUIDED_MODE)
    for attempt in range(1, ARMING_RETRIES + 1):
        print(f"Arming motors (Attempt {attempt})...")
        master.mav.command_long_send(master.target_system, master.target_component, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)
        try:
            master.motors_armed_wait(timeout=5)
            print("Motors armed!")
            break
        except Exception as e:
            print(f"Arming failed: {e}")
            if attempt == ARMING_RETRIES: return False
            time.sleep(ARMING_RETRY_DELAY)

    print(f"Taking off to {altitude}m...")
    master.mav.command_long_send(master.target_system, master.target_component, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, altitude)
    while True:
        msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
        if (msg.relative_alt / 1000.0) >= altitude * 0.95: break
        time.sleep(0.1)
    print("Target altitude reached.")
    return True

def land_normally(master):
    print("Executing normal landing...")
    master.mav.command_long_send(master.target_system, master.target_component, mavutil.mavlink.MAV_CMD_NAV_LAND, 0, 0, 0, 0, 0, 0, 0, 0)
    master.motors_disarmed_wait()
    print("Landed and disarmed.")

def set_velocity(master, vx=0, vy=0, vz=0, yaw_rate=0):
    master.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(
        0, master.target_system, master.target_component, mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
        VELOCITY_CONTROL_BITMASK, 0, 0, 0, vx, vy, vz, 0, 0, 0, 0, yaw_rate))

def execute_turn(master, direction, angle_degrees=90, rate_dps=20):
    print(f"Executing {angle_degrees}-degree turn to the {direction}...")
    yaw_rate_rad_s = math.radians(rate_dps) * (-1 if direction == 'left' else 1)
    set_velocity(master, yaw_rate=yaw_rate_rad_s)
    time.sleep(angle_degrees / rate_dps)
    set_velocity(master) # Stop turn
    time.sleep(1)
    print("Turn complete.")

def flush_socket_buffer(sock):
    sock.setblocking(False)
    while True:
        try: sock.recvfrom(1024)
        except BlockingIOError: break
    sock.setblocking(True)
    sock.settimeout(0.5)

def get_dynamic_gain(current_alt):
    if current_alt >= GAIN_MAX_ALT: return MAX_HORIZONTAL_GAIN
    if current_alt <= GAIN_MIN_ALT: return MIN_HORIZONTAL_GAIN
    return MIN_HORIZONTAL_GAIN + (MAX_HORIZONTAL_GAIN - MIN_HORIZONTAL_GAIN) * ((current_alt - GAIN_MIN_ALT) / (GAIN_MAX_ALT - GAIN_MIN_ALT))

# ############################################################################
# ## Vision-Based Mission Actions
# ############################################################################

def pickup_logistic(master, sock):
    print(f"\n--- Initiating pickup for any logistic (ignoring barrel ID: {DROP_ZONE_ID}) ---")
    flush_socket_buffer(sock)
    send_control_command('resume')
    
    start_time, last_detection_time = time.time(), time.time()
    last_known_alt = TAKEOFF_ALTITUDE

    while time.time() - start_time < LANDING_TIMEOUT:
        try:
            alt_msg = master.recv_match(type='DISTANCE_SENSOR', blocking=False, timeout=0.05)
            if alt_msg: last_known_alt = alt_msg.current_distance / 100.0
            current_altitude = last_known_alt

            data, _ = sock.recvfrom(1024)
            detection = json.loads(data.decode())
            
            if detection.get("class_id") == DROP_ZONE_ID:
                raise socket.timeout

            last_detection_time, start_time = time.time(), time.time()
            x, y = detection["x_center"], detection["frame_width"] - detection["y_center"]
            w, h = detection["frame_width"], detection["frame_height"]
            
            x_offset, y_offset = (x - w / 2) / (w / 2), (y - h / 2) / (h / 2)
            gain = get_dynamic_gain(current_altitude)
            fwd_vel, right_vel = -TRACKING_SPEED * y_offset * FWD_GAIN * gain, TRACKING_SPEED * x_offset * gain
            area_error = 1.0 - (detection.get("area", 0) / (0.2 * w * h))
            down_vel = TRACKING_SPEED * area_error * ALT_GAIN
            
            set_velocity(master, fwd_vel, right_vel, down_vel)
            
            center_error = abs(x - w / 2) / w
            print(f"\rPICKUP: Alt: {current_altitude:.2f}m, Err: {center_error:.2%}", end="")

            if current_altitude < LANDING_APPROACH_ALT and center_error < 0.10:
                print("\nTarget centered at low altitude. Pickup successful.")
                set_velocity(master)
                time.sleep(1)
                send_control_command('pause')
                return True
        except (socket.timeout, json.JSONDecodeError, KeyError):
            time_since_lost = time.time() - last_detection_time
            vz = -REACQUIRE_ASCEND_SPEED if time_since_lost > TARGET_LOST_HOVER_DURATION else 0
            set_velocity(master, vz=vz)
            print(f"\rSearching for logistic... Last seen {time_since_lost:.1f}s ago", end="")
    
    print("\nPickup timeout. Aborting.")
    set_velocity(master)
    send_control_command('pause')
    return False

def drop_logistic(master, sock, target_class_id):
    print(f"\n--- Initiating drop for target ID: {target_class_id} ---")
    flush_socket_buffer(sock)
    send_control_command('resume')
    
    start_time, last_detection_time = time.time(), time.time()
    last_known_alt = CENTERING_ALTITUDE

    while time.time() - start_time < CENTERING_TIMEOUT:
        try:
            alt_msg = master.recv_match(type='DISTANCE_SENSOR', blocking=False, timeout=0.05)
            if alt_msg: last_known_alt = alt_msg.current_distance / 100.0
            current_altitude = last_known_alt

            data, _ = sock.recvfrom(1024)
            detection = json.loads(data.decode())
            
            if detection.get("class_id") != target_class_id:
                raise socket.timeout

            last_detection_time, start_time = time.time(), time.time()
            x, y = detection["x_center"], detection["frame_width"] - detection["y_center"]
            w, h = detection["frame_width"], detection["frame_height"]

            x_offset, y_offset = (x - w / 2) / (w / 2), (y - h / 2) / (h / 2)
            gain = get_dynamic_gain(current_altitude)
            fwd_vel, right_vel = -TRACKING_SPEED * y_offset * gain, TRACKING_SPEED * x_offset * gain
            down_vel = -ALT_GAIN * (CENTERING_ALTITUDE - current_altitude)
            
            set_velocity(master, fwd_vel, right_vel, down_vel)

            center_error = abs(x - w / 2) / w
            print(f"\rDROP (ID {target_class_id}): Alt: {current_altitude:.2f}m, Err: {center_error:.2%}", end="")

            if center_error < 0.05 and abs(CENTERING_ALTITUDE - current_altitude) < 0.10:
                print("\nTarget centered. Drop successful.")
                set_velocity(master)
                time.sleep(1)
                send_control_command('pause')
                return True
        except (socket.timeout, json.JSONDecodeError, KeyError):
            time_since_lost = time.time() - last_detection_time
            vz = -REACQUIRE_ASCEND_SPEED if time_since_lost > TARGET_LOST_HOVER_DURATION else 0
            set_velocity(master, vz=vz)
            print(f"\rSearching for drop-zone ID {target_class_id}... Last seen {time_since_lost:.1f}s ago", end="")
            
    print("\nDrop timeout. Aborting.")
    set_velocity(master)
    send_control_command('pause')
    return False

# ############################################################################
# ## LiDAR Navigation Functions
# ############################################################################

def follow_wall(master, lidar, pid, wall_side, direction='forward'):
    speed, obstacle_sensor = (FORWARD_SPEED, 'front') if direction == 'forward' else (-FORWARD_SPEED, 'back')
    print(f"\nWall-following ({direction}) on '{wall_side}' side...")
    pid.last_time = time.time()

    while True:
        distances = lidar.get_all_distances()
        if distances[obstacle_sensor] < FRONT_WALL_THRESHOLD:
            print(f"\nCorner detected by {obstacle_sensor} LiDAR.")
            set_velocity(master)
            return True
        
        vy_correction = pid.update(distances[wall_side])
        if wall_side == 'left': vy_correction = -vy_correction
        set_velocity(master, vx=speed, vy=max(-0.5, min(0.5, vy_correction)))
        time.sleep(0.05)
    return False

# ############################################################################
# ## Main Execution Logic
# ############################################################################

def main():
    global data_sock
    data_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    data_sock.bind((UDP_RECEIVE_IP, UDP_RECEIVE_PORT))
    data_sock.settimeout(0.5)

    master = mavutil.mavlink_connection(CONNECTION_STRING, baud=BAUD_RATE)
    master.wait_heartbeat()
    print(f"Heartbeat from system (system {master.target_system})")
    
    master.mav.command_long_send(master.target_system, master.target_component, mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0, mavutil.mavlink.MAVLINK_MSG_ID_DISTANCE_SENSOR, 100000, 0, 0, 0, 0, 0)

    lidar = LidarManager(LIDAR_PORTS, LIDAR_BAUDRATE)
    
    try:
        if not arm_and_takeoff(master, TAKEOFF_ALTITUDE): raise Exception("Takeoff failed.")
        
        # === FIRST DELIVERY (LOGISTIC 1) ===
        print(f"\n--- MISSION 1: Acquiring and Delivering Logistic #1 ---")
        print(f"Centering until front LiDAR <= {LOGISTIC_1_STOP_DISTANCE_FRONT}m")
        centering_pid = PIDController(CENTERING_KP, CENTERING_KI, CENTERING_KD, setpoint=0)
        while True:
            distances = lidar.get_all_distances()
            if distances['front'] <= LOGISTIC_1_STOP_DISTANCE_FRONT: break
            error = distances['left'] - distances['right']
            vy = -centering_pid.update(error)
            set_velocity(master, vx=FORWARD_SPEED, vy=max(-0.5, min(0.5, vy)))
            time.sleep(0.05)
        
        if not pickup_logistic(master, data_sock): raise Exception("Pickup of L1 failed.")
        if not arm_and_takeoff(master, TAKEOFF_ALTITUDE): raise Exception("Second takeoff failed.")
        
        wall_follow_pid = PIDController(WALL_FOLLOW_KP, WALL_FOLLOW_KI, WALL_FOLLOW_KD, setpoint=0.8)
        if not follow_wall(master, lidar, wall_follow_pid, 'right', 'forward'): raise Exception("Failed to find corner.")
        execute_turn(master, TURN_DIRECTION)
        
        while lidar.get_all_distances()['back'] < DROP_ZONE_DISTANCE_FROM_CORNER:
            distances = lidar.get_all_distances()
            vy = wall_follow_pid.update(distances['right'])
            set_velocity(master, vx=FORWARD_SPEED, vy=max(-0.5, min(0.5, vy)))
            time.sleep(0.05)
        
        if not drop_logistic(master, data_sock, DROP_ZONE_ID): raise Exception("Drop of L1 failed.")

        # === SECOND DELIVERY (LOGISTIC 2) ===
        print(f"\n--- MISSION 2: Acquiring and Delivering Logistic #2 ---")
        if not follow_wall(master, lidar, wall_follow_pid, 'right', 'backward'): raise Exception("Failed to return to corner.")
        
        if not pickup_logistic(master, data_sock): raise Exception("Pickup of L2 failed.")
        if not arm_and_takeoff(master, TAKEOFF_ALTITUDE): raise Exception("Third takeoff failed.")

        while lidar.get_all_distances()['back'] < DROP_ZONE_DISTANCE_FROM_CORNER:
            distances = lidar.get_all_distances()
            vy = wall_follow_pid.update(distances['right'])
            set_velocity(master, vx=FORWARD_SPEED, vy=max(-0.5, min(0.5, vy)))
            time.sleep(0.05)

        if not drop_logistic(master, data_sock, DROP_ZONE_ID): raise Exception("Drop of L2 failed.")
        
        print("\n--- Flying past target for safe landing ---")
        set_velocity(master, vx=0.3)
        time.sleep(2.0)
        
        print("--- All deliveries complete. Landing. ---")
        land_normally(master)
        
        print("\nMission finished successfully!")

    except Exception as e:
        print(f"\nMISSION FAILED: {e}")
        set_velocity(master) # Stop all movement
        time.sleep(1)
        land_normally(master)
    finally:
        if data_sock: data_sock.close()
        print("Script finished.")

if __name__ == "__main__":
    main()
