import time
import board
from adafruit_pca9685 import PCA9685
from adafruit_motor import servo
from pymavlink import mavutil

# =================================================================================
# --- 1. CONFIGURATION ---
# =================================================================================

# --- Servo Configuration ---
# This dictionary now defines all servo mechanisms.
SERVO_CONFIG = {
    'lower_gripper': {
        'name': 'LOWER GRIPPER',
        'channels': [0, 1, 2, 3],
        'open_angles':  [0, 0, 0, 0],
        'close_angles': [130, 130, 130, 130],
    },
    'outdoor_drop_1': {
        'name': 'OUTDOOR DROP 1',
        'channel': 4, # Use singular 'channel' for single servos
        'hold_angle': 90,
        'drop_angle': 0,
    },
    'outdoor_drop_2': {
        'name': 'OUTDOOR DROP 2',
        'channel': 5,
        'hold_angle': 0,
        'drop_angle': 90,
    }
}

# --- MAVLink Connection Configuration ---
MAVLINK_CONNECTION_STRING = 'udp:127.0.0.1:14550'

# --- MAVLink Channel Mapping ---
# Maps an ArduPilot AUX channel to a function and its PWM threshold.
# 'channel' MUST match the "Ser No" in your Mission Planner `DO_SET_SERVO` command.
# AUX1 = 9, AUX2 = 10, AUX3 = 11, etc.
MAVLINK_CONTROL_MAPPING = {
    'gripper': {
        'channel': 9,
        'pwm_threshold': 1500 # Above threshold -> open, Below -> close
    },
    'outdoor_1': {
        'channel': 10,
        'pwm_threshold': 1500 # Above threshold -> drop, Below -> hold
    },
    'outdoor_2': {
        'channel': 11,
        'pwm_threshold': 1500 # Above threshold -> drop, Below -> hold
    }
}


# =================================================================================
# --- 2. GLOBAL VARIABLES ---
# =================================================================================

pca = None
servos = {} # Dictionary to hold all initialized servo objects

# =================================================================================
# --- 3. HARDWARE CONTROL FUNCTIONS ---
# =================================================================================

def setup():
    """Initializes the I2C connection and ALL configured servo motors."""
    global pca, servos
    print("Initializing I2C and PCA9685 for all servo control...")
    try:
        i2c = board.I2C()
        pca = PCA9685(i2c)
        pca.frequency = 50

        # Loop through all configurations to initialize every servo
        for key, config in SERVO_CONFIG.items():
            print(f"  - Initializing: {config['name']}...")
            if 'channels' in config: # Multi-servo setup (gripper)
                for i, channel_num in enumerate(config['channels']):
                    servo_key = f"{key}_{i}"
                    servos[servo_key] = servo.Servo(pca.channels[channel_num])
            elif 'channel' in config: # Single-servo setup
                servos[key] = servo.Servo(pca.channels[config['channel']])
        
        # Set outdoor drop servos to their 'hold' position initially
        hold_package_outdoor_1(silent=True)
        hold_package_outdoor_2(silent=True)

        print(f"✅ All servos initialized.")
        return True
    except Exception as e:
        print(f"❌ ERROR: Failed to initialize hardware: {e}")
        return False

def _set_angles(servo_objects, target_angles):
    """Internal helper to move multiple servos and then de-energize them."""
    if not pca: return
    try:
        for servo_obj, angle in zip(servo_objects, target_angles):
            if angle is not None:
                servo_obj.angle = angle
        time.sleep(1.0) # Wait for servos to move
        for servo_obj in servo_objects:
            servo_obj.angle = None # De-energize
    except Exception as e:
        print(f"Error moving servos: {e}")

def open_gripper(silent=False):
    """Moves the 4-servo lower gripper to the 'open' position."""
    config = SERVO_CONFIG['lower_gripper']
    if not silent: print(f"OPENING {config['name']}...")
    servo_keys = [f"lower_gripper_{i}" for i in range(len(config['channels']))]
    servo_objects = [servos[key] for key in servo_keys]
    _set_angles(servo_objects, config['open_angles'])
    if not silent: print(f"{config['name']} is OPEN.")

def close_gripper(silent=False):
    """Moves the 4-servo lower gripper to the 'close' position."""
    config = SERVO_CONFIG['lower_gripper']
    if not silent: print(f"CLOSING {config['name']}...")
    servo_keys = [f"lower_gripper_{i}" for i in range(len(config['channels']))]
    servo_objects = [servos[key] for key in servo_keys]
    _set_angles(servo_objects, config['close_angles'])
    if not silent: print(f"{config['name']} is CLOSED.")

def drop_package_outdoor_1(silent=False):
    """Moves the first outdoor servo to its DROP position."""
    config = SERVO_CONFIG['outdoor_drop_1']
    if not silent: print(f"Actuating {config['name']} to DROP...")
    _set_angles([servos['outdoor_drop_1']], [config['drop_angle']])
    if not silent: print(f"{config['name']} is in DROP position.")

def hold_package_outdoor_1(silent=False):
    """Moves the first outdoor servo to its HOLD position."""
    config = SERVO_CONFIG['outdoor_drop_1']
    if not silent: print(f"Resetting {config['name']} to HOLD...")
    _set_angles([servos['outdoor_drop_1']], [config['hold_angle']])
    if not silent: print(f"{config['name']} is in HOLD position.")

def drop_package_outdoor_2(silent=False):
    """Moves the second outdoor servo to its DROP position."""
    config = SERVO_CONFIG['outdoor_drop_2']
    if not silent: print(f"Actuating {config['name']} to DROP...")
    _set_angles([servos['outdoor_drop_2']], [config['drop_angle']])
    if not silent: print(f"{config['name']} is in DROP position.")

def hold_package_outdoor_2(silent=False):
    """Moves the second outdoor servo to its HOLD position."""
    config = SERVO_CONFIG['outdoor_drop_2']
    if not silent: print(f"Resetting {config['name']} to HOLD...")
    _set_angles([servos['outdoor_drop_2']], [config['hold_angle']])
    if not silent: print(f"{config['name']} is in HOLD position.")

def cleanup():
    """De-initializes the PCA9685 board."""
    if pca:
        print("De-initializing PCA9685...")
        pca.deinit()
    print("Cleanup complete.")

# =================================================================================
# --- 4. MAVLINK LISTENER FUNCTION ---
# =================================================================================

def mavlink_listener():
    """Listens for SERVO_OUTPUT_RAW messages and controls all mechanisms."""
    print(f"Connecting to MAVLink on '{MAVLINK_CONNECTION_STRING}'...")
    master = mavutil.mavlink_connection(MAVLINK_CONNECTION_STRING, wait_ready=True)
    master.wait_heartbeat()
    print("✅ MAVLink Heartbeat received! Listening for servo commands...")

    # State variables to avoid sending repeated commands
    states = {'gripper': None, 'outdoor_1': None, 'outdoor_2': None}

    while True:
        msg = master.recv_match(type='SERVO_OUTPUT_RAW', blocking=True)
        if not msg: continue

        # --- Check Gripper Channel ---
        gripper_map = MAVLINK_CONTROL_MAPPING['gripper']
        pwm_val = getattr(msg, f"servo{gripper_map['channel']}_raw", 0)
        if pwm_val > 0:
            if pwm_val > gripper_map['pwm_threshold'] and states['gripper'] != 'open':
                print(f"Gripper PWM {pwm_val} (> {gripper_map['pwm_threshold']}). Opening.")
                open_gripper()
                states['gripper'] = 'open'
            elif pwm_val < gripper_map['pwm_threshold'] and states['gripper'] != 'closed':
                print(f"Gripper PWM {pwm_val} (< {gripper_map['pwm_threshold']}). Closing.")
                close_gripper()
                states['gripper'] = 'closed'

        # --- Check Outdoor Drop 1 Channel ---
        outdoor_1_map = MAVLINK_CONTROL_MAPPING['outdoor_1']
        pwm_val = getattr(msg, f"servo{outdoor_1_map['channel']}_raw", 0)
        if pwm_val > 0:
            if pwm_val > outdoor_1_map['pwm_threshold'] and states['outdoor_1'] != 'dropped':
                print(f"Outdoor 1 PWM {pwm_val} (> {outdoor_1_map['pwm_threshold']}). Dropping.")
                drop_package_outdoor_1()
                states['outdoor_1'] = 'dropped'
            elif pwm_val < outdoor_1_map['pwm_threshold'] and states['outdoor_1'] != 'held':
                print(f"Outdoor 1 PWM {pwm_val} (< {outdoor_1_map['pwm_threshold']}). Holding.")
                hold_package_outdoor_1()
                states['outdoor_1'] = 'held'

        # --- Check Outdoor Drop 2 Channel ---
        outdoor_2_map = MAVLINK_CONTROL_MAPPING['outdoor_2']
        pwm_val = getattr(msg, f"servo{outdoor_2_map['channel']}_raw", 0)
        if pwm_val > 0:
            if pwm_val > outdoor_2_map['pwm_threshold'] and states['outdoor_2'] != 'dropped':
                print(f"Outdoor 2 PWM {pwm_val} (> {outdoor_2_map['pwm_threshold']}). Dropping.")
                drop_package_outdoor_2()
                states['outdoor_2'] = 'dropped'
            elif pwm_val < outdoor_2_map['pwm_threshold'] and states['outdoor_2'] != 'held':
                print(f"Outdoor 2 PWM {pwm_val} (< {outdoor_2_map['pwm_threshold']}). Holding.")
                hold_package_outdoor_2()
                states['outdoor_2'] = 'held'

# =================================================================================
# --- 5. MAIN EXECUTION BLOCK ---
# =================================================================================

if __name__ == '__main__':
    try:
        if setup():
            mavlink_listener()
    except KeyboardInterrupt:
        print("\nProgram interrupted by user.")
    except Exception as e:
        print(f"\nAn unexpected error occurred: {e}")
    finally:
        cleanup()
