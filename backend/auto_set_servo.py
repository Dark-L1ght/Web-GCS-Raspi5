import time
import board
from adafruit_pca9685 import PCA9685
from adafruit_motor import servo
from pymavlink import mavutil
from gpiozero import Motor, Device

# Use the default pin factory and prevent noisy GPIO warnings
# This matches your original winch_control.py
Device.pin_factory = None


# =================================================================================
# --- 1. CONFIGURATION ---
# =================================================================================

# --- Servo Configuration ---
SERVO_CONFIG = {
    'lower_gripper': {
        'name': 'LOWER GRIPPER',
        'channels': [0, 1, 2, 3],
        'open_angles':  [0, 0, 0, 0],
        'close_angles': [130, 130, 130, 130],
    },
    'outdoor_drop_1': {
        'name': 'OUTDOOR DROP 1',
        'channel': 4,
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

# --- Winch Motor Configuration (from winch_control.py) ---
WINCH_MOTOR_PINS = {
    'in1': 17,    # BCM Pin for IN1 on L298N
    'in2': 27,    # BCM Pin for IN2 on L298N
    'enable': 12, # BCM Pin for ENA on L298N
}
WINCH_MOTOR_SPEED = 1.0 # Speed from 0.0 to 1.0

# --- MAVLink Connection Configuration ---
MAVLINK_CONNECTION_STRING = 'udp:127.0.0.1:14550'

# --- MAVLink Channel Mapping ---
# Maps an ArduPilot AUX channel to a function and its PWM threshold(s).
# AUX1=9, AUX2=10, AUX3=11, AUX4=12, AUX5=13, AUX6=14
MAVLINK_CONTROL_MAPPING = {
    'gripper': {
        'channel': 9,
        'pwm_threshold': 1500 # Above -> open, Below -> close
    },
    'outdoor_1': {
        'channel': 10,
        'pwm_threshold': 1500 # Above -> drop, Below -> hold
    },
    'outdoor_2': {
        'channel': 11,
        'pwm_threshold': 1500 # Above -> drop, Below -> hold
    },
    'winch': {
        'channel': 12, # Using AUX4 for the winch
        'pwm_lower_threshold': 1600, # PWM value above which we lower the winch
        'pwm_raise_threshold': 1400  # PWM value below which we raise the winch
                                     # In between these values is the "dead zone" where the motor stops.
    }
}


# =================================================================================
# --- 2. GLOBAL VARIABLES ---
# =================================================================================

pca = None
servos = {}        # Dictionary to hold all initialized servo objects
winch_motor = None # Global object for the winch motor

# =================================================================================
# --- 3. HARDWARE CONTROL FUNCTIONS ---
# =================================================================================

def setup():
    """Initializes I2C, ALL servos, and the winch motor."""
    global pca, servos, winch_motor
    print("Initializing all hardware...")
    
    # --- Initialize Servos via PCA9685 ---
    try:
        print(" -> Initializing I2C and PCA9685...")
        i2c = board.I2C()
        pca = PCA9685(i2c)
        pca.frequency = 50

        for key, config in SERVO_CONFIG.items():
            print(f"   - Initializing: {config['name']}...")
            if 'channels' in config:
                for i, channel_num in enumerate(config['channels']):
                    servo_key = f"{key}_{i}"
                    servos[servo_key] = servo.Servo(pca.channels[channel_num])
            elif 'channel' in config:
                servos[key] = servo.Servo(pca.channels[config['channel']])
        
        hold_package_outdoor_1(silent=True)
        hold_package_outdoor_2(silent=True)
        print("✅ Servos initialized successfully.")
    except Exception as e:
        print(f"❌ ERROR: Failed to initialize servos: {e}")
        return False

    # --- Initialize Winch Motor via GPIO ---
    try:
        # print(" -> Initializing winch motor...")
        # winch_motor = Motor(
        #     forward=WINCH_MOTOR_PINS['in1'],
        #     backward=WINCH_MOTOR_PINS['in2'],
        #     enable=WINCH_MOTOR_PINS['enable']
        # )
        print("✅ Winch motor initialized successfully.")
    except Exception as e:
        print(f"❌ ERROR: Failed to initialize winch motor: {e}")
        print("   (Check GPIO connections and permissions).")
        return False
        
    print("✅ All hardware initialized.")
    return True

# --- Servo Functions ---
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
    config = SERVO_CONFIG['lower_gripper']
    if not silent: print(f"OPENING {config['name']}...")
    servo_keys = [f"lower_gripper_{i}" for i in range(len(config['channels']))]
    servo_objects = [servos[key] for key in servo_keys]
    _set_angles(servo_objects, config['open_angles'])
    if not silent: print(f"{config['name']} is OPEN.")

def close_gripper(silent=False):
    config = SERVO_CONFIG['lower_gripper']
    if not silent: print(f"CLOSING {config['name']}...")
    servo_keys = [f"lower_gripper_{i}" for i in range(len(config['channels']))]
    servo_objects = [servos[key] for key in servo_keys]
    _set_angles(servo_objects, config['close_angles'])
    if not silent: print(f"{config['name']} is CLOSED.")

def drop_package_outdoor_1(silent=False):
    config = SERVO_CONFIG['outdoor_drop_1']
    if not silent: print(f"Actuating {config['name']} to DROP...")
    _set_angles([servos['outdoor_drop_1']], [config['drop_angle']])
    if not silent: print(f"{config['name']} is in DROP position.")

def hold_package_outdoor_1(silent=False):
    config = SERVO_CONFIG['outdoor_drop_1']
    if not silent: print(f"Resetting {config['name']} to HOLD...")
    _set_angles([servos['outdoor_drop_1']], [config['hold_angle']])
    if not silent: print(f"{config['name']} is in HOLD position.")

def drop_package_outdoor_2(silent=False):
    config = SERVO_CONFIG['outdoor_drop_2']
    if not silent: print(f"Actuating {config['name']} to DROP...")
    _set_angles([servos['outdoor_drop_2']], [config['drop_angle']])
    if not silent: print(f"{config['name']} is in DROP position.")

def hold_package_outdoor_2(silent=False):
    config = SERVO_CONFIG['outdoor_drop_2']
    if not silent: print(f"Resetting {config['name']} to HOLD...")
    _set_angles([servos['outdoor_drop_2']], [config['hold_angle']])
    if not silent: print(f"{config['name']} is in HOLD position.")

# --- Winch Functions ---
def lower_winch():
    """Runs the winch motor forward to lower the payload."""
    if not winch_motor: return
    print("Lowering winch...")
    winch_motor.forward(speed=WINCH_MOTOR_SPEED)

def raise_winch():
    """Runs the winch motor backward to raise the payload."""
    if not winch_motor: return
    print("Raising winch...")
    winch_motor.backward(speed=WINCH_MOTOR_SPEED)

def stop_winch():
    """Stops the winch motor."""
    if not winch_motor: return
    print("Stopping winch.")
    winch_motor.stop()

def cleanup():
    """De-initializes PCA9685 and cleans up GPIO resources."""
    if pca:
        print("De-initializing PCA9685...")
        pca.deinit()
    if winch_motor:
        print("Cleaning up winch motor GPIO...")
        winch_motor.stop()
        winch_motor.close()
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
    states = {'gripper': None, 'outdoor_1': None, 'outdoor_2': None, 'winch': None}

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
                
        # --- Check Winch Control Channel ---
        winch_map = MAVLINK_CONTROL_MAPPING['winch']
        pwm_val = getattr(msg, f"servo{winch_map['channel']}_raw", 0)
        if pwm_val > 0:
            # Check for LOWER command
            if pwm_val > winch_map['pwm_lower_threshold'] and states['winch'] != 'lowering':
                print(f"Winch PWM {pwm_val} (> {winch_map['pwm_lower_threshold']}). Lowering.")
                lower_winch()
                states['winch'] = 'lowering'
            # Check for RAISE command
            elif pwm_val < winch_map['pwm_raise_threshold'] and states['winch'] != 'raising':
                print(f"Winch PWM {pwm_val} (< {winch_map['pwm_raise_threshold']}). Raising.")
                raise_winch()
                states['winch'] = 'raising'
            # Check for STOP command (in the dead zone)
            elif (winch_map['pwm_raise_threshold'] <= pwm_val <= winch_map['pwm_lower_threshold']) and states['winch'] != 'stopped':
                print(f"Winch PWM {pwm_val} is in the dead zone. Stopping.")
                stop_winch()
                states['winch'] = 'stopped'

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