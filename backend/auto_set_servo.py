import time
import board
import json
import sys
import os
from adafruit_pca9685 import PCA9685
from adafruit_motor import servo
from pymavlink import mavutil
from gpiozero import Motor, Device

# Use the default pin factory and prevent noisy GPIO warnings
# This matches your original winch_control.py
Device.pin_factory = None

# For keyboard testing, pynput is required.
# Install it with: pip install pynput
try:
    from pynput import keyboard
except ImportError:
    print("Warning: 'pynput' library not found. The keyboard test function will not work.")
    print("Install it with: 'pip install pynput'")
    keyboard = None


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

# --- Winch State Configuration ---
WINCH_STATE_FILE = 'winch_state.json'
WINCH_LOWER_DURATION = 3.0  # seconds to lower from 'stay' to 'low'
WINCH_RAISE_DURATION = 1.5  # seconds to raise from 'stay' to 'drop'

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
        # PWM > 1600 triggers 'low' state
        # PWM < 1400 triggers 'drop' state
        # PWM between 1400-1600 triggers 'stay' state
        'pwm_lower_threshold': 1600,
        'pwm_raise_threshold': 1400
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
        print(" -> Initializing winch motor...")
        winch_motor = Motor(
            forward=WINCH_MOTOR_PINS['in1'],
            backward=WINCH_MOTOR_PINS['in2'],
            enable=WINCH_MOTOR_PINS['enable']
        )
        print("✅ Winch motor initialized successfully (simulation mode).")
    except Exception as e:
        print(f"❌ ERROR: Failed to initialize winch motor: {e}")
        print("   (Check GPIO connections and permissions).")
        return False

    print("✅ All hardware initialized.")
    return True

# --- Servo Functions (omitted for brevity, they are unchanged from your original) ---
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


# --- Winch Low-Level Functions ---
def lower_winch():
    """Runs the winch motor forward to lower the payload."""
    if not winch_motor:
        print("SIM: Winch motor lowering.")
        return
    winch_motor.forward(speed=WINCH_MOTOR_SPEED)

def raise_winch():
    """Runs the winch motor backward to raise the payload."""
    if not winch_motor:
        print("SIM: Winch motor raising.")
        return
    winch_motor.backward(speed=WINCH_MOTOR_SPEED)

def stop_winch():
    """Stops the winch motor."""
    if not winch_motor:
        print("SIM: Winch motor stopped.")
        return
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

# --- Winch State Management ---
def load_winch_state():
    """Loads the winch state from the JSON file."""
    if not os.path.exists(WINCH_STATE_FILE):
        print(f"State file not found. Creating '{WINCH_STATE_FILE}' with default state 'stay'.")
        save_winch_state('stay')
        return 'stay'
    try:
        with open(WINCH_STATE_FILE, 'r') as f:
            state_data = json.load(f)
            return state_data.get('current_state', 'stay')
    except (json.JSONDecodeError, IOError) as e:
        print(f"Warning: Could not read state file: {e}. Assuming 'stay' state.")
        return 'stay'

def save_winch_state(new_state):
    """Saves the new winch state to the JSON file."""
    try:
        with open(WINCH_STATE_FILE, 'w') as f:
            json.dump({'current_state': new_state}, f)
    except IOError as e:
        print(f"ERROR: Could not write to state file: {e}")

def set_winch_state(target_state):
    """
    Manages the winch state machine. Always returns to 'stay' before moving to a new state.
    Valid states: 'stay', 'low', 'drop'.
    """
    current_state = load_winch_state()
    print(f"Request to move winch to '{target_state}' state. Current state is '{current_state}'.")

    if current_state == target_state:
        print(f"Winch is already in the '{target_state}' state. No action taken.")
        return

    # --- Step 1: Return to 'stay' position from the current state ---
    if current_state == 'low':
        print(f"Returning from 'low' to 'stay' (raising for {WINCH_LOWER_DURATION}s)...")
        raise_winch()
        time.sleep(WINCH_LOWER_DURATION)
        stop_winch()
    elif current_state == 'drop':
        print(f"Returning from 'drop' to 'stay' (lowering for {WINCH_RAISE_DURATION}s)...")
        lower_winch()
        time.sleep(WINCH_RAISE_DURATION)
        stop_winch()

    print("Winch is now at 'stay' position.")

    # --- Step 2: Move from 'stay' to the target state ---
    if target_state == 'low':
        print(f"Moving from 'stay' to 'low' (lowering for {WINCH_LOWER_DURATION}s)...")
        lower_winch()
        time.sleep(WINCH_LOWER_DURATION)
        stop_winch()
        print("✅ Winch is now in 'low' state.")
    elif target_state == 'drop':
        print(f"Moving from 'stay' to 'drop' (raising for {WINCH_RAISE_DURATION}s)...")
        raise_winch()
        time.sleep(WINCH_RAISE_DURATION)
        stop_winch()
        print("✅ Winch is now in 'drop' state.")
    elif target_state == 'stay':
        print("✅ Winch is now in 'stay' state.")

    # --- Step 3: Save the new state ---
    save_winch_state(target_state)


# =================================================================================
# --- 4. MAVLINK LISTENER & KEYBOARD TESTER ---
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

        # --- Check Gripper Channel (Unchanged) ---
        gripper_map = MAVLINK_CONTROL_MAPPING['gripper']
        pwm_val = getattr(msg, f"servo{gripper_map['channel']}_raw", 0)
        if pwm_val > 0:
            if pwm_val > gripper_map['pwm_threshold'] and states['gripper'] != 'open':
                open_gripper()
                states['gripper'] = 'open'
            elif pwm_val < gripper_map['pwm_threshold'] and states['gripper'] != 'closed':
                close_gripper()
                states['gripper'] = 'closed'
        
        # --- Other Servo Channels (Unchanged, omitted for brevity) ---
        # ...

        # --- Check Winch Control Channel (UPDATED LOGIC) ---
        winch_map = MAVLINK_CONTROL_MAPPING['winch']
        pwm_val = getattr(msg, f"servo{winch_map['channel']}_raw", 0)
        if pwm_val > 0:
            # Check for LOWER command
            if pwm_val > winch_map['pwm_lower_threshold'] and states['winch'] != 'low':
                print(f"Winch PWM {pwm_val} -> Commanding 'low' state.")
                set_winch_state('low')
                states['winch'] = 'low'
            # Check for RAISE/DROP command
            elif pwm_val < winch_map['pwm_raise_threshold'] and states['winch'] != 'drop':
                print(f"Winch PWM {pwm_val} -> Commanding 'drop' state.")
                set_winch_state('drop')
                states['winch'] = 'drop'
            # Check for STAY command (in the dead zone)
            elif (winch_map['pwm_raise_threshold'] <= pwm_val <= winch_map['pwm_lower_threshold']) and states['winch'] != 'stay':
                print(f"Winch PWM {pwm_val} -> Commanding 'stay' state.")
                set_winch_state('stay')
                states['winch'] = 'stay'

def test_with_keyboard():
    """Provides a keyboard interface to test the winch state machine."""
    if not keyboard:
        print("ERROR: 'pynput' library is required for keyboard test mode.")
        return

    print("\n--- Winch Keyboard Test Mode ---")
    print("Press the following keys to change the winch state:")
    print("  'l' : Move to LOW state")
    print("  'd' : Move to DROP state")
    print("  's' : Move to STAY state (return to home)")
    print("  'q' or 'Esc' : Quit")
    print("---------------------------------")

    def on_press(key):
        try:
            char = key.char
            if char == 'l':
                set_winch_state('low')
            elif char == 'd':
                set_winch_state('drop')
            elif char == 's':
                set_winch_state('stay')
            elif char == 'q':
                print("Quit key pressed. Exiting...")
                return False # Stop listener
        except AttributeError:
            if key == keyboard.Key.esc:
                print("Escape key pressed. Exiting...")
                return False # Stop listener

    with keyboard.Listener(on_press=on_press) as listener:
        listener.join()


# =================================================================================
# --- 5. MAIN EXECUTION BLOCK ---
# =================================================================================

if __name__ == '__main__':
    mode = 'mavlink'
    if len(sys.argv) > 1 and sys.argv[1].lower() == '--test':
        mode = 'test'

    try:
        if setup():
            if mode == 'mavlink':
                mavlink_listener()
            elif mode == 'test':
                test_with_keyboard()
    except KeyboardInterrupt:
        print("\nProgram interrupted by user.")
    except Exception as e:
        print(f"\nAn unexpected error occurred: {e}")
    finally:
        cleanup()