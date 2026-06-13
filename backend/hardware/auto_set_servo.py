import time
import board
import json
import sys
import os
from adafruit_pca9685 import PCA9685
from adafruit_motor import servo
from pymavlink import mavutil
from gpiozero import Motor, Device
import threading
import serial

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

SERIAL_PORT = '/dev/ttyUSB0' # This is where your Arduino connects
BAUD_RATE = 115200

# --- Servo Configuration ---
SERVO_CONFIG = {
    'lower_gripper': {
        'name': 'LOWER GRIPPER',
        'channels': [0, 1, 2, 3],
        'open_angles':  [0, 0, 0, 0],
        'close_angles': [130, 130, 130, 180], # Note the 180 for the last servo
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
        'drop_angle': 120,
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
WINCH_LOWER_DURATION = 3  # seconds to lower from 'stay' to 'low'
WINCH_RAISE_DURATION = 3  # seconds to raise from 'stay' to 'drop'

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

mechanism_states = {
    'gripper': 'closed',  # Assume closed on startup
    'outdoor_1': 'held',    # Assume held on startup
    'outdoor_2': 'held',    # Assume held on startup
    'winch': 'stay'       # Assume stay on startup
}

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

# MODIFIED FUNCTION
def close_gripper(silent=False):
    """
    Closes the gripper using a multi-step sequence to prevent a servo from
    blocking the camera view during the initial phase of closing.
    """
    config = SERVO_CONFIG['lower_gripper']
    if not silent: print(f"CLOSING {config['name']} (multi-step)...")
    if not pca: return

    try:
        # Identify the servo objects
        servo_keys = [f"lower_gripper_{i}" for i in range(len(config['channels']))]
        all_servos = [servos[key] for key in servo_keys]
        
        # The last servo in the list (channel 3) is the one that moves to 180°
        special_servo = all_servos[3]
        other_servos = all_servos[0:3]

        # Define angles for clarity
        intermediate_angle = 50
        special_servo_final_angle = config['close_angles'][3] # 180
        other_servos_final_angle = config['close_angles'][0] # 130

        # Step 1: Move the special servo to an intermediate position to clear camera view
        if not silent: print(f"  -> Moving servo 4 to intermediate position ({intermediate_angle}°)...")
        special_servo.angle = intermediate_angle
        time.sleep(0.5)  # Wait for it to move

        # Step 2: Move the other servos to their final position and the special one to its final
        if not silent: print(f"  -> Moving other servos to final position ({other_servos_final_angle}°)...")
        for servo_obj in other_servos:
            servo_obj.angle = other_servos_final_angle

        if not silent: print(f"  -> Moving servo 4 to final position ({special_servo_final_angle}°)...")
        special_servo.angle = special_servo_final_angle

        # Step 3: Wait for all movements to complete
        time.sleep(1.0)

        # Step 4: De-energize all servos
        if not silent: print("  -> De-energizing all gripper servos.")
        for servo_obj in all_servos:
            servo_obj.angle = None

    except Exception as e:
        print(f"ERROR: An error occurred during the multi-step gripper close: {e}")

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

def serial_listener():
    """Listens for commands from the Arduino over serial and toggles mechanisms."""
    global mechanism_states
    print(f"Attempting to connect to Arduino on {SERIAL_PORT}...")
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        print("✅ Serial connection to Arduino established.")
    except serial.SerialException as e:
        print(f"❌ ERROR: Could not open serial port {SERIAL_PORT}: {e}")
        print("   Physical button control will be disabled.")
        return

    while True:
        try:
            # Read a line from the Arduino, decode it, and strip whitespace
            line = ser.readline().decode('utf-8').strip()
            
            if line: # If we received a command
                print(f"[SERIAL] Received command: {line}")
                
                if line == "GRIPPER_TOGGLE":
                    if mechanism_states['gripper'] == 'closed':
                        open_gripper()
                        mechanism_states['gripper'] = 'open'
                    else:
                        close_gripper()
                        mechanism_states['gripper'] = 'closed'
                
                elif line == "OUTDOOR_1_TOGGLE":
                    if mechanism_states['outdoor_1'] == 'held':
                        drop_package_outdoor_1()
                        mechanism_states['outdoor_1'] = 'dropped'
                    else:
                        hold_package_outdoor_1()
                        mechanism_states['outdoor_1'] = 'held'

                elif line == "OUTDOOR_2_TOGGLE":
                    if mechanism_states['outdoor_2'] == 'held':
                        drop_package_outdoor_2()
                        mechanism_states['outdoor_2'] = 'dropped'
                    else:
                        hold_package_outdoor_2()
                        mechanism_states['outdoor_2'] = 'held'
        except serial.SerialException:
            print("ERROR: Arduino disconnected. Attempting to reconnect...")
            ser.close()
            time.sleep(3)
            try:
                ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
            except:
                pass # Suppress errors during reconnection attempts
        except Exception as e:
            print(f"An error occurred in the serial listener: {e}")
            time.sleep(1)

def mavlink_listener():
    """Listens for SERVO_OUTPUT_RAW messages and controls all mechanisms."""
    print(f"Connecting to MAVLink on '{MAVLINK_CONNECTION_STRING}'...")
    master = mavutil.mavlink_connection(MAVLINK_CONNECTION_STRING, wait_ready=True)
    master.wait_heartbeat()
    print("✅ MAVLink Heartbeat received! Listening for servo commands...")

    # State variables to avoid sending repeated commands
    last_command_states = {'gripper': None, 'outdoor_1': None, 'outdoor_2': None, 'winch': None}

    while True:
        msg = master.recv_match(type='SERVO_OUTPUT_RAW', blocking=True)
        if not msg: continue

        # --- Check Gripper Channel ---
        gripper_map = MAVLINK_CONTROL_MAPPING['gripper']
        pwm_val = getattr(msg, f"servo{gripper_map['channel']}_raw", 0)
        if pwm_val > 0:
            if pwm_val > gripper_map['pwm_threshold'] and last_command_states['gripper'] != 'open':
                open_gripper()
                mechanism_states['gripper'] = 'open' # Update global state
                last_command_states['gripper'] = 'open'
            elif pwm_val < gripper_map['pwm_threshold'] and last_command_states['gripper'] != 'closed':
                close_gripper()
                mechanism_states['gripper'] = 'closed' # Update global state
                last_command_states['gripper'] = 'closed'
        
        # --- Check Outdoor Drop 1 Channel ---
        outdoor_1_map = MAVLINK_CONTROL_MAPPING['outdoor_1']
        pwm_val = getattr(msg, f"servo{outdoor_1_map['channel']}_raw", 0)
        if pwm_val > 0:
            if pwm_val > outdoor_1_map['pwm_threshold'] and last_command_states['outdoor_1'] != 'dropped':
                drop_package_outdoor_1()
                mechanism_states['outdoor_1'] = 'dropped' # Update global state
                last_command_states['outdoor_1'] = 'dropped'
            elif pwm_val < outdoor_1_map['pwm_threshold'] and last_command_states['outdoor_1'] != 'held':
                hold_package_outdoor_1()
                mechanism_states['outdoor_1'] = 'held' # Update global state
                last_command_states['outdoor_1'] = 'held'

        outdoor_2_map = MAVLINK_CONTROL_MAPPING['outdoor_2']
        pwm_val = getattr(msg, f"servo{outdoor_2_map['channel']}_raw", 0)
        if pwm_val > 0:
            if pwm_val > outdoor_2_map['pwm_threshold'] and last_command_states['outdoor_2'] != 'dropped':
                drop_package_outdoor_2()
                mechanism_states['outdoor_2'] = 'dropped' # Update global state
                last_command_states['outdoor_2'] = 'dropped'
            elif pwm_val < outdoor_2_map['pwm_threshold'] and last_command_states['outdoor_2'] != 'held':
                hold_package_outdoor_2()
                mechanism_states['outdoor_2'] = 'held' # Update global state
                last_command_states['outdoor_2'] = 'held'

        # --- Check Winch Control Channel ---
        winch_map = MAVLINK_CONTROL_MAPPING['winch']
        pwm_val = getattr(msg, f"servo{winch_map['channel']}_raw", 0)
        if pwm_val > 0:
            # --- BUG 2 FIX: Replaced 'states' with 'last_command_states' ---
            if pwm_val > winch_map['pwm_lower_threshold'] and last_command_states['winch'] != 'low':
                print(f"Winch PWM {pwm_val} -> Commanding 'low' state.")
                set_winch_state('low')
                last_command_states['winch'] = 'low'
            elif pwm_val < winch_map['pwm_raise_threshold'] and last_command_states['winch'] != 'drop':
                print(f"Winch PWM {pwm_val} -> Commanding 'drop' state.")
                set_winch_state('drop')
                last_command_states['winch'] = 'drop'
            elif (winch_map['pwm_raise_threshold'] <= pwm_val <= winch_map['pwm_lower_threshold']) and last_command_states['winch'] != 'stay':
                print(f"Winch PWM {pwm_val} -> Commanding 'stay' state.")
                set_winch_state('stay')
                last_command_states['winch'] = 'stay'

def test_with_keyboard():
    """Provides a keyboard interface to test the winch state machine."""
    if not keyboard:
        print("ERROR: 'pynput' library is required for keyboard test mode.")
        return

    print("\n--- Mechanism Keyboard Test Mode ---")
    print("Press the following keys to test actions:")
    print("  'o' : OPEN gripper")
    print("  'c' : CLOSE gripper (multi-step)")
    print("  'l' : Move winch to LOW state")
    print("  'd' : Move winch to DROP state")
    print("  's' : Move winch to STAY state")
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
            elif char == 'o':
                open_gripper()
            elif char == 'c':
                close_gripper()
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
    mode = 'mavlink' # Default mode now runs both listeners
    if len(sys.argv) > 1 and sys.argv[1].lower() == '--test':
        mode = 'test'

    try:
        if setup():
            if mode == 'mavlink':
                # Create the MAVLink listener thread
                mavlink_thread = threading.Thread(target=mavlink_listener, daemon=True)
                
                # Create the Serial listener thread
                serial_thread = threading.Thread(target=serial_listener, daemon=True)
                
                # Start both threads
                print("Starting MAVLink and Serial listener threads...")
                mavlink_thread.start()
                serial_thread.start()
                
                # Keep the main program running while the threads do their work
                while True:
                    time.sleep(1)

            elif mode == 'test':
                test_with_keyboard()
    except KeyboardInterrupt:
        print("\nProgram interrupted by user.")
    except Exception as e:
        print(f"\nAn unexpected error occurred: {e}")
    finally:
        cleanup()
