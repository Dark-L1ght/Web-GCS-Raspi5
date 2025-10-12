import time
import board
from adafruit_pca9685 import PCA9685
from adafruit_motor import servo

# --- Required for Hotkey Control on Linux/macOS ---
import sys
import tty
import termios

# --- MODIFIED: Expanded Configuration for ALL Servos ---
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

# --- Global Variables ---
pca = None
servos = {} # This will now store all initialized servo objects

# --- Functions ---

def setup():
    """Initializes the I2C connection and ALL configured servo motors."""
    global pca, servos
    print("Initializing I2C and PCA9685 for all servo control...")
    
    i2c = board.I2C()
    pca = PCA9685(i2c)
    pca.frequency = 50

    # MODIFIED: Loop through all configurations to initialize every servo
    for key, config in SERVO_CONFIG.items():
        print(f"Initializing servos for: {config['name']}...")
        if 'channels' in config: # This is a multi-servo setup like the gripper
            for i, channel_num in enumerate(config['channels']):
                servo_key = f"{key}_{i}"
                servos[servo_key] = servo.Servo(pca.channels[channel_num])
        elif 'channel' in config: # This is a single-servo setup
            servo_key = key
            servos[servo_key] = servo.Servo(pca.channels[config['channel']])
    
    # NEW: Set outdoor drop servos to their 'hold' position initially
    servos['outdoor_drop_1'].angle = SERVO_CONFIG['outdoor_drop_1']['hold_angle']
    servos['outdoor_drop_2'].angle = SERVO_CONFIG['outdoor_drop_2']['hold_angle']
    time.sleep(1.0)
    servos['outdoor_drop_1'].angle = None # De-energize
    servos['outdoor_drop_2'].angle = None # De-energize
    
    print("All servos have been initialized.")

def _set_all_servo_angles_fast(servo_objects, target_angles):
    """Internal function to move multiple servos to target angles at maximum speed."""
    if not pca:
        print("Error: Servos not initialized.")
        return
        
    for servo_obj, angle in zip(servo_objects, target_angles):
        if angle is not None:
            servo_obj.angle = angle
    
    time.sleep(1.0) # Give servos time to move
    
    # De-energize servos to save power and reduce heat
    for servo_obj in servo_objects:
        servo_obj.angle = None

def open_gripper():
    """Moves the 4-servo lower gripper to the 'open' position (FAST)."""
    config = SERVO_CONFIG['lower_gripper']
    print(f"Opening {config['name']}...")
    servo_keys = [f"lower_gripper_{i}" for i in range(len(config['channels']))]
    servo_objects = [servos[key] for key in servo_keys]
    _set_all_servo_angles_fast(servo_objects, config['open_angles'])
    print(f"{config['name']} is OPEN.")

def close_gripper():
    """MODIFIED: Moves the 4-servo lower gripper to the 'close' position (FAST)."""
    config = SERVO_CONFIG['lower_gripper']
    print(f"Closing {config['name']}...")
    servo_keys = [f"lower_gripper_{i}" for i in range(len(config['channels']))]
    servo_objects = [servos[key] for key in servo_keys]
    _set_all_servo_angles_fast(servo_objects, config['close_angles'])
    print(f"{config['name']} is CLOSED.")

# --- NEW FUNCTIONS for Outdoor Drops ---

def drop_package_outdoor_1():
    """Controls the servo for the first outdoor drop."""
    config = SERVO_CONFIG['outdoor_drop_1']
    print(f"Actuating {config['name']}...")
    servo_obj = servos['outdoor_drop_1']
    
    servo_obj.angle = config['drop_angle']
    servo_obj.angle = None # De-energize
    print(f"{config['name']} sequence complete.")

def drop_package_outdoor_2():
    """Controls the servo for the second outdoor drop."""
    config = SERVO_CONFIG['outdoor_drop_2']
    print(f"Actuating {config['name']}...")
    servo_obj = servos['outdoor_drop_2']
    
    servo_obj.angle = config['drop_angle']
    servo_obj.angle = None
    print(f"{config['name']} sequence complete.")

# --- NEW FUNCTIONS for Resetting Outdoor Drops ---

def hold_package_outdoor_1():
    """Explicitly moves the first outdoor drop servo to its HOLD position."""
    config = SERVO_CONFIG['outdoor_drop_1']
    print(f"Resetting {config['name']} to HOLD position...")
    servo_obj = servos['outdoor_drop_1']
    
    servo_obj.angle = config['hold_angle']
    time.sleep(1.0)
    servo_obj.angle = None # De-energize
    print(f"{config['name']} is now in HOLD position.")

def hold_package_outdoor_2():
    """Explicitly moves the second outdoor drop servo to its HOLD position."""
    config = SERVO_CONFIG['outdoor_drop_2']
    print(f"Resetting {config['name']} to HOLD position...")
    servo_obj = servos['outdoor_drop_2']
    
    servo_obj.angle = config['hold_angle']
    time.sleep(1.0)
    servo_obj.angle = None
    print(f"{config['name']} is now in HOLD position.")


def cleanup():
    """De-initializes the PCA9685 board."""
    if pca:
        print("De-initializing PCA9685...")
        pca.deinit()
    print("Cleanup complete.")

def getch():
    """Gets a single character from standard input on Unix-like systems."""
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setcbreak(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

# --- Main Execution with Hotkey Control ---
if __name__ == '__main__':
    try:
        setup()
        
        # MODIFIED: Updated help text with new hotkeys
        print("\n--- Servo Hotkey Control ---")
        print("  c: Close Gripper (Fast)")
        print("  o: Open Gripper (Fast)")
        print("  1: Open Outdoor Drop 1")
        print("  2: Open Outdoor Drop 2")
        print("  8: Reset Outdoor Drop 1 to HOLD position")
        print("  9: Reset Outdoor Drop 2 to HOLD position")
        print("  q: Quit Program")
        print("----------------------------")
        print("Waiting for key press...")

        while True:
            char = getch()
            if char.lower() == 'c':
                close_gripper()
                print("Waiting for key press...")
            elif char.lower() == 'o':
                open_gripper()
                print("Waiting for key press...")
            elif char == '1':
                drop_package_outdoor_1()
                print("Waiting for key press...")
            elif char == '2':
                drop_package_outdoor_2()
                print("Waiting for key press...")
            # MODIFIED: Added elif blocks for the new hotkeys
            elif char == '8':
                hold_package_outdoor_1()
                print("Waiting for key press...")
            elif char == '9':
                hold_package_outdoor_2()
                print("Waiting for key press...")
            elif char.lower() == 'q':
                print("Quitting program.")
                break
            
    except KeyboardInterrupt:
        print("\nProgram interrupted by user (Ctrl+C).")
    finally:
        cleanup()