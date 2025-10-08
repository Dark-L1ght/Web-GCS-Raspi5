import time
import board
from adafruit_pca9685 import PCA9685
from adafruit_motor import servo

# --- Required for Hotkey Control on Linux/macOS ---
import sys
import tty
import termios

# --- Configuration for ONE Gripper with FOUR Servos ---
GRIPPER_CONFIG = {
    'lower': {
        'name': 'LOWER GRIPPER',
        'channels': [0, 1, 2, 3],
        
        'open_angles':  [180, 180, 180, 180],
        'close_angles': [50, 50, 50, 50],
        
        # --- NEW: Parameters for controlling the closing speed ---
        # Adjust these values to make the closing action faster or slower.
        # Total time = steps * delay. E.g., 50 * 0.04 = 2 seconds.
        'slow_close_steps': 50,   # Number of small steps for the movement
        'slow_close_delay': 0.02  # Delay between each step in seconds
    }
}

# --- Global Variables ---
pca = None
servos = {}

# --- Functions ---

def setup():
    """Initializes the I2C connection and ALL servo motors."""
    global pca, servos
    print("Initializing I2C and PCA9685 for 4-servo gripper control...")
    
    i2c = board.I2C()
    pca = PCA9685(i2c)
    pca.frequency = 50

    config = GRIPPER_CONFIG['lower']
    print(f"Initializing servos for {config['name']} on channels {config['channels']}...")
    for i, channel_num in enumerate(config['channels']):
        servo_key = f"lower_{i}"
        servos[servo_key] = servo.Servo(pca.channels[channel_num])
    
    print("All servos for the lower gripper have been initialized.")

def _set_all_servo_angles_fast(servo_objects, target_angles):
    """Internal function to move servos to target angles at maximum speed."""
    if not pca:
        print("Error: Servos not initialized.")
        return
        
    for servo_obj, angle in zip(servo_objects, target_angles):
        if angle is not None:
            servo_obj.angle = angle
    
    time.sleep(1.0)
    
    for servo_obj in servo_objects:
        servo_obj.angle = None

def _set_all_servo_angles_slowly(servo_objects, start_angles, target_angles, steps, delay):
    """Internal function to move servos slowly using step-by-step interpolation."""
    if not pca:
        print("Error: Servos not initialized.")
        return
    
    # Loop through each step of the movement
    for i in range(steps + 1):
        fraction = i / steps  # Calculate the fraction of the movement completed (0.0 to 1.0)
        
        # For each servo, calculate its intermediate angle for the current step
        for j in range(len(servo_objects)):
            start = start_angles[j]
            target = target_angles[j]
            
            # Linear interpolation formula
            current_angle = start + (target - start) * fraction
            servo_objects[j].angle = int(current_angle)
            
        time.sleep(delay) # Pause briefly at this step

    # De-energize servos at the final position
    for servo_obj in servo_objects:
        servo_obj.angle = None

def open_gripper():
    """Moves the 4-servo lower gripper to the 'open' position (FAST)."""
    config = GRIPPER_CONFIG['lower']
    print(f"Opening {config['name']}...")
    servo_objects = [servos[f"lower_{i}"] for i in range(len(config['channels']))]
    _set_all_servo_angles_fast(servo_objects, config['open_angles'])

def close_gripper():
    """Moves the 4-servo lower gripper to the 'close' position (SLOWLY)."""
    config = GRIPPER_CONFIG['lower']
    print(f"Closing {config['name']} slowly...")
    servo_objects = [servos[f"lower_{i}"] for i in range(len(config['channels']))]
    
    # Call the new slow function, providing start/end angles and speed parameters
    _set_all_servo_angles_slowly(
        servo_objects,
        start_angles=config['open_angles'],
        target_angles=config['close_angles'],
        steps=config['slow_close_steps'],
        delay=config['slow_close_delay']
    )

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
        
        print("\n--- Gripper Hotkey Control ---")
        print("  c: Close Gripper (Slowly)")
        print("  o: Open Gripper (Fast)")
        print("  q: Quit Program")
        print("------------------------------")
        print("Waiting for key press...")

        while True:
            char = getch()
            if char.lower() == 'c':
                close_gripper()
                print("Waiting for key press...")
            elif char.lower() == 'o':
                open_gripper()
                print("Waiting for key press...")
            elif char.lower() == 'q':
                print("Quitting program.")
                break
            
    except KeyboardInterrupt:
        print("\nProgram interrupted by user (Ctrl+C).")
    finally:
        cleanup()