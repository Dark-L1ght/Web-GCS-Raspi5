import time
import board
from adafruit_pca9685 import PCA9685
from adafruit_motor import servo

# --- Configuration for TWO Servos ---
SERVO_LEFT_CHANNEL = 0
SERVO_RIGHT_CHANNEL = 1

# --- Gripper Angle Configuration ---
# Both servos will move to the same angle values.
GRIPPER_OPEN_ANGLE = 35
GRIPPER_CLOSE_ANGLE = 140

# --- Global Variables ---
pca = None
servo_left = None
servo_right = None

# --- Functions ---

def setup():
    """
    Initializes the I2C connection and BOTH servo motors on the PCA9685 board.
    """
    global pca, servo_left, servo_right
    print("Initializing I2C and PCA9685 for dual servo control...")
    
    i2c = board.I2C()
    pca = PCA9685(i2c)
    pca.frequency = 50

    # Create two servo objects, one for each channel
    servo_left = servo.Servo(pca.channels[SERVO_LEFT_CHANNEL])
    servo_right = servo.Servo(pca.channels[SERVO_RIGHT_CHANNEL])
    
    print(f"Servo control initialized on channels {SERVO_LEFT_CHANNEL} (L) and {SERVO_RIGHT_CHANNEL} (R).")

def _set_gripper_angles(left_angle, right_angle):
    """
    Internal function to move both servos to their respective angles.
    """
    if not pca:
        print("Error: Servos not initialized. Call setup() first.")
        return
        
    # Command both servos to move to their target angles
    servo_left.angle = left_angle
    servo_right.angle = right_angle
    

    # Wait for the servos to move
    time.sleep(1.0)
    
    # De-energize both servos to prevent jitter
    servo_left.angle = None
    servo_right.angle = None

def open_gripper():
    """Moves both servos to the 'open' position."""
    print(f"Opening gripper (Angle: {GRIPPER_OPEN_ANGLE})...")
    # Both servos move to the SAME open angle
    _set_gripper_angles(GRIPPER_OPEN_ANGLE + 20, GRIPPER_OPEN_ANGLE)

def close_gripper():
    """Moves both servos to the 'close' position."""
    print(f"Closing gripper (Angle: {GRIPPER_CLOSE_ANGLE})...")
    # Both servos move to the SAME close angle
    _set_gripper_angles(GRIPPER_CLOSE_ANGLE + 20, GRIPPER_CLOSE_ANGLE)

def cleanup():
    """
    De-initializes the PCA9685 board, stopping all PWM signals.
    """
    if pca:
        print("De-initializing PCA9685...")
        pca.deinit()
    print("Cleanup complete.")

# --- Main Execution Example ---
if __name__ == '__main__':
    try:
        setup()
        while True:
            open_gripper()
    except KeyboardInterrupt:
        print("\nProgram interrupted.")
    finally:
        cleanup()