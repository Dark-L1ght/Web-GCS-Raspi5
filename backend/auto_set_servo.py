import time
import board
from adafruit_pca9685 import PCA9685
from adafruit_motor import servo
from pymavlink import mavutil

# =================================================================================
# --- 1. CONFIGURATION ---
# =================================================================================

# --- Gripper Servo Configuration ---
# Set the PCA9685 channels for your two servos.
SERVO_LEFT_CHANNEL = 0
SERVO_RIGHT_CHANNEL = 1

# Set the angle values for the open and close positions.
# You may need to fine-tune these for your specific gripper.
GRIPPER_OPEN_ANGLE = 20
GRIPPER_CLOSE_ANGLE = 145

# --- MAVLink Connection Configuration ---
# This script connects to MAVProxy running locally on the Pi.
# Ensure MAVProxy is forwarding to this UDP port.
MAVLINK_CONNECTION_STRING = 'udp:127.0.0.1:14550'

# The ArduPilot servo output channel to monitor (e.g., AUX1 = 9, AUX2 = 10).
# This MUST match the "Ser No" in your Mission Planner `DO_SET_SERVO` command.
MAVLINK_SERVO_CHANNEL_TO_MONITOR = 9

# The PWM value that acts as a switch between open and close.
# Values above this will open; values below will close.
PWM_THRESHOLD = 1500

# =================================================================================
# --- 2. GLOBAL VARIABLES ---
# =================================================================================

pca = None
servo_left = None
servo_right = None

# =================================================================================
# --- 3. HARDWARE CONTROL FUNCTIONS ---
# =================================================================================

def setup():
    """Initializes the I2C connection and BOTH servo motors on the PCA9685 board."""
    global pca, servo_left, servo_right
    print("Initializing I2C and PCA9685 for dual servo control...")
    try:
        i2c = board.I2C()
        pca = PCA9685(i2c)
        pca.frequency = 50

        servo_left = servo.Servo(pca.channels[SERVO_LEFT_CHANNEL])
        servo_right = servo.Servo(pca.channels[SERVO_RIGHT_CHANNEL])
        
        print(f"✅ Servo control initialized on channels {SERVO_LEFT_CHANNEL} (L) and {SERVO_RIGHT_CHANNEL} (R).")
        return True
    except Exception as e:
        print(f"❌ ERROR: Failed to initialize hardware: {e}")
        print("   - Is I2C enabled? (sudo raspi-config)")
        print("   - Is the PCA9685 board wired correctly?")
        return False

def _set_gripper_angles(left_angle, right_angle):
    """Internal function to move both servos and then de-energize them to prevent jitter."""
    if not pca:
        print("Error: Servos not initialized. Call setup() first.")
        return
    try:
        servo_left.angle = left_angle
        servo_right.angle = right_angle
        time.sleep(1.0) # Wait for the servos to reach the position
        servo_left.angle = None # De-energize to prevent jitter and save power
        servo_right.angle = None
    except Exception as e:
        print(f"Error moving servos: {e}")

def open_gripper():
    """Moves both servos to the 'open' position."""
    print(f"OPENING gripper (Angle: {GRIPPER_OPEN_ANGLE})...")
    # Note: For some symmetrical grippers, one servo might need the opposite angle.
    # e.g., right_angle = 180 - GRIPPER_OPEN_ANGLE
    _set_gripper_angles(GRIPPER_OPEN_ANGLE + 20, GRIPPER_OPEN_ANGLE)

def close_gripper():
    """Moves both servos to the 'close' position."""
    print(f"CLOSING gripper (Angle: {GRIPPER_CLOSE_ANGLE})...")
    _set_gripper_angles(GRIPPER_CLOSE_ANGLE + 20, GRIPPER_CLOSE_ANGLE)

def cleanup():
    """De-initializes the PCA9685 board, stopping all PWM signals."""
    if pca:
        print("De-initializing PCA9685...")
        pca.deinit()
    print("Cleanup complete.")

# =================================================================================
# --- 4. MAVLINK LISTENER FUNCTION ---
# =================================================================================

def mavlink_listener():
    """Listens for MAVLink SERVO_OUTPUT_RAW messages and controls the gripper."""
    print(f"Connecting to MAVLink on '{MAVLINK_CONNECTION_STRING}'...")
    master = mavutil.mavlink_connection(MAVLINK_CONNECTION_STRING, wait_ready=True)
    master.wait_heartbeat()
    print("✅ MAVLink Heartbeat received! Listening for servo commands...")

    gripper_is_open = None  # Use a state variable to avoid repeated commands

    while True:
        # Wait for a SERVO_OUTPUT_RAW message
        msg = master.recv_match(type='SERVO_OUTPUT_RAW', blocking=True)
        if not msg:
            continue

        # Construct the attribute name we're looking for (e.g., 'servo9_raw')
        attribute_name = f'servo{MAVLINK_SERVO_CHANNEL_TO_MONITOR}_raw'

        # Check if the message object actually contains the data we need.
        # This is robust for different ArduPilot MAVLink dialect versions.
        if hasattr(msg, attribute_name):
            pwm_value = getattr(msg, attribute_name)
            
            # --- This is the main control logic ---
            if pwm_value > PWM_THRESHOLD and gripper_is_open is not True:
                print(f"Received PWM {pwm_value} (> {PWM_THRESHOLD}). Calling open_gripper().")
                open_gripper()
                gripper_is_open = True
            elif pwm_value < PWM_THRESHOLD and gripper_is_open is not False:
                # We check for pwm_value > 0 to ignore startup/uninitialized signals
                if pwm_value > 0:
                    print(f"Received PWM {pwm_value} (< {PWM_THRESHOLD}). Calling close_gripper().")
                    close_gripper()
                    gripper_is_open = False

# =================================================================================
# --- 5. MAIN EXECUTION BLOCK ---
# =================================================================================

if __name__ == '__main__':
    try:
        # First, initialize the hardware
        if setup():
            # If hardware is ready, start listening for MAVLink commands
            mavlink_listener()
    except KeyboardInterrupt:
        print("\nProgram interrupted by user.")
    except Exception as e:
        print(f"\nAn unexpected error occurred: {e}")
    finally:
        # This block will run no matter how the script exits
        cleanup()