import RPi.GPIO as GPIO
import time

# --- Configuration ---
# The GPIO pin connected to the servo's signal wire.
SERVO_PIN = 18

# The angle values for the open and closed positions of your gripper.
# Tune these values to match your gripper's physical limits.
GRIPPER_OPEN_ANGLE = 55
GRIPPER_CLOSE_ANGLE = 115

# --- Global Variables ---
# We use a global variable for the PWM object so it can be accessed by all functions.
pwm = None

# --- Functions ---

def setup():
    """
    Initializes the GPIO pin for servo control.
    This must be called once when your main script starts.
    """
    global pwm
    print("Initializing GPIO for servo control...")
    GPIO.setwarnings(False) # Disable warnings
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(SERVO_PIN, GPIO.OUT)
    
    # Set up PWM on the servo pin with a 50Hz frequency.
    pwm = GPIO.PWM(SERVO_PIN, 50)
    pwm.start(0) # Start with no signal
    print("Servo control initialized.")

def _set_angle(angle):
    """
    Internal function to move the servo to a specific angle.
    """
    if not pwm:
        print("Error: Servo not initialized. Call setup() first.")
        return
        
    # Calculate the duty cycle required for the given angle
    duty_cycle = (angle / 18) + 2
    
    # Send the signal
    GPIO.output(SERVO_PIN, True)
    pwm.ChangeDutyCycle(duty_cycle)
    
    # Wait for the servo to move
    time.sleep(1.0)
    
    # Stop sending the signal to prevent servo jitter
    GPIO.output(SERVO_PIN, False)
    pwm.ChangeDutyCycle(0)

def open_gripper():
    """Moves the servo to the 'open' position."""
    print(f"Opening gripper (Angle: {GRIPPER_OPEN_ANGLE})...")
    _set_angle(GRIPPER_OPEN_ANGLE)

def close_gripper():
    """Moves the servo to the 'close' position."""
    print(f"Closing gripper (Angle: {GRIPPER_CLOSE_ANGLE})...")
    _set_angle(GRIPPER_CLOSE_ANGLE)

def cleanup():
    """
    Cleans up the GPIO pins.
    This must be called once when your main script exits.
    """
    if pwm:
        print("Stopping servo PWM and cleaning up GPIO...")
        pwm.stop()
    GPIO.cleanup()
    print("GPIO cleanup complete.")