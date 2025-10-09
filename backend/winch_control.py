import time
import datetime
import csv
from gpiozero import Motor, Device
from pynput import keyboard

# To prevent noisy GPIO warnings
Device.pin_factory = None

# =======================================================================
# PIN ASSIGNMENTS (BCM numbering)
# =======================================================================
MOTOR_A_IN1_PIN = 17  # IN1 (Connect to L298N IN1 or IN3)
MOTOR_A_IN2_PIN = 27  # IN2 (Connect to L298N IN2 or IN4)
MOTOR_A_ENABLE_PIN = 12 # ENA (Connect to L298N ENA or ENB)

# =======================================================================
# CONFIGURATION
# =======================================================================
MOTOR_SPEED = 1.0 # Speed from 0.0 to 1.0

# --- Global motor object ---
winch_motor = None

# =======================================================================
# CORE WINCH FUNCTIONS
# =======================================================================

def setup():
    """Initializes the motor object for the winch."""
    global winch_motor
    print("Setting up winch motor...")
    try:
        winch_motor = Motor(forward=MOTOR_A_IN1_PIN, backward=MOTOR_A_IN2_PIN, enable=MOTOR_A_ENABLE_PIN)
        print("Winch motor setup complete.")
    except Exception as e:
        print(f"ERROR: Could not initialize winch motor. Is pigpiod running? Error: {e}")
        # Exit if we can't control the motor, as it's critical
        exit()


def lower_winch(duration_sec):
    """Runs the winch motor forward to lower the magnet."""
    if not winch_motor:
        print("ERROR: Winch not set up.")
        return
    print(f"Lowering winch for {duration_sec} seconds...")
    winch_motor.forward(speed=MOTOR_SPEED)
    time.sleep(duration_sec)
    winch_motor.stop()
    print("Winch stopped.")

def raise_winch(duration_sec):
    """Runs the winch motor backward to raise the magnet."""
    if not winch_motor:
        print("ERROR: Winch not set up.")
        return
    print(f"Raising winch for {duration_sec} seconds...")
    winch_motor.backward(speed=MOTOR_SPEED)
    time.sleep(duration_sec)
    winch_motor.stop()
    print("Winch stopped.")

def cleanup():
    """Stops the motor and cleans up GPIO resources."""
    if winch_motor:
        print("Cleaning up winch motor GPIO...")
        winch_motor.stop()
        winch_motor.close()
    print("Winch cleanup complete.")


# =======================================================================
# INDEPENDENT TESTING SCRIPT (using your hotkey example)
# =======================================================================
if __name__ == '__main__':
    def on_press(key):
        try:
            if key.char == 'w':
                print("Manual Override: Raising winch...")
                winch_motor.backward(speed=MOTOR_SPEED)
            elif key.char == 's':
                print("Manual Override: Lowering winch...")
                winch_motor.forward(speed=MOTOR_SPEED)
        except AttributeError:
            pass

    def on_release(key):
        try:
            if key.char in ['w', 's']:
                print("Key released. Stopping winch.")
                winch_motor.stop()
        except AttributeError:
            pass
        if key == keyboard.Key.esc:
            return False # Stop listener

    try:
        setup()
        print("\n--- Winch Motor Test Mode ---")
        print("Controls: [W] Raise, [S] Lower")
        print("Hold down keys to move. Release to stop.")
        print("Press [Esc] to quit.")
        
        listener = keyboard.Listener(on_press=on_press, on_release=on_release)
        listener.start()
        listener.join()
        
    finally:
        print("\nExiting test mode.")
        cleanup()