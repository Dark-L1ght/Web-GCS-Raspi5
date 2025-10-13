import serial
import time
import json
import servo_control

# --- CONFIGURATION ---
SERIAL_PORT = '/dev/ttyACM0' # Common name for Arduino Nano, could also be /dev/ttyUSB0
BAUD_RATE = 115200

# --- Global state to hold the latest lidar data ---
latest_lidar_data = {
    "d1": -1,
    "d2": -1,
    "d3": -1,
    "d4": -1
}

def handle_lidar_data(json_string):
    """Parses a JSON string and updates the global lidar data state."""
    global latest_lidar_data
    try:
        data = json.loads(json_string)
        # You could add more validation here if needed
        latest_lidar_data.update(data)
        
        # For now, we will just print it. In your main program, you would
        # access the `latest_lidar_data` dictionary.
        print(f"Lidar Update -> F: {data.get('d1', -1):>4}cm, R: {data.get('d2', -1):>4}cm, B: {data.get('d3', -1):>4}cm, L: {data.get('d4', -1):>4}cm")

    except json.JSONDecodeError:
        print(f"Warning: Received a malformed JSON string: {json_string}")

def handle_button_command(command):
    """Maps a button command string to the correct servo_control function."""
    print(f"Button Command Received: '{command}'")
    if command == "GRIPPER_TOGGLE":
        servo_control.toggle_gripper()
    elif command == "OUTDOOR_1_TOGGLE":
        servo_control.toggle_package_outdoor_1()
    elif command == "OUTDOOR_2_TOGGLE":
        servo_control.toggle_package_outdoor_2()
    else:
        # This can happen if the first part of a JSON string is read alone
        pass

def main():
    """
    Sets up servo control and listens for BOTH button and lidar data
    from the Arduino.
    """
    print("--- Unified Device Listener (Buttons & Lidars) ---")
    
    servo_control.setup()
    
    print("Setting initial servo states...")
    servo_control.close_gripper()
    servo_control.hold_package_outdoor_1()
    servo_control.hold_package_outdoor_2()
    print("Initial state: Gripper CLOSED, Outdoor Drops in HOLD position.")
    
    while True:
        try:
            print(f"Attempting to connect to Arduino on {SERIAL_PORT}...")
            with serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1) as ser:
                print("Successfully connected to Arduino. Listening for all data...")
                
                while True:
                    line = ser.readline()
                    if line:
                        message = line.decode('utf-8').strip()
                        
                        if not message:
                            continue
                        
                        # --- Smart Message Handling ---
                        # If the message starts with '{', it's JSON from the lidars.
                        if message.startswith('{'):
                            handle_lidar_data(message)
                        # Otherwise, it's a command from a button press.
                        else:
                            handle_button_command(message)

        except serial.SerialException:
            print(f"Error: Could not open serial port {SERIAL_PORT}. Retrying in 5 seconds...")
            time.sleep(5)
        except KeyboardInterrupt:
            print("\nKeyboard interrupt received. Shutting down.")
            break
        except Exception as e:
            print(f"An unexpected error occurred: {e}")
            break

if __name__ == '__main__':
    try:
        main()
    finally:
        print("\nCleaning up GPIO resources...")
        servo_control.cleanup()
        print("Cleanup complete. Program finished.")