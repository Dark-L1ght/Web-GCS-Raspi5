import serial
import time
import sys

# ==============================================================================
# --- Configuration ---
# ==============================================================================
# These should match the settings in your main controller and Arduino sketch.
ARDUINO_PORT = '/dev/ttyUSB0'
ARDUINO_BAUDRATE = 115200
RETRY_DELAY_SECONDS = 5

# ==============================================================================
# --- Test Script Main Logic ---
# ==============================================================================
def main():
    """
    Connects to the Arduino and continuously prints the received LiDAR data.
    """
    print(f"--- Arduino LiDAR Test Utility ---")
    print(f"Attempting to connect to {ARDUINO_PORT} at {ARDUINO_BAUDRATE} baud.")
    print("Press Ctrl+C to exit.")

    while True:
        ser = None
        try:
            # Establish a connection to the serial port
            ser = serial.Serial(ARDUINO_PORT, ARDUINO_BAUDRATE, timeout=1)
            print(f"\nSuccessfully connected to Arduino on {ARDUINO_PORT}.")
            print("Waiting for data...")

            while True:
                # Check if there's data waiting to be read
                if ser.in_waiting > 0:
                    try:
                        # Read a line from the serial port
                        line = ser.readline().decode('utf-8').strip()

                        # Skip empty lines that might be sent on connection start
                        if not line:
                            continue

                        # Parse the comma-separated data
                        parts = line.split(',')
                        
                        if len(parts) == 3:
                            front_cm = int(parts[0])
                            left_cm = int(parts[1])
                            right_cm = int(parts[2])

                            # Print the data in a clean, updating format
                            sys.stdout.write(
                                f"\rFront: {front_cm:4d} cm   |   Left: {left_cm:4d} cm   |   Right: {right_cm:4d} cm      "
                            )
                            sys.stdout.flush()

                        else:
                            # Handle cases where the data is not in the expected format
                            print(f"\n[Warning] Received malformed data: '{line}'")

                    except (ValueError, IndexError):
                        print(f"\n[Warning] Could not parse data line: '{line}'")
                    except UnicodeDecodeError:
                         print(f"\n[Warning] Unicode decode error on received data.")
                
                # Small delay to prevent maxing out the CPU
                time.sleep(0.01)

        except serial.SerialException:
            print(f"\n[Error] Could not connect to Arduino on {ARDUINO_PORT}.")
            print(f"Is it connected and is the port correct? Retrying in {RETRY_DELAY_SECONDS} seconds...")
            time.sleep(RETRY_DELAY_SECONDS)
        except KeyboardInterrupt:
            print("\nExiting program.")
            break
        finally:
            if ser and ser.is_open:
                ser.close()
                print(f"\nSerial port {ARDUINO_PORT} closed.")

if __name__ == "__main__":
    main()