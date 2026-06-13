"""Servo driver library for the King Phoenix lower gripper and outdoor drops.

Imported by mission scripts. To run as a standalone test tool, use:
    python backend/utils/servo_test.py
"""

import time

import board
from adafruit_motor import servo
from adafruit_pca9685 import PCA9685

# --- Configuration for ALL Servos ---
SERVO_CONFIG = {
    "lower_gripper": {
        "name": "LOWER GRIPPER",
        "channels": [0, 1, 2, 3],
        "open_angles": [0, 0, 0, 0],
        "close_angles": [130, 130, 130, 130],
    },
    "outdoor_drop_1": {
        "name": "OUTDOOR DROP 1",
        "channel": 4,
        "hold_angle": 90,
        "drop_angle": 0,
    },
    "outdoor_drop_2": {
        "name": "OUTDOOR DROP 2",
        "channel": 5,
        "hold_angle": 0,
        "drop_angle": 90,
    },
}

# --- Global Variables ---
pca = None
servos = {}
gripper_is_open = False
outdoor_1_is_dropped = False
outdoor_2_is_dropped = False


# --- Core Functions ---


def setup():
    """Initializes the I2C connection and ALL configured servo motors."""
    global pca, servos
    print("Initializing I2C and PCA9685 for all servo control...")

    i2c = board.I2C()
    pca = PCA9685(i2c)
    pca.frequency = 50

    for key, config in SERVO_CONFIG.items():
        print(f"  Initializing: {config['name']}...")
        if "channels" in config:
            for i, channel_num in enumerate(config["channels"]):
                servo_key = f"{key}_{i}"
                servos[servo_key] = servo.Servo(pca.channels[channel_num])
        elif "channel" in config:
            servos[key] = servo.Servo(pca.channels[config["channel"]])

    # Set outdoor drops to hold position initially, then de-energize
    _set_angle_safe(
        servos["outdoor_drop_1"], SERVO_CONFIG["outdoor_drop_1"]["hold_angle"]
    )
    _set_angle_safe(
        servos["outdoor_drop_2"], SERVO_CONFIG["outdoor_drop_2"]["hold_angle"]
    )
    time.sleep(1.0)
    _deenergize([servos["outdoor_drop_1"], servos["outdoor_drop_2"]])

    print("All servos initialized.")


def _set_angle_safe(servo_obj, angle):
    """Set a single servo angle (no-op if servo_obj is None)."""
    if servo_obj and angle is not None:
        servo_obj.angle = angle


def _deenergize(servo_objects):
    """De-energize servos to save power and reduce heat."""
    for s in servo_objects:
        if s:
            s.angle = None


def _set_all_servo_angles_fast(servo_objects, target_angles):
    """Internal function to move multiple servos to target angles at maximum speed."""
    if not pca:
        print("Error: Servos not initialized.")
        return

    for servo_obj, angle in zip(servo_objects, target_angles):
        _set_angle_safe(servo_obj, angle)

    time.sleep(1.0)
    _deenergize(servo_objects)


def open_gripper():
    """Moves the 4-servo lower gripper to the 'open' position."""
    global gripper_is_open
    config = SERVO_CONFIG["lower_gripper"]
    print(f"Opening {config['name']}...")
    servo_keys = [f"lower_gripper_{i}" for i in range(len(config["channels"]))]
    servo_objects = [servos[key] for key in servo_keys]
    _set_all_servo_angles_fast(servo_objects, config["open_angles"])
    print(f"{config['name']} is OPEN.")
    gripper_is_open = True


def close_gripper():
    """Moves the 4-servo lower gripper to the 'close' position."""
    global gripper_is_open
    config = SERVO_CONFIG["lower_gripper"]
    print(f"Closing {config['name']}...")
    servo_keys = [f"lower_gripper_{i}" for i in range(len(config["channels"]))]
    servo_objects = [servos[key] for key in servo_keys]
    _set_all_servo_angles_fast(servo_objects, config["close_angles"])
    print(f"{config['name']} is CLOSED.")
    gripper_is_open = False


def toggle_gripper():
    """Toggles the gripper between open and closed states."""
    if gripper_is_open:
        close_gripper()
    else:
        open_gripper()


# --- Outdoor Drop Functions ---


def drop_package_outdoor_1():
    """Moves the first outdoor servo to its DROP position."""
    global outdoor_1_is_dropped
    config = SERVO_CONFIG["outdoor_drop_1"]
    print(f"Actuating {config['name']} to DROP...")
    _set_angle_safe(servos["outdoor_drop_1"], config["drop_angle"])
    time.sleep(1.0)
    _deenergize([servos["outdoor_drop_1"]])
    print(f"{config['name']} is in DROP position.")
    outdoor_1_is_dropped = True


def hold_package_outdoor_1():
    """Moves the first outdoor servo to its HOLD position."""
    global outdoor_1_is_dropped
    config = SERVO_CONFIG["outdoor_drop_1"]
    print(f"Resetting {config['name']} to HOLD...")
    _set_angle_safe(servos["outdoor_drop_1"], config["hold_angle"])
    time.sleep(1.0)
    _deenergize([servos["outdoor_drop_1"]])
    print(f"{config['name']} is in HOLD position.")
    outdoor_1_is_dropped = False


def toggle_package_outdoor_1():
    if outdoor_1_is_dropped:
        hold_package_outdoor_1()
    else:
        drop_package_outdoor_1()


def drop_package_outdoor_2():
    """Moves the second outdoor servo to its DROP position."""
    global outdoor_2_is_dropped
    config = SERVO_CONFIG["outdoor_drop_2"]
    print(f"Actuating {config['name']} to DROP...")
    _set_angle_safe(servos["outdoor_drop_2"], config["drop_angle"])
    time.sleep(1.0)
    _deenergize([servos["outdoor_drop_2"]])
    print(f"{config['name']} is in DROP position.")
    outdoor_2_is_dropped = True


def hold_package_outdoor_2():
    """Moves the second outdoor servo to its HOLD position."""
    global outdoor_2_is_dropped
    config = SERVO_CONFIG["outdoor_drop_2"]
    print(f"Resetting {config['name']} to HOLD...")
    _set_angle_safe(servos["outdoor_drop_2"], config["hold_angle"])
    time.sleep(1.0)
    _deenergize([servos["outdoor_drop_2"]])
    print(f"{config['name']} is in HOLD position.")
    outdoor_2_is_dropped = False


def toggle_package_outdoor_2():
    if outdoor_2_is_dropped:
        hold_package_outdoor_2()
    else:
        drop_package_outdoor_2()


def cleanup():
    """De-initializes the PCA9685 board."""
    if pca:
        print("De-initializing PCA9685...")
        pca.deinit()
    print("Cleanup complete.")
