#!/usr/bin/env python3
"""Standalone CLI utility for manually testing servo mechanisms.

Run this directly on the Raspberry Pi to debug servo angles without running
a full mission script.

Keys:
    c : Close gripper
    o : Open gripper
    1 : Drop outdoor 1
    2 : Drop outdoor 2
    8 : Hold outdoor 1
    9 : Hold outdoor 2
    q : Quit
"""

import sys
import termios
import tty

from backend.hardware import servo_control


def getch():
    """Get a single character from stdin on Unix-like systems."""
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setcbreak(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch


def main():
    try:
        servo_control.setup()

        print("\n--- Servo Hotkey Control ---")
        print("  c: Close Gripper")
        print("  o: Open Gripper")
        print("  1: Drop Outdoor 1")
        print("  2: Drop Outdoor 2")
        print("  8: Hold Outdoor 1")
        print("  9: Hold Outdoor 2")
        print("  q: Quit")
        print("----------------------------")

        while True:
            char = getch()
            if char.lower() == "c":
                servo_control.close_gripper()
            elif char.lower() == "o":
                servo_control.open_gripper()
            elif char == "1":
                servo_control.drop_package_outdoor_1()
            elif char == "2":
                servo_control.drop_package_outdoor_2()
            elif char == "8":
                servo_control.hold_package_outdoor_1()
            elif char == "9":
                servo_control.hold_package_outdoor_2()
            elif char.lower() == "q":
                print("Quitting.")
                break
    except KeyboardInterrupt:
        print("\nInterrupted by user.")
    finally:
        servo_control.cleanup()


if __name__ == "__main__":
    main()
