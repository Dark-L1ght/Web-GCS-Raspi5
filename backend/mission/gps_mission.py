#!/usr/bin/env python3
"""Strategy 1: GPS + Vision mission execution.

Connects to the drone via MAVLink, receives waypoints from the GCS (or uses
defaults), and executes an 8-phase autonomous mission with precision landing
and vision-guided targeting.
"""

import json
import logging
import socket
import sys

from pymavlink import mavutil

from backend.config_loader import MISSION, NETWORK
from backend.hardware import servo_control, winch_control
from backend.mission.shared import (
    TAKEOFF_ALTITUDE,
    arm_and_takeoff,
    center_above_target,
    execute_precision_landing,
    handle_window_approach,
    land_normally,
    navigate_to_waypoint,
    send_control_command,
)

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
    datefmt="%H:%M:%S",
)
logger = logging.getLogger(__name__)

# Default mission waypoints (lat, lon, alt)
DEFAULT_WAYPOINTS = [
    (-7.8332912, 110.3842767, 1.1),  # 0: Logistic 1
    (-7.8333110, 110.3842802, 1.1),  # 1: Logistic 2
    (-7.8333401, 110.3842762, 1.1),  # 2: Barrel Drop
    (-7.8333536, 110.3842784, 1.6),  # 3: Before Exit
    (-7.8333700, 110.3842700, 1.6),  # 4: After Exit
    (-7.8333800, 110.3842600, 1.1),  # 5: Outdoor Drop 1
    (-7.8333900, 110.3842500, 1.1),  # 6: Outdoor Drop 2
    (-7.8334000, 110.3842400, 1.1),  # 7: Final Land
]

EXPECTED_WAYPOINT_COUNT = 8


def load_waypoints_from_args():
    """Parse waypoints from CLI args (JSON string) or fall back to defaults."""
    if len(sys.argv) > 1:
        try:
            waypoints_from_gcs = json.loads(sys.argv[1])
            if len(waypoints_from_gcs) == EXPECTED_WAYPOINT_COUNT:
                parsed = [
                    (wp["lat"], wp["lon"], wp["alt"]) for wp in waypoints_from_gcs
                ]
                logger.info("Using %d waypoints from GCS.", EXPECTED_WAYPOINT_COUNT)
                return parsed
            logger.error(
                "Expected %d waypoints, got %d. Using defaults.",
                EXPECTED_WAYPOINT_COUNT,
                len(waypoints_from_gcs),
            )
        except (json.JSONDecodeError, IndexError, KeyError) as e:
            logger.error("Failed to parse GCS waypoints: %s. Using defaults.", e)
    else:
        logger.info("No waypoints from GCS. Using defaults.")
    return list(DEFAULT_WAYPOINTS)


def setup_mavlink_connection():
    """Establish MAVLink connection and request telemetry streams."""
    conn = NETWORK["mavlink_connection"]
    baud = NETWORK["baud_rate"]
    master = mavutil.mavlink_connection(conn, baud=baud)
    master.wait_heartbeat()
    logger.info(
        "Heartbeat from system (system %s component %s)",
        master.target_system,
        master.target_component,
    )

    logger.info("Requesting GLOBAL_POSITION_INT stream at 10 Hz...")
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
        0,
        mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT,
        100_000,
        0,
        0,
        0,
        0,
        0,
    )
    return master


def setup_vision_socket():
    """Bind the UDP socket that receives detection data from video_streamer."""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((NETWORK["udp_receive_ip"], NETWORK["udp_receive_port"]))
    sock.settimeout(0.5)
    return sock


def run_mission(master, waypoints, data_sock):
    """Execute the full 8-phase mission."""
    servo_control.setup()
    winch_control.setup()
    # Quick jog test to verify gripper servo is responsive
    servo_control.open_gripper()
    servo_control.close_gripper()
    send_control_command("pause")

    if not arm_and_takeoff(master, TAKEOFF_ALTITUDE):
        raise RuntimeError("Failed initial takeoff. Aborting mission.")

    # --- PHASE 1: Acquiring Logistic 1 ---
    logger.info("--- PHASE 1: Acquiring Logistic 1 ---")
    navigate_to_waypoint(master, waypoints[0][0], waypoints[0][1], waypoints[0][2])
    if not execute_precision_landing(master, sock=data_sock, pickup_type="winch"):
        raise RuntimeError("Failed to land at Logistic 1. Aborting.")

    if not arm_and_takeoff(master, TAKEOFF_ALTITUDE):
        raise RuntimeError("Failed takeoff after Logistic 1. Aborting.")
    logger.info("Closing gripper for transit.")
    servo_control.close_gripper()

    # --- PHASE 2: Acquiring Logistic 2 ---
    logger.info("--- PHASE 2: Acquiring Logistic 2 ---")
    navigate_to_waypoint(master, waypoints[1][0], waypoints[1][1], waypoints[1][2])
    if not execute_precision_landing(master, sock=data_sock, pickup_type="gripper"):
        raise RuntimeError("Failed to land at Logistic 2. Aborting.")

    if not arm_and_takeoff(master, TAKEOFF_ALTITUDE):
        raise RuntimeError("Failed takeoff after Logistic 2. Aborting.")
    logger.info("Closing gripper for transit.")
    servo_control.close_gripper()

    # --- PHASE 3: Dropping at Barrel ---
    logger.info("--- PHASE 3: Dropping at Barrel ---")
    navigate_to_waypoint(master, waypoints[2][0], waypoints[2][1], waypoints[2][2])
    if not center_above_target(master, sock=data_sock, target_alt=waypoints[2][2]):
        logger.warning("Failed to center on barrel, but continuing mission.")

    # --- PHASE 4: Navigating Exit ---
    logger.info("--- PHASE 4: Navigating Exit ---")
    navigate_to_waypoint(master, waypoints[3][0], waypoints[3][1], waypoints[3][2])
    if not handle_window_approach(master, waypoints[3][2]):
        raise RuntimeError("Altitude check failed at window. Aborting.")
    navigate_to_waypoint(master, waypoints[4][0], waypoints[4][1], waypoints[4][2])

    logger.info("Switching EKF back to Source Set 1 (Rangefinder)...")
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_DO_AUX_FUNCTION,
        0,
        90,
        0,
        0,
        0,
        0,
        0,
        0,
    )

    # --- PHASE 5 & 6: Outdoor Drops ---
    logger.info("--- PHASE 5: Outdoor Drop 1 ---")
    navigate_to_waypoint(master, waypoints[5][0], waypoints[5][1], waypoints[5][2])
    servo_control.drop_package_outdoor_1()

    logger.info("--- PHASE 6: Outdoor Drop 2 ---")
    navigate_to_waypoint(master, waypoints[6][0], waypoints[6][1], waypoints[6][2])
    servo_control.drop_package_outdoor_2()

    # --- FINAL PHASE: Landing ---
    logger.info("--- FINAL PHASE: Landing ---")
    navigate_to_waypoint(master, waypoints[7][0], waypoints[7][1], waypoints[7][2])
    land_normally(master)

    logger.info("Full mission finished successfully!")


def main():
    waypoints = load_waypoints_from_args()
    data_sock = setup_vision_socket()
    master = setup_mavlink_connection()

    try:
        run_mission(master, waypoints, data_sock)
    except RuntimeError as e:
        logger.error("MISSION FAILED: %s", e)
        logger.info("Attempting to land at current position...")
        land_normally(master)
    except KeyboardInterrupt:
        logger.info("Keyboard interrupt received. Landing immediately...")
        land_normally(master)
        servo_control.open_gripper()
    finally:
        # Ensure EKF is reset to source set 1 regardless of where the mission aborted
        try:
            master.mav.command_long_send(
                master.target_system,
                master.target_component,
                mavutil.mavlink.MAV_CMD_DO_AUX_FUNCTION,
                0,
                90,
                0,
                0,
                0,
                0,
                0,
                0,
            )
            logger.info("EKF source set 1 restored in cleanup.")
        except Exception:
            logger.warning("Could not restore EKF source set during cleanup.")

        servo_control.cleanup()
        winch_control.cleanup()
        data_sock.close()
        logger.info("Resources cleaned up.")


if __name__ == "__main__":
    main()
