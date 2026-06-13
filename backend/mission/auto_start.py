#!/usr/bin/env python3
"""Strategy 3: Mission Planner AUTO mode launcher.

Connects to the vehicle, sets AUTO mode, and starts the pre-loaded mission.
Used by gcs_server.py when started with the 'auto' argument.
"""

import logging
import sys
import time

from pymavlink import mavutil

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
    datefmt="%H:%M:%S",
)
logger = logging.getLogger(__name__)

CONNECTION_STRING = "udp:127.0.0.1:14550"


def connect_to_vehicle():
    logger.info("Connecting to vehicle on: %s", CONNECTION_STRING)
    try:
        master = mavutil.mavlink_connection(CONNECTION_STRING)
        master.wait_heartbeat()
        logger.info(
            "Heartbeat from system (system %s component %s)",
            master.target_system,
            master.target_component,
        )
        return master
    except Exception as e:
        logger.error("Failed to connect: %s", e)
        return None


def set_mode(master, mode_name):
    logger.info("Setting mode to %s...", mode_name)
    if mode_name not in master.mode_mapping():
        logger.error("Mode '%s' not available.", mode_name)
        return False

    mode_id = master.mode_mapping()[mode_name]
    master.mav.set_mode_send(
        master.target_system, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, mode_id
    )

    start_time = time.time()
    while time.time() - start_time < 5:
        msg = master.recv_match(type="HEARTBEAT", blocking=True, timeout=1)
        if msg and msg.custom_mode == mode_id:
            logger.info("Mode successfully changed to %s.", mode_name)
            return True

    logger.error("Failed to set mode to %s.", mode_name)
    return False


def arm_vehicle(master):
    logger.info("Arming vehicle...")
    try:
        master.mav.command_long_send(
            master.target_system,
            master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            1,
            0,
            0,
            0,
            0,
            0,
            0,
        )
        master.motors_armed_wait()
        logger.info("Vehicle ARMED.")
        return True
    except Exception as e:
        logger.error("Failed to arm vehicle: %s", e)
        return False


def start_mission(master):
    logger.info("Sending MISSION_START command...")
    try:
        master.mav.command_long_send(
            master.target_system,
            master.target_component,
            mavutil.mavlink.MAV_CMD_MISSION_START,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
        )
        logger.info("Mission start command sent.")
        return True
    except Exception as e:
        logger.error("Failed to send MISSION_START: %s", e)
        return False


def main():
    master = connect_to_vehicle()
    if not master:
        sys.exit(1)

    if not set_mode(master, "AUTO"):
        logger.error("Cannot set AUTO mode — aborting.")
        sys.exit(1)

    if not start_mission(master):
        sys.exit(1)

    logger.info("Mission initiated. Monitor progress in your GCS.")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        logger.info("Interrupted by user.")
        sys.exit(0)
