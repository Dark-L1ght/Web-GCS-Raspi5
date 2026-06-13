#!/usr/bin/env python3
"""Unified WebSocket GCS server.

Handles both custom mission (strategy 1) and AUTO mode (strategy 3) via a single
configurable entry point.  MAVLink connection is established lazily on startup.
"""

import asyncio
import json
import logging
import sys

import paramiko
import websockets
from pymavlink import mavutil

from backend.config_loader import NETWORK, PATHS

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
    datefmt="%H:%M:%S",
)
logger = logging.getLogger(__name__)

# ---------------------------------------------------------------------------
# Config-driven constants
# ---------------------------------------------------------------------------
DRONE_IP = NETWORK["drone_ip"]
DRONE_USER = NETWORK["drone_user"]
MAVLINK_SERVER = NETWORK["mavlink_server"]
WEBSOCKET_PORT = NETWORK["websocket_port"]

MISSION_SCRIPT = PATHS["project_root"] + "/" + PATHS["scripts"]["mission"]
AUTO_SCRIPT = PATHS["project_root"] + "/" + PATHS["scripts"]["auto"]

# Mission strategy set at startup
STRATEGY = "mission"  # or "auto"

# Global mutable state
connected_clients = set()
vehicle_state = {}
master = None

# ---------------------------------------------------------------------------
# MAVLink setup
# ---------------------------------------------------------------------------


def connect_mavlink():
    """Establish MAVLink connection and configure telemetry streams."""
    global master
    m = mavutil.mavlink_connection(MAVLINK_SERVER)
    logger.info("Waiting for heartbeat on %s...", MAVLINK_SERVER)
    m.wait_heartbeat()
    logger.info("Heartbeat received. MAVLink connection established.")

    logger.info("Requesting data streams...")
    for msg_id, interval in [
        (mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE, 500_000),
        (mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT, 500_000),
        (mavutil.mavlink.MAVLINK_MSG_ID_VFR_HUD, 500_000),
        (mavutil.mavlink.MAVLINK_MSG_ID_SYS_STATUS, 1_000_000),
        (mavutil.mavlink.MAVLINK_MSG_ID_STATUSTEXT, 1_000_000),
        (mavutil.mavlink.MAVLINK_MSG_ID_RANGEFINDER, 1_000_000),
    ]:
        m.mav.command_long_send(
            m.target_system,
            m.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
            0,
            msg_id,
            interval,
            0,
            0,
            0,
            0,
            0,
        )
    logger.info("Data streams requested.")
    return m


# ---------------------------------------------------------------------------
# SSH helpers
# ---------------------------------------------------------------------------


def execute_ssh_command(command: str):
    """Connect to the drone via SSH and execute a command."""
    try:
        ssh = paramiko.SSHClient()
        ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        logger.info("Connecting to %s@%s via SSH...", DRONE_USER, DRONE_IP)
        ssh.connect(DRONE_IP, username=DRONE_USER)
        logger.info("Executing: %s", command)
        stdin, stdout, stderr = ssh.exec_command(command)

        out = stdout.readlines()
        err = stderr.readlines()
        if out:
            logger.info("SSH stdout: %s", "".join(out).strip())
        if err:
            logger.warning("SSH stderr: %s", "".join(err).strip())

        logger.info("SSH command completed.")
    except Exception as e:
        logger.error("SSH Execution Failed: %s", e)
    finally:
        if (
            "ssh" in locals()
            and ssh.get_transport()
            and ssh.get_transport().is_active()
        ):
            ssh.close()
            logger.info("SSH connection closed.")


def stop_mission_and_land():
    """Terminate the remote mission script and command the drone to land."""
    process_name = "gps_mission.py" if STRATEGY == "mission" else "auto_start.py"
    logger.info("Stopping remote mission process (%s)...", process_name)

    # Try graceful SIGTERM first, then SIGKILL as fallback
    kill_cmd = f"pgrep -f {process_name} | xargs kill -15 2>/dev/null; sleep 2; pgrep -f {process_name} | xargs kill -9 2>/dev/null"
    execute_ssh_command(kill_cmd)

    logger.info("Commanding drone to LAND.")
    if master:
        master.mav.command_long_send(
            master.target_system,
            master.target_component,
            mavutil.mavlink.MAV_CMD_NAV_LAND,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
        )


# ---------------------------------------------------------------------------
# WebSocket handlers
# ---------------------------------------------------------------------------


async def mavlink_loop():
    """Continuously read MAVLink messages and forward them to all clients."""
    while True:
        msg = master.recv_match(
            type=[
                "ATTITUDE",
                "GLOBAL_POSITION_INT",
                "VFR_HUD",
                "HEARTBEAT",
                "STATUSTEXT",
                "SYS_STATUS",
                "RANGEFINDER",
            ],
            blocking=False,
        )
        if msg:
            data = None
            msg_type = msg.get_type()

            if msg_type == "STATUSTEXT":
                log_data = {
                    "type": "log",
                    "severity": msg.severity,
                    "text": msg.text.strip(),
                }
                if connected_clients:
                    websockets.broadcast(connected_clients, json.dumps(log_data))
                continue

            elif msg_type == "HEARTBEAT":
                is_armed = msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
                data = {
                    "flight_mode": master.flightmode,
                    "system_status": msg.system_status,
                    "armed": bool(is_armed),
                }

            elif msg_type == "ATTITUDE":
                data = {
                    "pitch": msg.pitch * 180.0 / 3.14,
                    "roll": msg.roll * 180.0 / 3.14,
                    "yaw": msg.yaw * 180.0 / 3.14,
                }

            elif msg_type == "GLOBAL_POSITION_INT":
                data = {
                    "lat": msg.lat / 1e7,
                    "lon": msg.lon / 1e7,
                    "alt_msl": msg.alt / 1000.0,
                    "alt_rel": msg.relative_alt / 1000.0,
                    "heading": msg.hdg / 100.0,
                }

            elif msg_type == "VFR_HUD":
                data = {
                    "airspeed": msg.airspeed,
                    "groundspeed": msg.groundspeed,
                    "throttle": msg.throttle,
                    "climb": msg.climb,
                }

            elif msg_type == "SYS_STATUS":
                data = {
                    "voltage": msg.voltage_battery / 1000.0,
                    "current": msg.current_battery / 100.0,
                    "level": msg.battery_remaining,
                }

            elif msg_type == "RANGEFINDER":
                data = {"distance": msg.distance}

            if data:
                vehicle_state.update(data)
                if connected_clients:
                    state_update = {"type": "state", "data": vehicle_state}
                    websockets.broadcast(connected_clients, json.dumps(state_update))

        await asyncio.sleep(0.01)


def _validate_waypoints(waypoints: list) -> tuple[bool, str]:
    """Server-side waypoint validation."""
    if not isinstance(waypoints, list):
        return False, "Waypoints must be a list."
    if len(waypoints) != 8:
        return False, f"Expected 8 waypoints, received {len(waypoints)}."

    for i, wp in enumerate(waypoints):
        if not isinstance(wp, dict):
            return False, f"Waypoint {i} is not an object."
        for key in ("lat", "lon", "alt"):
            if key not in wp:
                return False, f"Waypoint {i} missing key '{key}'."
            if not isinstance(wp[key], (int, float)):
                return False, f"Waypoint {i} '{key}' must be numeric."

        # Geographic bounds (approximate for Indonesia / KRTI venue)
        if not (-90.0 <= wp["lat"] <= 90.0):
            return False, f"Waypoint {i} latitude out of range."
        if not (-180.0 <= wp["lon"] <= 180.0):
            return False, f"Waypoint {i} longitude out of range."
        if not (0.5 <= wp["alt"] <= 10.0):
            return False, f"Waypoint {i} altitude must be 0.5-10 m."

    return True, ""


async def handler(websocket):
    """Handle a single WebSocket connection."""
    logger.info("Client connected from %s", websocket.remote_address)
    connected_clients.add(websocket)
    try:
        if vehicle_state:
            await websocket.send(json.dumps({"type": "state", "data": vehicle_state}))

        async for message in websocket:
            try:
                command = json.loads(message)
                action = command.get("action")

                if action == "start_mission":
                    if STRATEGY == "mission":
                        waypoints = command.get("waypoints")
                        ok, reason = _validate_waypoints(waypoints)
                        if not ok:
                            logger.warning("Mission start rejected: %s", reason)
                            continue

                        waypoints_json = json.dumps(waypoints)
                        mission_cmd = f"{MISSION_SCRIPT} '{waypoints_json}'"
                        asyncio.create_task(
                            asyncio.to_thread(execute_ssh_command, mission_cmd)
                        )
                        logger.info("Mission start command sent (custom GPS).")
                    else:
                        asyncio.create_task(
                            asyncio.to_thread(
                                execute_ssh_command, f"bash {AUTO_SCRIPT}"
                            )
                        )
                        logger.info("Mission start command sent (AUTO mode).")

                elif action == "stop_mission":
                    logger.info("Received stop_mission command.")
                    stop_mission_and_land()

            except json.JSONDecodeError:
                logger.warning("Received invalid JSON from client: %s", message)
            except Exception as e:
                logger.error("Error processing command: %s", e)
    finally:
        logger.info("Client disconnected from %s", websocket.remote_address)
        connected_clients.remove(websocket)


async def main():
    global master
    master = connect_mavlink()

    server = await websockets.serve(handler, "0.0.0.0", WEBSOCKET_PORT)
    logger.info("WebSocket server started on ws://0.0.0.0:%s", WEBSOCKET_PORT)
    mavlink_task = asyncio.create_task(mavlink_loop())
    await asyncio.gather(server.wait_closed(), mavlink_task)


if __name__ == "__main__":
    if len(sys.argv) > 1 and sys.argv[1].lower() == "auto":
        STRATEGY = "auto"
        logger.info("Strategy set to AUTO mode (Mission Planner).")
    else:
        logger.info("Strategy set to custom MISSION mode.")

    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        logger.info("Shutting down.")
