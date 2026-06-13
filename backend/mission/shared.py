"""Shared mission control functions and utilities.

All common logic used by mission scripts (GPS, LiDAR, or future variants) lives here
so it is maintained in a single place.
"""

import json
import logging
import math
import socket
import sys
import time

from pymavlink import mavutil

from backend.config_loader import MISSION, NETWORK
from backend.hardware import servo_control, winch_control

logger = logging.getLogger(__name__)

# ---------------------------------------------------------------------------
# MAVLink / ArduPilot protocol constants (not deployment-configurable)
# ---------------------------------------------------------------------------
GUIDED_MODE = 4
VELOCITY_CONTROL_BITMASK = 0b0000111111000111  # Ignore VX,VY,VZ,AX,AY,AZ,YAW,YAW_RATE
POSITION_CONTROL_BITMASK = 0b110111111000  # Use lat/lon/alt, ignore velocity/accel

# ---------------------------------------------------------------------------
# Mission tuning — from config.json
# ---------------------------------------------------------------------------
TAKEOFF_ALTITUDE = MISSION["takeoff_altitude"]
WAYPOINT_RADIUS = MISSION["waypoint_radius"]
ARMING_RETRIES = MISSION["arming_retries"]
ARMING_RETRY_DELAY = MISSION["arming_retry_delay"]
TRACKING_SPEED = MISSION["tracking_speed"]
CENTERING_SPEED = MISSION["centering_speed"]
FWD_GAIN = MISSION["fwd_gain"]
BLIND_FORWARD_SPEED = MISSION["blind_forward_speed"]
LANDING_VERTICAL_RATIO = MISSION["landing_vertical_ratio"]
CENTERING_VERTICAL_RATIO = MISSION["centering_vertical_ratio"]
CENTERING_SUCCESS_THRESHOLD = MISSION["centering_success_threshold"]
LANDING_D_GAIN = MISSION["landing_d_gain"]
CENTERING_D_GAIN = MISSION["centering_d_gain"]
LANDING_APPROACH_ALT = MISSION["landing_approach_alt"]
LANDING_TIMEOUT = MISSION["landing_timeout"]
FORCED_LAND_ALTITUDE = MISSION["forced_land_altitude"]
MAX_DOWN_VEL = MISSION["max_down_vel"]
MIN_DOWN_VEL = MISSION["min_down_vel"]
CENTERING_ERROR_THRESHOLD = MISSION["centering_error_threshold"]

WINCH_LOWER_DURATION = MISSION["winch_lower_duration"]
WINCH_LIFT_DURATION = MISSION["winch_lift_duration"]
WINCH_PULL_UP_DURATION = MISSION["winch_pull_up_duration"]

CENTERING_TIMEOUT = MISSION["centering_timeout"]
CENTERING_TARGET_AREA_RATIO = MISSION["centering_target_area_ratio"]
CENTERING_CONFIRMATION_DURATION = MISSION["centering_confirmation_duration"]
FINAL_VERIFICATION_TIMEOUT = MISSION["final_verification_timeout"]
FINAL_VERIFICATION_THRESHOLD = MISSION["final_verification_threshold"]

TARGET_LOST_HOVER_DURATION = MISSION["target_lost_hover_duration"]
REACQUIRE_ASCEND_SPEED = MISSION["reacquire_ascend_speed"]
ALTITUDE_CHECK_TOLERANCE = MISSION["altitude_check_tolerance"]

GAIN_MAX_ALT = MISSION["gain_max_alt"]
GAIN_MIN_ALT = MISSION["gain_min_alt"]
MAX_HORIZONTAL_GAIN = MISSION["max_horizontal_gain"]
MIN_HORIZONTAL_GAIN = MISSION["min_horizontal_gain"]

CONTROL_SERVER_IP = NETWORK["control_server_ip"]
CONTROL_SERVER_PORT = NETWORK["control_server_port"]

# Safety timeouts
NAVIGATION_TIMEOUT = 60  # seconds per waypoint
TAKEOFF_TIMEOUT = 15  # seconds to reach takeoff altitude
LAND_DISARM_TIMEOUT = 10  # seconds to wait for disarm after landing

# Vision detection filter constants (match video_streamer.py output)
TRACKING_STATE = "TRACKING"
VALID_CLASS_IDS = [0, 1]

# ---------------------------------------------------------------------------
# Core classes
# ---------------------------------------------------------------------------


class VelocityController:
    """A Proportional-Derivative (PD) controller for smooth drone velocities."""

    def __init__(self, p_gain: float, d_gain: float):
        self.p_gain = p_gain
        self.d_gain = d_gain
        self.prev_x_error = 0.0
        self.prev_y_error = 0.0
        self.current_x_error = 0.0
        self.current_y_error = 0.0
        logger.info("PD VelocityController initialized P=%.3f D=%.3f", p_gain, d_gain)

    def calculate_pd_velocities(
        self, x_center, y_center, frame_w, frame_h, vertical_ratio
    ):
        corrected_x_center = frame_w - x_center
        self.current_x_error = (corrected_x_center - frame_w / 2) / (frame_w / 2)

        target_y = frame_h * vertical_ratio
        self.current_y_error = (y_center - target_y) / (frame_h / 2)

        x_derivative = self.current_x_error - self.prev_x_error
        y_derivative = self.current_y_error - self.prev_y_error

        self.prev_x_error = self.current_x_error
        self.prev_y_error = self.current_y_error

        x_output = (self.p_gain * self.current_x_error) + (self.d_gain * x_derivative)
        y_output = (self.p_gain * self.current_y_error) + (self.d_gain * y_derivative)

        right_vel = x_output
        forward_vel = -y_output * FWD_GAIN

        if abs(right_vel) < 0.02:
            right_vel = 0
        if abs(forward_vel) < 0.02:
            forward_vel = 0

        return forward_vel, right_vel


# ---------------------------------------------------------------------------
# Low-level helpers
# ---------------------------------------------------------------------------


def send_control_command(command: str):
    """Send a command to the detection script via TCP."""
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.connect((CONTROL_SERVER_IP, CONTROL_SERVER_PORT))
            s.sendall(command.encode("utf-8"))
            logger.info("Control command sent: '%s'", command)
    except ConnectionRefusedError:
        logger.error("Detection script not reachable on port %s", CONTROL_SERVER_PORT)
    except Exception as e:
        logger.error("Failed to send control command: %s", e)


def flush_socket_buffer(sock: socket.socket):
    """Clear stale data from the UDP socket buffer."""
    logger.info("Flushing UDP socket buffer...")
    packets_cleared = 0
    start_time = time.time()
    while True:
        try:
            sock.recvfrom(1024)
            packets_cleared += 1
        except socket.timeout:
            break
        except Exception as e:
            logger.error("Error while flushing buffer: %s", e)
            break
    if packets_cleared > 0:
        logger.info(
            "Buffer flushed: %d packets in %.2fs",
            packets_cleared,
            time.time() - start_time,
        )
    else:
        logger.info("Buffer was already clear.")


def _send_zero_velocity(master):
    """Convenience: command the drone to hold position."""
    master.mav.send(
        mavutil.mavlink.MAVLink_set_position_target_local_ned_message(
            0,
            master.target_system,
            master.target_component,
            mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
            VELOCITY_CONTROL_BITMASK,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
        )
    )


# ---------------------------------------------------------------------------
# Flight control
# ---------------------------------------------------------------------------


def arm_and_takeoff(master: mavutil.mavlink_connection, altitude: float) -> bool:
    """Arm the vehicle and command a takeoff to the given altitude."""
    logger.info("Setting mode to GUIDED...")
    master.mav.set_mode_send(
        master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        GUIDED_MODE,
    )
    for attempt in range(1, ARMING_RETRIES + 1):
        logger.info("Arming motors (attempt %d/%d)...", attempt, ARMING_RETRIES)
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
        try:
            master.motors_armed_wait()
            logger.info("Motors armed.")
            break
        except Exception:
            if attempt == ARMING_RETRIES:
                logger.error("Could not arm motors after %d attempts.", ARMING_RETRIES)
                return False
            time.sleep(ARMING_RETRY_DELAY)

    logger.info("Taking off to %.2f m...", altitude)
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        altitude,
    )
    takeoff_start = time.time()
    while time.time() - takeoff_start < TAKEOFF_TIMEOUT:
        msg = master.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=2)
        if not msg:
            logger.warning("No altitude data during takeoff. Retrying...")
            continue
        current_altitude = msg.relative_alt / 1000.0
        logger.info("Takeoff altitude: %.2f m", current_altitude)
        if current_altitude >= altitude * 0.75:
            logger.info("Target altitude reached.")
            return True
        time.sleep(0.1)

    logger.error(
        "Takeoff timeout — did not reach %.2f m in %d s.", altitude, TAKEOFF_TIMEOUT
    )
    return False


def land_normally(master: mavutil.mavlink_connection):
    """Command the drone to land and wait for disarm."""
    logger.info("Executing normal landing...")
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
    start = time.time()
    while time.time() - start < LAND_DISARM_TIMEOUT:
        try:
            # poll heartbeat to check arm state
            msg = master.recv_match(type="HEARTBEAT", blocking=True, timeout=2)
            if msg and not (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
                logger.info("Landed and disarmed.")
                return
        except Exception:
            pass
    logger.warning(
        "Did not receive disarm within %d s — drone should auto-disarm on landing.",
        LAND_DISARM_TIMEOUT,
    )


def navigate_to_waypoint(
    master: mavutil.mavlink_connection, lat: float, lon: float, alt: float
):
    """Fly to a GPS waypoint and wait for arrival.  Resends position periodically."""
    logger.info("Navigating to (%.7f, %.7f) at %.2f m", lat, lon, alt)
    waypoint_start = time.time()
    last_command_time = 0

    while time.time() - waypoint_start < NAVIGATION_TIMEOUT:
        # Resend position target every 3 s (ArduPilot requires periodic refresh)
        if time.time() - last_command_time > 3.0:
            master.mav.set_position_target_global_int_send(
                0,
                master.target_system,
                master.target_component,
                mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                POSITION_CONTROL_BITMASK,
                int(lat * 1e7),
                int(lon * 1e7),
                alt,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
            )
            last_command_time = time.time()

        msg = master.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=1)
        if not msg:
            continue
        current_lat, current_lon = msg.lat / 1e7, msg.lon / 1e7
        dlat = math.radians(lat - current_lat)
        dlon = math.radians(lon - current_lon)
        a = (
            math.sin(dlat / 2) ** 2
            + math.cos(math.radians(current_lat))
            * math.cos(math.radians(lat))
            * math.sin(dlon / 2) ** 2
        )
        distance = 6371000 * 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
        logger.info("Distance to waypoint: %.1f m", distance)
        if distance <= WAYPOINT_RADIUS:
            logger.info("Waypoint reached.")
            break
        time.sleep(0.2)
    else:
        logger.error(
            "Navigation timeout — did not reach waypoint in %d s.", NAVIGATION_TIMEOUT
        )
        _send_zero_velocity(master)
        raise RuntimeError(f"Navigation timeout at waypoint ({lat:.7f}, {lon:.7f})")

    time.sleep(2)  # brief stabilization


# ---------------------------------------------------------------------------
# Vision-based helpers
# ---------------------------------------------------------------------------


def get_dynamic_gain(current_alt):
    """Calculate a dynamic gain scaling factor based on altitude."""
    if current_alt is None:
        return MIN_HORIZONTAL_GAIN
    if current_alt >= GAIN_MAX_ALT:
        return MAX_HORIZONTAL_GAIN
    if current_alt <= GAIN_MIN_ALT:
        return MIN_HORIZONTAL_GAIN
    gain_range = MAX_HORIZONTAL_GAIN - MIN_HORIZONTAL_GAIN
    alt_range = GAIN_MAX_ALT - GAIN_MIN_ALT
    return MIN_HORIZONTAL_GAIN + gain_range * ((current_alt - GAIN_MIN_ALT) / alt_range)


def verify_final_position(sock, target_class_ids, frame_w, frame_h):
    """Final high-precision check before committing an action (e.g. dropping)."""
    logger.info("Performing final position verification...")
    if not isinstance(target_class_ids, list):
        target_class_ids = [target_class_ids]

    flush_socket_buffer(sock)
    verification_start = time.time()

    while time.time() - verification_start < FINAL_VERIFICATION_TIMEOUT:
        try:
            data, _ = sock.recvfrom(1024)
            detection = json.loads(data.decode())

            if detection.get("state") != TRACKING_STATE:
                continue
            if detection.get("class_id") not in target_class_ids:
                continue

            error = abs(detection["x_center"] - frame_w / 2) / (frame_w / 2)
            if error < FINAL_VERIFICATION_THRESHOLD:
                logger.info("Verification PASSED (error: %.1f%%)", error * 100)
                return True
            else:
                logger.warning(
                    "Verification FAILED — target drifted (%.1f%%)", error * 100
                )
                return False

        except (socket.timeout, json.JSONDecodeError, KeyError):
            pass

    logger.warning("Verification timed out.")
    return False


def center_above_target(master, sock, target_alt):
    """Center the drone over a target and execute a dual drop once verified."""
    flush_socket_buffer(sock)
    send_control_command(f"set_ratio:{CENTERING_VERTICAL_RATIO}")
    time.sleep(0.1)
    send_control_command("resume")
    logger.info("Centering above target at %.2f m...", target_alt)

    controller = VelocityController(p_gain=CENTERING_SPEED, d_gain=CENTERING_D_GAIN)
    start_time = time.time()
    last_detection_time = time.time()
    last_known_alt = target_alt
    centered_confirmation_start = None
    was_centered = False

    while time.time() - start_time < CENTERING_TIMEOUT:
        alt_msg = master.recv_match(
            type="DISTANCE_SENSOR", blocking=False, timeout=0.05
        )
        if alt_msg and alt_msg.orientation == 25:
            last_known_alt = alt_msg.current_distance / 100.0
        current_altitude = last_known_alt

        fwd_vel, right_vel = 0, 0

        try:
            data, _ = sock.recvfrom(1024)
            detection = json.loads(data.decode())

            detected_id = detection.get("class_id")
            if (
                detection.get("state") != TRACKING_STATE
                or detected_id not in VALID_CLASS_IDS
            ):
                raise socket.timeout

            last_detection_time = time.time()

            # Reset confirmation timer if target was reacquired after being lost
            if was_centered:
                centered_confirmation_start = None
                was_centered = False

            x, y = detection["x_center"], detection["y_center"]
            w, h = detection["frame_width"], detection["frame_height"]

            fwd_vel, right_vel = controller.calculate_pd_velocities(
                x, y, w, h, CENTERING_VERTICAL_RATIO
            )
            horizontal_gain = get_dynamic_gain(current_altitude)
            fwd_vel *= horizontal_gain
            right_vel *= horizontal_gain

            h_err = abs(x - w / 2) / (w / 2)
            target_y = h * CENTERING_VERTICAL_RATIO
            v_err = abs(y - target_y) / (h / 2)
            area_ratio = min(
                detection["area"] / (w * h * CENTERING_TARGET_AREA_RATIO), 1.0
            )

            is_h_ok = h_err < CENTERING_SUCCESS_THRESHOLD
            is_v_ok = v_err < CENTERING_SUCCESS_THRESHOLD
            is_close = detection["area"] >= w * h * CENTERING_TARGET_AREA_RATIO

            logger.info(
                "Centering (ID %s): Alt %.2f  H-Err %.1f%%  V-Err %.1f%%  Area %.1f%%",
                detected_id,
                current_altitude,
                h_err * 100,
                v_err * 100,
                area_ratio * 100,
            )

            if is_h_ok and is_v_ok and is_close:
                if centered_confirmation_start is None:
                    centered_confirmation_start = time.time()
                    was_centered = True

                if (
                    time.time() - centered_confirmation_start
                    > CENTERING_CONFIRMATION_DURATION
                ):
                    logger.info("Target fully centered. Stabilizing...")
                    _send_zero_velocity(master)
                    time.sleep(1)
                    send_control_command("pause")

                    if verify_final_position(sock, VALID_CLASS_IDS, w, h):
                        logger.info("Commencing dual drop sequence.")
                        servo_control.open_gripper()
                        logger.info("Package 2 (gripper) dropped.")
                        winch_control.raise_winch(WINCH_PULL_UP_DURATION)
                        logger.info("Package 1 (winch) dropped.")
                        send_control_command("pause")
                        return True
                    else:
                        logger.info("Verification failed — resuming centering.")
                        send_control_command("resume")
                        centered_confirmation_start = None
                        was_centered = False
            else:
                # No longer centered — reset confirmation timer
                centered_confirmation_start = None
                was_centered = False

        except (socket.timeout, json.JSONDecodeError, KeyError):
            time_since_lost = time.time() - last_detection_time
            if time_since_lost >= TARGET_LOST_HOVER_DURATION:
                logger.info(
                    "Target lost for %.1f s. Hovering in place.", time_since_lost
                )
            fwd_vel, right_vel = 0, 0
            centered_confirmation_start = None
            was_centered = False

        master.mav.send(
            mavutil.mavlink.MAVLink_set_position_target_local_ned_message(
                0,
                master.target_system,
                master.target_component,
                mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
                VELOCITY_CONTROL_BITMASK,
                0,
                0,
                0,
                fwd_vel,
                right_vel,
                0,
                0,
                0,
                0,
                0,
                0,
            )
        )
        time.sleep(0.05)

    logger.error("Centering timeout — aborting drop sequence.")
    send_control_command("pause")
    _send_zero_velocity(master)
    return False


def execute_precision_landing(master, sock, pickup_type):
    """Precision landing using vision guidance.  pickup_type = 'winch' | 'gripper'."""
    logger.info("Opening gripper for landing approach...")
    servo_control.open_gripper()
    time.sleep(1.0)

    flush_socket_buffer(sock)
    send_control_command(f"set_ratio:{LANDING_VERTICAL_RATIO}")
    time.sleep(0.1)
    send_control_command("resume")
    logger.info("Precision landing for '%s' pickup...", pickup_type)

    pd_controller = VelocityController(p_gain=TRACKING_SPEED, d_gain=LANDING_D_GAIN)
    search_start = time.time()
    last_detection_time = time.time()
    last_known_alt = TAKEOFF_ALTITUDE

    def perform_pickup_action():
        logger.info("Landed — securing package.")
        servo_control.close_gripper()
        time.sleep(1.5)
        if pickup_type == "winch":
            logger.info("Winch pickup.")
            winch_control.lower_winch(WINCH_LOWER_DURATION)
            time.sleep(1.0)
            winch_control.raise_winch(WINCH_LIFT_DURATION)
        elif pickup_type == "gripper":
            logger.info("Gripper pickup — package secured.")
            time.sleep(1.0)
        return True

    while time.time() - search_start < LANDING_TIMEOUT:
        alt_msg = master.recv_match(
            type="DISTANCE_SENSOR", blocking=False, timeout=0.05
        )
        if alt_msg and alt_msg.orientation == 25:
            last_known_alt = alt_msg.current_distance / 100.0
        current_altitude = last_known_alt

        if current_altitude < FORCED_LAND_ALTITUDE:
            logger.info(
                "Altitude below %.2f m — forcing immediate landing.",
                FORCED_LAND_ALTITUDE,
            )
            land_normally(master)
            return perform_pickup_action()

        try:
            data, _ = sock.recvfrom(1024)
            detection = json.loads(data.decode())

            detected_id = detection.get("class_id")
            if (
                detection.get("state") != TRACKING_STATE
                or detected_id not in VALID_CLASS_IDS
            ):
                raise socket.timeout

            last_detection_time = time.time()
            x, y = detection["x_center"], detection["y_center"]
            w, h = detection["frame_width"], detection["frame_height"]

            fwd_vel, right_vel = pd_controller.calculate_pd_velocities(
                x, y, w, h, LANDING_VERTICAL_RATIO
            )
            horizontal_gain = get_dynamic_gain(current_altitude)
            fwd_vel *= horizontal_gain
            right_vel *= horizontal_gain

            center_error_ratio = abs(x - w / 2) / (w / 2)
            error_scale = min(center_error_ratio / CENTERING_ERROR_THRESHOLD, 1.0)
            dynamic_down_vel = MAX_DOWN_VEL - error_scale * (
                MAX_DOWN_VEL - MIN_DOWN_VEL
            )

            master.mav.send(
                mavutil.mavlink.MAVLink_set_position_target_local_ned_message(
                    0,
                    master.target_system,
                    master.target_component,
                    mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
                    VELOCITY_CONTROL_BITMASK,
                    0,
                    0,
                    0,
                    fwd_vel,
                    right_vel,
                    dynamic_down_vel,
                    0,
                    0,
                    0,
                    0,
                    0,
                )
            )
            logger.info(
                "LANDING (ID %s): Alt %.2f  Gain %.2f  Err %.1f%%  Vz %.2f",
                detected_id,
                current_altitude,
                horizontal_gain,
                center_error_ratio * 100,
                dynamic_down_vel,
            )

            if current_altitude < LANDING_APPROACH_ALT and center_error_ratio < 0.05:
                logger.info("Centered at low altitude — switching to LAND mode.")
                land_normally(master)
                return perform_pickup_action()

        except (socket.timeout, json.JSONDecodeError, KeyError):
            time_since_lost = time.time() - last_detection_time
            vx, vy, vz = 0, 0, 0
            if time_since_lost < TARGET_LOST_HOVER_DURATION:
                logger.info("Target lost. Hovering... (%.1f s)", time_since_lost)
            else:
                vz = -REACQUIRE_ASCEND_SPEED
                vx = BLIND_FORWARD_SPEED
                logger.info("Searching — ascending & moving forward.")

            master.mav.send(
                mavutil.mavlink.MAVLink_set_position_target_local_ned_message(
                    0,
                    master.target_system,
                    master.target_component,
                    mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
                    VELOCITY_CONTROL_BITMASK,
                    0,
                    0,
                    0,
                    vx,
                    vy,
                    vz,
                    0,
                    0,
                    0,
                    0,
                    0,
                )
            )

    logger.error("Landing timeout — aborting and hovering.")
    _send_zero_velocity(master)
    return False


def handle_window_approach(master, target_altitude):
    """Switch EKF to source set 2 (GPS altitude) and verify reading."""
    logger.info("--- WINDOW APPROACH ---")
    logger.info("Switching EKF to Source Set 2 (GPS for altitude)...")

    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_DO_AUX_FUNCTION,
        0,
        90,
        1,
        0,
        0,
        0,
        0,
        0,
    )
    time.sleep(1)

    msg = master.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=3)
    if not msg:
        logger.error("No altitude data after EKF switch.")
        return False

    gps_alt = msg.relative_alt / 1000.0
    error = abs(gps_alt - target_altitude)

    logger.info(
        "Target Alt: %.2f m  |  GPS Alt: %.2f m  |  Error: %.2f m",
        target_altitude,
        gps_alt,
        error,
    )

    if error > ALTITUDE_CHECK_TOLERANCE:
        logger.error(
            "Altitude check FAILED (error %.2f m > tolerance %.2f m).",
            error,
            ALTITUDE_CHECK_TOLERANCE,
        )
        return False

    logger.info("Altitude check PASSED.")
    return True
