# King Phoenix AIO GCS -- KRTI 2025

Ground Control Station and onboard mission execution for the King Phoenix drone team.
Runs on a Raspberry Pi 5 companion computer with Hailo AI accelerator.

## Project Structure

```
Web GCS - Raspi 5/
├── config.json                # Single source of truth for all tuning & network params
├── frontend/                  # Web GCS interface (Leaflet + WebSocket)
│   ├── index.html
│   ├── main.js
│   └── style.css
├── backend/
│   ├── config_loader.py       # Loads config.json; imported by any module that needs settings
│   ├── server/
│   │   └── gcs_server.py      # Unified WebSocket server (strategy = mission or auto)
│   ├── mission/
│   │   ├── shared.py          # All common mission functions (VelocityController, arm, land, ...)
│   │   ├── gps_mission.py     # Strategy 1: Full GPS + vision
│   │   └── auto_start.py      # Strategy 3: Mission Planner AUTO mode
│   ├── hardware/
│   │   ├── servo_control.py   # Library: gripper + outdoor drops (no CLI)
│   │   ├── auto_set_servo.py  # MAVLink-driven servo control (used by strategy 3)
│   │   └── winch_control.py   # Winch motor driver (used by all strategies)
│   ├── detection/
│   │   ├── video_streamer.py  # Hailo AI inference -> UDP
│   │   └── target.json        # Detection labels
│   └── utils/
│       ├── servo_test.py      # Standalone CLI for debugging servos
│       └── lidar_test.py      # Arduino LiDAR test utility
├── scripts/                   # Shell wrappers
│   ├── setup_env.sh           # Sets venv, PYTHONPATH, kernel check
│   ├── run_mission.sh         # Sources env -> runs gps_mission.py
│   ├── run_auto.sh            # Sources env -> runs auto_start.py
│   └── run_services.sh        # Starts MAVProxy + video_streamer
├── models/                    # Hailo .hef model files
├── archive/                   # Old / unused code
│   ├── lidar_mission.py
│   ├── gcs_mission.py
│   └── gcs_auto.py
├── run.py                     # Local test orchestrator
└── requirements.txt
```

## Configuration

All network addresses, tuning constants, and hardware pins live in `config.json` at the project root.
Edit this file instead of hunting through Python scripts.

Key sections:
- `network` -- drone IP, SSH user, MAVLink ports, UDP ports
- `mission` -- altitudes, timeouts, gains, thresholds
- `paths` -- model & script paths
- `hardware` -- winch GPIO pins
- `detection` -- camera device, Hailo architecture, valid class IDs

## Three Mission Strategies

| # | Strategy | Server command | Navigation | Vision | Servo |
|---|---|---|---|---|---|
| 1 | **Vision + GPS** | `python backend/server/gcs_server.py` | GPS waypoints (8 phases) | `video_streamer.py` (UDP) | `servo_control.py` |
| 3 | **Mission Planner AUTO** | `python backend/server/gcs_server.py auto` | ArduPilot pre-loaded mission | None | `auto_set_servo.py` (MAVLink) |

All strategies use `winch_control.py` for the winch.

## Quick Start

1. **Create the virtual environment** (required by `setup_env.sh`):

   The venv **must** be at `<project-root>/venv_hailo_rpi_examples` -- this path is hardcoded in `scripts/setup_env.sh`.

   ```bash
   # From the project root:
   python3 -m venv venv_hailo_rpi_examples
   source venv_hailo_rpi_examples/bin/activate
   pip install -r requirements.txt
   ```

2. **On the drone**, source the environment:

   ```bash
   source scripts/setup_env.sh
   ```

3. **Start onboard services** (MAVProxy + video streamer):

   ```bash
   bash scripts/run_services.sh
   ```

4. **On the GCS laptop**, start the unified WebSocket server:

   ```bash
   # Strategy 1 (custom mission with waypoints)
   python backend/server/gcs_server.py

   # Strategy 3 (Mission Planner AUTO mode)
   python backend/server/gcs_server.py auto
   ```

5. **Open the frontend** in a browser:

   ```
   file:///path/to/frontend/index.html
   ```

## Servo Debugging

Run the standalone CLI tool directly on the Pi:

```bash
python backend/utils/servo_test.py
# Keys: C=close, O=open, 1=drop1, 2=drop2, 8=hold1, 9=hold2, Q=quit
```

## What Changed in the Refactor

- **Centralized config** -- `config.json` replaces 30+ hardcoded constants; MAVLink protocol constants are hardcoded in code (not user-configurable)
- **Shared mission module** -- `backend/mission/shared.py` extracts all common flight/vision logic (~750 lines)
- **Unified server** -- `gcs_server.py` replaces the two nearly-identical server files; strategy selected via CLI arg
- **Logging** -- `print()` replaced with Python `logging` module everywhere
- **Graceful shutdown** -- `kill -9` replaced with `kill -15` (SIGTERM) + 2-second grace period, then `kill -9` fallback
- **Server-side waypoint validation** -- validates lat/lon bounds, altitude limits, and waypoint count
- **Relative paths** -- `video_streamer.py` resolves model/labels paths relative to project root
- **Library / CLI separation** -- `servo_control.py` is a pure library; `servo_test.py` is the standalone CLI tool
- **No module-scope MAVLink** -- connection established lazily in `main()`, not at import time
- **RTSP support removed** -- USB camera only
- **Mission safety fixes** -- timeouts added for navigation (60s), takeoff (15s), and landing disarm (10s); position targets are resent periodically (3s) as required by ArduPilot; EKF source set restored in cleanup
- **`auto_start.py` bugfix** -- now calls `set_mode("AUTO")` before starting the mission (previously silently failed if not already in AUTO)
- **Stale confirmation timer fix** -- `center_above_target` resets confirmation timer on target reacquisition to avoid premature drops
- **Dead code removed** -- `calculate_velocities()` function (unused)
- **Archive** -- old `lidar_mission.py`, `gcs_mission.py`, `gcs_auto.py` moved to `archive/`
