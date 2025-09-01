#!/bin/bash

# start_mission_looped.sh

SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )

while true; do
    echo "----------------------------------------"
    echo "---      STARTING NEW MISSION        ---"
    echo "----------------------------------------"

    echo "Configuring camera resolution to 640x480..."
    v4l2-ctl --device /dev/video0 --set-fmt-video=width=640,height=480,pixelformat='MJPG'

    echo "Sourcing environment..."
    source "$SCRIPT_DIR/setup_env.sh"

    echo "Starting video_streamer.py in the background..."
    python3 "$SCRIPT_DIR/backend/video_streamer.py" &
    VIDEO_PID=$! # Save the Process ID (PID) of the video streamer

    # Give the video streamer a few seconds to initialize
    sleep 5

    echo "Starting movement.py..."
    # This will run until it finishes or is cancelled
    python3 "$SCRIPT_DIR/backend/movement.py"

    echo "--- MISSION FINISHED OR CANCELLED ---"
    echo "Stopping video_streamer.py (PID: $VIDEO_PID)..."
    kill $VIDEO_PID
    wait $VIDEO_PID 2>/dev/null # Wait for it to shut down cleanly

    read -p "Press Enter to start the next mission, or Ctrl+C to exit."
    echo
done