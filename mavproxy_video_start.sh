#!/bin/bash

# This script automates the startup of essential drone services:
# 1. MAVProxy for telemetry forwarding.
# 2. The video_streamer.py for object detection and video feed.
# It ensures both are terminated cleanly when the script is stopped.

echo "--- Drone Onboard Services Startup ---"

# --- Configuration ---
# Set the IP addresses for your devices here for easy management.

# The IP of the Flight Controller (e.g., Cube Orange via WiFi)
FLIGHT_CONTROLLER_IP="192.168.144.31"

# The IP of the Ground Control Station (GCS) Laptop
GCS_LAPTOP_IP="192.168.10.237"


# --- Cleanup Function ---
# This function is called when the script is terminated (e.g., via Ctrl+C)
cleanup() {
    echo -e "\n--- Shutting down services... ---"
    
    # Kill the MAVProxy process if its Process ID (PID) exists
    if [ -n "$MAVPROXY_PID" ]; then
        echo "Stopping MAVProxy (PID: $MAVPROXY_PID)..."
        # Use kill -TERM for a graceful shutdown, then -9 if it fails
        kill -TERM $MAVPROXY_PID 2>/dev/null || kill -9 $MAVPROXY_PID 2>/dev/null
    fi
    
    # Kill the video streamer using its known script name as a safety measure
    # This helps catch any orphaned processes if the main one fails to exit
    pkill -f video_streamer.py
    
    echo "All services stopped."
    exit 0
}

# Trap the INT (Ctrl+C) and TERM signals and run the cleanup function
trap cleanup INT TERM

# Get the directory where this script is located to reliably find other files
SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )

# --- Environment Setup ---
# Source the environment setup script required for the Hailo AI application
echo "Sourcing environment from setup_env.sh..."
source "$SCRIPT_DIR/setup_env.sh"
echo "Environment ready."
sleep 1

# --- Start MAVProxy as a Background Process ---
echo "Starting MAVProxy in the background..."
mavproxy.py --master=udp:${FLIGHT_CONTROLLER_IP}:14550 \
            --out=udp:127.0.0.1:14550 \
            --out=udp:${GCS_LAPTOP_IP}:14552 \
            --out=udp:${GCS_LAPTOP_IP}:14553 &

# Save the Process ID (PID) of the MAVProxy process
MAVPROXY_PID=$!
echo "MAVProxy started with PID: $MAVPROXY_PID"
sleep 2 # Give MAVProxy a moment to initialize before starting the next process

# --- Start Video Streamer as the Foreground Process ---
echo "Starting video streamer..."
python3 "$SCRIPT_DIR/backend/video_streamer.py"

# The script will pause here, running the video streamer.
# When you press Ctrl+C, the 'trap' command will catch it and run the cleanup function.
echo "Video streamer has stopped. Exiting."
