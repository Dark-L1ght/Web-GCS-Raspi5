#!/bin/bash

# This script sets up the environment and then executes the movement script,
# passing along any arguments it receives.

# Find the directory where this script is located
SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )

echo "Wrapper script started. Sourcing environment..."
# Source the setup script to prepare the environment
source "$SCRIPT_DIR/setup_env.sh"

echo "Environment sourced. Starting auto_mode_start.py with waypoints..."
python3 "$SCRIPT_DIR/backend/auto_mode_start.py"