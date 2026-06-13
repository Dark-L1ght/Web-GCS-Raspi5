#!/usr/bin/env python3
"""Local orchestrator: starts video streamer + GPS mission for testing.

For production, use the shell wrappers in scripts/ instead.
"""

import subprocess
import sys
import time


def main():
    video_process = None
    try:
        print("Starting video_streamer.py...")
        video_process = subprocess.Popen(
            [sys.executable, "backend/detection/video_streamer.py"]
        )

        time.sleep(5)  # Allow camera / Hailo pipeline to initialize

        print("Starting gps_mission.py...")
        subprocess.run([sys.executable, "backend/mission/gps_mission.py"], check=True)

        print("Mission complete.")

    except subprocess.CalledProcessError:
        print("Mission script failed!")
    except KeyboardInterrupt:
        print("Mission interrupted by user.")
    finally:
        if video_process:
            print("Stopping video_streamer.py...")
            video_process.terminate()
            video_process.wait()
            print("Video streamer stopped.")


if __name__ == "__main__":
    main()
