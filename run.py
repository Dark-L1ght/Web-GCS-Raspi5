# run_mission.py
import subprocess
import time
import sys

def main():
    video_process = None
    try:
        # 1. Start the video streamer
        print("Starting video_streamer.py...")
        video_process = subprocess.Popen([sys.executable, "backend/video_streamer.py"])
        
        # 2. Give it time to initialize the camera
        time.sleep(5) # Adjust as needed

        # 3. Start the movement script
        print("Starting movement.py...")
        # .wait() makes this script pause until movement.py is finished
        subprocess.run([sys.executable, "backend/movement.py"], check=True)

        print("Mission complete.")

    except subprocess.CalledProcessError:
        print("Movement script failed!")
    except KeyboardInterrupt:
        print("Mission interrupted by user.")
    finally:
        # 5. Cleanly terminate the video streamer
        if video_process:
            print("Stopping video_streamer.py...")
            video_process.terminate()
            video_process.wait() # Wait for it to fully close
            print("Video streamer stopped.")

if __name__ == "__main__":
    main()