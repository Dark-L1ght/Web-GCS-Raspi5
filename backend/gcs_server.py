import asyncio
import json
import websockets
from pymavlink import mavutil
import paramiko

# --- SSH Configuration ---
DRONE_IP = '192.168.10.212' # IP DRONE
DRONE_USER = 'kingphoenix' # USERNAME DRONE

# --- MAVLink Connection ---
connection_string = 'udp:0.0.0.0:14552'
master = mavutil.mavlink_connection(connection_string)
print(f"Waiting for heartbeat on {connection_string}...")
master.wait_heartbeat()
print("Heartbeat received! MAVLink connection established.")

# --- REQUEST DATA STREAMS ---
print("Requesting data streams...")
# (Your data stream requests remain the same)
master.mav.command_long_send(master.target_system, master.target_component, mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0, mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE, 500000, 0, 0, 0, 0, 0)
master.mav.command_long_send(master.target_system, master.target_component, mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0, mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT, 500000, 0, 0, 0, 0, 0)
master.mav.command_long_send(master.target_system, master.target_component, mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0, mavutil.mavlink.MAVLINK_MSG_ID_VFR_HUD, 500000, 0, 0, 0, 0, 0)
master.mav.command_long_send(master.target_system, master.target_component, mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0, mavutil.mavlink.MAVLINK_MSG_ID_SYS_STATUS, 1000000, 0, 0, 0, 0, 0)
master.mav.command_long_send(master.target_system, master.target_component, mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0, mavutil.mavlink.MAVLINK_MSG_ID_STATUSTEXT, 1000000, 0, 0, 0, 0, 0)
master.mav.command_long_send(master.target_system, master.target_component, mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0, mavutil.mavlink.MAVLINK_MSG_ID_RANGEFINDER, 1000000, 0, 0, 0, 0, 0)
print("Data streams requested.")

# Global variables
connected_clients = set()
vehicle_state = {}

def execute_ssh_command(command):
    """Connects to the drone via SSH and executes a command."""
    try:
        ssh = paramiko.SSHClient()
        ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        print(f"Connecting to {DRONE_USER}@{DRONE_IP} via SSH...")
        ssh.connect(DRONE_IP, username=DRONE_USER) # Assumes SSH key setup
        print(f"Executing remote command: {command}")
        stdin, stdout, stderr = ssh.exec_command(command)
        
        stdout_lines = stdout.readlines()
        stderr_lines = stderr.readlines()
        if stdout_lines:
            print("SSH Command Output:", "".join(stdout_lines))
        if stderr_lines:
            print("SSH Command Error:", "".join(stderr_lines))
        
        print("SSH command sent successfully.")
    except Exception as e:
        print(f"SSH Execution Failed: {e}")
    finally:
        if 'ssh' in locals() and ssh.get_transport().is_active():
            ssh.close()
            print("SSH connection closed.")

# ### NEW FUNCTION ###
def stop_mission_and_land():
    """Terminates the remote mission script and commands the drone to land."""
    print("Attempting to stop remote mission script...")
    # Command to find the process ID of movement.py and kill it forcefully
    kill_command = "pgrep -f movement.py | xargs kill -9"
    execute_ssh_command(kill_command)
    
    print("Remote script terminated. Commanding drone to LAND.")
    # Send a land command as a safety measure
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_LAND, 0, 0, 0, 0, 0, 0, 0, 0
    )

async def mavlink_loop():
    """Continuously read MAVLink messages and forward them."""
    while True:
        msg = master.recv_match(
            type=['ATTITUDE', 'GLOBAL_POSITION_INT', 'VFR_HUD', 'HEARTBEAT',
                  'STATUSTEXT', 'SYS_STATUS', 'RANGEFINDER'],
            blocking=False
        )
        if msg:
            # (Your message handling logic remains the same)
            data = None
            msg_type = msg.get_type()

            if msg_type == 'STATUSTEXT':
                log_data = {'type': 'log', 'severity': msg.severity, 'text': msg.text.strip()}
                if connected_clients:
                    websockets.broadcast(connected_clients, json.dumps(log_data))
            
            elif msg_type == 'HEARTBEAT':
                is_armed = msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
                data = {'flight_mode': master.flightmode, 'system_status': msg.system_status, 'armed': bool(is_armed)}
                vehicle_state.update(data)

            elif msg_type == 'ATTITUDE':
                data = {'pitch': msg.pitch * 180.0 / 3.14, 'roll': msg.roll * 180.0 / 3.14, 'yaw': msg.yaw * 180.0 / 3.14}
                vehicle_state.update(data)

            elif msg_type == 'GLOBAL_POSITION_INT':
                data = {'lat': msg.lat / 1e7, 'lon': msg.lon / 1e7, 'alt_msl': msg.alt / 1000.0, 'alt_rel': msg.relative_alt / 1000.0, 'heading': msg.hdg / 100.0}
                vehicle_state.update(data)

            elif msg_type == 'VFR_HUD':
                data = {'airspeed': msg.airspeed, 'groundspeed': msg.groundspeed, 'throttle': msg.throttle, 'climb': msg.climb}
                vehicle_state.update(data)

            elif msg_type == 'SYS_STATUS':
                data = {'voltage': msg.voltage_battery / 1000.0, 'current': msg.current_battery / 100.0, 'level': msg.battery_remaining}
                vehicle_state.update(data)
                
            elif msg_type == 'RANGEFINDER':
                data = {'distance': msg.distance}
                vehicle_state.update(data)

            if data and connected_clients:
                state_update = {'type': 'state', 'data': vehicle_state}
                websockets.broadcast(connected_clients, json.dumps(state_update))
        await asyncio.sleep(0.01)

async def handler(websocket):
    """Handle new WebSocket connections and listen for incoming commands."""
    print(f"Client connected from {websocket.remote_address}")
    connected_clients.add(websocket)
    try:
        if vehicle_state:
            initial_state = {'type': 'state', 'data': vehicle_state}
            await websocket.send(json.dumps(initial_state))
        
        async for message in websocket:
            try:
                command = json.loads(message)
                action = command.get('action')

                if action == 'start_mission':
                    print("Received 'start_mission' command from web client.")
                    
                    waypoints = command.get('waypoints')
                    if not waypoints or len(waypoints) != 4:
                        print(f"Mission start aborted: Invalid waypoints received: {waypoints}")
                        continue # Ignore the command

                    # Convert waypoints to a JSON string for command-line argument
                    # The outer single quotes are for the shell command
                    waypoints_json_str = json.dumps(waypoints)
                    
                    # IMPORTANT: Update this path to be the correct one on your drone
                    script_path = '/home/kingphoenix/Web-GCS/backend/movement.py'
                    
                    mission_command = f"python3 {script_path} '{waypoints_json_str}'"
                    
                    # Run the blocking SSH call in a separate thread
                    asyncio.create_task(asyncio.to_thread(execute_ssh_command, mission_command))

                elif action == 'stop_mission':
                    print("Received 'stop_mission' command from web client.")
                    stop_mission_and_land()

            except json.JSONDecodeError:
                print(f"Received invalid JSON from client: {message}")
            except Exception as e:
                print(f"Error processing command: {e}")
    finally:
        print(f"Client disconnected from {websocket.remote_address}")
        connected_clients.remove(websocket)

async def main():
    """Start the MAVLink loop and the WebSocket server to run in parallel."""
    server = await websockets.serve(handler, "0.0.0.0", 8765)
    print("WebSocket server started on ws://0.0.0.0:8765")
    mavlink_task = asyncio.create_task(mavlink_loop())
    await asyncio.gather(server.wait_closed(), mavlink_task)

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("Shutting down.")
