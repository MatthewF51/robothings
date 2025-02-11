import os
import subprocess
import websocket
import json
import time

# Global publisher dictionary
publishers = {}

def get_wsl_ip():
    """Gets the current WSL IP address dynamically."""
    try:
        # Run a WSL command to get the eth0 IP address
        result = subprocess.run(
            ["wsl", "ip addr show eth0 | grep 'inet ' | awk '{print $2}' | cut -d/ -f1"],
            capture_output=True, text=True, shell=True
        )
        wsl_ip = result.stdout.strip()
        if wsl_ip:
            return wsl_ip
    except Exception as e:
        print(f"Error getting WSL IP: {e}")
    return "127.0.0.1"  # Default to localhost if WSL IP cannot be found

# Set ROSBridge connection dynamically
ROSBRIDGE_IP = get_wsl_ip()
ROSBRIDGE_PORT = 9090
ROSBRIDGE_URL = f"ws://{ROSBRIDGE_IP}:{ROSBRIDGE_PORT}"


def send_command(topic, message, duration=None):
    """Sends a command to ROS via WebSockets and stops after duration (if needed)."""
    ws = websocket.WebSocket()
    ws.connect(ROSBRIDGE_URL)

    # Send the actual command
    command_msg = {"op": "publish", "topic": topic, "msg": message}
    ws.send(json.dumps(command_msg))
    print(f"[DEBUG] Sent to {topic}: {message}")

    # If duration is provided, wait and then stop the robot
    if duration and duration > 0:
        time.sleep(duration)
        stop_msg = {"op": "publish", "topic": topic, "msg": {
            "linear": {"x": 0.0, "y": 0.0, "z": 0.0},
            "angular": {"x": 0.0, "y": 0.0, "z": 0.0}
        }}
        ws.send(json.dumps(stop_msg))
        print(f"[DEBUG] Stopped movement on {topic}")

    ws.close()

def stop_robot(ws=None):
    """Stops the robot by sending zero velocity."""
    stop_msg = {
        "op": "publish",
        "topic": "/cmd_vel",
        "msg": {
            "linear": {"x": 0.0, "y": 0.0, "z": 0.0},
            "angular": {"x": 0.0, "y": 0.0, "z": 0.0}
        }
    }

    try:
        if ws is None:
            ws = websocket.WebSocket()
            ws.connect(ROSBRIDGE_URL)

        ws.send(json.dumps(stop_msg))
        ws.close()
        print("[DEBUG] Emergency stop sent!")
    except Exception as e:
        print(f"[ERROR] Failed to send stop command: {e}")



def load_modules():
    """Loads commands and colors from all module files dynamically."""
    import importlib
    modules = {}
    modules_path = "Modules"  # Path to the modules folder
    for file in os.listdir(modules_path):
        if file.endswith(".py") and file != "__init__.py":
            module_name = file[:-3]  # Remove .py extension
            try:
                module = importlib.import_module(f"Modules.{module_name}")
                commands = getattr(module, "COMMANDS", {})
                color = getattr(module, "colour", "#000000")  # Default color is black
                modules[module_name] = {"commands": commands, "color": color}
            except AttributeError as e:
                print(f"Error loading {module_name}: {e}")
    return modules

def publish_command(command):
    """Publishes a command to ROS topic via WebSockets."""
    send_command("/robot_command", {"data": command})
