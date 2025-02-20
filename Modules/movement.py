import time
import threading
from utils import send_command

def send_timed_command(command, duration):
    """
    Sends a command for a set duration, then stops movement.

    :param command: The movement command string to send.
    :param duration: Duration (in seconds) to send the command.
    """
    process = send_command(command)  # Send movement command
    time.sleep(duration)  # Wait for duration
    send_command("rostopic pub -1 /cmd_vel geometry_msgs/Twist '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'")  # Stop

COMMANDS = {
    "Move Forward": {
        "function": lambda speed, distance: threading.Thread(
            target=send_timed_command,
            args=(f"rostopic pub /mobile_base_controller/cmd_vel geometry_msgs/Twist '{{linear: {{x: {speed}, y: 0.0, z: 0.0}}, angular: {{x: 0.0, y: 0.0, z: 0.0}}}}' -r 10", float(distance) / float(speed)),
            daemon=True
        ).start(),
        "inputs": {"Speed": "speed", "Distance": "distance"},
    },
    "Move Backward": {
        "function": lambda speed, distance: threading.Thread(
            target=send_timed_command,
            args=(f"rostopic pub /mobile_base_controller/cmd_vel geometry_msgs/Twist '{{linear: {{x: {-speed}, y: 0.0, z: 0.0}}, angular: {{x: 0.0, y: 0.0, z: 0.0}}}}' -r 10", float(distance) / float(speed)),
            daemon=True
        ).start(),
        "inputs": {"Speed": "speed", "Distance": "distance"},
    },
    "Rotate Left": {
        "function": lambda speed, angle: threading.Thread(
            target=send_timed_command,
            args=(f"rostopic pub /mobile_base_controller/cmd_vel geometry_msgs/Twist '{{linear: {{x: 0.0, y: 0.0, z: 0.0}}, angular: {{x: 0.0, y: 0.0, z: {speed}}}}}' -r 10", float(angle) / float(speed)),
            daemon=True
        ).start(),
        "inputs": {"Speed": "speed", "Angle": "angle"},
    },
    "Rotate Right": {
        "function": lambda speed, angle: threading.Thread(
            target=send_timed_command,
            args=(f"rostopic pub /mobile_base_controller/cmd_vel geometry_msgs/Twist '{{linear: {{x: 0.0, y: 0.0, z: 0.0}}, angular: {{x: 0.0, y: 0.0, z: {-speed}}}}}' -r 10", float(angle) / float(speed)),
            daemon=True
        ).start(),
        "inputs": {"Speed": "speed", "Angle": "angle"},
    },
    "Stop": {
        "function": lambda: send_command(
            "rostopic pub -1 /cmd_vel geometry_msgs/Twist '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'"
        ),
        "inputs": {},
    }
}

colour = "#FF5733"  # Orange color for UI
