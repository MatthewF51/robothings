import threading
from utils import send_command

# Set a fixed rotation speed in rad/s
FIXED_ROTATION_SPEED = 0.5  # Adjust this if needed

COMMANDS = {
    "Move Forward": {
        "function": lambda distance: threading.Thread(
            target=send_command,
            args=(
                f"rostopic pub /mobile_base_controller/cmd_vel geometry_msgs/Twist "
                f"'{{linear: {{x: 0.5, y: 0.0, z: 0.0}}, angular: {{x: 0.0, y: 0.0, z: 0.0}}}}' -r 10",
                float(distance) / 0.5  # Speed is fixed at 0.5 m/s
            ),
            daemon=True
        ).start(),
        "inputs": {"Distance": "distance"},
    },
    "Move Backward": {
        "function": lambda distance: threading.Thread(
            target=send_command,
            args=(
                f"rostopic pub /mobile_base_controller/cmd_vel geometry_msgs/Twist "
                f"'{{linear: {{x: -0.5, y: 0.0, z: 0.0}}, angular: {{x: 0.0, y: 0.0, z: 0.0}}}}' -r 10",
                float(distance) / 0.5
            ),
            daemon=True
        ).start(),
        "inputs": {"Distance": "distance"},
    },
    "Rotate Left": {
        "function": lambda degrees: threading.Thread(
            target=send_command,
            args=(
                f"rostopic pub /mobile_base_controller/cmd_vel geometry_msgs/Twist "
                f"'{{linear: {{x: 0.0, y: 0.0, z: 0.0}}, angular: {{x: 0.0, y: 0.0, z: {FIXED_ROTATION_SPEED}}}}}' -r 10",
                (float(degrees) * 3.14159265 / 180) / FIXED_ROTATION_SPEED  # Convert degrees to radians and calculate duration
            ),
            daemon=True
        ).start(),
        "inputs": {"Degrees": "degrees"},  # User inputs only degrees
    },
    "Rotate Right": {
        "function": lambda degrees: threading.Thread(
            target=send_command,
            args=(
                f"rostopic pub /mobile_base_controller/cmd_vel geometry_msgs/Twist "
                f"'{{linear: {{x: 0.0, y: 0.0, z: 0.0}}, angular: {{x: 0.0, y: 0.0, z: {-FIXED_ROTATION_SPEED}}}}}' -r 10",
                (float(degrees) * 3.14159265 / 180) / FIXED_ROTATION_SPEED
            ),
            daemon=True
        ).start(),
        "inputs": {"Degrees": "degrees"},  # User inputs only degrees
    },
    "Stop": {
        "function": lambda: send_command(
            "rostopic pub -1 /cmd_vel geometry_msgs/Twist "
            "'{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'"
        ),
        "inputs": {},
    }
}

colour = "#FF5733"  # Orange color for UI
