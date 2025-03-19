import threading
from utils import send_command

# Constants
FIXED_ROTATION_SPEED = 1  # rad/s
COMPENSATION_TIME = 1.5  # Extra time to account for delay

COMMANDS = {
    "Move Forward": {
        "function": lambda distance: threading.Thread(
            target=send_command,
            args=(
                f"rostopic pub /mobile_base_controller/cmd_vel geometry_msgs/Twist "
                f"'{{linear: {{x: 0.5, y: 0.0, z: 0.0}}, angular: {{x: 0.0, y: 0.0, z: 0.0}}}}' -r 10",
                (float(distance) / 0.5) + COMPENSATION_TIME,  # Compensate for delay
            ),
            daemon=True,
        ).start(),
        "inputs": {"Distance": "distance"},
    },
    "Move Backward": {
        "function": lambda distance: threading.Thread(
            target=send_command,
            args=(
                f"rostopic pub /mobile_base_controller/cmd_vel geometry_msgs/Twist "
                f"'{{linear: {{x: -0.5, y: 0.0, z: 0.0}}, angular: {{x: 0.0, y: 0.0, z: 0.0}}}}' -r 10",
                (float(distance) / 0.5) + COMPENSATION_TIME,
            ),
            daemon=True,
        ).start(),
        "inputs": {"Distance": "distance"},
    },
    "Rotate Left": {
        "function": lambda degrees: threading.Thread(
            target=send_command,
            args=(
                f"rostopic pub /mobile_base_controller/cmd_vel geometry_msgs/Twist "
                f"'{{linear: {{x: 0.0, y: 0.0, z: 0.0}}, angular: {{x: 0.0, y: 0.0, z: {FIXED_ROTATION_SPEED}}}}}' -r 10",
                ((float(degrees) * 3.14159265 / 180) / FIXED_ROTATION_SPEED)
                + COMPENSATION_TIME,
            ),
            daemon=True,
        ).start(),
        "inputs": {"Degrees": "degrees"},
    },
    "Rotate Right": {
        "function": lambda degrees: threading.Thread(
            target=send_command,
            args=(
                f"rostopic pub /mobile_base_controller/cmd_vel geometry_msgs/Twist "
                f"'{{linear: {{x: 0.0, y: 0.0, z: 0.0}}, angular: {{x: 0.0, y: 0.0, z: {-FIXED_ROTATION_SPEED}}}}}' -r 10",
                ((float(degrees) * 3.14159265 / 180) / FIXED_ROTATION_SPEED)
                + COMPENSATION_TIME,
            ),
            daemon=True,
        ).start(),
        "inputs": {"Degrees": "degrees"},
    },
}

colour = "#FF5733"  # Orange color for UI
