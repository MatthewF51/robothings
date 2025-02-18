from utils import send_command
from geometry_msgs.msg import Twist

COMMANDS = {
    "Move Forward": {
        "function": lambda speed, distance: send_command(
            "/cmd_vel",
            Twist,
            linear_x=float(speed),  # ✅ Correct field name
            linear_y=0.0,
            linear_z=0.0,
            angular_x=0.0,
            angular_y=0.0,
            angular_z=0.0
        ),
        "inputs": {"Speed": "speed", "Distance": "distance"},
        "topic": "/cmd_vel",
        "msg_type": Twist
    },
    "Move Backward": {
        "function": lambda speed, distance: send_command(
            "/cmd_vel",
            Twist,
            linear_x=-float(speed),  # ✅ Correct field name
            linear_y=0.0,
            linear_z=0.0,
            angular_x=0.0,
            angular_y=0.0,
            angular_z=0.0
        ),
        "inputs": {"Speed": "speed", "Distance": "distance"},
        "topic": "/cmd_vel",
        "msg_type": Twist
    },
    "Rotate Left": {
        "function": lambda speed, angle: send_command(
            "/cmd_vel",
            Twist,
            linear_x=0.0,
            linear_y=0.0,
            linear_z=0.0,
            angular_x=0.0,
            angular_y=0.0,
            angular_z=float(speed)  # ✅ Correct field name
        ),
        "inputs": {"Speed": "speed", "Angle": "angle"},
        "topic": "/cmd_vel",
        "msg_type": Twist
    },
    "Rotate Right": {
        "function": lambda speed, angle: send_command(
            "/cmd_vel",
            Twist,
            linear_x=0.0,
            linear_y=0.0,
            linear_z=0.0,
            angular_x=0.0,
            angular_y=0.0,
            angular_z=-float(speed)  # ✅ Correct field name
        ),
        "inputs": {"Speed": "speed", "Angle": "angle"},
        "topic": "/cmd_vel",
        "msg_type": Twist
    },
    "Stop": {
        "function": lambda: send_command(
            "/cmd_vel",
            Twist,
            linear_x=0.0,
            linear_y=0.0,
            linear_z=0.0,
            angular_x=0.0,
            angular_y=0.0,
            angular_z=0.0
        ),
        "inputs": {},
        "topic": "/cmd_vel",
        "msg_type": Twist
    }
}

colour = "#FF5733"  # Orange color for UI
