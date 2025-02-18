COMMANDS = {
    "Move Forward": {
        "function": lambda speed, distance: send_command(
            f"rostopic pub -1 /cmd_vel geometry_msgs/Twist '{{linear: {{x: {speed}, y: 0.0, z: 0.0}}, angular: {{x: 0.0, y: 0.0, z: 0.0}}}}'"
        ),
        "inputs": {"Speed": "speed", "Distance": "distance"},
    },
    "Move Backward": {
        "function": lambda speed, distance: send_command(
            f"rostopic pub -1 /cmd_vel geometry_msgs/Twist '{{linear: {{x: {-speed}, y: 0.0, z: 0.0}}, angular: {{x: 0.0, y: 0.0, z: 0.0}}}}'"
        ),
        "inputs": {"Speed": "speed", "Distance": "distance"},
    },
    "Rotate Left": {
        "function": lambda speed, angle: send_command(
            f"rostopic pub -1 /cmd_vel geometry_msgs/Twist '{{linear: {{x: 0.0, y: 0.0, z: 0.0}}, angular: {{x: 0.0, y: 0.0, z: {speed}}}}}'"
        ),
        "inputs": {"Speed": "speed", "Angle": "angle"},
    },
    "Rotate Right": {
        "function": lambda speed, angle: send_command(
            f"rostopic pub -1 /cmd_vel geometry_msgs/Twist '{{linear: {{x: 0.0, y: 0.0, z: 0.0}}, angular: {{x: 0.0, y: 0.0, z: {-speed}}}}}'"
        ),
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
