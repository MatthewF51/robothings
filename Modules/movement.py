from utils import send_command

COMMANDS = {
    "Move Forward": {
        "function": lambda speed, distance: send_command(
            "/cmd_vel",
            {
                "linear": {"x": float(speed), "y": 0.0, "z": 0.0},
                "angular": {"x": 0.0, "y": 0.0, "z": 0.0}
            },
            duration=(float(distance) / float(speed)) if float(speed) != 0 else 0
        ),
        "inputs": {"Speed": "speed", "Distance": "distance"},
        "topic": "/cmd_vel"
    },
    "Move Backward": {
        "function": lambda speed, distance: send_command(
            "/cmd_vel",
            {
                "linear": {"x": -float(speed), "y": 0.0, "z": 0.0},
                "angular": {"x": 0.0, "y": 0.0, "z": 0.0}
            },
            duration=(float(distance) / float(speed)) if float(speed) != 0 else 0
        ),
        "inputs": {"Speed": "speed", "Distance": "distance"},
        "topic": "/cmd_vel"
    },
    "Rotate Left": {
        "function": lambda speed, angle: send_command(
            "/cmd_vel",
            {
                "linear": {"x": 0.0, "y": 0.0, "z": 0.0},
                "angular": {"x": 0.0, "y": 0.0, "z": float(speed)}
            },
            duration=(float(angle) / float(speed)) if float(speed) != 0 else 0
        ),
        "inputs": {"Speed": "speed", "Angle": "angle"},
        "topic": "/cmd_vel"
    },
    "Rotate Right": {
        "function": lambda speed, angle: send_command(
            "/cmd_vel",
            {
                "linear": {"x": 0.0, "y": 0.0, "z": 0.0},
                "angular": {"x": 0.0, "y": 0.0, "z": -float(speed)}
            },
            duration=(float(angle) / float(speed)) if float(speed) != 0 else 0
        ),
        "inputs": {"Speed": "speed", "Angle": "angle"},
        "topic": "/cmd_vel"
    },
    "Stop": {
        "function": lambda: send_command(
            "/cmd_vel",
            {
                "linear": {"x": 0.0, "y": 0.0, "z": 0.0},
                "angular": {"x": 0.0, "y": 0.0, "z": 0.0}
            }
        ),
        "inputs": {},
        "topic": "/cmd_vel"
    }
}

colour = "#FF5733"
