from utils import send_command

COMMANDS = {
    "Move Arm Forward": {
        "function": lambda distance: send_command(
            "rosservice call /arm_teleop/move_rel", f"'x: {distance} y: 0.0 z: 0.0'"
        ),
        "inputs": {"Distance (m)": "distance"},
    },
    "Move Arm Backward": {
        "function": lambda distance: send_command(
            "rosservice call /arm_teleop/move_rel", f"'x: -{distance} y: 0.0 z: 0.0'"
        ),
        "inputs": {"Distance (m)": "distance"},
    },
    "Move Arm Left": {
        "function": lambda distance: send_command(
            "rosservice call /arm_teleop/move_rel", f"'x: 0.0 y: {distance} z: 0.0'"
        ),
        "inputs": {"Distance (m)": "distance"},
    },
    "Move Arm Right": {
        "function": lambda distance: send_command(
            "rosservice call /arm_teleop/move_rel", f"'x: 0.0 y: -{distance} z: 0.0'"
        ),
        "inputs": {"Distance (m)": "distance"},
    },
    "Move Arm Up": {
        "function": lambda distance: send_command(
            "rosservice call /arm_teleop/move_rel", f"'x: 0.0 y: 0.0 z: {distance}'"
        ),
        "inputs": {"Distance (m)": "distance"},
    },
    "Move Arm Down": {
        "function": lambda distance: send_command(
            "rosservice call /arm_teleop/move_rel", f"'x: 0.0 y: 0.0 z: -{distance}'"
        ),
        "inputs": {"Distance (m)": "distance"},
    },
    "Rotate Wrist Clockwise": {
        "function": lambda angle: send_command(
            "rosservice call /arm_teleop/rotate_wrist", f"'angle: {angle}'"
        ),
        "inputs": {"Angle (degrees)": "angle"},
    },
    "Rotate Wrist Counterclockwise": {
        "function": lambda angle: send_command(
            "rosservice call /arm_teleop/rotate_wrist", f"'angle: -{angle}'"
        ),
        "inputs": {"Angle (degrees)": "angle"},
    },
    "Grasp (Close Hand)": {
        "function": lambda: send_command("rosservice call /gripper_controller/grasp"),
        "inputs": {},
    },
    "Release (Open Hand)": {
        "function": lambda: send_command("rosservice call /gripper_controller/release"),
        "inputs": {},
    },
}

colour = "#00A6FF"  # Light blue for UI
