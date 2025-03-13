from utils import send_command

COMMANDS = {
    "Move to Home Pose": {
        "function": lambda: send_command(
            "rosrun play_motion run_motion_python_node.py home"
        ),
        "inputs": {},
    },
    "Move to Offer Pose": {
        "function": lambda: send_command(
            "rosrun play_motion run_motion_python_node.py offer_right"
        ),
        "inputs": {},
    },
    "Open Grip": {
        "function": lambda: send_command(
            "rosrun play_motion run_motion_python_node.py open_right"
        ),
        "inputs": {},
    },
    "Grip": {
        "function": lambda: send_command(
            "rosrun play_motion run_motion_python_node.py close_right"
        ),
        "inputs": {},
    },
    "Move to Transport Pose": {
        "function": lambda: send_command(
            "rosrun play_motion run_motion_python_node.py home_right"
        ),
        "inputs": {},
    },
    "Wave": {
        "function": lambda: send_command(
            "rosrun play_motion run_motion_python_node.py wave"
        ),
        "inputs": {},
    },
}

colour = "#00A6FF"  # Light blue for UI
