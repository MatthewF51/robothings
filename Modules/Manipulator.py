from utils import send_command

COMMANDS = {
    "Move to Home Pose": {
        "function": lambda: send_command(
            "rosrun play_motion run_motion_python_node.py home"
        ),
        "inputs": {},
    },
    "Move to Pick Pose": {
        "function": lambda: send_command(
            "rosrun play_motion run_motion_python_node.py pick"
        ),
        "inputs": {},
    },
    "Move to Pregrasp Pose": {
        "function": lambda: send_command(
            "rosrun play_motion run_motion_python_node.py pregrasp"
        ),
        "inputs": {},
    },
    "Move to Transport Pose": {
        "function": lambda: send_command(
            "rosrun play_motion run_motion_python_node.py transport"
        ),
        "inputs": {},
    },
    "Move to Handover Pose": {
        "function": lambda: send_command(
            "rosrun play_motion run_motion_python_node.py handover"
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
