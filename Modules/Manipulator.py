from utils import send_command

COMMANDS = {
    "Move to Home Pose": {
        "function": lambda: send_command(
            "rosservice call /play_motion 'motion_name: \"home\" skip_planning: false'"
        ),
        "inputs": {},
    },
    "Move to Pick Pose": {
        "function": lambda: send_command(
            "rosservice call /play_motion 'motion_name: \"pick\" skip_planning: false'"
        ),
        "inputs": {},
    },
    "Move to Pregrasp Pose": {
        "function": lambda: send_command(
            "rosservice call /play_motion 'motion_name: \"pregrasp\" skip_planning: false'"
        ),
        "inputs": {},
    },
    "Move to Transport Pose": {
        "function": lambda: send_command(
            "rosservice call /play_motion 'motion_name: \"transport\" skip_planning: false'"
        ),
        "inputs": {},
    },
    "Move to Handover Pose": {
        "function": lambda: send_command(
            "rosservice call /play_motion 'motion_name: \"handover\" skip_planning: false'"
        ),
        "inputs": {},
    },
    "Wave": {
        "function": lambda: send_command(
            "rosservice call /play_motion 'motion_name: \"wave\" skip_planning: false'"
        ),
        "inputs": {},
    },
}

colour = "#00A6FF"  # Light blue for UI
