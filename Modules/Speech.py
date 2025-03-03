import threading
from utils import send_command

COMMANDS = {
    "Speak Text": {
        "function": lambda text: threading.Thread(
            target=send_command,
            args=(
                f"""rostopic pub -1 /tts/goal pal_interaction_msgs/TtsActionGoal "{{
                    goal: {{
                        rawtext: {{
                            text: '{text}',
                            lang_id: 'en_GB'
                        }}
                    }}
                }}" """,
            ),
            daemon=True,
        ).start(),
        "inputs": {"Enter Text": "text"},  # Match movement input format
    }
}

colour = "#FFA500"  # Orange color for UI
