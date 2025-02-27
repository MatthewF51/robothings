from utils import send_command

def speak(text):
    """Sends a text-to-speech command using send_command() to execute in the terminal."""
    formatted_text = f"'{text}'"  # Ensure proper quoting

    # Construct the correct rostopic pub command for Tiago's TTS system
    tts_command = f"""rostopic pub -1 /tts/goal pal_interaction_msgs/TtsActionGoal "{{
        goal: {{
            rawtext: {{
                text: {formatted_text},
                lang_id: 'en_GB'
            }}
        }}
    }}" """

    send_command(tts_command)  # Execute the command

# GUI Command Configuration
COMMANDS = {
    "Speak Text": {
        "function": speak,
        "inputs": {"Text": "text"},
    }
}

colour = "#FFA500"  # Orange for UI
