from utils import send_command

def speak(text):
    """Sends a text-to-speech command using send_command() to execute in the terminal."""
    formatted_text = f"'{text}'"  # Ensure the text is properly quoted
    send_command(f"rosservice call /tts \"{{text: {formatted_text}}}\"")  # Adjust if Tiago's service expects different input

COMMANDS = {
    "Speak Text": {
        "function": lambda text: speak(text),  # Explicitly pass the text argument
        "inputs": {"Text to Speak": "text"},  # Ensure the UI recognizes it
    }
}

colour = "#FFA500"  # Orange for UI

