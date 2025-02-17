from utils import send_command

def speak(text):
    """Sends a text-to-speech command using the existing send_command method."""
    send_command("/tts", {"data": text})  # Format as a dictionary with "data" key

# Example Commands for GUI Integration
COMMANDS = {
    "Speak Text": {
        "function": lambda text: speak(text),
        "inputs": {"Text": "text"},
        "topic": "/tts"  # Explicitly define the topic
    }
}

colour = "#FFA500"  # Orange color for UI
