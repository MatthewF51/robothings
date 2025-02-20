from utils import send_command

def speak(text):
    """Sends a text-to-speech command using send_command() to execute in the terminal."""
    send_command(f"rosservice call /tts '{text}'")  # Adjust this based on Tiago's TTS service

# GUI Command Configuration
COMMANDS = {
    "Speak Text": {
        "function": speak,
        "inputs": {"Text": "text"},
    }
}

colour = "#FFA500"  # Orange for UI
