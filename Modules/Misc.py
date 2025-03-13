import time

COMMANDS = {
    "Pause": {
        "function": lambda inputs: time.sleep(inputs["Time"]),
        "inputs": {"Time": "time"},
    }
}

colour = "#9D00FF"  # Purple for UI
