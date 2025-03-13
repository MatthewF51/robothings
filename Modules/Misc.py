import time

COMMANDS = {
    "Pause": {
        "function": lambda Time: time.sleep(Time),
        "inputs": {"Time": "time"},
    }
}

colour = "#9D00FF"  # Purple for UI
