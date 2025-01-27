def forward(distance):
    return f"Move Forward {distance} meters"


def backward(distance):
    return f"Move Backward {distance} meters"


def turn_left(degrees):
    return f"Turn Left {degrees} degrees"


def turn_right(degrees):
    return f"Turn Right {degrees} degrees"


COMMANDS = {
    "Forward": {
        "func": forward,
        "inputs": {"Distance (meters)": "distance"},
    },
    "Backward": {
        "func": backward,
        "inputs": {"Distance (meters)": "distance"},
    },
    "Turn Left": {
        "func": turn_left,
        "inputs": {"Degrees": "degrees"},
    },
    "Turn Right": {
        "func": turn_right,
        "inputs": {"Degrees": "degrees"},
    },
}

colour = "#FF5733"
