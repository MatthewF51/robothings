import rospy
import importlib
import subprocess
import os

# Dictionary to store publishers
publishers = {}

def initialize_ros():
    """Initializes ROS and sets up publishers for multiple topics."""
    if not rospy.core.is_initialized():
        rospy.init_node("tiago_gui", anonymous=True)

def get_publisher(topic, msg_type):
    """Returns a publisher for the given topic and message type, creating one if needed."""
    if topic not in publishers:
        publishers[topic] = rospy.Publisher(topic, msg_type, queue_size=10)
        rospy.sleep(0.5)  # Allow publisher to register
    return publishers[topic]


def send_command(command):
    """
    Sends a ROS command to the terminal.

    :param command: The full ROS command as a string (e.g., "rostopic pub /cmd_vel geometry_msgs/Twist ...").
    """
    try:
        print(f"Executing: {command}")  # Debugging output
        subprocess.run(command, shell=True, check=True)
    except subprocess.CalledProcessError as e:
        print(f"Error executing command: {e}")


def load_modules():
    """Dynamically loads modules from the 'Modules' folder."""
    modules_path = os.path.join(os.path.dirname(__file__), "Modules")
    modules = {}

    for module_name in os.listdir(modules_path):
        if module_name.endswith(".py") and module_name != "__init__.py":
            module_name = module_name[:-3]  # Remove .py extension
            module = importlib.import_module(f"Modules.{module_name}")
            
            if hasattr(module, "COMMANDS") and hasattr(module, "colour"):
                modules[module_name] = {
                    "commands": module.COMMANDS,
                    "color": module.colour
                }

    return modules
