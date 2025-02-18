import rospy
import importlib
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

def send_command(topic, msg_type, **kwargs):
    """
    Publishes a message to a given topic.

    :param topic: The ROS topic to publish to.
    :param msg_type: The message type (e.g., Twist, String).
    :param kwargs: The fields and values for the message.
    """
    if rospy.is_shutdown():
        rospy.logerr("ROS is not running!")
        return

    pub = get_publisher(topic, msg_type)
    msg = msg_type()  # Create an instance of the message type

    # Dynamically set message attributes
    for key, value in kwargs.items():
        # Convert field names (linear_x → linear.x)
        key = key.replace("_", ".")  # ✅ Ensure proper ROS nested field format
        
        field_path = key.split(".")
        sub_msg = msg

        for sub_field in field_path[:-1]:  # Navigate to nested field
            if hasattr(sub_msg, sub_field):
                sub_msg = getattr(sub_msg, sub_field)
            else:
                rospy.logwarn(f"Invalid field '{key}' for message type {msg_type}")
                return

        if hasattr(sub_msg, field_path[-1]):  # Assign final value
            setattr(sub_msg, field_path[-1], value)
        else:
            rospy.logwarn(f"Invalid field '{key}' for message type {msg_type}")
            return

    pub.publish(msg)

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
