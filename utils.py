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

def send_command(topic, msg_type, message=None, **kwargs):
    """
    Publishes a message to a given topic.

    :param topic: The ROS topic to publish to.
    :param msg_type: The message type (e.g., Twist, String).
    :param message: A pre-built message (optional).
    :param kwargs: The fields and values for the message if not using a pre-built message.
    """
    if rospy.is_shutdown():
        rospy.logerr("ROS is not running!")
        return

    pub = get_publisher(topic, msg_type)

    # If a raw message object is provided, publish it directly
    if message:
        pub.publish(message)
        return

    # Otherwise, construct the message from kwargs
    msg = msg_type()
    
    for key, value in kwargs.items():
        field_path = key.split('.')
        sub_msg = msg
        for sub_field in field_path[:-1]:  # Navigate into nested fields
            sub_msg = getattr(sub_msg, sub_field)
        setattr(sub_msg, field_path[-1], value)  # Assign the final value

    pub.publish(msg)
