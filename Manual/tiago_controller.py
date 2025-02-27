import pygame
import subprocess
import time

# Initialize pygame and joystick
pygame.init()
pygame.joystick.init()

# Check if a joystick is connected
if pygame.joystick.get_count() == 0:
    print("No joystick detected. Please connect a Logitech controller.")
    exit()

# Initialize the first joystick
joystick = pygame.joystick.Joystick(0)
joystick.init()
print(f"Connected to: {joystick.get_name()}")

# Axis mappings
AXIS_LEFT_Y = 1  # Forward/Backward movement
AXIS_LEFT_X = 0  # Rotation left/right
AXIS_RIGHT_Y = 3 # Head up/down
AXIS_RIGHT_X = 2 # Head left/right
AXIS_RT = 5      # Right Trigger (Tighten Grip)
AXIS_LT = 4      # Left Trigger (Loosen Grip)

# D-Pad mappings (hat switch)
DPAD_UP = (0, 1)
DPAD_DOWN = (0, -1)

# Button mappings
BUTTON_A = 0   # Make Tiago speak
BUTTON_L_STICK = 8  # Left stick click
BUTTON_R_STICK = 9  # Right stick click

# Speed settings
MAX_LINEAR_SPEED = 0.5  # Maximum forward/backward speed
MAX_ANGULAR_SPEED = 1.0 # Maximum rotation speed
MAX_HEAD_SPEED = 0.5    # Maximum head movement speed
TOROSO_STEP = 0.05      # Step size for torso movement
GRIP_STEP = 0.1         # Step size for grip control
DEADZONE = 0.1          # Ignore small movements

# Default grip state
grip_position = 0.0  # Open hand

def send_cmd_vel(linear_speed, angular_speed):
    """ Send movement command to Tiago via ROS """
    command = f"rostopic pub -1 /cmd_vel geometry_msgs/Twist '{{linear: {{x: {linear_speed}, y: 0.0, z: 0.0}}, angular: {{x: 0.0, y: 0.0, z: {angular_speed}}}}}'"
    print(f"Executing: {command}")
    subprocess.run(command, shell=True, executable="/bin/bash")

def send_head_command(pan, tilt):
    """ Move Tiago's head """
    command = f"rostopic pub -1 /head_controller/command trajectory_msgs/JointTrajectory '{{joint_names: [\"head_1_joint\", \"head_2_joint\"], points: [{{positions: [{pan}, {tilt}], time_from_start: {{secs: 1, nsecs: 0}}}}]}}'"
    print(f"Moving head: pan={pan}, tilt={tilt}")
    subprocess.run(command, shell=True, executable="/bin/bash")

def send_torso_command(height):
    """ Raise or lower Tiago's torso """
    command = f"rostopic pub -1 /torso_controller/command std_msgs/Float64 '{height}'"
    print(f"Adjusting torso to height: {height}")
    subprocess.run(command, shell=True, executable="/bin/bash")

def send_grip_command(position):
    """ Control Tiago's right hand grip """
    command = f"rostopic pub -1 /gripper_controller/command std_msgs/Float64 '{position}'"
    print(f"Adjusting right hand grip: {position}")
    subprocess.run(command, shell=True, executable="/bin/bash")

def emergency_stop():
    """ Send emergency stop command """
    command = "rostopic pub -1 /emergency_stop std_msgs/Bool 'true'"
    print("Emergency Stop Activated!")
    subprocess.run(command, shell=True, executable="/bin/bash")

def make_tiago_speak():
    """ Make Tiago say a phrase """
    command = "rostopic pub -1 /sound/play_sound std_msgs/String '\"Hello, I am Tiago!\"'"
    print("Tiago speaking: 'Hello, I am Tiago!'")
    subprocess.run(command, shell=True, executable="/bin/bash")

# Main loop
running = True
while running:
    linear_speed = 0.0
    angular_speed = 0.0
    head_pan = 0.0
    head_tilt = 0.0
    torso_height = 0.35  # Default torso height

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        
        elif event.type == pygame.JOYAXISMOTION:
            if event.axis == AXIS_LEFT_Y:
                linear_speed = -event.value  # Invert Y-axis for forward/backward
            elif event.axis == AXIS_LEFT_X:
                angular_speed = event.value  # Left = negative, Right = positive
            elif event.axis == AXIS_RIGHT_X:
                head_pan = event.value * MAX_HEAD_SPEED  # Move head left/right
            elif event.axis == AXIS_RIGHT_Y:
                head_tilt = -event.value * MAX_HEAD_SPEED  # Move head up/down
            elif event.axis == AXIS_RT:
                grip_position += GRIP_STEP  # Tighten grip
                if grip_position > 1.0:
                    grip_position = 1.0  # Max closed
            elif event.axis == AXIS_LT:
                grip_position -= GRIP_STEP  # Loosen grip
                if grip_position < 0.0:
                    grip_position = 0.0  # Fully open

        elif event.type == pygame.JOYHATMOTION:
            dpad = event.value
            if dpad == DPAD_UP:
                torso_height += TOROSO_STEP
            elif dpad == DPAD_DOWN:
                torso_height -= TOROSO_STEP

        elif event.type == pygame.JOYBUTTONDOWN:
            if event.button == BUTTON_A:
                make_tiago_speak()
            elif event.button == BUTTON_L_STICK and event.button == BUTTON_R_STICK:
                emergency_stop()

    # Apply deadzone
    if abs(linear_speed) < DEADZONE:
        linear_speed = 0.0
    if abs(angular_speed) < DEADZONE:
        angular_speed = 0.0
    if abs(head_pan) < DEADZONE:
        head_pan = 0.0
    if abs(head_tilt) < DEADZONE:
        head_tilt = 0.0

    # Scale speed values
    linear_speed = round(linear_speed * MAX_LINEAR_SPEED, 2)
    angular_speed = round(angular_speed * MAX_ANGULAR_SPEED, 2)

    # Send movement commands
    send_cmd_vel(linear_speed, angular_speed)
    send_head_command(head_pan, head_tilt)
    send_torso_command(torso_height)
    send_grip_command(grip_position)

pygame.quit()
