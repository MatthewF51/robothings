import keyboard
import subprocess

# Base Movement Commands

linear_speed = 0.0  # Forward/Backward movement
angular_speed = 0.0  # Left/Right rotation
speed_step = 0.1  # Step size for speed changes

def move_base():
    """Send velocity command to Tiago's base."""
    command = f"""rostopic pub /cmd_vel geometry_msgs/Twist '
linear:
  x: {linear_speed}
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: {angular_speed}'"""
    
    subprocess.run(command, shell=True, text=True)

def moveForward():
    global linear_speed
    linear_speed += speed_step
    print("Moving Forward")
    move_base()

def moveBackward():
    global linear_speed
    linear_speed -= speed_step
    print("Moving Backward")
    move_base()

def turnLeft():
    global angular_speed
    angular_speed += speed_step
    print("Turning Left")
    move_base()

def turnRight():
    global angular_speed
    angular_speed -= speed_step
    print("Turning Right")
    move_base()
    
# Head Movement Commands

head_x = 0.0  # Left (-) / Right (+)
head_y = 0.0  # Down (+) / Up (-)
step = 0.1  # Step size for each movement

def move_head():
    """Send the head position command to Tiago."""
    command = f"""rostopic pub /head_controller/command trajectory_msgs/JointTrajectory '
joint_names: ["head_1_joint", "head_2_joint"]
points:
  - positions: [{head_x}, {head_y}]
    time_from_start: {{sec: 2, nanosec: 0}}'"""
    
    subprocess.run(command, shell=True, text=True)

def headUp():
    global head_y
    head_y -= step
    print("Looking Up")
    move_head()

def headDown():
    global head_y
    head_y += step
    print("Looking Down")
    move_head()

def headLeft():
    global head_x
    head_x += step
    print("Looking Left")
    move_head()

def headRight():
    global head_x
    head_x -= step
    print("Looking Right")
    move_head()
    
    
# Moving Torso Commands

torso_pos = 0.0  # 0.0 (down) to 0.34 (up)
step = 0.05  # How much to move per key press

def move_torso():
    """Send the torso lift position command to Tiago."""
    command = f"""rostopic pub /torso_controller/command trajectory_msgs/JointTrajectory '
joint_names: ["torso_lift_joint"]
points:
  - positions: [{torso_pos}]
    time_from_start: {{sec: 2, nanosec: 0}}'"""
    
    subprocess.run(command, shell=True, text=True)

# Define key actions
def torsoUp():
    global torso_pos
    if torso_pos < 0.34:
        torso_pos += step
        print("Rising")
        move_torso()

def torsoDown():
    global torso_pos
    if torso_pos > 0.0:
        torso_pos -= step
        print("Lowering")
        move_torso()
        
# Grip Command        
 
grip_position = 0.0 
GRIP_STEP = 0.1      
        
def grip(position):
    """ Control Tiago's right hand grip """
    command = f"rostopic pub -1 /gripper_controller/command std_msgs/Float64 '{position}'"
    print(f"Adjusting right hand grip: {position}")
    subprocess.run(command, shell=True, executable="/bin/bash") 
                
def tightenGrip():
    global grip_position
    grip_position += GRIP_STEP  # Tighten grip
    if grip_position > 1.0:
        grip_position = 1.0  # Max closed
    print("Tightening")
    grip(grip_position)      
    
def releaseGrip():
    global grip_position
    grip_position -= GRIP_STEP  # Release grip
    if grip_position < 0.0:
        grip_position = 0.0  # Max open
    print("Releasing")
    grip(grip_position)                            
      
# Speech Command       
def speak():
    """Capture text input and send it to Tiago's TTS system."""
    print("Type a message for Tiago to say:")
    user_text = input("> ")  # Wait for user input
    command = f"""rostopic pub /tts/say std_msgs/String '{{data: "{user_text}"}}'"""
    subprocess.run(command, shell=True, text=True)
    print(f'Tiago said: "{user_text}"')
    
# Emergency Stop Commands
emergency_stop_active = False

def toggle_emergency_stop():
    """Toggle emergency stop for Tiago."""
    global emergency_stop_active
    emergency_stop_active = not emergency_stop_active  # Toggle state

    command = f"""rostopic pub /emergency_stop std_msgs/Bool "data: {str(emergency_stop_active).lower()}" """
    subprocess.run(command, shell=True, text=True)

    if emergency_stop_active:
        print("EMERGENCY STOP ACTIVATED")
    else:
        print("EMERGENCY STOP DEACTIVATED")

# Base Movement Keys
keyboard.add_hotkey('w', moveForward)
keyboard.add_hotkey('a', turnLeft)
keyboard.add_hotkey('s', moveBackward)
keyboard.add_hotkey('d', turnRight)

# Head Movement Keys
keyboard.add_hotkey('up', headUp)
keyboard.add_hotkey('left', headLeft)
keyboard.add_hotkey('down', headDown)
keyboard.add_hotkey('right', headRight)

# Torso Movement Keys
keyboard.add_hotkey("shift+up", torsoUp) 
keyboard.add_hotkey("shift+down", torsoDown)  

# Grip Keys
keyboard.add_hotkey('r', releaseGrip)
keyboard.add_hotkey('t', tightenGrip)

# Speech Mode Key
keyboard.add_hotkey("f", speak)

# Emergency Stop Key
keyboard.add_hotkey("shift+e", toggle_emergency_stop)

print("Press 'F' to type a message for Tiago to say. Press 'Esc' to exit.")

keyboard.wait('esc')  # Wait for 'esc' key to exit
