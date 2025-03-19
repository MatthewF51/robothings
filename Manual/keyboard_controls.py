import tkinter as tk

def key_controls():   
    controls_window = tk.Toplevel()
    controls_window.title("Keyboard Controls")
    controls_window.geometry("300x200")
    controls_window.configure(bg="white")

    label = tk.Label(controls_window, text="Keyboard Control Instructions", font=("Arial", 14), bg="white")
    label.pack(pady=20)

    instructions = """
    You must hit enter to send keyboard command. CASE SENSITIVE COMMANDS.
    For the following 4 controls, press the key more to move further.
    
    w - Move Forward
    s - Move Backward
    a - Turn Left
    d - Turn Right
    i - Look Up
    k - Look Down
    j - Look Left
    l - Look Right
    
    Only send one key for the following
    
    W - Raise Torso
    S - Lower Torso
    R - Release Grip
    T - Tighten Grip
    y - Say Hello
    o - Offer Hand
    p - Return Hand
    E - Cancel Action
    EE - Emergency Stop
        
    Press 'q' to enter advanced arm control mode. Then select a joint by pressing a number.
    Then control using r and t (send multiple of same key to move further). After selecting
    a joint, press 'q' to go back to joint selection, and press again to go back to basic
    control.
    
    1 -
    2 -
    3 -
    4 -
    5 - 
    6 - 
    7 - 
    8 - 
    9 -
    
    """
    text_widget = tk.Label(controls_window, text=instructions, font=("Arial", 12), bg="white", justify="left")
    text_widget.pack(pady=10)