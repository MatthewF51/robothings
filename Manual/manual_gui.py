import tkinter as tk
from tkinter import messagebox
import multiprocessing
import tiago_keyboard
import tiago_controller
import keyboard_controls
import controller_controls

# Globals to hold the process objects
keyboard_process = None
controller_process = None

# Track which process is active
active_process = None  # 'keyboard', 'controller'

def run_keyboard_control():
    global keyboard_process, controller_process, active_process

    # Stop the controller process if it's running
    if controller_process is not None and controller_process.is_alive():
        controller_process.terminate()
        controller_process.join()
        controller_process = None
        print("Stopped Controller Control")

    # Start the keyboard process if not already running
    if keyboard_process is None or not keyboard_process.is_alive():
        keyboard_process = multiprocessing.Process(target=tiago_keyboard.runKeyboard)
        keyboard_process.start()
        active_process = 'keyboard'
        print("Started Keyboard Control")

def run_controller_control():
    global keyboard_process, controller_process, active_process

    # Stop the keyboard process if it's running
    if keyboard_process is not None and keyboard_process.is_alive():
        keyboard_process.terminate()
        keyboard_process.join()
        keyboard_process = None
        print("Stopped Keyboard Control")

    # Start the controller process if not already running
    if controller_process is None or not controller_process.is_alive():
        controller_process = multiprocessing.Process(target=tiago_controller.runController)
        controller_process.start()
        active_process = 'controller'
        print("Started Controller Control")

def show_controls_window():
    if active_process == 'keyboard':
        show_keyboard_controls()
    elif active_process == 'controller':
        show_controller_controls()
    else:
        messagebox.showinfo("No Active Process", "Please start either Keyboard or Controller Control first.")

def show_keyboard_controls():
    print("Opening Keyboard Controls Window...")
    keyboard_controls.key_controls()

def show_controller_controls():
    print("Opening Controller Controls Window...")
    controller_controls.con_controls()

def create_gui():
    # Create the main window
    window = tk.Tk()
    window.title("Tiago Control Panel")
    window.geometry("600x400")  # Bigger window to fit extra button
    window.configure(bg="white")

    # Button styling
    button_style = {
        "bg": "#4CAF50",          # Nice green color
        "fg": "white",            # White text
        "activebackground": "#45a049",  # Darker green when pressed
        "font": ("Arial", 14),    # Bigger font
        "width": 20,              # Wider button
        "height": 3,              # Taller button
        "bd": 0,                  # No border
        "relief": "flat",         # Flat look
        "cursor": "hand2"         # Pointer cursor on hover
    }

    # Create a frame to hold the two control buttons side by side
    button_frame = tk.Frame(window, bg="white")
    button_frame.pack(expand=True)

    # Button 1 - Run Keyboard Control
    button1 = tk.Button(
        button_frame, 
        text="Run Keyboard Control", 
        command=run_keyboard_control,
        **button_style
    )
    button1.pack(side="left", padx=20, pady=10)

    # Button 2 - Run Controller Control
    button2 = tk.Button(
        button_frame, 
        text="Run Controller Control", 
        command=run_controller_control,
        **button_style
    )
    button2.pack(side="left", padx=20, pady=10)

    # Show Controls Button - Centered below the two
    show_controls_button = tk.Button(
        window,
        text="Show Controls",
        command=show_controls_window,
        bg="#2196F3",                # Blue color for difference
        fg="white",
        activebackground="#1976D2",  # Darker blue on click
        font=("Arial", 14),
        width=20,
        height=2,
        bd=0,
        relief="flat",
        cursor="hand2"
    )
    show_controls_button.pack(pady=40)

    # Handle closing the window gracefully
    def on_closing():
        # Terminate both processes if they are running
        if keyboard_process is not None and keyboard_process.is_alive():
            keyboard_process.terminate()
            keyboard_process.join()
        if controller_process is not None and controller_process.is_alive():
            controller_process.terminate()
            controller_process.join()
        window.destroy()

    window.protocol("WM_DELETE_WINDOW", on_closing)

    # Run the window loop
    window.mainloop()

if __name__ == "__main__":
    multiprocessing.set_start_method('spawn')  # Ensures compatibility across OSes
    create_gui()
