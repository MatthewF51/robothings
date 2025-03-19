import tkinter as tk

def con_controls():   
    controls_window = tk.Toplevel()
    controls_window.title("Controller Controls")
    controls_window.geometry("300x200")
    controls_window.configure(bg="white")

    label = tk.Label(controls_window, text="Controller Control Instructions", font=("Arial", 14), bg="white")
    label.pack(pady=20)

    instructions = """
    Left Stick - Move
    Right Stick - Look Around
    A Button - Start
    B Button - Stop
    """
    text_widget = tk.Label(controls_window, text=instructions, font=("Arial", 12), bg="white", justify="left")
    text_widget.pack(pady=10)