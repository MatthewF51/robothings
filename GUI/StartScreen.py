import tkinter as tk


class StartScreen:
    def __init__(self, parent, controller):
        # Initialize the start screen
        self.controller = controller

        # Create the frame for this page
        self.frame = tk.Frame(parent, bg="white")
        self.setup_ui()

    def setup_ui(self):
        # Set up the UI for the start screen
        self.frame.rowconfigure([0, 1, 2], weight=1)
        self.frame.columnconfigure(0, weight=1)

        # Create a title label
        title_label = tk.Label(
            self.frame,
            text="CtrlWizard",
            font=("Arial", 24, "bold"),
            fg="black",
            bg="white",
        )
        title_label.grid(row=0, column=0, pady=20, sticky="n")

        # Create Pre-Programming button
        pre_programming_button = tk.Button(
            self.frame,
            text="Pre-Programming",
            width=20,
            height=2,
            bg="#4CAF50",  # Green
            fg="white",
            font=("Arial", 12, "bold"),
            command=lambda: self.controller.show_page("PreProgrammingPage"),
        )
        pre_programming_button.grid(row=1, column=0, pady=10, sticky="n")

        # Create Manual Control button (future functionality)
        manual_control_button = tk.Button(
            self.frame,
            text="Manual Control",
            width=20,
            height=2,
            bg="#007BFF",  # Blue
            fg="white",
            font=("Arial", 12, "bold"),
            # command=lambda: self.controller.show_page("ManualControlPage")
        )
        manual_control_button.grid(row=2, column=0, pady=10, sticky="n")
