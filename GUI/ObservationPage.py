import tkinter as tk
from tkinter import ttk, messagebox


class ObservationPage:
    def __init__(self, root, controller):
        # Initialize the observation page
        self.root = root
        self.controller = controller

        # Create the frame for the observation page
        self.frame = tk.Frame(root, bg="white")
        self.setup_ui()

    def setup_ui(self):
        # Set up the UI for the observation page
        self.frame.rowconfigure(0, weight=1)  # Full-height row
        self.frame.columnconfigure(0, weight=1)  # Column for Box A
        self.frame.columnconfigure(
            1, weight=5
        )  # Column for Middle Frame (Boxes B, C, D)

        # Box A: Emergency Controls
        box_a = tk.LabelFrame(
            self.frame,
            text="Emergency Controls",
            bg="white",
            fg="black",
            font=("Arial", 10, "bold"),
        )
        box_a.grid(row=0, column=0, sticky="nsew", padx=5, pady=5)

        # Emergency Stop Button
        tk.Button(
            box_a,
            text="Emergency Stop",
            bg="#FF0000",
            fg="white",
            font=("Arial", 12, "bold"),
            height=3,  # Make the button taller
            command=lambda: self.show_message("Emergency Stop Activated"),
        ).pack(pady=10, padx=10, fill="x")

        # Navigate to Programming Page Button
        tk.Button(
            box_a,
            text="Back to Programming Page",
            bg="#4CAF50",
            fg="white",
            font=("Arial", 12, "bold"),
            command=lambda: self.controller.show_page("PreProgrammingPage"),
        ).pack(pady=10, padx=10, fill="x")

        # Navigate to Start Page Button
        tk.Button(
            box_a,
            text="Back to Start Page",
            bg="#007BFF",
            fg="white",
            font=("Arial", 12, "bold"),
            command=lambda: self.controller.show_page("StartScreen"),
        ).pack(pady=10, padx=10, fill="x")

        # Middle Frame: Contains Boxes B, C, and D
        middle_frame = tk.Frame(self.frame, bg="white")
        middle_frame.grid(row=0, column=1, sticky="nsew", padx=5, pady=5)
        middle_frame.rowconfigure(0, weight=1)  # Box B: Progress Bar
        middle_frame.rowconfigure(1, weight=6)  # Box C: Video Feed (60%)
        middle_frame.rowconfigure(2, weight=4)  # Box D: Command Log (40%)
        middle_frame.columnconfigure(0, weight=1)

        # Box B: Progress Bar
        box_b = tk.Frame(middle_frame, bg="white")
        box_b.grid(row=0, column=0, sticky="ew", padx=5, pady=5)
        progress = ttk.Progressbar(box_b, orient="horizontal", mode="determinate")
        progress.pack(fill="x", pady=10, padx=5)
        progress["value"] = 0  # Simulated progress value

        # Box C: Video Feed
        box_c = tk.LabelFrame(
            middle_frame,
            text="Video Feed",
            bg="white",
            fg="black",
            font=("Arial", 10, "bold"),
        )
        box_c.grid(row=1, column=0, sticky="nsew", padx=5, pady=5)

        # Box D: Command Log
        box_d = tk.LabelFrame(
            middle_frame,
            text="Command Log",
            bg="white",
            fg="black",
            font=("Arial", 10, "bold"),
        )
        box_d.grid(row=2, column=0, sticky="nsew", padx=5, pady=5)

    def show_message(self, message):
        # Display a pop-up message
        messagebox.showinfo("Info", message)
