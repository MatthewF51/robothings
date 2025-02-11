import tkinter as tk
from tkinter import ttk, messagebox
from PIL import Image, ImageTk
import cv2
import threading

# ROS Web Video Server URL (Change IP if needed)
VIDEO_STREAM_URL = "http://192.168.1.100:8080/stream?topic=/camera/image_raw"

class ObservationPage:
    def __init__(self, root, controller):
        # Initialize the observation page
        self.root = root
        self.controller = controller

        # Create the frame for the observation page
        self.frame = tk.Frame(root, bg="white")
        self.video_label = None  # Placeholder for video frame
        self.video_thread = None  # Thread for updating video
        self.running = True  # Flag to control video streaming

        self.setup_ui()
        self.start_video_stream()

    def setup_ui(self):
        # Set up the UI for the observation page
        self.frame.rowconfigure(0, weight=1)
        self.frame.columnconfigure(0, weight=1)  # Column for Emergency Controls
        self.frame.columnconfigure(1, weight=5)  # Column for Video & Command Log

        # Emergency Controls (Left Side)
        box_a = tk.LabelFrame(self.frame, text="Emergency Controls", bg="white", font=("Arial", 10, "bold"))
        box_a.grid(row=0, column=0, sticky="nsew", padx=5, pady=5)

        # Emergency Stop Button
        tk.Button(box_a, text="Emergency Stop", bg="#FF0000", fg="white", font=("Arial", 12, "bold"),
                  height=3, command=lambda: self.show_message("Emergency Stop Activated")).pack(pady=10, padx=10, fill="x")

        # Back to Programming Page
        tk.Button(box_a, text="Back to Programming Page", bg="#4CAF50", fg="white", font=("Arial", 12, "bold"),
                  command=lambda: self.controller.show_page("PreProgrammingPage")).pack(pady=10, padx=10, fill="x")

        # Back to Start Page
        tk.Button(box_a, text="Back to Start Page", bg="#007BFF", fg="white", font=("Arial", 12, "bold"),
                  command=lambda: self.controller.show_page("StartScreen")).pack(pady=10, padx=10, fill="x")

        # Middle Frame: Contains Video & Command Log
        middle_frame = tk.Frame(self.frame, bg="white")
        middle_frame.grid(row=0, column=1, sticky="nsew", padx=5, pady=5)
        middle_frame.rowconfigure(0, weight=6)  # Video (60%)
        middle_frame.rowconfigure(1, weight=4)  # Command Log (40%)
        middle_frame.columnconfigure(0, weight=1)

        # Box C: Video Feed
        box_c = tk.LabelFrame(middle_frame, text="Video Feed", bg="white", font=("Arial", 10, "bold"))
        box_c.grid(row=0, column=0, sticky="nsew", padx=5, pady=5)

        # Placeholder for video feed
        self.video_label = tk.Label(box_c, bg="black")
        self.video_label.pack(fill="both", expand=True)

        # Box D: Command Log
        box_d = tk.LabelFrame(middle_frame, text="Command Log", bg="white", font=("Arial", 10, "bold"))
        box_d.grid(row=1, column=0, sticky="nsew", padx=5, pady=5)

    def start_video_stream(self):
        """Starts a thread to fetch and display video frames."""
        self.video_thread = threading.Thread(target=self.update_video_feed, daemon=True)
        self.video_thread.start()

    def update_video_feed(self):
        """Fetches frames from the ROS video stream and updates the UI."""
        cap = cv2.VideoCapture(VIDEO_STREAM_URL)
        while self.running:
            ret, frame = cap.read()
            if ret:
                frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                image = Image.fromarray(frame)
                photo = ImageTk.PhotoImage(image=image)

                self.video_label.config(image=photo)
                self.video_label.image = photo  # Prevent garbage collection

        cap.release()

    def stop_video_stream(self):
        """Stops the video feed and closes the stream."""
        self.running = False
        if self.video_thread and self.video_thread.is_alive():
            self.video_thread.join()

    def show_message(self, message):
        """Displays a pop-up message."""
        messagebox.showinfo("Info", message)
