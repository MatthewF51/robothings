import tkinter as tk
from tkinter import ttk, messagebox
from PIL import Image, ImageTk
import cv2
import threading
import rospy  # type: ignore
from sensor_msgs.msg import Image as ROSImage  # type: ignore
from cv_bridge import CvBridge  # type: ignore
from geometry_msgs.msg import Twist  # type: ignore
import os
import time
import signal
from tkinter import messagebox
import subprocess

# ROS Topic for Tiago's Camera
CAMERA_TOPIC = "/xtion/rgb/image_raw"


class ObservationPage:
    # Sets up the Observation Page
    def __init__(self, root, controller):
        self.root = root
        self.controller = controller
        self.frame = tk.Frame(root, bg="white")

        # Video feed variables
        self.video_label = None
        self.bridge = CvBridge()
        self.running = True  # Flag for video stream
        self.progress_value = tk.DoubleVar()  # Progress bar value

        # ROS Setup
        rospy.init_node("observation_page", anonymous=True)
        self.video_subscriber = rospy.Subscriber(
            CAMERA_TOPIC, ROSImage, self.update_video_feed
        )

        self.setup_ui()

    def setup_ui(self):
        # Setup the UI Layout
        self.frame.rowconfigure(0, weight=1)
        self.frame.columnconfigure(0, weight=1)  # Emergency Controls
        self.frame.columnconfigure(1, weight=5)  # Video & Command Log

        # Emergency Controls (Left Panel)
        box_a = tk.LabelFrame(
            self.frame,
            text="Emergency Controls",
            bg="white",
            font=("Arial", 10, "bold"),
        )
        box_a.grid(row=0, column=0, sticky="nsew", padx=5, pady=5)

        tk.Button(
            box_a,
            text="Emergency Stop",
            bg="#FF0000",
            fg="white",
            font=("Arial", 12, "bold"),
            height=3,
            command=self.emergency_stop,
        ).pack(pady=10, padx=10, fill="x")

        tk.Button(
            box_a,
            text="Back to Programming Page",
            bg="#4CAF50",
            fg="white",
            font=("Arial", 12, "bold"),
            command=lambda: self.controller.show_page("PreProgrammingPage"),
        ).pack(pady=10, padx=10, fill="x")

        tk.Button(
            box_a,
            text="Back to Start Page",
            bg="#007BFF",
            fg="white",
            font=("Arial", 12, "bold"),
            command=lambda: self.controller.show_page("StartScreen"),
        ).pack(pady=10, padx=10, fill="x")

        # Middle Frame (Video + Command Log)
        middle_frame = tk.Frame(self.frame, bg="white")
        middle_frame.grid(row=0, column=1, sticky="nsew", padx=5, pady=5)
        middle_frame.rowconfigure(0, weight=1)  # Progress Bar (10%)
        middle_frame.rowconfigure(1, weight=5)  # Video Feed (50%)
        middle_frame.rowconfigure(2, weight=4)  # Command Log (40%)
        middle_frame.columnconfigure(0, weight=1)

        # **NEW** Progress Bar (TOP of Page)
        progress_frame = tk.Frame(middle_frame, bg="white")
        progress_frame.grid(row=0, column=0, sticky="nsew", padx=5, pady=5)

        self.progress_bar = ttk.Progressbar(
            progress_frame,
            variable=self.progress_value,
            maximum=100,
            mode="determinate",
        )
        self.progress_bar.pack(fill="x", padx=10, pady=5)

        self.progress_label = tk.Label(
            progress_frame,
            text="Execution Progress: 0%",
            bg="white",
            font=("Arial", 10),
        )
        self.progress_label.pack(pady=5)

        # Video Feed
        box_c = tk.LabelFrame(
            middle_frame, text="Video Feed", bg="white", font=("Arial", 10, "bold")
        )
        box_c.grid(row=1, column=0, sticky="nsew", padx=5, pady=5)

        self.video_label = tk.Label(box_c, bg="black")
        self.video_label.pack(fill="both", expand=True)

        # Command Log
        box_d = tk.LabelFrame(
            middle_frame, text="Command Log", bg="white", font=("Arial", 10, "bold")
        )
        box_d.grid(row=2, column=0, sticky="nsew", padx=5, pady=5)

    def update_video_feed(self, ros_image):
        # Receives and updates video feed from ROS camera topic
        try:
            frame = self.bridge.imgmsg_to_cv2(ros_image, "bgr8")
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            image = Image.fromarray(frame)
            photo = ImageTk.PhotoImage(image=image)

            self.video_label.config(image=photo)
            self.video_label.image = photo  # Prevent garbage collection
        except Exception as e:
            rospy.logerr(f"Error processing video frame: {e}")

    def update_progress(self, percent):
        # Updates the progress bar dynamically
        self.progress_value.set(percent)
        self.progress_label.config(text=f"Execution Progress: {int(percent)}%")

        self.frame.update_idletasks()  # Ensure smooth UI update

    def emergency_stop(self):
        # Sends Ctrl+C to stop running commands without closing the UI
        print("!!! Emergency Stop Activated !!!")

        # Find the process running `ros`
        try:
            result = subprocess.run(
                ["pgrep", "-f", "ros"], capture_output=True, text=True
            )
            pids = result.stdout.strip().split("\n")

            if pids and pids[0]:  # If there are PIDs found
                for pid in pids:
                    print(f" Stopping process {pid} (rostopic pub)")
                    os.kill(int(pid), signal.SIGINT)  # Send Ctrl+C (SIGINT)

                self.log_action(" All commands stopped successfully.")
            else:
                print(" No active commands found to stop.")
                self.log_action(" No active commands were running.")

        except Exception as e:
            print(f" Failed to send Ctrl+C: {e}")
            self.log_action(f" Error stopping commands: {e}")

    def stop_video_stream(self):
        # Stops the video feed and closes the stream.
        self.running = False
        self.video_subscriber.unregister()  # Unsubscribe from ROS topic
