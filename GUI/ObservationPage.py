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

# ROS Topic for Tiago's Camera
CAMERA_TOPIC = "/xtion/rgb/image_raw"

class ObservationPage:
    def __init__(self, root, controller):
        """Initialize Observation Page"""
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
        self.video_subscriber = rospy.Subscriber(CAMERA_TOPIC, ROSImage, self.update_video_feed)

        self.setup_ui()

    def setup_ui(self):
        """Setup the UI Layout"""
        self.frame.rowconfigure(0, weight=1)
        self.frame.columnconfigure(0, weight=1)  # Emergency Controls
        self.frame.columnconfigure(1, weight=5)  # Video & Command Log

        # 🚨 Emergency Controls (Left Panel)
        box_a = tk.LabelFrame(self.frame, text="Emergency Controls", bg="white", font=("Arial", 10, "bold"))
        box_a.grid(row=0, column=0, sticky="nsew", padx=5, pady=5)

        tk.Button(box_a, text="Emergency Stop", bg="#FF0000", fg="white", font=("Arial", 12, "bold"),
                  height=3, command=self.emergency_stop).pack(pady=10, padx=10, fill="x")

        tk.Button(box_a, text="Back to Programming Page", bg="#4CAF50", fg="white", font=("Arial", 12, "bold"),
                  command=lambda: self.controller.show_page("PreProgrammingPage")).pack(pady=10, padx=10, fill="x")

        tk.Button(box_a, text="Back to Start Page", bg="#007BFF", fg="white", font=("Arial", 12, "bold"),
                  command=lambda: self.controller.show_page("StartScreen")).pack(pady=10, padx=10, fill="x")

        # 📹 Middle Frame (Video + Command Log)
        middle_frame = tk.Frame(self.frame, bg="white")
        middle_frame.grid(row=0, column=1, sticky="nsew", padx=5, pady=5)
        middle_frame.rowconfigure(0, weight=6)  # Video Feed (60%)
        middle_frame.rowconfigure(1, weight=4)  # Command Log (40%)
        middle_frame.columnconfigure(0, weight=1)

        # 🎥 Video Feed
        box_c = tk.LabelFrame(middle_frame, text="Video Feed", bg="white", font=("Arial", 10, "bold"))
        box_c.grid(row=0, column=0, sticky="nsew", padx=5, pady=5)

        self.video_label = tk.Label(box_c, bg="black")
        self.video_label.pack(fill="both", expand=True)

        # 📜 Command Log
        box_d = tk.LabelFrame(middle_frame, text="Command Log", bg="white", font=("Arial", 10, "bold"))
        box_d.grid(row=1, column=0, sticky="nsew", padx=5, pady=5)

        # 🔄 Progress Bar
        self.progress_bar = ttk.Progressbar(box_d, variable=self.progress_value, maximum=100, mode="determinate")
        self.progress_bar.pack(fill="x", padx=10, pady=10)

        self.progress_label = tk.Label(box_d, text="Execution Progress: 0%", bg="white", font=("Arial", 10))
        self.progress_label.pack(pady=5)

    def update_video_feed(self, ros_image):
        """Receives and updates video feed from ROS camera topic."""
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
        """Updates the progress bar dynamically."""
        self.progress_value.set(percent)
        self.progress_label.config(text=f"Execution Progress: {int(percent)}%")
        self.root.update_idletasks()  # Ensure smooth UI update

    def emergency_stop(self):
        """🔴 Emergency Stop - Stops all robot movement and disables controllers using Ctrl-C."""
        try:
            rospy.logwarn("🚨 Emergency Stop Activated! Sending Ctrl-C to stop running processes.")

            # 1️⃣ Find and kill movement-related ROS nodes
            os.system("rosnode kill /move_base")  # Navigation node (if running)
            os.system("rosnode kill /teleop_twist_keyboard")  # Teleoperation node (if running)

            # 2️⃣ Send Ctrl-C to stop all running processes
            os.kill(os.getpid(), signal.SIGINT)  

            # 3️⃣ Ensure robot stops completely by publishing zero velocity
            rospy.sleep(1)  # Give time for the kill command to take effect
            stop_twist = Twist()
            pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
            pub.publish(stop_twist)

            pub = rospy.Publisher("/mobile_base_controller/cmd_vel", Twist, queue_size=10)
            pub.publish(stop_twist)

            messagebox.showinfo("Emergency Stop", "🚨 All robot functions stopped!")

        except Exception as e:
            rospy.logerr(f"Emergency Stop Failed: {e}")
            messagebox.showerror("Emergency Stop Error", f"Failed to stop robot: {e}")


    def stop_video_stream(self):
        """Stops the video feed and closes the stream."""
        self.running = False
        self.video_subscriber.unregister()  # Unsubscribe from ROS topic
