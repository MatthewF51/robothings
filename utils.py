import importlib
import subprocess
import os
import time
import signal
import threading

# Dictionary to store active ROS processes
active_processes = {}

def send_command(command, duration=None):
    """
    Sends a ROS command using subprocess.Popen() for non-blocking execution.

    :param command: The full ROS command as a string (e.g., "rostopic pub /cmd_vel geometry_msgs/Twist ...").
    :param duration: (Optional) Time in seconds to keep sending the command before stopping.
    """

    try:
        print(f"Executing: {command}")  # Debugging output
        process = subprocess.Popen(command, shell=True, preexec_fn=os.setsid)  # Run in new process group
        active_processes[command] = process  # Store process for possible termination

        if duration:
            def stop_after_duration():
                time.sleep(duration)
                stop_command(command)
                print(f"Command '{command}' stopped after {duration} seconds.")

            threading.Thread(target=stop_after_duration, daemon=True).start()

    except Exception as e:
        print(f"Error executing command: {e}")

def stop_command(command):
    """
    Stops a running ROS command by sending SIGINT (Ctrl-C).
    
    :param command: The command string to stop.
    """
    process = active_processes.get(command)
    if process:
        print(f"Stopping command: {command}")
        os.killpg(os.getpgid(process.pid), signal.SIGINT)  # Send Ctrl-C to process group
        process.wait()  # Ensure process has stopped
        del active_processes[command]  # Remove from active processes

def load_modules():
    """Dynamically loads modules from the 'Modules' folder."""
    modules_path = os.path.join(os.path.dirname(__file__), "Modules")
    modules = {}

    for module_name in os.listdir(modules_path):
        if module_name.endswith(".py") and module_name != "__init__.py":
            module_name = module_name[:-3]  # Remove .py extension
            module = importlib.import_module(f"Modules.{module_name}")
            
            if hasattr(module, "COMMANDS") and hasattr(module, "colour"):
                modules[module_name] = {
                    "commands": module.COMMANDS,
                    "color": module.colour
                }

    return modules
