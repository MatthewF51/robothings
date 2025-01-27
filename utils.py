import importlib
import os


def load_modules():
    # Load commands and colors from all module files
    modules = {}
    modules_path = "Modules"  # Path to the modules folder
    for file in os.listdir(modules_path):
        if file.endswith(".py") and file != "__init__.py":
            module_name = file[:-3]  # Remove .py extension
            try:
                module = importlib.import_module(f"Modules.{module_name}")
                commands = getattr(module, "COMMANDS", {})
                color = getattr(module, "colour", "#000000")  # Default color is black
                modules[module_name] = {"commands": commands, "color": color}
            except AttributeError as e:
                print(f"Error loading {module_name}: {e}")
    return modules
