import tkinter as tk
from tkinter import ttk, messagebox
from utils import load_modules, send_command
import threading
from Modules.Movement import COMMANDS
import os
import time


class PreProgrammingPage:
    # Programming grid setup
    GRID_ROWS = 10
    GRID_COLS = 1
    CELL_HEIGHT = 60
    CELL_WIDTH = 750

    def __init__(self, container, controller):
        # Initialize the pre-programming page
        self.frame = tk.Frame(container, bg="white")
        self.controller = controller

        # Drag-and-drop tracking
        self.drag_data = {"widget": None, "row": None, "col": None}

        # Grid tracking
        self.grid_cells = [
            [None for _ in range(self.GRID_COLS)] for _ in range(self.GRID_ROWS)
        ]
        self.command_list = []
        self.undo_list = []
        self.redo_list = []

        # To handle row highlighting
        self.highlight_rect = None
        
        # Scrolling stuff
        self.visible_rows = 8  # Number of visible rows at a time
        self.scroll_position = 0  # Current scroll index


        self.module_colors = {}

        self.setup_ui()

    def setup_ui(self):
        self.frame.rowconfigure([0, 1], weight=1)
        self.frame.columnconfigure(0, weight=1)  # Commands section
        self.frame.columnconfigure(1, weight=4)  # Programming area
        self.frame.columnconfigure(2, weight=2)  # Log area

        # Box A: Commands list
        self.command_section = tk.LabelFrame(
            self.frame, text="Commands", bg="white", font=("Arial", 10, "bold")
        )
        self.command_section.grid(
            row=0, column=0, rowspan=2, sticky="nsew", padx=5, pady=5
        )
        self.populate_command_section(self.command_section)

        # Middle frame (Programming area)
        middle_frame = tk.Frame(self.frame, bg="white")
        middle_frame.grid(row=0, column=1, rowspan=2, sticky="nsew", padx=5, pady=5)
        middle_frame.rowconfigure(0, weight=0)
        middle_frame.rowconfigure(1, weight=1)
        middle_frame.columnconfigure(0, weight=1)

        # Functional buttons
        box_b = tk.Frame(middle_frame, bg="white", height=50)
        box_b.grid(row=0, column=0, sticky="ew", padx=5, pady=5)
        button_colors = {
            "Home": "#808080",
            "Save": "#007BFF",
            "Load": "#007BFF",
            "Undo": "#FFA500",
            "Redo": "#FFA500",
            "Clear": "#FF0000",
            "Run": "#32CD32",
        }
        for btn_text, btn_color in button_colors.items():
            tk.Button(
                box_b,
                text=btn_text,
                bg=btn_color,
                fg="white",
                width=10,
                font=("Arial", 10, "bold"),
                command=lambda t=btn_text: self.handle_button_click(t),
            ).pack(side="left", padx=10, pady=5)

        # File name input
        tk.Label(box_b, text="File Name:", bg="white", font=("Arial", 10)).pack(
            side="left", padx=5
        )
        self.file_name_entry = tk.Entry(box_b, width=20, font=("Arial", 10))
        self.file_name_entry.pack(side="left", padx=10)
        self.file_name_entry.insert(0, "program")  # Default file name

        # Box C: Programming area (Fixed Grid)
        programming_frame = tk.Frame(middle_frame, bg="white")
        programming_frame.grid(row=1, column=0, sticky="nsew", padx=5, pady=5)

        self.programming_area = tk.Frame(programming_frame, bg="lightgray")
        self.programming_area.pack(fill="both", expand=True)

        # Create visible grid slots
        self.grid_slots = []
        for i in range(self.visible_rows):
            slot = tk.Frame(self.programming_area, bg="lightgray", height=self.CELL_HEIGHT)
            slot.grid(row=i, column=0, sticky="ew", padx=5, pady=2)
            self.grid_slots.append(slot)

        # Scroll buttons
        scroll_controls = tk.Frame(programming_frame, bg="white")
        scroll_controls.pack(fill="x", pady=5)

        tk.Button(scroll_controls, text="⬆ Scroll Up", command=self.scroll_up).pack(side="left", expand=True)
        tk.Button(scroll_controls, text="⬇ Scroll Down", command=self.scroll_down).pack(side="right", expand=True)



        # Box D: Log Area
        self.log_section = tk.LabelFrame(
            self.frame, text="Action Log", bg="white", font=("Arial", 10, "bold")
        )
        self.log_section.grid(row=0, column=2, rowspan=2, sticky="nsew", padx=5, pady=5)

        self.log_text = tk.Text(
            self.log_section, wrap="word", height=20, state="disabled", bg="lightgray"
        )
        self.log_text.pack(fill="both", expand=True, padx=5, pady=5)
        self.log_scroll = ttk.Scrollbar(self.log_section, command=self.log_text.yview)
        self.log_scroll.pack(side="right", fill="y")
        self.log_text.config(yscrollcommand=self.log_scroll.set)

    def log_action(self, message):
        # Appends a message to the log section
        self.log_text.config(state="normal")
        self.log_text.insert("end", f"{message}\n")
        self.log_text.config(state="disabled")
        self.log_text.yview("end")

    def populate_command_section(self, command_frame):
        modules_commands = load_modules()  # Load all available modules

        for module_name, module_data in modules_commands.items():
            commands = module_data["commands"]
            color = module_data["color"]

            print(
                f"Loading Module: {module_name}, Available Commands: {list(commands.keys())}"
            )  # Debugging print

            module_label = tk.Label(
                command_frame, text=module_name, font=("Arial", 10, "bold")
            )
            module_label.pack(pady=5)

            for command_name in commands.keys():
                print(
                    f"Adding command: '{command_name}' from Module: {module_name}"
                )  # Debugging print

                command_label = tk.Label(
                    command_frame,
                    text=command_name.strip(),
                    bg=color,
                    fg="white",
                    font=("Arial", 10),
                    relief="raised",
                    padx=5,
                    pady=2,
                )
                command_label.pack(pady=2, padx=5, fill="x")

                # Ensure correct module is passed when clicking the command
                command_label.bind(
                    "<Button-1>",
                    lambda e, label=command_label, mod=commands: self.add_block_to_grid(
                        label, mod
                    ),
                )

    def add_block_to_grid(self, command_label, command_module):
        # Add a new block to the command list
        self.undo_list.append(self.capture_state())
        self.redo_list.clear()

        # Get the next available row and column (always col=0)
        next_row = len(self.command_list)
        col = 0  # Always column 0

        # Create the block with correct parameters
        block = self.create_block(command_label, next_row, col, command_module)
        self.command_list.append(block)

        self.refresh_visible_blocks()  # Refresh UI after adding



    def create_block(self, command_label, row, col, command_module):
        command_name = command_label.cget("text").strip()

        if command_name not in command_module:
            print(f"ERROR: '{command_name}' not found in module. Available: {list(command_module.keys())}")
            return None

        inputs = command_module[command_name].get("inputs", {})
        color = command_label.cget("bg")

        block = tk.Frame(self.programming_area, bg=color, relief="raised", bd=2, width=self.CELL_WIDTH, height=self.CELL_HEIGHT)

        content_frame = tk.Frame(block, bg=color)
        content_frame.pack(fill="both", expand=True, padx=10, pady=5)

        block.command_name = command_name
        block.command_module = command_module

        label = tk.Label(content_frame, text=command_name, bg=color, fg="white", font=("Arial", 12, "bold"), anchor="w")
        label.pack(side="left", padx=5)

        input_vars = {}
        for label_text, var_name in inputs.items():
            tk.Label(content_frame, text=label_text, bg=color, fg="white", font=("Arial", 10)).pack(side="left", padx=5)
            input_var = tk.StringVar()
            tk.Entry(content_frame, textvariable=input_var, width=20).pack(side="left", padx=5)
            input_vars[var_name] = input_var

        block.input_vars = input_vars
        block.grid_row = row
        block.grid_col = col

        return block


    def on_drag_start(self, event, block):
        self.undo_list.append(self.capture_state())
        self.redo_list.clear()

        # Start dragging a block
        self.drag_data["widget"] = block
        self.drag_data["row"] = block.grid_row
        self.drag_data["col"] = block.grid_col

        # Temporarily free the current grid cell
        self.grid_cells[block.grid_row][block.grid_col] = None

        # Clear the highlight rectangle if it exists
        self.clear_highlight()

    def on_drag_motion(self, event, block):
        # Handle motion of a dragged block
        # Adjustments to account for consistent offset
        fixed_offset_x = 260
        fixed_offset_y = 80

        # Calculate mouse position relative to the programming_area
        x = (
            self.programming_area.canvasx(event.x_root - self.frame.winfo_rootx())
            - fixed_offset_x
        )
        y = (
            self.programming_area.canvasy(event.y_root - self.frame.winfo_rooty())
            - fixed_offset_y
        )

        # Constrain the cursor position within the grid
        x = max(0, min(x, self.CELL_WIDTH - block.winfo_width()))
        y = max(0, min(y, self.GRID_ROWS * self.CELL_HEIGHT - block.winfo_height()))

        # Move the block to follow the cursor with adjusted position
        block.place(x=x, y=y)

        # Determine the row under the cursor
        target_row = max(0, min(int(y // self.CELL_HEIGHT), self.GRID_ROWS - 1))

        # Highlight the target row
        self.highlight_target_row(target_row)

    def highlight_target_row(self, target_row):
        # Highlight the target row
        if self.highlight_rect:
            self.programming_area.coords(
                self.highlight_rect,
                0,
                target_row * self.CELL_HEIGHT,
                self.CELL_WIDTH,
                (target_row + 1) * self.CELL_HEIGHT,
            )
        else:
            self.highlight_rect = self.programming_area.create_rectangle(
                0,
                target_row * self.CELL_HEIGHT,
                self.CELL_WIDTH,
                (target_row + 1) * self.CELL_HEIGHT,
                outline="blue",
                width=2,
                dash=(4, 2),
            )

    def on_drop(self, event, block):
        # Capture state only if a block was actually moved
        if block and self.highlight_rect:
            coords = self.programming_area.coords(self.highlight_rect)
            target_row = int(coords[1] // self.CELL_HEIGHT)

            # Move block into position
            if self.grid_cells[target_row][0]:
                self.move_blocks_down(target_row)

            block.grid_row = target_row
            block.grid_col = 0
            self.grid_cells[target_row][0] = block
            block.place(x=0, y=target_row * self.CELL_HEIGHT)

            # Capture only if the position has changed
            if block.grid_row != target_row:
                new_state = self.capture_state()
                if new_state and (
                    not self.undo_list or self.undo_list[-1] != new_state
                ):
                    self.undo_list.append(new_state)
                    self.redo_list.clear()

            self.update_command_list()
            self.move_blocks_up()

        self.drag_data = {"widget": None, "row": None, "col": None}
        self.clear_highlight()

    def move_blocks_down(self, start_row):
        # Move all blocks from start_row down by one row
        # Add a new row if needed
        if len(self.grid_cells) <= start_row:
            self.add_row()

        # Iterate from bottom to top, shifting blocks down
        for row in range(len(self.grid_cells) - 1, start_row, -1):
            if self.grid_cells[row - 1][0]:  # Check if the row above has a block
                block = self.grid_cells[row - 1][0]
                self.grid_cells[row][0] = block  # Move the block down
                self.grid_cells[row - 1][0] = None  # Clear the original cell
                block.grid_row = row
                block.place(x=0, y=row * self.CELL_HEIGHT)

    def move_blocks_up(self):
        # Move blocks up recursively to fill all empty rows
        moved = False  # Flag to track if any block was moved

        # Iterate through all rows except the last
        for row in range(len(self.grid_cells) - 1):
            if (
                not self.grid_cells[row][0] and self.grid_cells[row + 1][0]
            ):  # If current row is empty and next row has a block
                # Move the block from the next row up
                block = self.grid_cells[row + 1][0]
                self.grid_cells[row][0] = block
                self.grid_cells[row + 1][0] = None
                block.grid_row = row
                block.place(x=0, y=row * self.CELL_HEIGHT)
                moved = True

        # If a block was moved, call the method recursively
        if moved:
            self.move_blocks_up()

    def clear_highlight(self):
        # Remove the highlight rectangle
        if self.highlight_rect:
            self.programming_area.delete(self.highlight_rect)
            self.highlight_rect = None

    def handle_button_click(self, button_text):
        # Handles button clicks and logs actions
        self.log_action(f"Button clicked: {button_text}")
        if button_text == "Run":
            self.run_program()
        elif button_text == "Save":
            file_name = self.file_name_entry.get().strip()
            if file_name:
                self.save_program(file_name)
            else:
                messagebox.showerror("Save Error", "Please enter a file name.")
        elif button_text == "Load":
            file_name = self.file_name_entry.get().strip()
            if file_name:
                self.load_program(file_name)
            else:
                messagebox.showerror("Load Error", "Please enter a file name.")
        elif button_text == "Clear":
            self.clear_programming_area()
        elif button_text == "Home":
            self.controller.show_page("StartScreen")
        elif button_text == "Undo":
            self.undo_last_action()
        elif button_text == "Redo":
            self.redo_last_action()

    import threading

    def run_program(self):
        # Executes commands using send_command() in a background thread and logs it
        self.controller.show_page("ObservationPage")
        self.log_action("Program started...")

        # Start execute_commands in a new thread
        threading.Thread(target=self.execute_commands, daemon=True).start()

    def execute_commands(self):
        # Executes commands sequentially, waiting until each finishes. Speech runs in parallel
        total_commands = len(self.command_list)
        if total_commands == 0:
            self.controller.pages["ObservationPage"].update_progress(0)
            return

        for index, block in enumerate(self.command_list):
            command_name = getattr(block, "command_name", None)
            command_module = getattr(block, "command_module", None)

            if not command_name:
                print(f"ERROR: Block at row {block.grid_row} has no command_name!")
                continue

            if not command_module:
                print(f"ERROR: Block '{command_name}' has no module reference!")
                continue

            print(f"Running Command: {command_name} from Module: {command_module}")

            if command_name not in command_module:
                print(
                    f"ERROR: '{command_name}' not found in module! Available: {list(command_module.keys())}"
                )
                self.log_action(f"Error: Unknown command {command_name}")
                continue

            command_info = command_module[command_name]
            command_function = command_info.get("function")

            inputs = {}
            for var_name, var in block.input_vars.items():
                input_value = var.get().strip()
                inputs[var_name] = input_value

            try:
                print(f"Executing: {command_name} with inputs {inputs}")
                self.log_action(f"Executing: {command_name} with {inputs}")

                if command_name == "Speak Text":
                    # Speech runs in parallel (does not block execution)
                    threading.Thread(
                        target=command_function, args=inputs.values(), daemon=True
                    ).start()
                else:
                    # Other commands execute one after the other
                    start_time = time.time()  # ⏳ Start timing execution

                    command_function(*inputs.values())  # Run the function

                    # Check for command completion dynamically
                    execution_time = self.estimate_execution_time(command_name, inputs)
                    print(
                        f"Estimated Execution Time for {command_name}: {execution_time:.2f} seconds"
                    )
                    time.sleep(execution_time)  # Wait for the command to finish

            except Exception as e:
                print(f"Execution Failed for {command_name}: {e}")
                self.log_action(f"Error running {command_name}: {e}")

            # Update progress bar
            percent_complete = ((index + 1) / total_commands) * 100
            self.controller.pages["ObservationPage"].update_progress(percent_complete)

        self.controller.pages["ObservationPage"].update_progress(100)  # Complete

    def estimate_execution_time(self, command_name, inputs):
        # Estimates execution time dynamically based on command type and input values
        # Example: Movement commands → Use distance & speed for estimation
        if command_name in ["Move Forward", "Move Backward"]:
            distance = float(inputs.get("distance", 0))  # Get distance
            speed = 0.5
            return (distance / speed) + 1

        # Example: Rotation commands → Use degrees & rotation speed
        elif command_name in ["Rotate Left", "Rotate Right"]:
            degrees = float(inputs.get("degrees", 0))
            rotation_speed = 1
            return ((degrees * 3.14159265 / 180) / rotation_speed) + 1

        # Default time if no estimation is available
        return 10  # Assume a safe default execution time

    def save_program(self, file_name):
        os.makedirs("SavedFiles", exist_ok=True)  # Ensure the directory exists

        if not file_name:
            messagebox.showwarning("Save", "Please enter a valid file name.")
            return

        file_path = os.path.join("SavedFiles", f"{file_name}.txt")

        try:
            with open(file_path, "w") as file:
                for block in self.command_list:
                    command_name = block.command_name
                    module_name = getattr(
                        block, "command_module_name", "Unknown"
                    )  # Store module name
                    inputs = {
                        var_name: var.get()
                        for var_name, var in block.input_vars.items()
                    }

                    input_line = (
                        f"{command_name};{module_name};"
                        + ";".join(f"{k}={v}" for k, v in inputs.items())
                        + "\n"
                    )
                    file.write(input_line)

            messagebox.showinfo("Save", f"Program saved to {file_path}!")
        except Exception as e:
            messagebox.showerror("Save Error", f"Error saving program: {e}")

    def load_program(self, file_name):
        try:
            self.clear_programming_area()  # Clear previous blocks

            file_path = os.path.join("SavedFiles", f"{file_name}.txt")

            with open(file_path, "r") as file:
                for line in file:
                    line = line.strip()
                    if not line:
                        continue

                    # Read command_name, module_name, and input values
                    parts = line.split(";")
                    command_name = parts[0]
                    module_name = parts[1]
                    inputs = {kv.split("=")[0]: kv.split("=")[1] for kv in parts[2:]}

                    # Find correct module
                    modules_commands = load_modules()
                    if module_name not in modules_commands:
                        print(f"ERROR: Module '{module_name}' not found!")
                        continue

                    command_module = modules_commands[module_name]["commands"]
                    if command_name not in command_module:
                        print(
                            f"ERROR: Command '{command_name}' not found in '{module_name}'!"
                        )
                        continue

                    # Create block with correct module and inputs
                    block = self.create_block_from_data(
                        command_name, command_module, inputs
                    )

                    for row in range(len(self.grid_cells)):  # Find empty row
                        if not self.grid_cells[row][0]:
                            block.grid_row = row
                            block.grid_col = 0
                            block.place(x=0, y=row * self.CELL_HEIGHT)
                            self.grid_cells[row][0] = block
                            break

                    self.command_list.append(block)

                self.move_blocks_up()

            messagebox.showinfo(
                "Load Success", f"Program '{file_name}.txt' loaded successfully."
            )

        except FileNotFoundError:
            messagebox.showerror("Load Error", f"File '{file_name}.txt' not found.")
        except Exception as e:
            messagebox.showerror("Load Error", f"Error loading program: {e}")

    def undo_last_action(self):
        if not self.undo_list:
            messagebox.showinfo("Undo", "Nothing to undo")
            return

        # Capture current state before undoing, ensuring it's not a duplicate
        current_state = self.capture_state()
        if current_state and (
            not self.redo_list or self.redo_list[-1] != current_state
        ):
            self.redo_list.append(current_state)

        last_state = self.undo_list.pop()
        self.restore_state(last_state)

    def redo_last_action(self):
        if not self.redo_list:
            messagebox.showinfo("Redo", "Nothing to redo")
            return

        # Capture current state before redoing, ensuring it's not a duplicate
        current_state = self.capture_state()
        if current_state and (
            not self.undo_list or self.undo_list[-1] != current_state
        ):
            self.undo_list.append(current_state)

        last_state = self.redo_list.pop()
        self.restore_state(last_state)

    def capture_state(self):
        # Capture the current state of the programming area
        state = []

        for i, block in enumerate(self.command_list):
            command_name = block.command_name
            command_module_name = getattr(block, "command_module_name", "Unknown")
            inputs = {var_name: var.get() for var_name, var in block.input_vars.items()}

            state.append((command_name, command_module_name, inputs, i))  # Store row index

        return state if not self.undo_list or self.undo_list[-1] != state else None


    def capture_input_change(self):
        # Capture state when a user modifies an input field
        self.undo_list.append(self.capture_state())
        self.redo_list.clear()

    def restore_state(self, state):
        # Restore a previous state of the programming area
        self.clear_programming_area()
        
        modules_commands = load_modules()  # Reload available modules

        for command_name, module_name, inputs, row in state:
            if module_name not in modules_commands:
                print(f"🚨 ERROR: Module '{module_name}' not found during restore!")
                continue

            command_module = modules_commands[module_name]["commands"]
            if command_name not in command_module:
                print(f"🚨 ERROR: Command '{command_name}' not found in '{module_name}'!")
                continue

            # Create block using stored data
            block = self.create_block_from_data(command_name, command_module, inputs)
            self.command_list.append(block)

        self.refresh_visible_blocks()


    def create_block_from_data(self, command_name, expected_inputs, input_values):
        # Get the module color or use a default
        color = self.module_colors.get(command_name.split()[0], "#FF5733")

        # Create a block
        row = len(self.command_list)  # Place it at the next available row
        col = 0
        block = tk.Frame(
            self.programming_area,
            bg=color,
            relief="raised",
            bd=2,
            width=self.CELL_WIDTH,
            height=self.CELL_HEIGHT,
        )
        block.place(x=col * self.CELL_WIDTH, y=row * self.CELL_HEIGHT)

        # Create a container frame for the block's content
        content_frame = tk.Frame(block, bg=color)
        content_frame.pack(fill="both", expand=True, padx=10, pady=5)

        # Add command label
        label = tk.Label(
            content_frame,
            text=command_name,
            bg=color,
            fg="white",
            font=("Arial", 12, "bold"),
            anchor="w",
        )
        label.pack(side="left", padx=5)
        block.command_label = label  # Add reference to the block for easy access

        # Add input fields dynamically based on the expected inputs
        input_vars = {}
        for label_text, var_name in expected_inputs.items():
            tk.Label(
                content_frame, text=label_text, bg=color, fg="white", font=("Arial", 10)
            ).pack(side="left", padx=5)
            input_var = tk.StringVar(
                value=input_values.get(var_name, "")
            )  # Set saved value
            tk.Entry(content_frame, textvariable=input_var, width=8).pack(
                side="left", padx=5
            )
            input_vars[var_name] = input_var

        # Store inputs in the block for later use
        block.input_vars = input_vars

        # Set grid attributes
        block.grid_row = row
        block.grid_col = col

        return block

    def create_command_label(self, command_name):
        # Helper to create a dummy label for loading commands.
        return tk.Label(
            self.command_section,
            text=command_name,
            bg="white",  # Default color
            fg="black",
            font=("Arial", 10),
        )

    def clear_programming_area(self):
        # Clear all commands from the programming area
        for widget in self.programming_area.winfo_children():
            widget.destroy()
        self.grid_cells = [
            [None for _ in range(self.GRID_COLS)] for _ in range(self.GRID_ROWS)
        ]
        self.command_list.clear()

    def add_row(self):
        # Add a new row to the grid and adjust the canvas size
        self.grid_cells.append([None for _ in range(self.GRID_COLS)])

        # Calculate the new height of the canvas content
        new_height = len(self.grid_cells) * self.CELL_HEIGHT
        self.canvas_content.config(height=new_height)

        # Update the scroll region of the canvas
        self.update_scroll_region()

    def check_and_add_rows(self):
        # Check if additional rows are needed and add them dynamically
        # Count the number of empty rows
        empty_rows = sum(1 for row in self.grid_cells if row[0] is None)

        # Add more rows if fewer than 2 rows are empty
        if empty_rows < 2:
            self.add_row()

    def scroll_up(self):
        # Scroll the programming area up
        if self.scroll_position > 0:
            self.scroll_position -= 1
            self.refresh_visible_blocks()

    def scroll_down(self):
        # Scroll the programming area down
        if self.scroll_position + self.visible_rows < len(self.command_list):
            self.scroll_position += 1
            self.refresh_visible_blocks()

    def refresh_visible_blocks(self):
        # Refresh visible blocks based on scroll position

        # Clear ALL grid slots before re-populating
        for slot in self.grid_slots:
            for widget in slot.winfo_children():
                widget.destroy()  # Remove any existing widgets inside the slot

        # Populate visible slots with blocks from command_list
        for i in range(self.visible_rows):
            row_index = self.scroll_position + i  # Get correct index in the command list
            if row_index < len(self.command_list):
                block = self.command_list[row_index]
                block.grid_row = i  # Update row position
                
                # Add block to the correct row slot and clear its previous position
                block.pack(in_=self.grid_slots[i], fill="x", padx=5, pady=2)

        # Update the UI to reflect changes
        self.programming_area.update_idletasks()

    def update_command_list(self):
        # Update the list of commands
        self.command_list = [
            self.grid_cells[row][0]
            for row in range(len(self.grid_cells))
            if self.grid_cells[row][0]
        ]
        print(self.command_list)
