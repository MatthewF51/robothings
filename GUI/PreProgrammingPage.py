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

        # Drag-and-drop tracking (with offset values)
        self.drag_data = {
            "widget": None,
            "row": None,
            "col": None,
            "offset_x": 0,
            "offset_y": 0,
        }

        # Grid tracking
        self.grid_cells = [
            [None for _ in range(self.GRID_COLS)] for _ in range(self.GRID_ROWS)
        ]
        self.command_list = []
        self.undo_list = []
        self.redo_list = []

        # Scrolling stuff
        self.visible_rows = 8  # Number of visible rows at a time
        self.scroll_position = 0  # Current scroll index

        self.module_colors = {}

        self.last_highlighted_row = None

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
            slot = tk.Frame(
                self.programming_area, bg="lightgray", height=self.CELL_HEIGHT
            )
            slot.grid(row=i, column=0, sticky="ew", padx=5, pady=2)
            self.grid_slots.append(slot)

        # Scroll buttons
        scroll_controls = tk.Frame(programming_frame, bg="white")
        scroll_controls.pack(fill="x", pady=5)
        tk.Button(scroll_controls, text="⬆ Scroll Up", command=self.scroll_up).pack(
            side="left", expand=True
        )
        tk.Button(scroll_controls, text="⬇ Scroll Down", command=self.scroll_down).pack(
            side="right", expand=True
        )

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
            )
            module_label = tk.Label(
                command_frame, text=module_name, font=("Arial", 10, "bold")
            )
            module_label.pack(pady=5)
            for command_name in commands.keys():
                print(f"Adding command: '{command_name}' from Module: {module_name}")
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
                # Pass the module name along with the commands dict.
                command_label.bind(
                    "<Button-1>",
                    lambda e, label=command_label, mod=commands, mod_name=module_name: self.add_block_to_grid(
                        label, mod, mod_name
                    ),
                )

    def add_block_to_grid(self, command_label, command_module, module_name):
        self.undo_list.append(self.capture_state())
        self.redo_list.clear()
        next_row = len(self.command_list)
        col = 0  # Always column 0
        block = self.create_block(
            command_label, next_row, col, command_module, module_name
        )
        if block is None:
            return
        # Ensure grid_cells has a row for this block.
        if next_row >= len(self.grid_cells):
            self.grid_cells.append([None for _ in range(self.GRID_COLS)])
        block.grid_row = next_row
        block.grid_col = col
        self.grid_cells[next_row][col] = block
        self.command_list.append(block)
        self.refresh_visible_blocks()
        print(
            f"[add_block_to_grid] Added block '{block.command_name}' in module '{module_name}' at row {next_row}"
        )

    def create_block(self, command_label, row, col, command_module, module_name):
        command_name = command_label.cget("text").strip()
        if command_name not in command_module:
            print(
                f"ERROR: '{command_name}' not found in module. Available: {list(command_module.keys())}"
            )
            return None
        inputs = command_module[command_name].get("inputs", {})
        color = command_label.cget("bg")
        block = tk.Frame(
            self.programming_area,
            bg=color,
            relief="raised",
            bd=2,
            width=self.CELL_WIDTH,
            height=self.CELL_HEIGHT,
        )
        # Create a content frame inside the block
        content_frame = tk.Frame(block, bg=color)
        content_frame.pack(fill="both", expand=True, padx=10, pady=5)
        block.command_name = command_name
        block.command_module = command_module
        # Store the module name so it can be saved later.
        block.command_module_name = module_name
        block.grid_row = row
        block.grid_col = col
        label = tk.Label(
            content_frame,
            text=command_name,
            bg=color,
            fg="white",
            font=("Arial", 12, "bold"),
            anchor="w",
        )
        label.pack(side="left", padx=5)
        input_vars = {}
        for label_text, var_name in inputs.items():
            tk.Label(
                content_frame, text=label_text, bg=color, fg="white", font=("Arial", 10)
            ).pack(side="left", padx=5)
            input_var = tk.StringVar()
            tk.Entry(content_frame, textvariable=input_var, width=20).pack(
                side="left", padx=5
            )
            input_vars[var_name] = input_var
        block.input_vars = input_vars

        # Bind drag events to both block and its content_frame
        block.bind(
            "<ButtonPress-1>", lambda event, blk=block: self.on_drag_start(event, blk)
        )
        block.bind(
            "<B1-Motion>", lambda event, blk=block: self.on_drag_motion(event, blk)
        )
        block.bind(
            "<ButtonRelease-1>", lambda event, blk=block: self.on_drop(event, blk)
        )
        content_frame.bind(
            "<ButtonPress-1>", lambda event, blk=block: self.on_drag_start(event, blk)
        )
        content_frame.bind(
            "<B1-Motion>", lambda event, blk=block: self.on_drag_motion(event, blk)
        )
        content_frame.bind(
            "<ButtonRelease-1>", lambda event, blk=block: self.on_drop(event, blk)
        )

        return block

    def on_drag_start(self, event, block):
        self.undo_list.append(self.capture_state())
        self.redo_list.clear()
        # Compute the offset using absolute coordinates
        self.drag_data["widget"] = block
        self.drag_data["offset_x"] = event.x_root - block.winfo_rootx()
        self.drag_data["offset_y"] = event.y_root - block.winfo_rooty()
        self.drag_data["row"] = block.grid_row
        self.drag_data["col"] = block.grid_col
        print(
            f"[on_drag_start] Block '{block.command_name}' at row {block.grid_row} started drag; offset=({self.drag_data['offset_x']}, {self.drag_data['offset_y']})"
        )
        # Free the grid cell
        self.grid_cells[block.grid_row][block.grid_col] = None
        self.clear_highlight()

    def on_drag_motion(self, event, block):
        prog_area_x = self.programming_area.winfo_rootx()
        prog_area_y = self.programming_area.winfo_rooty()
        new_x = event.x_root - prog_area_x - self.drag_data["offset_x"]
        new_y = event.y_root - prog_area_y - self.drag_data["offset_y"]
        new_x = max(0, min(new_x, self.CELL_WIDTH - block.winfo_width()))
        new_y = max(
            0, min(new_y, self.GRID_ROWS * self.CELL_HEIGHT - block.winfo_height())
        )
        block.place(x=new_x, y=new_y)
        target_row = max(0, min(int(new_y // self.CELL_HEIGHT), self.GRID_ROWS - 1))
        print(
            f"[on_drag_motion] Block '{block.command_name}' moved to x={new_x}, y={new_y}; target row: {target_row}"
        )
        self.highlight_target_row(target_row)

    def highlight_target_row(self, target_row):
        # Only update if the target row has changed.
        if self.last_highlighted_row == target_row:
            return
        self.last_highlighted_row = target_row

        # Reset all slots and blocks
        for i, slot in enumerate(self.grid_slots):
            slot.config(bg="lightgray")
            if self.grid_cells[i][0]:
                block = self.grid_cells[i][0]
                block.config(bg=block.original_bg)
                if block.winfo_children():
                    block.winfo_children()[0].config(bg=block.original_bg)
        # Now update the target row
        if 0 <= target_row < len(self.grid_slots):
            if self.grid_cells[target_row][0]:
                block = self.grid_cells[target_row][0]
                block.config(bg="darkgray")
                if block.winfo_children():
                    block.winfo_children()[0].config(bg="darkgray")
            else:
                self.grid_slots[target_row].config(bg="lightblue")
        print(f"[highlight_target_row] Highlighting row {target_row}")

    def clear_highlight(self):
        # Reset last highlighted row and UI elements
        self.last_highlighted_row = None
        for i, slot in enumerate(self.grid_slots):
            slot.config(bg="lightgray")
            if self.grid_cells[i][0]:
                block = self.grid_cells[i][0]
                block.config(bg=block.original_bg)
                if block.winfo_children():
                    block.winfo_children()[0].config(bg=block.original_bg)
        print("[clear_highlight] Cleared highlights.")

    def on_drop(self, event, block):
        if block:
            y = block.winfo_y()
            target_row = max(0, min(int(y // self.CELL_HEIGHT), self.GRID_ROWS - 1))
            print(
                f"[on_drop] Block '{block.command_name}' dropped at y={y}, computed target row: {target_row}"
            )
            if self.grid_cells[target_row][0]:
                print(
                    f"[on_drop] Target row {target_row} is occupied. Shifting blocks down."
                )
                self.move_blocks_down(target_row)
            else:
                print(f"[on_drop] Target row {target_row} is empty.")
            block.grid_row = target_row
            block.grid_col = 0
            self.grid_cells[target_row][0] = block
            block.place(x=0, y=target_row * self.CELL_HEIGHT)
            print(f"[on_drop] Placed block '{block.command_name}' in row {target_row}.")
            self.update_command_list()
            self.move_blocks_up()
            print(
                f"[on_drop] After moving blocks up, command_list: {self.command_list}"
            )
        self.drag_data = {
            "widget": None,
            "row": None,
            "col": None,
            "offset_x": 0,
            "offset_y": 0,
        }
        self.clear_highlight()

    def move_blocks_down(self, start_row):
        print(f"[move_blocks_down] Shifting blocks down starting from row {start_row}")
        if len(self.grid_cells) <= start_row:
            print("[move_blocks_down] start_row out of range.")
            return
        for row in range(len(self.grid_cells) - 1, start_row, -1):
            if self.grid_cells[row - 1][0]:
                block = self.grid_cells[row - 1][0]
                print(
                    f"[move_blocks_down] Moving block '{block.command_name}' from row {row-1} to row {row}"
                )
                self.grid_cells[row][0] = block
                self.grid_cells[row - 1][0] = None
                block.grid_row = row
                block.place(x=0, y=row * self.CELL_HEIGHT)
        print("[move_blocks_down] Completed shifting down.")

    def move_blocks_up(self):
        print("[move_blocks_up] Starting upward shift to remove gaps.")
        moved = True
        while moved:
            moved = False
            for row in range(len(self.grid_cells) - 1):
                if not self.grid_cells[row][0] and self.grid_cells[row + 1][0]:
                    block = self.grid_cells[row + 1][0]
                    print(
                        f"[move_blocks_up] Moving block '{block.command_name}' up from row {row+1} to row {row}"
                    )
                    self.grid_cells[row][0] = block
                    self.grid_cells[row + 1][0] = None
                    block.grid_row = row
                    block.place(x=0, y=row * self.CELL_HEIGHT)
                    moved = True
        print("[move_blocks_up] Completed upward shift.")
        self.update_command_list()
        self.refresh_visible_blocks()

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
        os.makedirs("SavedFiles", exist_ok=True)
        file_path = os.path.join("SavedFiles", f"{file_name}.txt")
        try:
            with open(file_path, "w") as file:
                for block in self.command_list:
                    if not block.winfo_exists():
                        continue
                    command_name = block.command_name
                    module_name = getattr(block, "command_module_name", "Unknown")
                    inputs = {var_name: var.get() for var_name, var in block.input_vars.items()}
                    input_line = f"{command_name};{module_name};" + ";".join(f"{k}={v}" for k, v in inputs.items()) + "\n"
                    file.write(input_line)
            messagebox.showinfo("Save", f"Program saved to {file_path}!")
            print(f"[save_program] Program saved to {file_path}")
        except Exception as e:
            messagebox.showerror("Save Error", f"Error saving program: {e}")
            print(f"[save_program] ERROR: {e}")


    def load_program(self, file_name):
        try:
            self.clear_programming_area()  # Clear previous blocks
            file_path = os.path.join("SavedFiles", f"{file_name}.txt")
            with open(file_path, "r") as file:
                for line in file:
                    line = line.strip()
                    if not line:
                        continue
                    # Expected format: command_name;module_name;key=value;key=value;...
                    parts = line.split(";")
                    if len(parts) < 2:
                        print(f"[load_program] Skipping malformed line: {line}")
                        continue
                    command_name = parts[0]
                    module_name = parts[1]
                    # Build inputs dictionary, ignoring empty parts
                    inputs = {}
                    for kv in parts[2:]:
                        if kv and "=" in kv:
                            key, value = kv.split("=", 1)
                            inputs[key] = value
                    print(f"[load_program] Loading block: {command_name} from module {module_name} with inputs {inputs}")
                    
                    modules_commands = load_modules()
                    if module_name not in modules_commands:
                        print(f"[load_program] ERROR: Module '{module_name}' not found!")
                        continue
                    
                    command_module = modules_commands[module_name]["commands"]
                    if command_name not in command_module:
                        print(f"[load_program] ERROR: Command '{command_name}' not found in '{module_name}'!")
                        continue

                    expected_inputs = command_module[command_name].get("inputs", {})

                    # Pass command_module along to create_block_from_data so the block has its module reference.
                    block = self.create_block_from_data(command_name, expected_inputs, inputs, module_name, command_module)
                    if block is None:
                        continue

                    # Find the first available row in grid_cells.
                    placed = False
                    for r in range(len(self.grid_cells)):
                        if self.grid_cells[r][0] is None:
                            block.grid_row = r
                            block.grid_col = 0
                            block.place(x=0, y=r * self.CELL_HEIGHT)
                            self.grid_cells[r][0] = block
                            placed = True
                            print(f"[load_program] Placed block '{command_name}' in row {r}")
                            break
                    if not placed:
                        self.add_row()
                        r = len(self.grid_cells) - 1
                        block.grid_row = r
                        block.grid_col = 0
                        block.place(x=0, y=r * self.CELL_HEIGHT)
                        self.grid_cells[r][0] = block
                        print(f"[load_program] Placed block '{command_name}' in new row {r}")
                    
                    self.command_list.append(block)
                    print(f"[load_program] Appended block '{command_name}' to command_list.")
            self.move_blocks_up()
            self.refresh_visible_blocks()
            self.update_command_list()
            self.move_blocks_up()
            messagebox.showinfo("Load Success", f"Program '{file_name}.txt' loaded successfully.")
            print(f"[load_program] Loaded program from {file_path}")
        except FileNotFoundError:
            messagebox.showerror("Load Error", f"File '{file_name}.txt' not found.")
            print(f"[load_program] ERROR: File '{file_name}.txt' not found.")
        except Exception as e:
            messagebox.showerror("Load Error", f"Error loading program: {e}")
            print(f"[load_program] ERROR: {e}")

    def undo_last_action(self):
        if not self.undo_list:
            messagebox.showinfo("Undo", "Nothing to undo")
            return
        current_state = self.capture_state()
        if current_state is not None:
            self.redo_list.append(current_state)
        last_state = self.undo_list.pop()
        print("[undo_last_action] Restoring state:", last_state)
        self.restore_state(last_state)

    def redo_last_action(self):
        if not self.redo_list:
            messagebox.showinfo("Redo", "Nothing to redo")
            return
        current_state = self.capture_state()
        if current_state is not None:
            self.undo_list.append(current_state)
        last_state = self.redo_list.pop()
        print("[redo_last_action] Restoring state:", last_state)
        self.restore_state(last_state)

    def capture_state(self):
        """
        Capture the current state as a list of tuples:
        (command_name, command_module_name, inputs, row_index)
        """
        state = []
        # Iterate through grid_cells rows.
        for row in range(len(self.grid_cells)):
            block = self.grid_cells[row][0]
            if block is not None and block.winfo_exists():
                command_name = block.command_name
                command_module_name = getattr(block, "command_module_name", "Unknown")
                inputs = {k: v.get() for k, v in block.input_vars.items()}
                state.append((command_name, command_module_name, inputs, row))
        print("[capture_state] Captured state:", state)
        return state

    def restore_state(self, state):
        """
        Restore a previously captured state.
        Clears the programming area and re-creates blocks in their saved order.
        """
        print("[restore_state] Restoring state:", state)
        # Clear the programming area completely.
        self.clear_programming_area()
        modules_commands = load_modules()
        for (command_name, module_name, inputs, row) in state:
            if module_name not in modules_commands:
                print(f"[restore_state] ERROR: Module '{module_name}' not found.")
                continue
            command_module = modules_commands[module_name]["commands"]
            if command_name not in command_module:
                print(f"[restore_state] ERROR: Command '{command_name}' not found in module '{module_name}'.")
                continue
            expected_inputs = command_module[command_name].get("inputs", {})
            # Create block using updated create_block_from_data (now with command_module)
            block = self.create_block_from_data(command_name, expected_inputs, inputs, module_name, command_module)
            if block is None:
                continue
            # Ensure grid_cells has enough rows.
            while row >= len(self.grid_cells):
                self.add_row()
            block.grid_row = row
            block.grid_col = 0
            block.place(x=0, y=row * self.CELL_HEIGHT)
            self.grid_cells[row][0] = block
            self.command_list.append(block)
            print(f"[restore_state] Restored block '{command_name}' in row {row}")
        self.refresh_visible_blocks()
        print("[restore_state] State restored. New command_list:", self.command_list)
        
    def capture_input_change(self):
        # Capture state when a user modifies an input field
        self.undo_list.append(self.capture_state())
        self.redo_list.clear()

    def create_block_from_data(self, command_name, expected_inputs, input_values, module_name, command_module):
        # Obtain the correct color for the module.
        modules_commands = load_modules()
        color = "#FF5733"  # Default color
        if module_name in modules_commands:
            color = modules_commands[module_name]["color"]
            self.module_colors[module_name] = color

        row = len(self.command_list)  # New block goes at the end of command_list
        col = 0
        block = tk.Frame(self.programming_area, bg=color, relief="raised", bd=2,
                        width=self.CELL_WIDTH, height=self.CELL_HEIGHT)
        block.place(x=col * self.CELL_WIDTH, y=row * self.CELL_HEIGHT)
        content_frame = tk.Frame(block, bg=color)
        content_frame.pack(fill="both", expand=True, padx=10, pady=5)
        label = tk.Label(content_frame, text=command_name, bg=color, fg="white",
                        font=("Arial", 12, "bold"), anchor="w")
        label.pack(side="left", padx=5)
        block.command_label = label

        # Store module information in the block.
        block.command_name = command_name
        block.command_module_name = module_name
        block.command_module = command_module

        input_vars = {}
        for label_text, var_name in expected_inputs.items():
            tk.Label(content_frame, text=label_text, bg=color, fg="white", font=("Arial", 10)).pack(side="left", padx=5)
            input_var = tk.StringVar(value=input_values.get(var_name, ""))
            tk.Entry(content_frame, textvariable=input_var, width=8).pack(side="left", padx=5)
            input_vars[var_name] = input_var
        block.input_vars = input_vars

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
        # Destroy all widgets inside the programming_area (grid slots, blocks, etc.)
        for widget in self.programming_area.winfo_children():
            widget.destroy()

        # Recreate grid slots
        self.grid_slots = []
        for i in range(self.visible_rows):
            slot = tk.Frame(
                self.programming_area, bg="lightgray", height=self.CELL_HEIGHT
            )
            slot.grid(row=i, column=0, sticky="ew", padx=5, pady=2)
            self.grid_slots.append(slot)

        # Reset internal tracking data structures
        self.grid_cells = [
            [None for _ in range(self.GRID_COLS)] for _ in range(self.GRID_ROWS)
        ]
        self.command_list.clear()
        self.undo_list.clear()
        self.redo_list.clear()
        self.scroll_position = 0

        print("[clear_programming_area] Programming area cleared.")

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
        if self.scroll_position > 0:
            self.scroll_position -= 1
            self.refresh_visible_blocks()

    def scroll_down(self):
        if self.scroll_position + self.visible_rows < len(self.command_list):
            self.scroll_position += 1
            self.refresh_visible_blocks()

    def refresh_visible_blocks(self):
        # Clear current display
        for slot in self.grid_slots:
            for widget in slot.winfo_children():
                widget.destroy()

        # Populate visible slots based on scroll position
        for i in range(self.visible_rows):
            row_index = self.scroll_position + i
            if row_index < len(self.command_list):
                block = self.command_list[row_index]
                block.grid_row = i  # Update position within visible rows
                block.place(in_=self.grid_slots[i], x=0, y=0)

    def update_command_list(self):
        # Sort the command_list by the block's current vertical position (y coordinate)
        sorted_blocks = sorted(self.command_list, key=lambda block: block.winfo_y())
        # Clear grid_cells and reassign blocks row-by-row
        self.grid_cells = [[None] for _ in range(self.GRID_ROWS)]
        for i, block in enumerate(sorted_blocks):
            block.grid_row = i
            self.grid_cells[i][0] = block
            block.place(x=0, y=i * self.CELL_HEIGHT)
        self.command_list = sorted_blocks
        print("[update_command_list] Command list updated:", self.command_list)
