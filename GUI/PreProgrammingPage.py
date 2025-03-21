import tkinter as tk
from tkinter import ttk, messagebox
from utils import load_modules, send_command
import threading
from Modules.Movement import COMMANDS
from Modules.Movement import COMPENSATION_TIME
import os
import time


class PreProgrammingPage:
    # Programming grid setup
    GRID_ROWS = 12
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
        self.visible_rows = 12  # Number of visible rows at a time
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
            module_label = tk.Label(
                command_frame, text=module_name, font=("Arial", 10, "bold")
            )
            module_label.pack(pady=5)
            for command_name in commands.keys():
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
        self.log_action(
            f"[add_block_to_grid] Added block '{block.command_name}' in module '{module_name}' at row {next_row}"
        )

    def create_block(self, command_label, row, col, command_module, module_name):
        command_name = command_label.cget("text").strip()
        if command_name not in command_module:
            self.log_action(
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
        block.original_bg = color
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
        self.drag_data["widget"] = block

        # Get programming area absolute coordinates.
        prog_area_x = self.programming_area.winfo_rootx()
        prog_area_y = self.programming_area.winfo_rooty()

        # Get block's absolute position (using its current parent's coordinate system).
        block_abs_x = block.winfo_rootx()
        block_abs_y = block.winfo_rooty()

        # Compute the offset between the cursor and the block's top-left.
        # (This is not used when centering, but might be useful if you switch modes.)
        self.drag_data["offset_x"] = event.x_root - block_abs_x
        self.drag_data["offset_y"] = event.y_root - block_abs_y

        # Store the parent's offset relative to programming_area.
        parent = block.master  # grid slot
        self.drag_data["parent_offset_x"] = parent.winfo_rootx() - prog_area_x
        self.drag_data["parent_offset_y"] = parent.winfo_rooty() - prog_area_y

        self.drag_data["row"] = block.grid_row
        self.drag_data["col"] = block.grid_col

        # Free the grid cell
        self.grid_cells[block.grid_row][block.grid_col] = None
        self.clear_highlight()


    def on_drag_motion(self, event, block):
        # Get programming area absolute coordinates.
        prog_area_x = self.programming_area.winfo_rootx()
        prog_area_y = self.programming_area.winfo_rooty()

        # Compute where the block should be placed in the programming area's coordinate system
        # so that its center is under the cursor.
        new_x_abs = event.x_root - prog_area_x - block.winfo_width() / 2
        new_y_abs = event.y_root - prog_area_y - block.winfo_height() / 2

        # Convert these absolute coordinates to coordinates relative to the block's parent.
        # This is where the parent's offset comes into play.
        parent_offset_x = self.drag_data.get("parent_offset_x", 0)
        parent_offset_y = self.drag_data.get("parent_offset_y", 0)
        new_x_local = new_x_abs - parent_offset_x
        
        if block.grid_row == 0:
            new_y_local = new_y_abs - parent_offset_y
        else:
            new_y_local = new_y_abs - parent_offset_y - 75*block.grid_row
        """
        # Optional: clamp values to keep the block within bounds.
        new_x_local = max(0, min(new_x_local, self.CELL_WIDTH - block.winfo_width()))
        
        if block.grid_row == 0:
            new_y_local = max(0, min(new_y_local, self.GRID_ROWS * self.CELL_HEIGHT - block.winfo_height()))
        else:
            new_y_local = max(0, min(new_y_local, self.GRID_ROWS * self.CELL_HEIGHT - block.winfo_height()))
        """
        
        block.place(x=new_x_local, y=new_y_local)

        target_row = max(0, min(int(new_y_abs // self.CELL_HEIGHT), self.GRID_ROWS - 1))
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


    def on_drop(self, event, block):
        # Determine target row based on highlighted row or block's position.
        if self.last_highlighted_row is not None:
            target_row = self.last_highlighted_row
        else:
            y = block.winfo_y()
            target_row = int(y // self.CELL_HEIGHT)
        
        # Clamp the target row to the visible region.
        visible_top = self.scroll_position
        visible_bottom = self.scroll_position + self.visible_rows - 1
        if target_row < visible_top:
            target_row = visible_top
        elif target_row > visible_bottom:
            target_row = visible_bottom

        print(f"Dropping block '{block.command_name}' at target row: {target_row}")

        # Update the command list by removing the block from its old position and inserting at target_row.
        if block in self.command_list:
            self.command_list.remove(block)
        # If target_row is beyond current length, append.
        if target_row >= len(self.command_list):
            self.command_list.append(block)
        else:
            self.command_list.insert(target_row, block)
        """
        # Rebuild the grid layout based on the updated command list.
        self.grid_cells = [[None for _ in range(self.GRID_COLS)] for _ in range(self.GRID_ROWS)]
        for idx, blk in enumerate(self.command_list):
            blk.grid_row = idx
            blk.grid_col = 0
            self.grid_cells[idx][0] = blk
            # Place each block in its proper row.
            blk.place(x=0, y=idx * self.CELL_HEIGHT)
        """
        # Update any UI elements related to scrolling or ordering.
        self.update_command_list()
        self.clear_highlight()
        self.refresh_visible_blocks()

        # Clear drag data and highlights.
        self.drag_data = {"widget": None, "row": None, "col": None, "offset_x": 0, "offset_y": 0}
        self.clear_highlight()


    def move_block_down(self, visible_row):
        # visible_row is the row index relative to the visible grid (0 <= visible_row < visible_rows)
        abs_index = self.scroll_position + visible_row
        if abs_index < len(self.command_list) - 1:
            # Swap the block with the next block in the absolute order
            self.command_list[abs_index], self.command_list[abs_index + 1] = (
                self.command_list[abs_index + 1],
                self.command_list[abs_index],
            )
            self.update_command_list()  # Update any state if needed
            self.refresh_visible_blocks()  # Rebuild grid_slots from command_list

    def move_block_up(self, visible_row):
        abs_index = self.scroll_position + visible_row
        if abs_index > 0:
            # Swap with the previous block
            self.command_list[abs_index], self.command_list[abs_index - 1] = (
                self.command_list[abs_index - 1],
                self.command_list[abs_index],
            )
            self.update_command_list()
            self.refresh_visible_blocks()


    def handle_button_click(self, button_text):
        # Handles button clicks and logs actions
        self.log_action(f"Button clicked: {button_text}")
        if button_text == "Run":
            self.controller.pages["ObservationPage"].log_action("Program started...")
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
        
        self.controller.pages["ObservationPage"].update_progress(0)  # Start
        
        self.controller.show_page("ObservationPage")

        # Start execute_commands in a new thread
        threading.Thread(target=self.execute_commands, daemon=True).start()

    def execute_commands(self):
        # Executes commands sequentially, waiting until each finishes. Speech runs in parallel
        self.stop_event = threading.Event()
        total_commands = len(self.command_list)
        if total_commands == 0:
            self.controller.pages["ObservationPage"].update_progress(0)
            return

        for index, block in enumerate(self.command_list):
            if self.controller.stop_flag:
                print("Stop event set")
                break
            	
            if self.stop_event.is_set():
                print("Stop event set")
                break
        
        
            command_name = getattr(block, "command_name", None)
            command_module = getattr(block, "command_module", None)

            if not command_name:
                print(f"ERROR: Block at row {block.grid_row} has no command_name!")
                continue

            if not command_module:
                print(f"ERROR: Block '{command_name}' has no module reference!")
                continue

            if command_name not in command_module:
                print(
                    f"ERROR: '{command_name}' not found in module! Available: {list(command_module.keys())}"
                )
                self.controller.pages["ObservationPage"].log_action(
                    f"Error: Unknown command {command_name}"
                )
                continue

            command_info = command_module[command_name]
            command_function = command_info.get("function")

            inputs = {}
            for var_name, var in block.input_vars.items():
                input_value = var.get().strip()
                inputs[var_name] = input_value

            try:
                self.controller.pages["ObservationPage"].log_action(
                    f"Executing: {command_name} with {inputs}"
                )

                if command_name == "Speak Text":
                    # Speech runs in parallel (does not block execution)
                    threading.Thread(
                        target=command_function, args=inputs.values(), daemon=True
                    ).start()
                elif command_name == "Pause":
                    command_function(*inputs.values())
                else:
                    # Other commands execute one after the other
                    start_time = time.time()  # ⏳ Start timing execution

                    command_function(*inputs.values())  # Run the function

                    # Check for command completion dynamically
                    execution_time = self.estimate_execution_time(command_name, inputs) + COMPENSATION_TIME
                    time.sleep(execution_time)  # Wait for the command to finish

            except Exception as e:
                self.controller.pages["ObservationPage"].log_action(
                    f"Error running {command_name}: {e}"
                )

            # Update progress bar
            percent_complete = ((index + 1) / total_commands) * 100
            self.controller.pages["ObservationPage"].update_progress(percent_complete)

        self.controller.pages["ObservationPage"].update_progress(100)  # Complete
        self.controller.pages["ObservationPage"].log_action(f"Execution Completed")
        self.controller.pages["ObservationPage"].save_command_log()

    def estimate_execution_time(self, command_name, inputs):
        # Estimates execution time dynamically based on command type and input values
        # Example: Movement commands → Use distance & speed for estimation
        
        
        if command_name in ["Move Forward", "Move Backward"]:
            distance = float(inputs.get("distance", 0))  # Get distance
            speed = 0.5
            return ((distance / speed) + COMPENSATION_TIME)

        # Example: Rotation commands → Use degrees & rotation speed
        elif command_name in ["Rotate Left", "Rotate Right"]:
            degrees = float(inputs.get("degrees", 0))
            rotation_speed = 1
            return (
                (degrees * 3.14159265 / 180) / COMPENSATION_TIME)

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

                    modules_commands = load_modules()
                    if module_name not in modules_commands:
                        print(
                            f"[load_program] ERROR: Module '{module_name}' not found!"
                        )
                        continue

                    command_module = modules_commands[module_name]["commands"]
                    if command_name not in command_module:
                        print(
                            f"[load_program] ERROR: Command '{command_name}' not found in '{module_name}'!"
                        )
                        continue

                    expected_inputs = command_module[command_name].get("inputs", {})

                    # Pass command_module along to create_block_from_data so the block has its module reference.
                    block = self.create_block_from_data(
                        command_name,
                        expected_inputs,
                        inputs,
                        module_name,
                        command_module,
                    )
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
                            break
                    if not placed:
                        self.add_row()
                        r = len(self.grid_cells) - 1
                        block.grid_row = r
                        block.grid_col = 0
                        block.place(x=0, y=r * self.CELL_HEIGHT)
                        self.grid_cells[r][0] = block

                    self.command_list.append(block)
            #self.move_block_up()
            self.refresh_visible_blocks()
            #self.update_command_list()
            #self.move_block_up()
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
        current_state = self.capture_state()
        if current_state is not None:
            self.redo_list.append(current_state)
        last_state = self.undo_list.pop()
        self.restore_state(last_state)

    def redo_last_action(self):
        if not self.redo_list:
            messagebox.showinfo("Redo", "Nothing to redo")
            return
        current_state = self.capture_state()
        if current_state is not None:
            self.undo_list.append(current_state)
        last_state = self.redo_list.pop()
        self.restore_state(last_state)

    def capture_state(self):
        # Capture the current state as a list of tuples: (command_name, command_module_name, inputs, row_index)
        state = []
        # Iterate through grid_cells rows.
        for row in range(len(self.grid_cells)):
            block = self.grid_cells[row][0]
            if block is not None and block.winfo_exists():
                command_name = block.command_name
                command_module_name = getattr(block, "command_module_name", "Unknown")
                inputs = {k: v.get() for k, v in block.input_vars.items()}
                state.append((command_name, command_module_name, inputs, row))
        return state

    def restore_state(self, state):
        # Restore a previously captured state.
        # Clear the programming area completely.
        self.clear_programming_area()
        modules_commands = load_modules()
        for command_name, module_name, inputs, row in state:
            if module_name not in modules_commands:
                print(f"[restore_state] ERROR: Module '{module_name}' not found.")
                continue
            command_module = modules_commands[module_name]["commands"]
            if command_name not in command_module:
                print(
                    f"[restore_state] ERROR: Command '{command_name}' not found in module '{module_name}'."
                )
                continue
            expected_inputs = command_module[command_name].get("inputs", {})
            # Create block using updated create_block_from_data (now with command_module)
            block = self.create_block_from_data(
                command_name, expected_inputs, inputs, module_name, command_module
            )
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
        self.refresh_visible_blocks()

    def capture_input_change(self):
        # Capture state when a user modifies an input field
        self.undo_list.append(self.capture_state())
        self.redo_list.clear()

    def create_block_from_data(
        self, command_name, expected_inputs, input_values, module_name, command_module
    ):
        # Obtain the correct color for the module.
        modules_commands = load_modules()
        color = "#FF5733"  # Default color
        if module_name in modules_commands:
            color = modules_commands[module_name]["color"]
            self.module_colors[module_name] = color

        row = len(self.command_list)  # New block goes at the end of command_list
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
        content_frame = tk.Frame(block, bg=color)
        content_frame.pack(fill="both", expand=True, padx=10, pady=5)
        label = tk.Label(
            content_frame,
            text=command_name,
            bg=color,
            fg="white",
            font=("Arial", 12, "bold"),
            anchor="w",
        )
        label.pack(side="left", padx=5)
        block.command_label = label

        # Store module information in the block.
        block.command_name = command_name
        block.command_module_name = module_name
        block.command_module = command_module

        input_vars = {}
        for label_text, var_name in expected_inputs.items():
            tk.Label(
                content_frame, text=label_text, bg=color, fg="white", font=("Arial", 10)
            ).pack(side="left", padx=5)
            input_var = tk.StringVar(value=input_values.get(var_name, ""))
            tk.Entry(content_frame, textvariable=input_var, width=8).pack(
                side="left", padx=5
            )
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
        # self.undo_list.clear()
        # self.redo_list.clear()
        self.scroll_position = 0

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
        # Clear current display in all visible grid slots.
        for slot in self.grid_slots:
            for widget in slot.winfo_children():
                widget.destroy()

        # Iterate through all blocks in the command_list.
        for idx, block in enumerate(self.command_list):
            # If the block is within the visible range, reparent and display it.
            if self.scroll_position <= idx < self.scroll_position + self.visible_rows:
                visible_index = idx - self.scroll_position
                block.grid_row = visible_index  # Update relative grid row.
                block.place(in_=self.grid_slots[visible_index], x=0, y=0)
            else:
                # Hide blocks outside the visible area.
                block.place_forget()



    def update_command_list(self):
        # Sort the command_list by the block's current vertical position (y coordinate)
        sorted_blocks = sorted(self.command_list, key=lambda block: block.winfo_y())
        
        # Reinitialize grid_cells with the correct dimensions.
        # This ensures that each row has GRID_COLS columns.
        self.grid_cells = [[None for _ in range(self.GRID_COLS)] for _ in range(self.GRID_ROWS)]
        
        # Reassign blocks to rows and update their placement.
        for i, block in enumerate(sorted_blocks):
            block.grid_row = i
            # Assign the block to the grid_cells list (column 0)
            if i < len(self.grid_cells):
                self.grid_cells[i][0] = block
            block.place(x=0, y=i * self.CELL_HEIGHT)
        
        self.command_list = sorted_blocks

