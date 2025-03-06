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

        # (Unused since we're now highlighting using grid slot backgrounds)
        # self.highlight_rect = None

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

                # Pass the module name along with the commands dict.
                command_label.bind(
                    "<Button-1>",
                    lambda e, label=command_label, mod=commands, mod_name=module_name: self.add_block_to_grid(
                        label, mod, mod_name
                    ),
                )

    def add_block_to_grid(self, command_label, command_module, module_name):
        # Add a new block to the command list
        self.undo_list.append(self.capture_state())
        self.redo_list.clear()

        # Get the next available row (always col=0)
        next_row = len(self.command_list)
        col = 0

        # Create the block with correct parameters
        block = self.create_block(command_label, next_row, col, command_module, module_name)
        if block:
            self.command_list.append(block)

        self.refresh_visible_blocks()  # Refresh UI after adding

    def create_block(self, command_label, row, col, command_module, module_name):
        command_name = command_label.cget("text").strip()

        if command_name not in command_module:
            print(f"ERROR: '{command_name}' not found in module. Available: {list(command_module.keys())}")
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

        # Bind events to the block
        block.bind("<ButtonPress-1>", lambda event, blk=block: self.on_drag_start(event, blk))
        block.bind("<B1-Motion>", lambda event, blk=block: self.on_drag_motion(event, blk))
        block.bind("<ButtonRelease-1>", lambda event, blk=block: self.on_drop(event, blk))

        # Create content_frame first
        content_frame = tk.Frame(block, bg=color)
        content_frame.pack(fill="both", expand=True, padx=10, pady=5)

        # Bind events to content_frame
        content_frame.bind("<ButtonPress-1>", lambda event, blk=block: self.on_drag_start(event, blk))
        content_frame.bind("<B1-Motion>", lambda event, blk=block: self.on_drag_motion(event, blk))
        content_frame.bind("<ButtonRelease-1>", lambda event, blk=block: self.on_drop(event, blk))

        block.command_name = command_name
        block.command_module = command_module  # Store module reference (dict)
        block.command_module_name = module_name   # Store module name for saving/restoring
        block.grid_row = row  # Assign row
        block.grid_col = col  # Assign column

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
            tk.Label(content_frame, text=label_text, bg=color, fg="white", font=("Arial", 10)).pack(side="left", padx=5)
            input_var = tk.StringVar()
            input_entry = tk.Entry(content_frame, textvariable=input_var, width=20)
            input_entry.pack(side="left", padx=5)
            input_vars[var_name] = input_var

        block.input_vars = input_vars

        return block

    def on_drag_start(self, event, block):
        # Fix dragging so the block stays attached to the cursor
        self.undo_list.append(self.capture_state())
        self.redo_list.clear()

        # Store original block position and the offset
        self.drag_data["widget"] = block
        self.drag_data["offset_x"] = event.x
        self.drag_data["offset_y"] = event.y
        self.drag_data["orig_x"] = block.winfo_x()
        self.drag_data["orig_y"] = block.winfo_y()

        # Ensure block is on top
        block.lift()

    def on_drag_motion(self, event, block):
        if not self.drag_data["widget"]:
            return

        # Calculate new position using the stored offsets
        new_x = event.x_root - self.frame.winfo_rootx() - self.drag_data["offset_x"]
        new_y = event.y_root - self.frame.winfo_rooty() - self.drag_data["offset_y"]

        new_x = max(0, min(new_x, self.CELL_WIDTH - block.winfo_width()))
        new_y = max(0, min(new_y, self.GRID_ROWS * self.CELL_HEIGHT - block.winfo_height()))

        block.place(x=new_x, y=new_y)

        # Calculate target row based on new_y and highlight it
        target_row = max(0, min(int(new_y // self.CELL_HEIGHT), self.GRID_ROWS - 1))
        self.highlight_target_row(target_row)

    def highlight_target_row(self, target_row):
        # Reset all grid slots to the default background
        for slot in self.grid_slots:
            slot.configure(bg="lightgray")
        # Highlight the target slot if within bounds
        if 0 <= target_row < len(self.grid_slots):
            self.grid_slots[target_row].configure(bg="lightblue")

    def clear_highlight(self):
        for slot in self.grid_slots:
            slot.configure(bg="lightgray")

    def clear_rows_above(self, target_row):
        for r in range(target_row):
            if self.grid_cells[r][0] is not None:
                block_to_clear = self.grid_cells[r][0]
                block_to_clear.destroy()  # Remove from UI
                self.grid_cells[r][0] = None
                if block_to_clear in self.command_list:
                    self.command_list.remove(block_to_clear)

    def on_drop(self, event, block):
        if not block:
            return

        # Determine the target row for dropping
        y = block.winfo_y()
        target_row = max(0, min(y // self.CELL_HEIGHT, self.GRID_ROWS - 1))
        
        # Clear rows above the target_row
        self.clear_rows_above(target_row)
        
        # If the target row is occupied, shift blocks down
        if self.grid_cells[target_row][0] is not None:
            self.move_blocks_down(target_row)

        # Place the block in the target row
        block.grid_row = target_row
        block.grid_col = 0
        self.grid_cells[target_row][0] = block
        block.place(x=0, y=target_row * self.CELL_HEIGHT)

        # Capture state for undo/redo
        self.undo_list.append(self.capture_state())
        self.redo_list.clear()

        # Adjust remaining blocks upward if there are gaps
        self.move_blocks_up()

        # Clear highlight and reset drag data
        self.clear_highlight()
        self.drag_data = {"widget": None, "row": None, "col": None}

    def move_blocks_down(self, start_row):
        if len(self.grid_cells) <= start_row:
            return  # Invalid row index

        # If the last row is full, add a new row at the bottom
        if self.grid_cells[-1][0] is not None:
            self.grid_cells.append([None for _ in range(self.GRID_COLS)])

        # Shift blocks downwards from bottom to start_row
        for row in range(len(self.grid_cells) - 1, start_row, -1):
            if self.grid_cells[row - 1][0]:
                block = self.grid_cells[row - 1][0]
                self.grid_cells[row][0] = block
                self.grid_cells[row - 1][0] = None
                block.grid_row = row
                block.place(x=0, y=row * self.CELL_HEIGHT)

        self.check_and_add_rows()

    def move_blocks_up(self):
        moved = True
        while moved:
            moved = False
            for row in range(len(self.grid_cells) - 1):
                if self.grid_cells[row][0] is None and self.grid_cells[row + 1][0] is not None:
                    block = self.grid_cells[row + 1][0]
                    self.grid_cells[row][0] = block
                    self.grid_cells[row + 1][0] = None
                    block.grid_row = row
                    block.place(x=0, y=row * self.CELL_HEIGHT)
                    moved = True

    def handle_button_click(self, button_text):
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

    def run_program(self):
        self.controller.show_page("ObservationPage")
        self.log_action("Program started...")
        threading.Thread(target=self.execute_commands, daemon=True).start()

    def execute_commands(self):
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
                print(f"ERROR: '{command_name}' not found in module! Available: {list(command_module.keys())}")
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
                    threading.Thread(
                        target=command_function, args=tuple(inputs.values()), daemon=True
                    ).start()
                else:
                    start_time = time.time()
                    command_function(*inputs.values())
                    execution_time = self.estimate_execution_time(command_name, inputs)
                    print(f"Estimated Execution Time for {command_name}: {execution_time:.2f} seconds")
                    time.sleep(execution_time)
            except Exception as e:
                print(f"Execution Failed for {command_name}: {e}")
                self.log_action(f"Error running {command_name}: {e}")

            percent_complete = ((index + 1) / total_commands) * 100
            self.controller.pages["ObservationPage"].update_progress(percent_complete)

        self.controller.pages["ObservationPage"].update_progress(100)

    def estimate_execution_time(self, command_name, inputs):
        if command_name in ["Move Forward", "Move Backward"]:
            distance = float(inputs.get("distance", 0))
            speed = 0.5
            return (distance / speed) + 1
        elif command_name in ["Rotate Left", "Rotate Right"]:
            degrees = float(inputs.get("degrees", 0))
            rotation_speed = 1
            return ((degrees * 3.14159265 / 180) / rotation_speed) + 1
        return 10

    def save_program(self, file_name):
        os.makedirs("SavedFiles", exist_ok=True)
        if not file_name:
            messagebox.showwarning("Save", "Please enter a valid file name.")
            return

        file_path = os.path.join("SavedFiles", f"{file_name}.txt")
        try:
            with open(file_path, "w") as file:
                for block in self.command_list:
                    command_name = block.command_name
                    module_name = getattr(block, "command_module_name", "Unknown")
                    inputs = {var_name: var.get() for var_name, var in block.input_vars.items()}
                    input_line = f"{command_name};{module_name};" + ";".join(f"{k}={v}" for k, v in inputs.items()) + "\n"
                    file.write(input_line)
            messagebox.showinfo("Save", f"Program saved to {file_path}!")
        except Exception as e:
            messagebox.showerror("Save Error", f"Error saving program: {e}")

    def load_program(self, file_name):
        try:
            self.clear_programming_area()
            file_path = os.path.join("SavedFiles", f"{file_name}.txt")
            with open(file_path, "r") as file:
                for line in file:
                    line = line.strip()
                    if not line:
                        continue
                    parts = line.split(";")
                    command_name = parts[0]
                    module_name = parts[1]
                    inputs = {kv.split("=")[0]: kv.split("=")[1] for kv in parts[2:]}
                    modules_commands = load_modules()
                    if module_name not in modules_commands:
                        print(f"ERROR: Module '{module_name}' not found!")
                        continue
                    command_module = modules_commands[module_name]["commands"]
                    if command_name not in command_module:
                        print(f"ERROR: Command '{command_name}' not found in '{module_name}'!")
                        continue
                    block = self.create_block_from_data(command_name, command_module, inputs, module_name)
                    for row in range(len(self.grid_cells)):
                        if not self.grid_cells[row][0]:
                            block.grid_row = row
                            block.grid_col = 0
                            block.place(x=0, y=row * self.CELL_HEIGHT)
                            self.grid_cells[row][0] = block
                            break
                    self.command_list.append(block)
                self.move_blocks_up()
            messagebox.showinfo("Load Success", f"Program '{file_name}.txt' loaded successfully.")
        except FileNotFoundError:
            messagebox.showerror("Load Error", f"File '{file_name}.txt' not found.")
        except Exception as e:
            messagebox.showerror("Load Error", f"Error loading program: {e}")

    def undo_last_action(self):
        if not self.undo_list:
            messagebox.showinfo("Undo", "Nothing to undo")
            return

        current_state = self.capture_state()
        if current_state and (not self.redo_list or self.redo_list[-1] != current_state):
            self.redo_list.append(current_state)
        last_state = self.undo_list.pop()
        self.restore_state(last_state)

    def redo_last_action(self):
        if not self.redo_list:
            messagebox.showinfo("Redo", "Nothing to redo")
            return

        current_state = self.capture_state()
        if current_state and (not self.undo_list or self.undo_list[-1] != current_state):
            self.undo_list.append(current_state)
        last_state = self.redo_list.pop()
        self.restore_state(last_state)

    def capture_state(self):
        state = []
        for i, block in enumerate(self.command_list):
            command_name = block.command_name
            command_module_name = getattr(block, "command_module_name", "Unknown")
            inputs = {var_name: var.get() for var_name, var in block.input_vars.items()}
            state.append((command_name, command_module_name, inputs, i))
        return state if not self.undo_list or self.undo_list[-1] != state else None

    def capture_input_change(self):
        self.undo_list.append(self.capture_state())
        self.redo_list.clear()

    def restore_state(self, state):
        self.clear_programming_area()
        modules_commands = load_modules()
        for command_name, module_name, inputs, row in state:
            if module_name not in modules_commands:
                print(f"🚨 ERROR: Module '{module_name}' not found during restore!")
                continue
            command_module = modules_commands[module_name]["commands"]
            if command_name not in command_module:
                print(f"🚨 ERROR: Command '{command_name}' not found in '{module_name}'!")
                continue
            block = self.create_block_from_data(command_name, command_module, inputs, module_name)
            self.grid_cells[row][0] = block
            block.grid_row = row
            block.grid_col = 0
            block.place(x=0, y=row * self.CELL_HEIGHT)
            self.command_list.append(block)
        self.move_blocks_up()

    def create_block_from_data(self, command_name, expected_inputs, input_values, module_name):
        color = self.module_colors.get(command_name.split()[0], "#FF5733")
        row = len(self.command_list)
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
        input_vars = {}
        for label_text, var_name in expected_inputs.items():
            tk.Label(content_frame, text=label_text, bg=color, fg="white", font=("Arial", 10)).pack(side="left", padx=5)
            input_var = tk.StringVar(value=input_values.get(var_name, ""))
            tk.Entry(content_frame, textvariable=input_var, width=8).pack(side="left", padx=5)
            input_vars[var_name] = input_var
        block.input_vars = input_vars
        block.grid_row = row
        block.grid_col = col
        block.command_name = command_name
        block.command_module_name = module_name
        return block

    def create_command_label(self, command_name):
        return tk.Label(
            self.command_section,
            text=command_name,
            bg="white",
            fg="black",
            font=("Arial", 10),
        )

    def clear_programming_area(self):
        for slot in self.grid_slots:
            for widget in slot.winfo_children():
                widget.destroy()
        self.grid_cells = [
            [None for _ in range(self.GRID_COLS)] for _ in range(self.GRID_ROWS)
        ]
        self.command_list.clear()
        self.undo_list.clear()
        self.redo_list.clear()
        self.scroll_position = 0
        self.refresh_visible_blocks()

    def add_row(self):
        self.grid_cells.append([None for _ in range(self.GRID_COLS)])

    def check_and_add_rows(self):
        empty_rows = sum(1 for row in self.grid_cells if row[0] is None)
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
        for slot in self.grid_slots:
            for widget in slot.winfo_children():
                widget.destroy()
        for i in range(self.visible_rows):
            row_index = self.scroll_position + i
            if row_index < len(self.command_list):
                block = self.command_list[row_index]
                block.grid_row = i
                block.place(in_=self.grid_slots[i], x=0, y=0)

    def update_command_list(self):
        self.command_list = [
            self.grid_cells[row][0]
            for row in range(len(self.grid_cells))
            if self.grid_cells[row][0]
        ]
        print(self.command_list)
