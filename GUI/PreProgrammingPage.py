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
        self.grid_cells = [[None for _ in range(self.GRID_COLS)] for _ in range(self.GRID_ROWS)]
        self.command_list = []  
        self.undo_list = []  
        self.redo_list = []  

        # To handle row highlighting
        self.highlight_rect = None
        
        self.module_colors = {}
        
        self.setup_ui()

    def setup_ui(self):
        self.frame.rowconfigure([0, 1], weight=1)
        self.frame.columnconfigure(0, weight=1)  # Commands section
        self.frame.columnconfigure(1, weight=4)  # Programming area (reduced width)
        self.frame.columnconfigure(2, weight=2)  # Log area

        # Box A: Commands list
        self.command_section = tk.LabelFrame(self.frame, text="Commands", bg="white", font=("Arial", 10, "bold"))
        self.command_section.grid(row=0, column=0, rowspan=2, sticky="nsew", padx=5, pady=5)
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
            tk.Button(box_b, text=btn_text, bg=btn_color, fg="white", width=10, font=("Arial", 10, "bold"),
                      command=lambda t=btn_text: self.handle_button_click(t)).pack(side="left", padx=10, pady=5)

        # File name input
        tk.Label(box_b, text="File Name:", bg="white", font=("Arial", 10)).pack(side="left", padx=5)
        self.file_name_entry = tk.Entry(box_b, width=20, font=("Arial", 10))
        self.file_name_entry.pack(side="left", padx=10)
        self.file_name_entry.insert(0, "program")  # Default file name

        # Box C: Programming area
        programming_frame = tk.Frame(middle_frame, bg="white")
        programming_frame.grid(row=1, column=0, sticky="nsew", padx=5, pady=5)

        self.scrollbar = ttk.Scrollbar(programming_frame, orient="vertical")
        self.scrollbar.pack(side="right", fill="y")

        self.programming_area = tk.Canvas(programming_frame, bg="lightgray",
                                          width=self.CELL_WIDTH, height=self.GRID_ROWS * self.CELL_HEIGHT,
                                          highlightthickness=0, yscrollcommand=self.scrollbar.set)
        self.programming_area.pack(side="left", fill="both", expand=True)
        self.scrollbar.config(command=self.programming_area.yview)

        self.canvas_content = tk.Frame(self.programming_area, bg="lightgray")
        self.programming_area.create_window((0, 0), window=self.canvas_content, anchor="nw")

        self.programming_area.bind("<MouseWheel>", self.on_mouse_wheel)
        self.update_scroll_region()

        # **Box D: Log Area**
        self.log_section = tk.LabelFrame(self.frame, text="Action Log", bg="white", font=("Arial", 10, "bold"))
        self.log_section.grid(row=0, column=2, rowspan=2, sticky="nsew", padx=5, pady=5)

        self.log_text = tk.Text(self.log_section, wrap="word", height=20, state="disabled", bg="lightgray")
        self.log_text.pack(fill="both", expand=True, padx=5, pady=5)
        self.log_scroll = ttk.Scrollbar(self.log_section, command=self.log_text.yview)
        self.log_scroll.pack(side="right", fill="y")
        self.log_text.config(yscrollcommand=self.log_scroll.set)

    def log_action(self, message):
        """Appends a message to the log section."""
        self.log_text.config(state="normal")
        self.log_text.insert("end", f"{message}\n")
        self.log_text.config(state="disabled")
        self.log_text.yview("end")  # Auto-scroll to the latest log


    def populate_command_section(self, command_frame):
        modules_commands = load_modules()  # Load all available modules

        for module_name, module_data in modules_commands.items():
            commands = module_data["commands"]
            color = module_data["color"]

            print(f"📦 Loading Module: {module_name}, Available Commands: {list(commands.keys())}")  # Debugging print

            module_label = tk.Label(command_frame, text=module_name, font=("Arial", 10, "bold"))
            module_label.pack(pady=5)

            for command_name in commands.keys():
                print(f"🔹 Adding command: '{command_name}' from Module: {module_name}")  # Debugging print

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
                command_label.bind("<Button-1>", lambda e, label=command_label, mod=commands: self.add_block_to_grid(label, mod))



    def add_block_to_grid(self, command_label, command_module):
        self.undo_list.append(self.capture_state())
        self.redo_list.clear()

        for row in range(len(self.grid_cells)):
            if not self.grid_cells[row][0]:  # Find an empty row
                block = self.create_block(command_label, row, 0, command_module)  # Pass correct module
                self.grid_cells[row][0] = block
                self.command_list.append(block)
                return

        self.add_row()
        self.add_block_to_grid(command_label, command_module)




    def create_block(self, command_label, row, col, command_module):
        command_name = command_label.cget("text").strip()

        print(f"🛠 Creating block for '{command_name}' from module {command_module}")  # Debugging print

        if command_name not in command_module:
            print(f"🚨 ERROR: '{command_name}' not found in module. Available: {list(command_module.keys())}")
            return None

        inputs = command_module[command_name].get("inputs", {})
        print(f"🎤 Inputs for '{command_name}': {inputs}")  # Debugging print

        color = command_label.cget("bg")
        block = tk.Frame(
            self.programming_area, bg=color, relief="raised", bd=2,
            width=self.CELL_WIDTH, height=self.CELL_HEIGHT
        )
        block.place(x=col * self.CELL_WIDTH, y=row * self.CELL_HEIGHT)

        content_frame = tk.Frame(block, bg=color)
        content_frame.pack(fill="both", expand=True, padx=10, pady=5)

        # ✅ Store both the command name and module reference in the block
        block.command_name = command_name
        block.command_module = command_module  # Store module reference

        label = tk.Label(
            content_frame, text=command_name, bg=color, fg="white", font=("Arial", 12, "bold"), anchor="w"
        )
        label.pack(side="left", padx=5)

        input_vars = {}

        for label_text, var_name in inputs.items():
            if not isinstance(var_name, str):  
                print(f"🚨 Error: Unexpected input key '{var_name}' in {command_name}. Skipping.")
                continue

            print(f"🎤 Adding input field: {label_text} -> {var_name}")  # Debugging print
            tk.Label(content_frame, text=label_text, bg=color, fg="white", font=("Arial", 10)).pack(side="left", padx=5)

            input_var = tk.StringVar()
            input_entry = tk.Entry(content_frame, textvariable=input_var, width=20)
            input_entry.pack(side="left", padx=5)

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
        """Capture state only if a block was actually moved."""
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
                if new_state and (not self.undo_list or self.undo_list[-1] != new_state):
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
        """Handles button clicks and logs actions."""
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
        """Executes commands using send_command() in a background thread and logs it."""
        self.controller.show_page("ObservationPage")
        self.log_action("Program started...")

        def execute_commands():
            """Executes commands without blocking the UI."""
            for block in self.command_list:
                command_name = getattr(block, "command_name", None)
                command_module = getattr(block, "command_module", None)

                if not command_name:
                    print(f"🚨 ERROR: Block at row {block.grid_row} has no command_name!")
                    continue

                if not command_module:
                    print(f"🚨 ERROR: Block '{command_name}' has no module reference!")
                    continue

                print(f"▶ Running Command: {command_name} from Module: {command_module}")

                if command_name not in command_module:
                    print(f"🚨 ERROR: '{command_name}' not found in module! Available: {list(command_module.keys())}")
                    self.log_action(f"Error: Unknown command {command_name}")
                    continue

                command_info = command_module[command_name]
                command_function = command_info.get("function")

                print(f"🛠 Function Retrieved: {command_function}")  # Debugging print

                inputs = {}
                for var_name, var in block.input_vars.items():
                    input_value = var.get().strip()
                    print(f"🔹 Input for {command_name}: {var_name} = '{input_value}'")  # Debugging print
                    inputs[var_name] = input_value

                try:
                    print(f"🚀 Executing: {command_name} with inputs {inputs}")  # Debugging print
                    self.log_action(f"Executing: {command_name} with {inputs}")
                    command_function(*inputs.values())  # Run the function with inputs
                except Exception as e:
                    print(f"❌ Execution Failed for {command_name}: {e}")
                    self.log_action(f"Error running {command_name}: {e}")


                    
        # 🔹 Start execute_commands in a new thread so it doesn't freeze the UI
        import threading
        threading.Thread(target=execute_commands, daemon=True).start()




    def save_program(self, file_name):
        os.makedirs("SavedFiles", exist_ok=True)  # Ensure the directory exists
        if not file_name:
            messagebox.showwarning("Save", "Please enter a valid file name.")
            return

        file_path = os.path.join("SavedFiles", f"{file_name}.txt")
        try:
            with open(file_path, "w") as file:
                for block in self.command_list:
                    command_name = block.command_label.cget("text")
                    inputs = {var_name: var.get() for var_name, var in block.input_vars.items()}
                    input_line = f"{command_name};" + ";".join(f"{k}={v}" for k, v in inputs.items()) + "\n"
                    file.write(input_line)
            messagebox.showinfo("Save", f"Program saved to {file_path}!")
        except Exception as e:
            messagebox.showerror("Save Error", f"Error saving program: {e}")



    def load_program(self, file_name):
        try:
            # Clear the programming area before loading
            self.clear_programming_area()
            
            file_path = os.path.join("SavedFiles", f"{file_name}.txt")
            with open(file_path, "r") as file:
                for line in file:
                    line = line.strip()
                    if not line:
                        continue
                    
                    # Split command and input data
                    parts = line.split(";")
                    command_name = parts[0]
                    inputs = {}
                    
                    # Parse inputs
                    for input_part in parts[1:]:
                        key, value = input_part.split("=")
                        inputs[key] = value
                    
                    # Validate the command exists
                    if command_name not in COMMANDS:
                        raise ValueError(f"Command '{command_name}' not found.")
                    
                    # Create the block using saved data
                    command_data = COMMANDS[command_name]
                    block = self.create_block_from_data(command_name, command_data["inputs"], inputs)
                    
                    # Find an empty row in the grid and place the block
                    for row in range(len(self.grid_cells)):
                        if not self.grid_cells[row][0]:  # Find the first empty row
                            block.grid_row = row
                            block.grid_col = 0
                            block.place(x=0, y=row * self.CELL_HEIGHT)
                            self.grid_cells[row][0] = block  # Update grid_cells
                            break
                    
                    self.command_list.append(block)  # Add to command list
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

        # Capture current state before undoing, ensuring it's not a duplicate
        current_state = self.capture_state()
        if current_state and (not self.redo_list or self.redo_list[-1] != current_state):
            self.redo_list.append(current_state)

        last_state = self.undo_list.pop()
        self.restore_state(last_state)

    def redo_last_action(self):
        if not self.redo_list:
            messagebox.showinfo("Redo", "Nothing to redo")
            return

        # Capture current state before redoing, ensuring it's not a duplicate
        current_state = self.capture_state()
        if current_state and (not self.undo_list or self.undo_list[-1] != current_state):
            self.undo_list.append(current_state)

        last_state = self.redo_list.pop()
        self.restore_state(last_state)

    def capture_state(self):
        """Captures the current state of the programming area."""
        state = []
        for row in range(len(self.grid_cells)):
            block = self.grid_cells[row][0]
            if block:
                command_name = getattr(block, "command_name", None)
                if not command_name:
                    print(f"🚨 Warning: Block at row {row} has no command_name. Skipping.")
                    continue  # Skip blocks without a valid command name

                inputs = {var_name: var.get() for var_name, var in block.input_vars.items()}
                state.append((command_name, inputs, row))  # Store row position

        return state

    
    def capture_input_change(self):
        """Capture state when a user modifies an input field."""
        self.undo_list.append(self.capture_state())
        self.redo_list.clear()

    
    def restore_state(self, state):
        """Restore a previous state of the programming area."""
        self.clear_programming_area()

        for command_name, inputs, row in state:
            if command_name not in COMMANDS:
                messagebox.showerror("Restore Error", f"Command '{command_name}' not found.")
                continue

            # Create block using stored data
            command_data = COMMANDS[command_name]
            block = self.create_block_from_data(command_name, command_data["inputs"], inputs)

            # Place block in the correct row
            self.grid_cells[row][0] = block
            block.grid_row = row
            block.grid_col = 0
            block.place(x=0, y=row * self.CELL_HEIGHT)

            self.command_list.append(block)

        self.move_blocks_up()  # Ensure blocks are positioned correctly

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
            input_var = tk.StringVar(value=input_values.get(var_name, ""))  # Set saved value
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

    def on_mouse_wheel(self, event):
        """Handles mouse wheel scrolling."""
        self.programming_area.yview_scroll(-1 * (event.delta // 120), "units")

    def update_scroll_region(self):
        """Updates the canvas scroll region."""
        self.programming_area.update_idletasks()
        self.programming_area.config(scrollregion=self.programming_area.bbox("all"))

    def update_command_list(self):
        # Update the list of commands
        self.command_list = [self.grid_cells[row][0] for row in range(len(self.grid_cells)) if self.grid_cells[row][0]];
        print(self.command_list)
