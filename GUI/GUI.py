import tkinter as tk
from GUI.StartScreen import StartScreen
from GUI.PreProgrammingPage import PreProgrammingPage
from GUI.ObservationPage import ObservationPage


class GUIController:
    def __init__(self):
        # Initialize the GUI controller
        self.root = tk.Tk()
        self.root.title("CtlrR")
        self.root.geometry(f"{self.root.winfo_screenwidth()}x{self.root.winfo_screenheight()}")

        # Create a container frame for all pages
        self.container = tk.Frame(self.root)
        self.container.pack(fill="both", expand=True)

        # Initialize pages
        self.pages = {}
        self.initialize_pages()

    def initialize_pages(self):
        # Initialize and store all pages
        self.pages["StartScreen"] = StartScreen(self.container, self)
        self.pages["PreProgrammingPage"] = PreProgrammingPage(self.container, self)
        self.pages["ObservationPage"] = ObservationPage(self.container, self)

        # Stack all pages in the container and ensure they fill the window
        for page in self.pages.values():
            page.frame.grid(row=0, column=0, sticky="nsew")

        self.container.rowconfigure(0, weight=1)
        self.container.columnconfigure(0, weight=1)

        # Show the start screen by default
        self.show_page("StartScreen")


    def show_page(self, page_name):
        # Bring the specified page to the front
        if page_name in self.pages:
            self.pages[page_name].frame.tkraise()

    def run(self):
        """Run the GUI application."""
        self.root.mainloop()


if __name__ == "__main__":
    GUIController().run()
