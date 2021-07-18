from tkinter import *
from camera import *

class gui:
    def __init__(self, frame1):
        self.root = Tk()
        self.root.title("Dinamika_99 Main Program")
        self.root.geometry("1280x768")
        self.frame1 = frame1 
    
    def display(self):
        self.frame1 = Canvas(
                self.root, 
                width = 480, 
                height = 270)
        self.frame1.pack()
        self.root.mainloop()

    def update(self):
        # Get a frame from the video source
        ret, frame = self.frame1.get_frame()
        self.window.after(5, self.update)

frame = camera(2)
window = gui(frame)
window.display()
