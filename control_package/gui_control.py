import rclpy
from TkOffice import *
from rclpy.node import Node

from annex_msgs.msg import Con2vcu

# Control Modes
MODE = {"SIM": 1, "REAL": 2}
OP_MODE = {"DRIVE": 1, "SPIDER": 2}
BG = '#1b1824'
MODES = {
    1.0: "walk forward",
    2.0: "walk left",
    3.0: "walk right",
    6.0: "drive forward",
    7.0: "drive right",
    8.0: "drive left",
    20.0: "stop",
}

class Control(Node):
    def __init__(self):
        super().__init__('control')
        self.publisher_ = self.create_publisher(Con2vcu, 'adsmt/manual_control', 10)
        self.logger = self.get_logger()

    def execute(self, cmd):
        """Send command to ROS 2 topic."""
        msg = Con2vcu()
        msg.mode = 1.0
        msg.dir = cmd * 1.0
        self.publisher_.publish(msg)
        return f"Command Sent: {cmd}"


class ControlGUI(tk.Tk):
    def __init__(self, control_node:Control):
        super().__init__(baseName="ADS-MT", className='ADS-MT')
        self.control_node = control_node
        package_share_directory =current_file_directory = os.path.dirname(os.path.abspath(__file__))
        file_to_icon = os.path.join(package_share_directory ,'imgs/icon_main.png')
        photo = PhotoImage(file=file_to_icon)
        og_icon = Image.open(file_to_icon)
        re_icon = og_icon.resize((20,20))
        self.photo = ImageTk.PhotoImage(re_icon)
        self.iconphoto(False, photo)
        self.mode = "none"
        self.title("Control Package - ADSMT")
        self.attributes("-alpha", 0.7)
        # Get screen width and height
        screen_width = self.winfo_screenwidth()
        screen_height = self.winfo_screenheight()

        # Set window size
        self.window_width = 600
        self.window_height = 400

        # Calculate position to center the window
        x_position = (screen_width // 2) - (self.window_width // 2)
        y_position = (screen_height // 2) - (self.window_height // 2)

        self.geometry(f"{self.window_width}x{self.window_height}+{x_position}+{y_position}")
        # self.resizable(False, False)

        self.control_node.logger.info(self.winfo_name())
        self._initialise_gui_widgets()

    def get_pos(self,event):
        x_win = self.winfo_x()
        y_win = self.winfo_y()
        start_x = event.x_root
        start_y = event.y_root

        ywin = y_win - start_y
        xwin = x_win - start_x

        def move_window(event_in):
            self.geometry("600x400" + '+{0}+{1}'.format(event_in.x_root + xwin, event_in.y_root + ywin))

        start_x = event.x_root
        start_y = event.y_root

        self.bind('<B1-Motion>', move_window)


    def _move_window(self, event):
        self.control_node.logger.info('called')
        self.geometry(f'+{event.x_root}+{event.y_root}')

    def _initialise_gui_widgets(self):
        self.configure(bg=BG)
        # kill default title bar
        self.overrideredirect(True)

        # lay out necessary widgets

        # make a ADS-Mt Titlebar
        title_bar = Frame(self, bg=BG,highlightthickness=0, borderwidth=0,)
        title_bar.pack(fill=X, side=TOP)
        # put a close button on the title bar
        close_button =Button(title_bar, text='[X]', command=self.destroy, width=10, foreground='#703b46',highlightthickness=0, borderwidth=0,bd=0, bg= BG, activebackground=BG, activeforeground='#ff5757')
        close_button.pack(side=RIGHT, padx=5, pady=5)

        # Load the icon
        icon_label = Label(title_bar, image=self.photo, width=20, height=20, highlightthickness=0, bg=BG, foreground="#aaa", borderwidth=0,)
        icon_label.pack(side="left", padx=10)

        # Add Title
        label = Label(title_bar, text="Control-Package: ADS-MT", font=("Consolas", 9, 'bold'), highlightthickness=0, bg=BG, foreground="#60627a")
        label.pack(side="left", padx=10)

        # bind to move around
        self.bind('<Button-1>', self.get_pos)

        self.label = Label(self, text=f"Current Command: {self.mode}", bg = BG, fg="#703b46",)
        self.label.pack(side=TOP, fill=BOTH, padx=5, pady=5)

        button = Button(self, text="walk straight ^", command= lambda: self.send_command(1.0), height=2, width=22, bg='#1b1824', fg="#703b46", bdcolor="#703b46",
                        bd= 2,  activebackground='#131119',  activeforeground='#eee', highlightthickness=0, font=("Terminal", 9),
                        )
        button.pack(padx=5, pady=5)

        button = Button(self, text="walk left <", command= lambda: self.send_command(2.0), height=2, width=22, bg='#1b1824', fg="#703b46", bdcolor="#703b46",
                        bd= 2,  activebackground='#131119',  activeforeground='#eee', highlightthickness=0,font=("Terminal", 9),
                        )
        button.pack(padx=5, pady=5)

        button = Button(self, text="walk right >", command= lambda: self.send_command(3.0), height=2, width=22, bg='#1b1824', fg="#703b46", bdcolor="#703b46",
                        bd= 2,  activebackground='#131119',  activeforeground='#eee', highlightthickness=0,font=("Terminal", 9),
                        )
        button.pack(padx=5, pady=5)

        button = Button(self, text="drive straight ^", command= lambda: self.send_command(6.0), height=2, width=22, bg='#1b1824', fg="#703b46", bdcolor="#703b46",
                        bd= 2,  activebackground='#131119',  activeforeground='#eee', highlightthickness=0,font=("Terminal", 9),
                        )
        button.pack(padx=5, pady=5)

        button = Button(self, text="drive left <", command= lambda: self.send_command(8.0), height=2, width=22, bg='#1b1824', fg="#703b46", bdcolor="#703b46",
                        bd= 2,  activebackground='#131119',  activeforeground='#eee', highlightthickness=0,font=("Terminal", 9),
                        )
        button.pack(padx=5, pady=5)

        button = Button(self, text="drive right >", command= lambda: self.send_command(7.0), height=2, width=22, bg='#1b1824', fg="#703b46", bdcolor="#703b46",
                        bd= 2,  activebackground='#131119',  activeforeground='#eee', highlightthickness=0,font=("Terminal", 9),
                        )
        button.pack(padx=5, pady=5)

        button = Button(self, text="stop [-]", command= lambda: self.send_command(20.0), height=2, width=22, bg='#1b1824', fg="#703b46", bdcolor="#703b46",
                        bd= 2,  activebackground='#131119',  activeforeground='#eee', highlightthickness=0,font=("Terminal", 9),
                        )
        button.pack(padx=5, pady=5)

    def send_command(self, command):
        """Send command to ROS 2 node and update UI."""
        response = self.control_node.execute(command)
        self.control_node.logger.info(f"called {MODES[command]}")
        self.mode = MODES[command]
        self.label.configure(text=f"Current Command: {self.mode}")
        self.label.update()

    def destroy(self):
        super().destroy()
        self.control_node.destroy_node()
        rclpy.shutdown()

def main():
    rclpy.init()

    control_node = Control()
    gui = ControlGUI(control_node,)
    gui.mainloop()

    try:
        rclpy.spin(control_node)  # Keep ROS 2 node running
    except KeyboardInterrupt:
        print("\nApplication interrupted using Ctrl+C")
    finally:
        control_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
