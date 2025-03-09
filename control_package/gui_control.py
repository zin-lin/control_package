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
        self.window_height = 378

        # Calculate position to center the window
        x_position = (screen_width // 2) - (self.window_width // 2)
        y_position = (screen_height // 2) - (self.window_height // 2)

        self.geometry(f"{self.window_width}x{self.window_height}+{x_position}+{y_position}")
        # self.resizable(False, False)

        self.control_node.logger.info(self.winfo_name())
        self._initialise_gui_widgets()


    def _dup (self, event):
        self.send_command(6.0) # drive forward

    def _dright (self, event):
        self.send_command(7.0) # drive right

    def _dleft (self, event):
        self.send_command(8.0) # drive left

    def _wfor (self, event):
        self.send_command(1.0) # walk forward

    def _wright (self, event):
        self.send_command(3.0) # turn right

    def _wleft (self, event):
        self.send_command(2.0) # turn left

    # stop
    def _stop (self, event):
        self.send_command(20.0) # turn left


    def _move_window(self, event):
        self.control_node.logger.info('called')
        self.geometry(f'+{event.x_root}+{event.y_root}')

    def _initialise_gui_widgets(self):
        self.configure(bg=BG)
        # kill default title bar
        # self.overrideredirect(True)

        self.resizable(False, False)

        # bind to move around
        # self.bind('<Button-1>', self.get_pos)

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
        self.self_bind()

    def self_bind(self):
        self.focus_set()
        # drive-
        self.bind('<Up>', self._dup)
        # self.bind('<Down>', self.get_pos)
        self.bind('<Right>', self._dright)
        self.bind('<Left>', self._dleft)

        # walk
        self.bind('<w>', self._wfor)
        # self.bind('<Down>', self.get_pos)
        self.bind('<a>', self._wleft)
        self.bind('<d>', self._wright)

        # stop
        self.bind('<space>', self._stop)

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
