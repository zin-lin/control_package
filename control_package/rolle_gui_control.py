import rclpy
from TkOffice import *
from rclpy.node import Node
import threading
import os

from annex_msgs.msg import Con2vcu, Aicon2vcu
from rclpy.executors import MultiThreadedExecutor

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
    13.0:"drive back",
    20.0: "stop",
}


class Control(Node):
    def __init__(self):
        super().__init__('control')
        self.rpm = 0.0
        self.ss_steering_ratio = 0.0

        self.publisher_ = self.create_publisher(
            Aicon2vcu,
            'a_a_v/ads_roll_e/ai_control',
            10
        )
        self.logger = self.get_logger()

        # flags controlled by GUI
        self.frpm = False
        self.brpm = False
        self.right = False
        self.left = False

        # KEEP A REFERENCE TO THE TIMER
        self.timer = self.create_timer(0.2, self.timer_callback)

    # --- reset helpers from GUI ---

    def reset_left(self):
        self.left = False
        self.ss_steering_ratio = 0.0

    def reset_right(self):
        self.right = False
        self.ss_steering_ratio = 0.0

    def reset_frpm(self):
        self.frpm = False   # <-- FIXED

    def reset_brpm(self):
        self.brpm = False

    def reset_steering(self):
        self.ss_steering_ratio = 0.0

    # --- periodic control loop ---

    def timer_callback(self):
        msg = Aicon2vcu()

        # log so you can SEE the timer is firing


        # no input → full stop
        if not (self.frpm or self.brpm or self.right or self.left):
            self.rpm = 0.0
            self.ss_steering_ratio = 0.0
            msg.rad_s_l = 0.0
            msg.rad_s_r = 0.0
            self.publisher_.publish(msg)
            return

        # forward / backward rpm ramp
        if self.frpm:
            if self.rpm < 10:
                self.rpm += 4.5
        elif self.brpm:
            if self.rpm > -18.0:
                self.rpm -= 4.5

        # left / right steering
        if self.right:
            if self.ss_steering_ratio > -1.0:
                self.ss_steering_ratio -= 0.25
        elif self.left:
            if self.ss_steering_ratio < 1.0:
                self.ss_steering_ratio += 0.25

        # wheel speeds from steering ratio
        if self.ss_steering_ratio == 0.0:
            msg.rad_s_l = self.rpm
            msg.rad_s_r = self.rpm
        else:
            if self.ss_steering_ratio > 0.0:
                msg.rad_s_l = self.rpm * (1 - abs(self.ss_steering_ratio))
                msg.rad_s_r = self.rpm
            elif self.ss_steering_ratio < 0.0:
                msg.rad_s_r = self.rpm * (1 - abs(self.ss_steering_ratio))
                msg.rad_s_l = self.rpm   # <-- FIXED

        self.publisher_.publish(msg)

    # --- commands from GUI ---

    def execute(self, cmd: float):
        """Set control flags based on command from GUI."""
        if cmd == 6.0:         # forward
            self.frpm = True
            self.brpm = False

        elif cmd == 7.0:       # right
            self.right = True
            self.left = False

        elif cmd == 8.0:       # left
            self.left = True
            self.right = False

        elif cmd == 13.0:      # backward
            self.brpm = True
            self.frpm = False

        else:                  # stop
            self.frpm = False
            self.brpm = False
            self.left = False
            self.right = False


class ControlGUI(tk.Tk):
    def __init__(self, control_node: Control):
        super().__init__(baseName="ADS-MT", className='ADS-ROLL-E')
        self.control_node = control_node

        package_share_directory = os.path.dirname(os.path.abspath(__file__))
        file_to_icon = os.path.join(package_share_directory, 'imgs/icon_main.png')
        photo = PhotoImage(file=file_to_icon)
        og_icon = Image.open(file_to_icon)
        re_icon = og_icon.resize((20, 20))
        self.photo = ImageTk.PhotoImage(re_icon)
        self.iconphoto(False, photo)

        self.mode = "none"
        self.title("Control Package - ADS-ROLL-E")
        self.attributes("-alpha", 0.7)

        # center window
        screen_width = self.winfo_screenwidth()
        screen_height = self.winfo_screenheight()
        self.window_width = 600
        self.window_height = 378
        x_position = (screen_width // 2) - (self.window_width // 2)
        y_position = (screen_height // 2) - (self.window_height // 2)
        self.geometry(f"{self.window_width}x{self.window_height}+{x_position}+{y_position}")

        self._initialise_gui_widgets()

    # --- key handlers ---

    def _dup(self, event):
        self.send_command(6.0)  # drive forward

    def _dright(self, event):
        self.send_command(7.0)  # drive right

    def _dleft(self, event):
        self.send_command(8.0)  # drive left

    def _ddown(self, event):
        self.send_command(13.0)  # drive backwards

    def _stop(self, event=None):
        self.send_command(20.0)  # stop

    def _resetter_left(self, event):
        self.control_node.reset_left()

    def _resetter_right(self, event):
        self.control_node.reset_right()

    def _move_window(self, event):
        self.control_node.logger.info('called')
        self.geometry(f'+{event.x_root}+{event.y_root}')

    # --- GUI setup ---

    def _initialise_gui_widgets(self):
        self.configure(bg=BG)
        self.resizable(False, False)

        self.label = Label(self, text=f"Current Command: {self.mode}", bg=BG, fg="#703b46")
        self.label.pack(side=TOP, fill=BOTH, padx=5, pady=5)

        btn = Button(
            self, text="drive straight ^",
            command=lambda: self.send_command(6.0),
            height=2, width=22,
            bg='#1b1824', fg="#703b46", bdcolor="#703b46",
            bd=2, activebackground='#131119',
            activeforeground='#eee', highlightthickness=0,
            font=("Terminal", 9),
        )
        btn.pack(padx=5, pady=5)

        btn = Button(
            self, text="drive back v",
            command=lambda: self.send_command(13.0),
            height=2, width=22,
            bg='#1b1824', fg="#703b46", bdcolor="#703b46",
            bd=2, activebackground='#131119',
            activeforeground='#eee', highlightthickness=0,
            font=("Terminal", 9),
        )
        btn.pack(padx=5, pady=5)

        btn = Button(
            self, text="drive left <",
            command=lambda: self.send_command(8.0),
            height=2, width=22,
            bg='#1b1824', fg="#703b46", bdcolor="#703b46",
            bd=2, activebackground='#131119',
            activeforeground='#eee', highlightthickness=0,
            font=("Terminal", 9),
        )
        btn.pack(padx=5, pady=5)

        btn = Button(
            self, text="drive right >",
            command=lambda: self.send_command(7.0),
            height=2, width=22,
            bg='#1b1824', fg="#703b46", bdcolor="#703b46",
            bd=2, activebackground='#131119',
            activeforeground='#eee', highlightthickness=0,
            font=("Terminal", 9),
        )
        btn.pack(padx=5, pady=5)

        btn = Button(
            self, text="stop",
            command=lambda: self.send_command(20.0),
            height=2, width=22,
            bg='#1b1824', fg="#703b46", bdcolor="#703b46",
            bd=2, activebackground='#131119',
            activeforeground='#eee', highlightthickness=0,
            font=("Terminal", 9),
        )
        btn.pack(padx=5, pady=5)

        self.self_bind()

    def self_bind(self):
        self.focus_set()
        # press
        self.bind('<Up>', self._dup)
        self.bind('<Down>', self._ddown)
        self.bind('<Right>', self._dright)
        self.bind('<Left>', self._dleft)

        # release
        self.bind('<KeyRelease-Right>', self._resetter_right)
        self.bind('<KeyRelease-Left>', self._resetter_left)
        self.bind('<KeyRelease-Up>', self._stop)
        self.bind('<KeyRelease-Down>', self._stop)

        self.bind('<space>', self._stop)

    def send_command(self, command: float):
        self.control_node.execute(command)
        self.mode = MODES[command]
        self.label.configure(text=f"Current Command: {self.mode}")


# main method
def main():
    rclpy.init()

    control_node = Control()

    # ROS executor in background thread
    executor = MultiThreadedExecutor()
    executor.add_node(control_node)

    def ros_spin():
        try:
            executor.spin()
        except Exception as e:
            print("Executor spin error:", e)

    ros_thread = threading.Thread(target=ros_spin, daemon=True)
    ros_thread.start()

    # Tkinter GUI in main thread
    gui = ControlGUI(control_node)
    gui.mainloop()

    # cleanup after window closed
    executor.shutdown()
    control_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
