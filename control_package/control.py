# Author: Zin Lin Htun


import time
import rclpy
from rclpy.node import Node
from annex_msgs.msg import Vcu2ai, Con2vcu

MODE = {"SIM": 1, "REAL": 2}
SERVO_IDS = [11, 12, 13, 21, 22, 23, 31, 32, 33, 41, 42, 43]
LEG_1 = [12, 13, 14]
LEG_2 = [22, 23, 24]
LEG_3 = [32, 33, 34]
LEG_4 = [42, 43, 44]
GEAR_IDS = [14, 24, 34, 44]
OP_MODE = {"DRIVE": 1, "SPIDER": 2}
TORQUE_ADDR = 64
POSITION_ADDR = 116


class Control(Node):
    mode = MODE["REAL"]
    op_mode = OP_MODE["SPIDER"]

    def __init__(self):
        super().__init__('control')
        self.publisher_ = self.create_publisher(Con2vcu, 'adsmt/manual_control', 10)
        self.mode = OP_MODE['SPIDER']

    def execute(self, cmd):
        msg = Con2vcu()
        msg.mode = 1.0
        msg.dir = cmd * 1.0
        self.publisher_.publish(msg)


# terminal application
def terminal_app(node):
    while True:
        cmd = input("Command w,s,a,d : ")
        if cmd == "w":
            node.execute(1.0)
        elif cmd == "a":
            node.execute(2.0)
        elif cmd == "d":
            node.execute(3.0)
        elif cmd == "s":
            node.execute(4.0)
        elif cmd == "jf":
            node.execute(5.0)
        elif cmd == "df":
            node.execute(6.0)
        elif cmd == "dr":
            node.execute(7.0)
        elif cmd == "dl":
            node.execute(8.0)
        elif cmd == "hf":
            node.execute(9.0)
        elif cmd == "hr":
            node.execute(10.0)
        elif cmd == "hl":
            node.execute(11.0)
        else:
            node.execute(20.0)


# main method
def main(args=None):
    rclpy.init(args=args)

    con = Control()
    try:
        # Start the command-line interface
        terminal_app(con)
    except KeyboardInterrupt:
        print("\nApplication interrupted using Ctrl+C")
    rclpy.spin(con)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    con.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
