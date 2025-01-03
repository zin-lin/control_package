# Author : Zin Lin Htun

from dynamixel_sdk import *  # Dynamixel SDK library import
import dynamixel_sdk as dxl
import time
import rclpy
from rclpy.node import Node
from annex_msgs.msg import Vcu2ai


MODE = {"SIM":1, "REAL":2}
SERVO_IDS = [11, 12, 13, 21, 22, 23, 31, 32, 33, 41, 42, 43]
LEG_1 = [ 12, 13, 14]
LEG_2 = [ 22, 23, 24]
LEG_3 = [ 32, 33, 34]
LEG_4 = [ 42, 43, 44]
GEAR_IDS = [14, 24, 34, 44]
OP_MODE = {"DRIVE":1, "SPIDER":2}
TORQUE_ADDR = 64
POSITION_ADDR = 116

class VehicleControl(Node):

    mode = MODE["REAL"]
    op_mode = OP_MODE["SPIDER"]
    def __init__(self):
        super().__init__('vehicle_control')
        if self.mode == MODE["REAL"]:

            # Set the port and baudrate
            DEVICENAME = '/dev/ttyUSB0'  # Modify this according to your setup
            BAUDRATE = 57600  # Modify this according to your Dynamixel configuration

            # Define protocol version
            PROTOCOL_VERSION = 2.0

            # Initialize PortHandler instance
            self.portHandler = PortHandler(DEVICENAME)

            # Initialize PacketHandler instance
            self.packetHandler = PacketHandler(PROTOCOL_VERSION)

            # Open the port
            if self.portHandler.openPort():
                print("Succeeded to open the port")
            else:
                print("Failed to open the port")
                exit(1)

            # Set the baudrate
            if self.portHandler.setBaudRate(BAUDRATE):
                print("Succeeded to change the baudrate")
            else:
                print("Failed to change the baudrate")
                exit(1)

            for DXL_ID in SERVO_IDS:
                dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, DXL_ID, TORQUE_ADDR,
                                                                               1)  # Torque enable
                if dxl_comm_result != COMM_SUCCESS:
                    print(f"TX is :: {dxl_comm_result} %s" % self.packetHandler.getTxRxResult(dxl_comm_result))
                elif dxl_error != 0:
                    print("Error:: %s" % self.packetHandler.getRxPacketError(dxl_error))
                else:
                    print("Torque changed")

            # Enable torque for multiple motors
            # self.swing_neutral()

        self.walk_forward()

    # single servo joint connection
    def writeGoalPos(self, val, DXL_ID):

            # Write goal position
            dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(self.portHandler, DXL_ID, 116, val)  # Write goal position
            if dxl_comm_result != COMM_SUCCESS:
                print(f"TX is :: {dxl_comm_result} %s" % self.packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("Error:: %s" % self.packetHandler.getRxPacketError(dxl_error))
            else:
                print(f"Goal position set to: {val} neutral")

    def transform (self, op_mode):
        self.op_mode = op_mode

        for DXL_ID in GEAR_IDS:
            dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, DXL_ID, TORQUE_ADDR, 0 if self.op_mode == OP_MODE["SPIDER"] else 1)  # Torque enable
            if dxl_comm_result != COMM_SUCCESS:
                print(f"TX is :: {dxl_comm_result} %s" % self.packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("Error:: %s" % self.packetHandler.getRxPacketError(dxl_error))
            else:
                print("Torque changed")

    def swing_forward_right(self, leg=[] ):
        for DID in leg:
            if DID == 22 or DID == 42:
                self.writeGoalPos(2050, DID)
            else:
                self.writeGoalPos(1350, DID)

    def swing_backward_right(self, leg=[] ):
        for DID in leg:
            if DID == 22 or DID == 42:
                self.writeGoalPos(1600, DID)
            else:
                self.writeGoalPos(1350, DID)

    def swing_forward_left(self, leg=[] ):
        for DID in leg:
            if DID == 12 or DID == 32:
                self.writeGoalPos(2050, DID)
            else:
                self.writeGoalPos(2750, DID)

    def swing_neutral(self):
        for DXL_ID in SERVO_IDS:
            if DXL_ID == 12 or DXL_ID == 13 or DXL_ID == 32 or DXL_ID == 33:
                if DXL_ID == 13 or DXL_ID == 33:
                    self.writeGoalPos(2500, DXL_ID)
                else:
                    self.writeGoalPos(2250, DXL_ID)
            # right legs
            elif DXL_ID == 22 or DXL_ID == 23 or DXL_ID == 42 or DXL_ID == 43:
                if DXL_ID == 23 or DXL_ID == 43:
                    self.writeGoalPos(1600, DXL_ID)
                else:
                    self.writeGoalPos(1825, DXL_ID)
            # wheels and joints
            # else:
            #     if DXL_ID == 11:
            #         self.writeGoalPos(2000, DXL_ID)
            #
            #     if DXL_ID == 21:
            #         self.writeGoalPos(2000, DXL_ID)
            #
            #     if DXL_ID == 31:
            #         self.writeGoalPos(2000, DXL_ID)
            #
            #     else:
            #         self.writeGoalPos(2000, DXL_ID)


    def swing_backward_left(self, leg=[] ):
        for DID in leg:
            if DID == 12 or DID == 32:
                self.writeGoalPos(2500, DID)
            else:
                self.writeGoalPos(2750, DID)

    # move forward
    def walk_forward(self,):
        # # first half of the cycle
        # # left legs
        # self.swing_backward_left(LEG_1)
        # time.sleep(0.3)
        # self.swing_backward_left(LEG_3)
        # time.sleep(0.3)
        #
        # # right legs
        # self.swing_forward_right(LEG_2)
        # time.sleep(0.3)
        #
        # self.swing_forward_right(LEG_4)
        # time.sleep(0.3)
        #
        # # neutral
        # self.swing_neutral()
        #
        # # second half of the cycle
        # # left legs
        # self.swing_backward_right(LEG_2)
        # time.sleep(0.3)
        #
        # self.swing_backward_right(LEG_4)
        # time.sleep(0.3)
        #
        # # right legs
        # self.swing_forward_left(LEG_1)
        # time.sleep(0.3)
        #
        # self.swing_forward_left(LEG_3)
        # time.sleep(0.3)
        #
        # # neutral
        # self.swing_neutral()
        self.writeGoalPos(2300,33)
        self.writeGoalPos(1800,43)
        return

    def leg(self,id, val):
        # Enable torque for a single motor
        ids = [id]
        for DXL_ID in ids:
            dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, DXL_ID, TORQUE_ADDR, 1)  # Torque enable
            if dxl_comm_result != COMM_SUCCESS:
                print(f"TX is :: {dxl_comm_result} %s" % self.packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("Error:: %s" % self.packetHandler.getRxPacketError(dxl_error))
            else:
                print("Torque enabled")

            # Write goal position
            dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(self.portHandler, DXL_ID, POSITION_ADDR, val)  # Write goal position
            if dxl_comm_result != COMM_SUCCESS:
                print(f"TX is :: {dxl_comm_result} %s" % self.packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("Error:: %s" % self.packetHandler.getRxPacketError(dxl_error))
            else:
                print(f"Goal position set to: {val} neutral")

    # Uncomment this section for continuous control
    # count = 1
    # while True:
    #     if count % 2 == 0:
    #         leg1_2_3(2000)
    #     else:
    #         leg1_2_3(0)
    #     count += 1

    # Set goal position for leg with ID 33 to 0
    # leg(33, 0)

    # Close port
    def end(self):
        self.portHandler.closePort()


# main method
def main(args=None):
    rclpy.init(args=args)

    vehicle_control = VehicleControl()

    rclpy.spin(vehicle_control)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    vehicle_control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


