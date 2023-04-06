import os
from dynamixel_sdk import * # Uses Dynamixel SDK library

if os.name == 'nt':
    import msvcrt
    def getch():
        return msvcrt.getch().decode()
else:
    import sys, tty, termios
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    def getch():
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch


MY_DXL = 'X_SERIES'       # X330 (5.0 V recommended), X430, X540, 2X430

# Control table address
if MY_DXL == 'X_SERIES' or MY_DXL == 'MX_SERIES':
    ADDR_TORQUE_ENABLE          = 64
    ADDR_GOAL_POSITION          = 116
    ADDR_PRESENT_POSITION       = 132
    DXL_MINIMUM_POSITION_VALUE  = 0         # Refer to the Minimum Position Limit of product
    DXL_MAXIMUM_POSITION_VALUE  = 4095      # Refer to the Maximum Position Limit of product
    BAUDRATE                    = 1000000

def pos_to_angle(pos):
    return int(pos/4095*365)

def angle_to_pos(angle):
    return int(4095/365 * angle)


PROTOCOL_VERSION            = 2.0

# Factory default ID of all DYNAMIXEL is 1
# DXL_ID                      = 13
DXL_ID_list                      = [11, 12, 13]
DEVICE_NUM = len(DXL_ID_list)

DEVICENAME                  = '/dev/ttyUSB1'


TORQUE_ENABLE               = 1     # Value for enabling the torque
TORQUE_DISABLE              = 0     # Value for disabling the torque
DXL_MOVING_STATUS_THRESHOLD = 10    # Dynamixel moving status threshold

index = 0

# portHandler_list = PortHandler(DEVICENAME)

portHandler_list = [PortHandler(DEVICENAME) for _ in range(DEVICE_NUM)]

# packetHandler_list[0] = PacketHandler(PROTOCOL_VERSION)
packetHandler_list = [PacketHandler(PROTOCOL_VERSION) for _ in range(DEVICE_NUM)]

def open_port_and_baud(portHandler_list):
    # Open port
    if portHandler_list.openPort():
        print("Succeeded to open the port")
    else:
        print("Failed to open the port")
        print("Press any key to terminate...")
        getch()
        quit()
    # Set port baudrate
    if portHandler_list.setBaudRate(BAUDRATE):
        print("Succeeded to change the baudrate")
    else:
        print("Failed to change the baudrate")
        print("Press any key to terminate...")
        getch()
        quit()

# Enable Dynamixel Torque
def enable_torque(packetHandler, portHandler_list, DXL_ID):
    global ADDR_TORQUE_ENABLE, TORQUE_ENABLE, COMM_SUCCESS
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler_list, DXL_ID, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    else:
        print("Dynamixel has been successfully connected")

for i in range(0, DEVICE_NUM):
    open_port_and_baud(portHandler_list[i])
    enable_torque(packetHandler_list[i], portHandler_list[i], DXL_ID_list[i])



for i in range(DEVICE_NUM):
    dxl_present_position, dxl_comm_result, dxl_error = packetHandler_list[i].read4ByteTxRx(portHandler_list[i], DXL_ID_list[i], ADDR_PRESENT_POSITION)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler_list[i].getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler_list[i].getRxPacketError(dxl_error))

    print(f'id:{DXL_ID_list[i]}, PresPos:{pos_to_angle(dxl_present_position)}({dxl_present_position})')
