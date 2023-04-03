import os
from dynamixel_sdk import * # Uses Dynamixel SDK library

def pos_to_angle(pos):
    return int(pos/4095*365)

def angle_to_pos(angle):
    return int(4095/365 * angle)

# Open port baudrate
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


if __name__ == '__main__':
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


    ADDR_TORQUE_ENABLE          = 64
    ADDR_GOAL_POSITION          = 116
    ADDR_PRESENT_POSITION       = 132
    DXL_MINIMUM_POSITION_VALUE  = 0         # Refer to the Minimum Position Limit of product
    DXL_MAXIMUM_POSITION_VALUE  = 4095      # Refer to the Maximum Position Limit of product
    BAUDRATE                    = 1000000
    PROTOCOL_VERSION            = 2.0

    DXL_ID_list = [11, 12, 13]
    DEVICE_NUM = len(DXL_ID_list)
    DEVICENAME = '/dev/ttyUSB1'


    TORQUE_ENABLE               = 1     # Value for enabling the torque
    TORQUE_DISABLE              = 0     # Value for disabling the torque
    DXL_MOVING_STATUS_THRESHOLD = 10    # Dynamixel moving status threshold

    index = 0
    # Goal position
    dxl_goal_position = [
                        [angle_to_pos(180), angle_to_pos(180)],
                        [angle_to_pos(180 - 20), angle_to_pos(180+20)],
                        [angle_to_pos(180), angle_to_pos(180)]]


    portHandler_list = [PortHandler(DEVICENAME) for _ in range(DEVICE_NUM)]
    packetHandler_list = [PacketHandler(PROTOCOL_VERSION) for _ in range(DEVICE_NUM)]

    for i in range(0, DEVICE_NUM):
        open_port_and_baud(portHandler_list[i])
        enable_torque(packetHandler_list[i], portHandler_list[i], DXL_ID_list[i])

    while True:
        # print("Press any key to continue! (or press ESC to quit!)")
        if getch() == chr(0x1b):
            break

        # Write goal position
        for i in range(0, DEVICE_NUM):
            dxl_comm_result, dxl_error = packetHandler_list[i].write4ByteTxRx(portHandler_list[i], 
                                                                    DXL_ID_list[i], ADDR_GOAL_POSITION, dxl_goal_position[i][index])
        
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler_list[0].getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler_list[0].getRxPacketError(dxl_error))

        # read goal and current pos
        break_bool = False
        while True:
            for i in range(0, DEVICE_NUM):
                dxl_present_position, dxl_comm_result, dxl_error = packetHandler_list[i].read4ByteTxRx(portHandler_list[i], DXL_ID_list[i], ADDR_PRESENT_POSITION)

                if dxl_comm_result != COMM_SUCCESS:
                    print("%s" % packetHandler_list[i].getTxRxResult(dxl_comm_result))
                elif dxl_error != 0:
                    print("%s" % packetHandler_list[i].getRxPacketError(dxl_error))

            # for i in range(0, DEVICE_NUM):

                print(f'id:{DXL_ID_list[i]}, GoalPos :{pos_to_angle(dxl_goal_position[i][index])}({dxl_goal_position[i][index]}) | PresPos:{pos_to_angle(dxl_present_position)}({dxl_present_position})')


                if not abs(dxl_goal_position[i][index] - dxl_present_position) > DXL_MOVING_STATUS_THRESHOLD:
                    break_bool = True
                    break
            print('=====================================')
            if break_bool:
                break

        if index == 0:
            index = 1
        else:
            index = 0


    # Disable Dynamixel Torque
    for i in range(0, DEVICE_NUM):
        dxl_comm_result, dxl_error = packetHandler_list[i].write1ByteTxRx(portHandler_list[i], DXL_ID_list[i], ADDR_TORQUE_ENABLE, TORQUE_DISABLE)


    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % pos_to_angle(packetHandler_list[0].getTxRxResult(dxl_comm_result)))
    elif dxl_error != 0:
        print("%s" % packetHandler_list[0].getRxPacketError(dxl_error))

    # Close port
    portHandler_list[0].closePort()