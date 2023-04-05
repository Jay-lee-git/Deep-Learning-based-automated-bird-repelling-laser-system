import os
from dynamixel_sdk import * # Uses Dynamixel SDK library
from ultralytics import YOLO
import cv2
import numpy as np
import pyrealsense2 as rs
from collections import deque


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

# ============================== openCV ================================
def get_camera_goal_pos(x1,y1, x2,y2,frame):
    frame_width, frame_height, _ = frame.shape
    frame_mid_point = (frame_height//2 + 53 , frame_width//2 - 58)
    target_mid_point = ((x1+x2)//2 , (y1+y2)//2)
    return frame_mid_point, target_mid_point

def get_predict_info(frame, model):
    predicted_results = model(frame)[0]

    # original predicted results image
    # result_image = predicted_results.plot()
    # get result
    clf_results = predicted_results.boxes.cls
    percentage_result = predicted_results.boxes.conf
    coordinate_result = predicted_results.boxes.xyxy

    # convert to list
    clf_results = clf_results.tolist()
    percentage_result = percentage_result.tolist()
    coordinate_result = coordinate_result.tolist()
    return clf_results, percentage_result, coordinate_result   

def get_realsense_frame(pipeline):
    frames = pipeline.wait_for_frames()
    color_frame = frames.get_color_frame()
    color_image = np.asanyarray(color_frame.get_data())
    return color_image

def draw_target(x1, y1, x2, y2, frame, frame_mid_point, target_mid_point):
    # Draw a point on the image
    cv2.circle(frame, frame_mid_point, radius=5, color=(0, 0, 255), thickness=-1)
    dx, dy = target_mid_point
    target_x = (frame_mid_point[0] + dx) 
    target_y = (frame_mid_point[1] + dy)
    cv2.circle(frame, (target_x, target_y), radius=5, color=(255, 0, 0), thickness=-1)

    # Draw a line on the image
    cv2.line(frame, frame_mid_point, (target_x, target_y), color=(0, 0, 255), thickness=2)

    # Draw a label on the image
    cv2.rectangle(frame, (x1, y1-100), (x1+100, y1), (0,0,0), -1)
    cv2.putText(frame, 'bird ' + str(round(percentage_result[label_index]*100, 2)) + '%',
                (x1, y1-20), cv2.FONT_HERSHEY_SIMPLEX, 2, (255,255,255), 2, cv2.LINE_AA)            
    cv2.rectangle(frame, (x1, y1), (x2, y2), (0,255,0), 3)

def realsense_config():
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    pipeline.start(config)
    return pipeline

def NOR(a, b): 
    if(a == False) and (b == False): 
        return False
    elif(a == False) and (b == True): 
        return True
    elif(a == True) and (b == False): 
        return True
    elif(a == True) and (b == True): 
        return False

import serial


if __name__ == '__main__':
    
    model = YOLO('./data/yolov8n.pt') # load a pretrained model (recommended for training)
    
    # cap = cv2.VideoCapture('./data/test3.avi')
    pipeline = realsense_config()

    mid_point_list = [deque([]) for _ in range(2)]
    step_size = 5

    py_serial = serial.Serial(port='/dev/ttyACM0', baudrate=9600)
    target_set_flag = False
    laser_status_flag = False
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

    TARGET_INDEX_CODE = 14.0
    DXL_ID_list = [11, 12, 13, 14]
    DEVICE_NUM = len(DXL_ID_list)
    DEVICENAME = '/dev/ttyUSB0'
    TORQUE_ENABLE               = 1     # Value for enabling the torque
    TORQUE_DISABLE              = 0     # Value for disabling the torque
    DXL_MOVING_STATUS_THRESHOLD = 60    # Dynamixel moving status threshold

    index = 0
    THRESHOLD = 10
    # Goal position
    dxl_goal_position = [angle_to_pos(180), angle_to_pos(100), angle_to_pos(250), angle_to_pos(180)]

    portHandler_list = [PortHandler(DEVICENAME) for _ in range(DEVICE_NUM)]
    packetHandler_list = [PacketHandler(PROTOCOL_VERSION) for _ in range(DEVICE_NUM)]

    for i in range(0, DEVICE_NUM):
        open_port_and_baud(portHandler_list[i])
        enable_torque(packetHandler_list[i], portHandler_list[i], DXL_ID_list[i])


    while True:
        # ret, frame = cap.read()
        frame = get_realsense_frame(pipeline)
        clf_results, percentage_result, coordinate_result  = get_predict_info(frame, model)
        max_arg = 0
        
        # Write goal position
        for i in range(0, DEVICE_NUM):
            dxl_comm_result, dxl_error = packetHandler_list[i].write4ByteTxRx(portHandler_list[i], 
                                                                    DXL_ID_list[i], ADDR_GOAL_POSITION, dxl_goal_position[i])
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler_list[0].getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler_list[0].getRxPacketError(dxl_error))

        if TARGET_INDEX_CODE in clf_results:    
            label_index = clf_results.index(TARGET_INDEX_CODE)
                    
            x1, y1, x2, y2 = map(int, coordinate_result[label_index])
            frame_mid_point, target_mid_point = get_camera_goal_pos(x1, y1, x2, y2, frame)
            
            dx = frame_mid_point[0] - target_mid_point[0]
            dy = frame_mid_point[1] - target_mid_point[1]
            

            mid_point_list[0].append(dx)
            mid_point_list[1].append(dy)
            
            # pop the first element
            if len(mid_point_list[0]) > step_size:
                mid_point_list[0].popleft()
                mid_point_list[1].popleft()
            target_mid_point = (sum(mid_point_list[0])//step_size*-1, sum(mid_point_list[1])//step_size*-1)
            # print(target_mid_point, frame_mid_point)
            fx, fy = (frame_mid_point[0] - target_mid_point[0], frame_mid_point[1] - target_mid_point[1])
            # print(fx, fy)

            if target_mid_point[0] < THRESHOLD and target_mid_point[1] < THRESHOLD:
                target_set_flag = True
            else:
                target_set_flag = False
            
            if angle_to_pos(180-100) < dxl_goal_position[0] < angle_to_pos(180+100):
                dxl_goal_position[0] += int(-1 * target_mid_point[0]* 0.156)
            else:
                dxl_goal_position[0] = angle_to_pos(179)
                
            if angle_to_pos(180-60) < dxl_goal_position[3] < angle_to_pos(180+60):
                dxl_goal_position[3] += int(target_mid_point[1] * 0.083)
            else:
                dxl_goal_position[3] = angle_to_pos(179)
                
            draw_target(x1, y1, x2, y2, frame, frame_mid_point, target_mid_point)
        else:
            target_set_flag = False
        key_input = cv2.waitKey(1)
    
        if key_input == 27:
            break
        elif key_input == ord('w'):
            dxl_goal_position[3] -= angle_to_pos(1)
        elif key_input == ord('s'):
            dxl_goal_position[3] += angle_to_pos(1)
        elif key_input == ord('a'):
            dxl_goal_position[0] += angle_to_pos(1)
        elif key_input == ord('d'):
            dxl_goal_position[0] -= angle_to_pos(1)
        elif key_input == ord('l'):
            # send l to arduino uno
            target_set_flag = not target_set_flag
        elif key_input == ord('p'):
            TARGET_INDEX_CODE = 14.0
        elif key_input == ord('h'):
            TARGET_INDEX_CODE = 0.0


        cv2.imshow('frame', frame)
        for i in range(0, DEVICE_NUM):
            dxl_present_position, dxl_comm_result, dxl_error = packetHandler_list[i].read4ByteTxRx(portHandler_list[i], DXL_ID_list[i], ADDR_PRESENT_POSITION)
            if dxl_comm_result != COMM_SUCCESS:     
                print("%s" % packetHandler_list[i].getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % packetHandler_list[i].getRxPacketError(dxl_error))
           
        if NOR(target_set_flag, laser_status_flag):
            py_serial.write(b'l')
            laser_status_flag = not laser_status_flag

    # Disable Dynamixel Torque
    for i in range(0, DEVICE_NUM):
        dxl_comm_result, dxl_error = packetHandler_list[i].write1ByteTxRx(portHandler_list[i], DXL_ID_list [i], ADDR_TORQUE_ENABLE, TORQUE_DISABLE)


    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % pos_to_angle(packetHandler_list[0].getTxRxResult(dxl_comm_result)))
    elif dxl_error != 0:
        print("%s" % packetHandler_list[0].getRxPacketError(dxl_error))

    # Close port
    portHandler_list[0].closePort()
    if laser_status_flag == True:
        py_serial.write(b'l')
    # cap.release()