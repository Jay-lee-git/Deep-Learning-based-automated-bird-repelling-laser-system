from ultralytics import YOLO
import cv2
import numpy as np
import timeit
import pyrealsense2 as rs
import time

def get_camera_goal_pos(x1,y1, x2,y2,frame):
    frame_width, frame_height, _ = frame.shape
    frame_mid_point = (frame_height//2, frame_width//2)
    target_mid_point = ((x1+x2)//2 , (y1+y2)//2)
    return frame_mid_point, target_mid_point



def get_predict_info(frame, model):
    predicted_results = model(frame)[0]

    # original predicted results image
    result_image = predicted_results.plot()

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
    # cv2.imshow('Color frame', color_image)
    return color_image

def draw_target(x1, y1, x2, y2, frame, frame_mid_point, target_mid_point):
    # Draw a point on the image
    cv2.circle(frame, frame_mid_point, radius=5, color=(0, 0, 255), thickness=-1)
    cv2.circle(frame, target_mid_point, radius=5, color=(255, 0, 0), thickness=-1)

    # Draw a line on the image
    cv2.line(frame, frame_mid_point, target_mid_point, color=(0, 0, 255), thickness=2)

    # Draw a label on the image
    cv2.rectangle(frame, (x1, y1-100), (x1+100, y1), (0,0,0), -1)
    cv2.putText(frame, 'bird ' + str(round(percentage_result[label_index]*100, 2)) + '%',
                (x1, y1-20), cv2.FONT_HERSHEY_SIMPLEX, 2, (255,255,255), 2, cv2.LINE_AA)            
    cv2.rectangle(frame, (x1, y1), (x2, y2), (0,255,0), 3)



if __name__ == '__main__':
    # Load a model
    model = YOLO('./data/yolov8n.pt') # load a pretrained model (recommended for training)
    cap = cv2.VideoCapture('./data/test3.avi')
    
    # realsense config
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    pipeline.start(config)

    # time_flag = True
    start_time = time.time()

    while True:
        frame = get_realsense_frame(pipeline)
        # ret, frame = cap.read()

        clf_results, percentage_result, coordinate_result  = get_predict_info(frame, model)
        
        max_arg = 0
        
        for label_index in range(len(clf_results)):
            x1, y1, x2, y2 = map(int, coordinate_result[label_index])
            if int(clf_results[label_index]) == 14:
                if max_arg < percentage_result[label_index]:
                    max_arg = percentage_result[label_index]
                    
                    current_time = time.time()
                    elapsed_time = current_time - start_time

                    if elapsed_time >= 2:  # if 2 seconds have passed
                        start_time = current_time
                        frame_mid_point, target_mid_point = get_camera_goal_pos(x1, y1, x2, y2, frame)
                    draw_target(x1, y1, x2, y2, frame, frame_mid_point, target_mid_point)


        print("--------------------------")


        cv2.imshow('frame', frame)

        if cv2.waitKey(1) == 27:
            break


    cap.release()