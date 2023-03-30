from ultralytics import YOLO
import cv2
import numpy as np
from PIL import Image
import timeit

# Load a model
model = YOLO('./data/yolov8n.pt')  # load a pretrained model (recommended for training)
cap = cv2.VideoCapture('./data/test3.avi')

def camera_goal_pos(x1,y1, x2,y2,frame):
    frame_width, frame_height, _ = frame.shape
    frame_mid_point = (frame_height//2, frame_width//2)
    target_mid_point = ((x1+x2)//2 , (y1+y2)//2)
    # Draw a point on the image
    cv2.circle(frame, frame_mid_point, radius=5, color=(0, 0, 255), thickness=-1)
    cv2.circle(frame, target_mid_point, radius=5, color=(255, 0, 0), thickness=-1)
    cv2.line(frame, frame_mid_point, target_mid_point, color=(0, 0, 255), thickness=2)

    

try:
    while True:
        ret, frame = cap.read()
        
        print(frame.shape)
        # need to cal frame
        start_t = timeit.default_timer()
        results = model(frame)[0]
        result_image = results.plot()  # predict on an image


        # label : int
        result_cls = results.boxes.cls
        # percentage
        result_conf = results.boxes.conf
        # two coordinate from the detected frame
        result_xyxy = results.boxes.xyxy

        result_cls = result_cls.tolist()
        result_conf = result_conf.tolist()
        result_xyxy = result_xyxy.tolist()
        
        max_arg = 0

        for c in range(len(result_cls)):
            if int(result_cls[c]) == 14:
                
                if max_arg < result_conf[c]:
                    max_arg = result_conf[c]
                    camera_goal_pos(int(result_xyxy[c][0]),int(result_xyxy[c][1]),int(result_xyxy[c][2]),int(result_xyxy[c][3]),frame)
                
                    cv2.rectangle(frame,
                                  (int(result_xyxy[c][0]),int(result_xyxy[c][1])-100),
                                  (int(result_xyxy[c][0]+100),int(result_xyxy[c][1])),
                                  (0,0,0), -1)
                    
                    cv2.putText(frame,
                                str(model.names[int(result_cls[c])]) + str(round(result_conf[c]*100,2)) + '%',
                                (int(result_xyxy[c][0]),int(result_xyxy[c][1]-20)), cv2.FONT_HERSHEY_SIMPLEX,
                                2, (255,255,255), 2, cv2.LINE_AA)
                    
                    cv2.rectangle(frame,
                                  (int(result_xyxy[c][0]),int(result_xyxy[c][1])),
                                  (int(result_xyxy[c][2]),int(result_xyxy[c][3])),
                                  (0,255,0), 3)

        print("--------------------------")

        terminate_t = timeit.default_timer()

        cv2.putText(frame, str(int(1/(terminate_t - start_t))), (50,50), cv2.FONT_HERSHEY_SIMPLEX, 2, (255,0,0), 2, cv2.LINE_4)

        cv2.imshow('frame', frame)

        if cv2.waitKey(1) == 27:
            break


finally:
    cap.release()