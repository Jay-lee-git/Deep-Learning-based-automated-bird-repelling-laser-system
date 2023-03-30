from ultralytics import YOLO
import cv2
import numpy as np
import timeit


def camera_goal_pos(x1,y1, x2,y2,frame):
    frame_width, frame_height, _ = frame.shape
    frame_mid_point = (frame_height//2, frame_width//2)
    target_mid_point = ((x1+x2)//2 , (y1+y2)//2)

    # Draw a point on the image
    cv2.circle(frame, frame_mid_point, radius=5, color=(0, 0, 255), thickness=-1)
    cv2.circle(frame, target_mid_point, radius=5, color=(255, 0, 0), thickness=-1)

    # Draw a line on the image
    cv2.line(frame, frame_mid_point, target_mid_point, color=(0, 0, 255), thickness=2)



if __name__ == '__main__':
    
    # Load a model
    model = YOLO('./data/yolov8n.pt')  # load a pretrained model (recommended for training)
    cap = cv2.VideoCapture('./data/test3.avi')
    try:
        while True:
            ret, frame = cap.read()
            
            # FPS measurement
            start_time = timeit.default_timer()

            predicted_results = model(frame)[0]
            # original predicted results image
            result_image = predicted_results.plot()

            # get result
            clf_results = predicted_results.boxes.cls
            percentage_result = predicted_results.boxes.conf
            result_coordinate = predicted_results.boxes.xyxy

            # convert to list
            clf_results = clf_results.tolist()
            percentage_result = percentage_result.tolist()
            result_coordinate = result_coordinate.tolist()
            
            max_arg = 0
            for c in range(len(clf_results)):
                x1, y1, x2, y2 = map(int, result_coordinate[c])
                if int(clf_results[c]) == 14:
                    
                    if max_arg < percentage_result[c]:
                        max_arg = percentage_result[c]
                        camera_goal_pos(x1, y1, x2, y2,frame)
                    
                        cv2.rectangle(frame, (x1, y1-100), (x1+100, y1), (0,0,0), -1)
                        
                        cv2.putText(frame, 'bird' + str(round(percentage_result[c]*100, 2)) + '%',
                                    (x1, y1-20), cv2.FONT_HERSHEY_SIMPLEX, 2, (255,255,255), 2, cv2.LINE_AA)
                                    
                        cv2.rectangle(frame, (x1, y1), (x2, y2), (0,255,0), 3)


            print("--------------------------")

            terminate_time = timeit.default_timer()

            # FPS
            cv2.putText(frame, str(int(1/(terminate_time - start_time))), (50,50), cv2.FONT_HERSHEY_SIMPLEX, 2, (255,0,0), 2, cv2.LINE_4)

            cv2.imshow('frame', frame)

            if cv2.waitKey(1) == 27:
                break

    finally:
        cap.release()