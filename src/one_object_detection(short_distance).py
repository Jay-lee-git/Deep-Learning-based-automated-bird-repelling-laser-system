from ultralytics import YOLO
import cv2
import numpy as np
from PIL import Image
import timeit

# Load a model
#model = YOLO("yolov8n.yaml")  # build a new model from scratch
model = YOLO("yolov8n.pt")  # load a pretrained model (recommended for training)

# Use the model
#model.train(data="coco128.yaml", epochs=3)  # train the model
#metrics = model.val()  # evaluate model performance on the validation set

# variable pre_detect_position
detect_xyxy = []

# video
cap = cv2.VideoCapture('./data/test2.avi')

# cam
#cap = cv2.VideoCapture(0)

try:
    while True:
        ret, frame = cap.read()

        # FPS measurement
        start_t = timeit.default_timer()

        results = model(frame)[0]
        result_image = results.plot()  # predict on an image(frame)

        result_cls = results.boxes.cls.tolist()
        result_conf = results.boxes.conf.tolist()
        result_xyxy = results.boxes.xyxy.tolist()
        
        max_arg = 0
        count = 0

        for c in range(len(result_cls)):
            if int(result_cls[c]) == 14:

                print("----------1-------------")
                print(result_xyxy[c])

                if not detect_xyxy:
                    #max_arg = result_conf[c]
                    detect_xyxy = result_xyxy[c]

                print(detect_xyxy)
                print(result_xyxy[c])
                print(abs(round(detect_xyxy[0],2) - round(result_xyxy[c][0],2)), abs(round(detect_xyxy[1],2) - round(result_xyxy[c][1],2)))
                
                if abs(round(detect_xyxy[0],2) - round(result_xyxy[c][0],2)) < 40 and abs(round(detect_xyxy[1],2) - round(result_xyxy[c][1],2)) < 40: 
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
                    
                    count += 1
            
                    detect_xyxy = result_xyxy[c]
                    
                elif abs(round(detect_xyxy[0],2) - round(result_xyxy[c][0],2)) < 200 and abs(round(detect_xyxy[1],2) - round(result_xyxy[c][1],2)) < 200:
                    if c == len(result_cls)-1 and count == 0:
                        #print(c)
                        #cv2.rectangle(frame,
                        #              (int(result_xyxy[c][0]),int(result_xyxy[c][1])-100),
                        #              (int(result_xyxy[c][0]+100),int(result_xyxy[c][1])),
                        #              (0,0,0), -1)
                        #cv2.putText(frame,
                        #            str(model.names[int(result_cls[c])]) + str(result_conf[c]),
                        #            (int(result_xyxy[c][0]),int(result_xyxy[c][1]-20)), cv2.FONT_HERSHEY_SIMPLEX,
                        #            2, (255,255,255), 2, cv2.LINE_AA)
                        #cv2.rectangle(frame,
                        #              (int(result_xyxy[c][0]),int(result_xyxy[c][1])),
                        #              (int(result_xyxy[c][2]),int(result_xyxy[c][3])),
                        #              (0,255,0), 3)
                        detect_xyxy = []
        print("--------------------------")

        # 아래 code의 주석은 results를 그대로 사용하여 출력하고자 할 때 사용 
        #pil_image=Image.fromarray(result_image[:, :, ::-1])
#
        ##pil_image.show()
        #numpy_image=np.array(pil_image)
        #opencv_image=cv2.cvtColor(numpy_image, cv2.COLOR_RGB2BGR)

        # FPS measurement
        terminate_t = timeit.default_timer()

        #cv2.putText(opencv_image, str(int(1/(terminate_t - start_t))), (50,50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,0,0), 1, cv2.LINE_4)
        #cv2.imshow('test',opencv_image)

        cv2.putText(frame, str(int(1/(terminate_t - start_t))), (50,50), cv2.FONT_HERSHEY_SIMPLEX, 2, (255,0,0), 2, cv2.LINE_4)
        cv2.imshow('frame', frame)

        if cv2.waitKey(1) == 27:
            break

        # model export
        #success = model.export(format="onnx")  # export the model to ONNX format
finally:
    cap.release()