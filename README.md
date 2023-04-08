# Deep Learning-based automated bird repelling laser system

automated laser system tracking bird using YOLOV8x

![pigeon_synced_highlight_gif](https://user-images.githubusercontent.com/117917498/230707315-8e3973ec-ad56-4bce-ad5b-18a61c6b8639.gif)



## Test Env.
---
The code is tested successfully at
- Python 3.8.10
- Linux 20.04 LTS
- YOLOv8
    - model: yolov8x.pt
### Hardware Requirements
- Dynamixel(XM430-W350-T) * 4
- U2D2
- RealSense D435
- Arduino Uno
- [Laser303](https://smartstore.naver.com/athlove1/products/8087044267?)
- 3d printed laser and D435 holder
    - stl file [download link](www.google.com)


## Requirements
---
### YOLOv8 Setting
- Install YOLOv8 on a machine by [ultralytics](https://github.com/ultralytics/ultralytics) git link

### Dynamixel Setting
- Install pip install dynamixel-sdk
 package by [ROBOTIS](https://github.com/ROBOTIS-GIT/DynamixelSDK)

```
pip install dynamixel-sdk
```

### Python Setting
- To run the python code, following pakages are necessary: pypcd, tqdm, scikit-learn, and tabulate

```
pip install serial
pip install pyrealsense2	
pip install cv2
```
## How to Run ERASOR
---
`bird_laser.py` : main code to run  
`get_current_pos.py` : to get current pose of each motors

## Results
---
영상상으로만 레이저로 쫓는 짤
오픈메니풀레이터가 쫓는 짤
