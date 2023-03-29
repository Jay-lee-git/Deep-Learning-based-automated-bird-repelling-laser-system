import cv2
import pyrealsense2 as rs
import sys
import numpy as np
import datetime

from datetime import datetime 

today = datetime.now()
today = today.strftime('%Y-%m-%d_%H-%M-%S')

pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
pipeline.start(config)

# Set up OpenCV video writer to save the recorded video
fourcc = cv2.VideoWriter_fourcc(*'XVID')
out = cv2.VideoWriter('output_'+ today + '.avi', fourcc, 30.0, (640, 480))

# try:
while True:
    # Wait for a new frame from the RealSense camera
    frames = pipeline.wait_for_frames()
    color_frame = frames.get_color_frame()
    depth_frame = frames.get_depth_frame()

    if not color_frame or not depth_frame:
        continue

    # Convert RealSense frame to OpenCV format
    color_image = np.asanyarray(color_frame.get_data())
    depth_image = np.asanyarray(depth_frame.get_data())

    # Write the color frame to the output video file
    out.write(color_image)

    # Display the color and depth frames (optional)
    cv2.imshow('Color frame', color_image)
    cv2.imshow('Depth frame', depth_image)
    if cv2.waitKey(1) == 27:
        break

out.release()
cv2.destroyAllWindows()