import pyrealsense2 as rs
import numpy as np
import cv2
import os
from datetime import datetime

# Initialize the pipeline for RealSense
pipeline = rs.pipeline()	
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

# Start the image stream
pipeline.start(config)




# Modify the path to create the 'camera' folder in a specific directory
output_folder = os.path.join(os.path.expanduser("~"), "Desktop", "yolov8", "camera")
os.makedirs(output_folder, exist_ok=True)


try:
    frame_number = 0
    while True:
        # Wait for the next frame
        frames = pipeline.wait_for_frames()
        
        # Get color frames
        color_frame = frames.get_color_frame()
        
        # Convert the color frame to a numpy array
        color_image = np.asanyarray(color_frame.get_data())
        
        # Display the image
        cv2.imshow('RealSense Camera', color_image)
        
        # Save the image to the 'images' folder
        timestamp = datetime.now().strftime('%Y%m%d%H%M%S')
        filename = os.path.join(output_folder, f'image_{timestamp}_{frame_number}.png')
        cv2.imwrite(filename, color_image)
        
        frame_number += 1
        
        # Break the loop by pressing 'q'
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
finally:
    # Stop Streaming
    pipeline.stop()
    cv2.destroyAllWindows()
