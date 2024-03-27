import rospy
from std_msgs.msg import Float32, UInt16
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import csv
import os
import datetime
import shutil
import cv2
import threading
import time
from ultralytics import YOLO

class DepthCalculator:
    def __init__(self):
        rospy.init_node('depth_calculator', anonymous=True)

        # Set up the publisher for the distance and pixel coordinates
        self.distance_pub = rospy.Publisher('/custom_node/distance', Float32, queue_size=10)
        self.pixel_x_pub = rospy.Publisher('/custom_node/pixel_x', UInt16, queue_size=10)
        self.pixel_y_pub = rospy.Publisher('/custom_node/pixel_y', UInt16, queue_size=10)

        # Set up the RealSense pipeline
        self.bridge = CvBridge()

        # Initialize YOLO model with the specified path
        yolo_model_path = "/home/vyana/codes/gates.pt"
        self.model = YOLO(yolo_model_path)

        # Create a new CSV file for logging data
        self.csv_file_path = self.get_unique_csv_filename()
        self.csv_file = open(self.csv_file_path, 'w')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(['Object Type', 'Center_X', 'Center_Y', 'Distance'])

        # Create a directory to store processed images
        self.output_dir = "/home/vyana/processed_images_gates"
        if not os.path.exists(self.output_dir):
            os.makedirs(self.output_dir)

        # Read source and project directories from currentFolder.txt
        self.read_folder_info()

        # Initialize threading lock
        self.lock = threading.Lock()

        # Create a thread for moving images
        self.move_thread = threading.Thread(target=self.move_images_thread)
        self.move_thread.daemon = True
        self.move_thread.start()

    def get_unique_csv_filename(self):
        # Generate a unique filename based on the current timestamp
        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"depth_data_{timestamp}.csv"
        return filename

    def read_folder_info(self):
        folder_info_file = "/home/vyana/Desktop/yolov8/currentFolder.txt"
        if os.path.exists(folder_info_file):
            with open(folder_info_file, 'r') as file:
                lines = file.readlines()
                if len(lines) >= 2:
                    source_folder = lines[0].split(": ")[1].strip()
                    project_folder = lines[1].split(": ")[1].strip()
                    base_folder = os.path.join(os.path.expanduser('~'), "Desktop", "yolov8")
                    self.source_folder = os.path.join(base_folder, source_folder)
                    self.project_folder = os.path.join(base_folder, project_folder)

    def depth_image_callback(self, depth_image_msg):
        try:
            # Convert the ROS Image message to a NumPy array
            depth_image_data = np.frombuffer(depth_image_msg.data, dtype=np.uint16)
            depth_image = depth_image_data.reshape((depth_image_msg.height, depth_image_msg.width))

            # Calculate the center pixel coordinates
            center_x, center_y = int(depth_image.shape[1] // 2), int(depth_image.shape[0] // 2)

            # Publish the pixel coordinates of the center
            self.pixel_x_pub.publish(UInt16(center_x))
            self.pixel_y_pub.publish(UInt16(center_y))

            # Extract the depth value at the center pixel
            depth_at_center = depth_image[center_y, center_x] * 0.001  # Convert to meters

            # Publish the depth value at the center
            self.distance_pub.publish(Float32(depth_at_center))

            if hasattr(self, 'model') and self.model is not None:
                # Perform object detection
                results = self.model.predict(source=self.source_folder, project= self.project_folder, name='test', exist_ok=True, save_crop=True, stream=True)

                for i, result in enumerate(results):
                    for j, box in enumerate(result.boxes):
                        class_name = result.names[box.cls[0].item()]
                        coords = box.xyxy[0].tolist()
                        coords = [round(x) for x in coords]
                        center_x = int((coords[0] + coords[2]) / 2)
                        center_y = int((coords[1] + coords[3]) / 2)

                        # Publish the center pixel coordinates of the detection
                        self.pixel_x_pub.publish(UInt16(center_x))
                        self.pixel_y_pub.publish(UInt16(center_y))

                        # Log coordinates and distance to CSV file
                        with self.lock:
                            self.csv_writer.writerow([class_name, center_x, center_y, depth_at_center])
                            # Log coordinates to the console
                            rospy.loginfo(f"Object type: {class_name}, Center_X: {center_x}, Center_Y: {center_y}")

                # Move the processed image to the output directory
                #timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S_%f")
                #image_filename = f"processed_image_{timestamp}.png"
                #cv2.imwrite(os.path.join(self.output_dir, image_filename), depth_image)

        except Exception as e:
            rospy.logerr(f"Error processing depth image: {e}")

    def move_images_thread(self):
        while not rospy.is_shutdown():
            try:
                # Move images from source to destination folder
                time.sleep(20)
                files = os.listdir(self.source_folder )
                files = sorted(files)
                for file in files:
                    if file.endswith('.jpg') or file.endswith('.png'):
                        time.sleep(0.75)
                        source_file_path = os.path.join(self.source_folder, file)
                        destination_file_path = os.path.join(self.output_dir, file)
                        shutil.move(source_file_path, destination_file_path)
                        rospy.loginfo(f"Moved image from {source_file_path} to {destination_file_path}")
            except Exception as e:
                rospy.logerr(f"Error moving images: {e}")

            # Sleep to achieve the desired frequency (1.2 Hz)

    def run(self):
        # Create a subscriber for the depth image stream
        rospy.Subscriber('/camera/depth/image_rect_raw', Image, self.depth_image_callback)

        rospy.spin()

        # Close the CSV file
        self.csv_file.close()

if __name__ == '__main__':
    depth_calculator = DepthCalculator()
    print("AI Node Initialized")
    depth_calculator.run()
