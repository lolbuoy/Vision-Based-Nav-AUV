import rospy
from std_msgs.msg import Float32, UInt16
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO
import csv
import os
import datetime

class DepthCalculator:
    def __init__(self):
        rospy.init_node('depth_calculator', anonymous=True)

        # Set up the publisher for the distance and pixel coordinates
        self.distance_pub = rospy.Publisher('/custom_node/distance', Float32, queue_size=10)
        self.pixel_x_pub = rospy.Publisher('/custom_node/pixel_x', UInt16, queue_size=10)
        self.pixel_y_pub = rospy.Publisher('/custom_node/pixel_y', UInt16, queue_size=10)

        # Set up the RealSense pipeline
        self.bridge = CvBridge()

        # Initialize YOLO model
        self.model = YOLO("/home/jetson/ai/ultralytics/best.pt")

        # Create a new CSV file for logging data
        self.csv_file_path = self.get_unique_csv_filename()
        self.csv_file = open(self.csv_file_path, 'w')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(['Object Type', 'Center_X', 'Center_Y', 'Distance'])

    def get_unique_csv_filename(self):
        # Generate a unique filename based on the current timestamp
        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"depth_data_{timestamp}.csv"
        return filename

    def depth_image_callback(self, depth_image_msg):
        try:
            # Convert the ROS Image message to a CV2 image
            depth_image = self.bridge.imgmsg_to_cv2(depth_image_msg, desired_encoding="passthrough")

            # Calculate the center pixel coordinates
            center_x, center_y = depth_image.shape[1] // 2, depth_image.shape[0] // 2

            # Publish the pixel coordinates of the center
            self.pixel_x_pub.publish(UInt16(center_x))
            self.pixel_y_pub.publish(UInt16(center_y))

            # Extract the depth value at the center pixel
            depth_at_center = depth_image[center_y, center_x] * 0.001  # Convert to meters

            # Publish the depth value at the center
            self.distance_pub.publish(Float32(depth_at_center))

            # Perform object detection
            results = self.model.predict(source='/home/jetson/Desktop/yolov8/camera', project='/home/jetson/Desktop/yolov8/new1', name='test1', exist_ok=True, save_crop=True, stream=True)

            for i, result in enumerate(results):
                for j, box in enumerate(result.boxes):
                    class_name = result.names[box.cls[0].item()]
                    coords = box.xyxy[0].tolist()
                    coords = [round(x) for x in coords]
                    center_x = (coords[0] + coords[2]) / 2
                    center_y = (coords[1] + coords[3]) / 2

                    # Log coordinates and distance to CSV file
                    self.csv_writer.writerow([class_name, center_x, center_y, depth_at_center])

                    # Log coordinates to the console
                    rospy.loginfo(f"Object type: {class_name}, Center_X: {center_x}, Center_Y: {center_y}")

        except Exception as e:
            rospy.logerr(f"Error processing depth image: {e}")

    def run(self):
        # Create a subscriber for the RealSense depth stream
        rospy.Subscriber('/camera/depth/image_rect_raw', Image, self.depth_image_callback)

        rospy.spin()

        # Close the CSV file
        self.csv_file.close()

if __name__ == '__main__':
    depth_calculator = DepthCalculator()
    print("Depth Calculator Node Initialized")
    depth_calculator.run()
