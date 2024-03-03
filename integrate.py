import rospy
from std_msgs.msg import Float32
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO
import csv

class DepthCalculator:
    def __init__(self):
        rospy.init_node('depth_calculator', anonymous=True)

        # Set up the publisher for the distance
        self.distance_pub = rospy.Publisher('/custom_node/distance', Float32, queue_size=10)

        # Set up the RealSense pipeline
        self.bridge = CvBridge()

        # Initialize YOLO model
        self.model = YOLO("/home/jetson/ai/ultralytics/best.pt")

        # Create a CSV file to log data
        self.csv_file_path = '/home/jetson/ai/depth_data.csv'
        self.csv_file = open(self.csv_file_path, 'w')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(['Object Type', 'Center_X', 'Center_Y', 'Distance'])

    def depth_image_callback(self, depth_image_msg):
        try:
            # Convert the ROS Image message to a CV2 image
            depth_image = self.bridge.imgmsg_to_cv2(depth_image_msg, desired_encoding="passthrough")

            # Define a square region around the center of the image
            center_x, center_y = depth_image.shape[1] // 2, depth_image.shape[0] // 2
            square_size = 100  # Adjust this value based on the size of the square you want
            top_left_x = center_x - square_size // 2
            top_left_y = center_y - square_size // 2
            bottom_right_x = center_x + square_size // 2
            bottom_right_y = center_y + square_size // 2

            # Extract the depth values within the square region
            depth_square = depth_image[top_left_y:bottom_right_y, top_left_x:bottom_right_x]

            # Calculate the average depth within the square
            average_depth = depth_square.mean() * 0.001  # Convert to meters

            # Publish the average depth within the square
            self.distance_pub.publish(Float32(average_depth))

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
                    self.csv_writer.writerow([class_name, center_x, center_y, average_depth])

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
