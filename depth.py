import rospy
from std_msgs.msg import Float32
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class DepthCalculator:
    def __init__(self):
        rospy.init_node('depth_calculator', anonymous=True)

        # Set up the publisher for the distance
        self.distance_pub = rospy.Publisher('/custom_node/distance', Float32, queue_size=10)

        # Set up the RealSense pipeline
        self.bridge = CvBridge()

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

            # Display the depth image with the square highlighted
            depth_image_with_square = depth_image.copy()
            cv2.rectangle(depth_image_with_square, (top_left_x, top_left_y), (bottom_right_x, bottom_right_y), (255, 255, 255), 2)
            cv2.imshow("Depth Image with Square", cv2.applyColorMap(cv2.convertScaleAbs(depth_image_with_square, alpha=0.03), cv2.COLORMAP_JET))
            cv2.waitKey(1)

        except Exception as e:
            rospy.logerr(f"Error processing depth image: {e}")

    def run(self):
        # Create a subscriber for the RealSense depth stream
        rospy.Subscriber('/camera/depth/image_rect_raw', Image, self.depth_image_callback)

        rospy.spin()

        # Close the OpenCV window when the node is shut down
        cv2.destroyAllWindows()

if __name__ == '__main__':
    depth_calculator = DepthCalculator()
    print("Depth Calculator Node Initialized")
    depth_calculator.run()
