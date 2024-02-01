import rospy
from std_msgs.msg import Float32
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from inf3_center import center_x

class DepthCalculator:
    def __init__(self):
        rospy.init_node('depth_calculator', anonymous=True)

        # Set up the publisher for the distance
        self.distance_pub = rospy.Publisher('/custom_node/distance', Float32, queue_size=10)

        # Set up the RealSense pipeline
        self.bridge = CvBridge()

        # Placeholder for center_x, replace with the actual value or logic
        self.center_x_value = 320  # Replace with the correct value

    def depth_image_callback(self, depth_image_msg):
        try:
            # Convert the ROS Image message to a CV2 image
            depth_image = self.bridge.imgmsg_to_cv2(depth_image_msg, desired_encoding="passthrough")

            # Use the center_x value obtained from inf3_center.py for depth calculation
            center_x_value = self.center_x_value  # Use the stored center_x_value

            # Extract the depth value at the specified x-coordinate
            depth_at_x = depth_image[center_x_value, :]

            # Calculate the average depth along the specified x-coordinate
            average_depth = depth_at_x.mean() * 0.001  # Convert to meters

            # Publish the average depth at the specified x-coordinate
            self.distance_pub.publish(Float32(average_depth))

            # Display the depth image with the specified x-coordinate highlighted
            depth_image_with_line = depth_image.copy()
            cv2.line(depth_image_with_line, (center_x_value, 0), (center_x_value, depth_image.shape[0]), (255, 255, 255), 2)
            cv2.imshow("Depth Image with Line", cv2.applyColorMap(cv2.convertScaleAbs(depth_image_with_line, alpha=0.03), cv2.COLORMAP_JET))
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
