import rospy
from std_msgs.msg import Float32
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from inf3_center import center_x  # Import center_x from inf3_center module

class DepthCalculator:
    def __init__(self, center_x):
        rospy.init_node('depth_calculator', anonymous=True)

        # Set up the publisher for the distance
        self.distance_pub = rospy.Publisher('/custom_node/distance', Float32, queue_size=10)

        # Set up the RealSense pipeline
        self.bridge = CvBridge()

        # Store the center_x value
        self.center_x = center_x

    def depth_image_callback(self, depth_image_msg):
        try:
            # Convert the ROS Image message to a CV2 image
            depth_image = self.bridge.imgmsg_to_cv2(depth_image_msg, desired_encoding="passthrough")

            # Get the depth value at the specified center_x
            depth_at_center_mm = depth_image[depth_image.shape[0] // 2, self.center_x]

            # Convert the depth value to meters
            depth_at_center_m = depth_at_center_mm * 0.001

            # Publish the depth at the specified center_x
            self.distance_pub.publish(Float32(depth_at_center_m))

            # Display the depth image with the specified center_x highlighted
            depth_image_with_center = depth_image.copy()
            cv2.circle(depth_image_with_center, (self.center_x, depth_image.shape[0] // 2), 5, (255, 255, 255), -1)  # Highlight specified center_x
            cv2.imshow("Depth Image with Specified Center_x", cv2.applyColorMap(cv2.convertScaleAbs(depth_image_with_center, alpha=0.03), cv2.COLORMAP_JET))
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
    # Use the center_x value from inf3_center module
    from inf3_center import center_x
    depth_calculator = DepthCalculator(center_x)
    print("Depth Calculator Node Initialized")
    depth_calculator.run()
