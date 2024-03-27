import rospy
import time
from std_msgs.msg import Float32
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
from PIL import Image as PILImage
from PIL import ImageDraw

class DepthCalculator:
    def __init__(self):
        rospy.init_node('depth_calculator', anonymous=True)

        # Set up the publisher for the distance
        self.distance_pub = rospy.Publisher('/custom_node/distance', Float32, queue_size=10)

        # Set up the RealSense pipeline
        self.bridge = CvBridge()

    def depth_image_callback(self, depth_image_msg):
        try:
            # Convert the ROS Image message to a NumPy array
            depth_image = np.frombuffer(depth_image_msg.data, dtype=np.uint16).reshape((depth_image_msg.height, depth_image_msg.width))

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
            time.sleep(0.5)
            # Convert NumPy array to PIL Image
            pil_img = PILImage.fromarray(depth_image)

            # Draw a rectangle on the image to highlight the square region
            draw = ImageDraw.Draw(pil_img)
            draw.rectangle([top_left_x, top_left_y, bottom_right_x, bottom_right_y], outline="white")

            # Display the depth image with the square highlighted
            #pil_img.show()

        except Exception as e:
            rospy.logerr(f"Error processing depth image: {e}")

    def run(self):
        # Create a subscriber for the RealSense depth stream
        rospy.Subscriber('/camera/depth/image_rect_raw', Image, self.depth_image_callback)

        rospy.spin()

if __name__ == '__main__':
    depth_calculator = DepthCalculator()
    print("Depth Calculator Node Initialized")
    depth_calculator.run()
