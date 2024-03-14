import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
import subprocess

class ImageSubscriber:
    def __init__(self):
        # Clear the folder at the beginning
        save_folder = '/home/lolbuoy/Desktop/yolov8/camera1'  # Update the default save folder path
        subprocess.call(['./clearboi.sh', save_folder])

        rospy.init_node('image_subscriber', anonymous=True)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber('/camera/color/image_raw', Image, self.image_callback)
        self.save_folder = save_folder
        if not os.path.exists(self.save_folder):
            os.makedirs(self.save_folder)
        self.image_count = 0
        self.save_rate = rospy.Rate(8)  # Set the save rate to match the image stream frequency

    def image_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            print(e)
            return

        # Display the color image using OpenCV
        #cv2.imshow("Color Image", cv_image)
        #cv2.waitKey(1)  # Keep the window open

        # Save every 5th image to the folder
        if self.image_count % 5 == 0:
            image_filename = os.path.join(self.save_folder, f"image_{self.image_count}.jpg")
            cv2.imwrite(image_filename, cv_image)
            print(f"Saved image {self.image_count} to {image_filename}")

        self.image_count += 1

def main():
    try:
        image_subscriber = ImageSubscriber()
        rospy.spin()  # Keep the node running
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
