import time
import rospy
from sensor_msgs.msg import Image
import numpy as np
from PIL import Image as PILImage
import os
from datetime import datetime

# Global variable to track the image count
image_count = 0

def create_folders():
    try:
        # Get the current date and time
        current_date = datetime.now().strftime("%Y-%m-%d")
        current_time = datetime.now().strftime("%H-%M-%S")

        # Specify the directory path
        SAVE_FOLDER = os.path.join(os.path.expanduser('~'), "Desktop", "yolov8")

        # Construct the folder names
        camera_folder_name = f"camera_pipes_{current_time}"
        results_folder_name = f"results_{current_date}_{current_time}"

        # Create the camera folder
        camera_folder_path = os.path.join(SAVE_FOLDER, camera_folder_name)
        os.makedirs(camera_folder_path)

        # Create the results folder
        results_folder_path = os.path.join(SAVE_FOLDER, results_folder_name)
        os.makedirs(results_folder_path)

        # Write folder names to a text file
        with open(os.path.join(SAVE_FOLDER, "currentFolder.txt"), "w") as file:
            file.write(f"Camera folder: {camera_folder_name}\n")
            file.write(f"Results folder: {results_folder_name}")

        return camera_folder_path

    except Exception as e:
        print(f"An error occurred while creating folders: {e}")

def main():
    global image_count  # Declare image_count as global
    image_count = 0  # Initialize image_count to 0
    
    # Initialize ROS node
    rospy.init_node('image_receiver', anonymous=True)

    # Create folders only once and get camera folder path
    camera_folder_path = create_folders()

    # Set the desired frequency (12Hz)
    rate = rospy.Rate(5)

    # Callback function to handle incoming image messages
    def image_callback(msg):
        global image_count  # Use the global image_count variable
        
        try:
            # Convert ROS Image message to NumPy array
            img_data = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, -1))

            # Convert NumPy array to PIL Image
            pil_img = PILImage.fromarray(img_data)

            # Get the current timestamp
            current_timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")

            # Increment the image count
            image_count += 1

            # Save the image to the camera folder with an incremented image count
            image_filename = os.path.join(camera_folder_path, f"image_{current_timestamp}_{image_count}.jpg")
            pil_img.save(image_filename)
            print(f"Saved image to {image_filename}")
            time.sleep(0.15)
        except Exception as e:
            print(e)

    # Subscribe to the image topic
    image_sub = rospy.Subscriber('/camera/color/image_raw', Image, image_callback)

    # Keep running until the node is shutdown
    while not rospy.is_shutdown():
        # Sleep to maintain the desired frequency
        time.sleep(0.15)

if __name__ == '__main__':
    print("IMAGE CAPTURE STARTED")
    main()
