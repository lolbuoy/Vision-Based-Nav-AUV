#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32

class AUVRepositioning:
    def __init__(self):
        rospy.init_node('auv_repositioning', anonymous=True)
        print("AUV Repositioning node initialized")  # Added print statement

        # Set up the subscriber for the depth information
        self.depth_sub = rospy.Subscriber('/custom_node/z_pixel', Float32, self.depth_callback)

        # Define the maximum depth (altitude) limit
        self.max_depth_limit = 8.0  # Set the limit to 2 meters
        
        # Flag to control the movement
        self.is_moving = True

    def depth_callback(self, depth_msg):
        try:
            # Get the depth value from the message
            depth = depth_msg.data

            # Print the depth value for reference
            rospy.loginfo(f"Received depth value: {depth}")

            # Check if the received depth value is zero
            if depth == 0:
                rospy.loginfo("Depth value is zero. Stopping movement.")
                self.is_moving = False  # Stop the movement
            else:
                # If the depth value is non-zero, resume the movement
                if not self.is_moving:
                    rospy.loginfo("Depth value is non-zero. Resuming movement.")
                    self.is_moving = True

                # Assuming you have a simple mapping from depth to desired altitude (z-coordinate)
                # You might need a more sophisticated mapping based on your AUV's dynamics
                desired_altitude = depth

                # Calculate the desired position based on maximum offsets
                y_offset = max(min(desired_altitude, y_distance_to_forward), y_distance_to_backward)

                # Determine the direction to move the AUV (forward or backward)
                y_direction = "forward" if y_offset >= 0 else "backward"

                # Print the information or use it to control the AUV
                rospy.loginfo(f"Move the AUV {abs(y_offset):.2f} meters {y_direction} in the y-axis direction")
                rospy.loginfo(f"Updated Depth: {depth:.3f} meters")

                # Check if the desired altitude exceeds the maximum depth limit
                if desired_altitude >= self.max_depth_limit:
                    rospy.loginfo("Maximum depth limit reached. Stopping movement.")
                    # You might add additional logic here to halt the AUV's movement

        except Exception as e:
            rospy.logerr(f"Error processing depth information: {e}")

# Define the maximum offsets for the y-axis
y_distance_to_forward = 64.0
y_distance_to_backward = -62.6

if __name__ == "__main__":
    try:
        auv_repositioning = AUVRepositioning()
        rospy.spin()  # Start ROS node

    except rospy.ROSInterruptException:
        pass


'''import rospy
from std_msgs.msg import Float32

class AUVRepositioning:
    def __init__(self):
        rospy.init_node('auv_repositioning', anonymous=True)
        print("AUV Repositioning node initialized")  # Added print statement

        # Set up the subscriber for the depth information
        self.depth_sub = rospy.Subscriber('/custom_node/z_pixel', Float32, self.depth_callback)

        # Define the maximum depth (altitude) limit
        self.max_depth_limit = 4.0  # Set the limit to 2 meters

    def depth_callback(self, depth_msg):
        try:
            # Get the depth value from the message
            depth = depth_msg.data

            # Assuming you have a simple mapping from depth to desired altitude (z-coordinate)
            # You might need a more sophisticated mapping based on your AUV's dynamics
            desired_altitude = depth

            # Calculate the desired position based on maximum offsets
            y_offset = max(min(desired_altitude, y_distance_to_forward), y_distance_to_backward)

            # Determine the direction to move the AUV (forward or backward)
            y_direction = "forward" if y_offset >= 0 else "backward"

            # Print the information or use it to control the AUV
            rospy.loginfo(f"Move the AUV {abs(y_offset):.2f} meters {y_direction} in the y-axis direction")
            rospy.loginfo(f"Updated Depth: {depth:.3f} meters")

            # Check if the desired altitude exceeds the maximum depth limit
            if desired_altitude >= self.max_depth_limit:
                rospy.loginfo("Maximum depth limit reached. Stopping movement.")
                # You might add additional logic here to halt the AUV's movement

            # Introduce a delay to slow down the process
            rospy.sleep(1)  # 1 second delay

        except Exception as e:
            rospy.logerr(f"Error processing depth information: {e}")

# Define the maximum offsets for the y-axis
y_distance_to_forward = 64.0
y_distance_to_backward = -62.6

if __name__ == "__main__":
    try:
        auv_repositioning = AUVRepositioning()
        rospy.spin()  # Start ROS node

    except rospy.ROSInterruptException:
        pass
'''









'''import rospy
from std_msgs.msg import Float32
import random

class AUVRepositioning:
    def __init__(self):
        rospy.init_node('auv_repositioning', anonymous=True)

        # Set up the subscriber for the depth information
        self.depth_sub = rospy.Subscriber('/custom_node/z_pixel', Float32, self.depth_callback)

        # Set up the publisher for random distance_x
        # self.distance_x_pub = rospy.Publisher('/custom_node/distance_x', Float32, queue_size=10)

        # List to store random values
        self.random_values_list = []

        # Set up a timer to generate random distance_x values at a rate of 2 Hz
        rospy.Timer(rospy.Duration(0.5), self.publish_random_distance_x)

    def depth_callback(self, depth_msg):
        try:
            # Get the depth value from the message
            depth = depth_msg.data

            # Assuming you have a simple mapping from depth to desired altitude (z-coordinate)
            # You might need a more sophisticated mapping based on your AUV's dynamics
            desired_altitude = depth

            # Map the desired position based on maximum offsets
            x_offset = max(min(desired_altitude, x_distance_to_left_side), x_distance_to_right_side)

            # Determine the direction to move the camera (left or right)
            x_direction = "right" if x_offset >= 0 else "left"

            # Print the information
            print(f"Move the camera {abs(x_offset):.2f} meters {x_direction} in the x-direction")
            print(f"Updated Distance: {depth:.2f} meters")

        except Exception as e:
            rospy.logerr(f"Error processing depth information: {e}")

    def publish_random_distance_x(self, event=None):
        # Generate a random distance_x in the range of 200 to 600
        random_distance_x = random.uniform(200, 600)

        # Append the random value to the list
        self.random_values_list.append(random_distance_x)
        
        # Publish the random distance_x
        # self.distance_x_pub.publish(Float32(random_distance_x))

    def get_random_values_list(self):
        # Method to retrieve the list of random values
        return self.random_values_list

    def run(self):
        rospy.spin()
    

# Define the maximum offsets
x_distance_to_left_side = 64.0
x_distance_to_right_side = -62.6
y_distance_to_top_side = 42.4
y_distance_to_bottom_side = -35.7

# Define the node name
node_name = 'auv_repositioning'

auv_repositioning = AUVRepositioning()
print(f"{node_name} Initialized")
auv_repositioning.run()'''
