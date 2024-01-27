#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32
import random

def publish_random_data():
    # Initialize the ROS node
    rospy.init_node('random_data_publisher', anonymous=True)

    # Create a publisher for the /custom_node/x_pixel topic
    pub = rospy.Publisher('/custom_node/z_pixel', Float32, queue_size=10)

    # Set the publishing rate to 3 Hz
    rate = rospy.Rate(0.3)

    while not rospy.is_shutdown():  # Continue looping until shutdown signal is received
        # Start from 4 and decrement by 0.5 until reaching 0
        for value in reversed(range(9)):
            random_value = value * 0.5  # Decrement by 0.5

            # Create a Float32 message and publish the random value
            msg = Float32(data=random_value)
            pub.publish(msg)

            # Print the published value for reference
            rospy.loginfo(f"Published z random value: {random_value}")

            # Sleep to maintain the publishing rate
            rate.sleep()

            # Check if the value reaches 0, and if so, break out of the loop
            if random_value == 0:
                break

if __name__ == '__main__':
    try:
        publish_random_data()
    except rospy.ROSInterruptException:
        pass




'''#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32
import random

def publish_random_data():
    # Initialize the ROS node
    rospy.init_node('random_data_publisher', anonymous=True)

    # Create a publisher for the /custom_node/x_pixel topic
    pub = rospy.Publisher('/custom_node/z_pixel', Float32, queue_size=10)

    # Set the publishing rate to 3 Hz
    rate = rospy.Rate(3)

    while not rospy.is_shutdown():
        # Generate a random float value between -2 and 2
        random_value = random.uniform(0,2)

        # Create a Float32 message and publish the random value
        msg = Float32(data=random_value)
        pub.publish(msg)

        # Print the published value for reference
        rospy.loginfo(f"Published z random value: {random_value}")

        # Sleep to maintain the publishing rate
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_random_data()
    except rospy.ROSInterruptException:
        pass'''
