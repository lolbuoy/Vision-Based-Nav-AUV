import rospy
from std_msgs.msg import Float32

class AUVRepositioning:
    def __init__(self):
        rospy.init_node('auv_repositioning', anonymous=True)

        # Set up the subscriber for the depth information
        self.depth_sub = rospy.Subscriber('/custom_node/distance', Float32, self.depth_callback)

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

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    # Define the maximum offsets
    x_distance_to_left_side = 64.0
    x_distance_to_right_side = -62.6
    y_distance_to_top_side = 42.4
    y_distance_to_bottom_side = -35.7

    auv_repositioning = AUVRepositioning()
    print("AUV Repositioning Node Initialized")
    auv_repositioning.run()
