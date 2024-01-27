from pymavlink import mavutil
from std_msgs.msg import Float32
import rospy

# Global variable to store depth value
depth_value = 0

def depth_callback(msg):
    global depth_value
    depth_value = msg.data

def send_rc_override_scaled(value):
    # Define the RC channel index (assuming it's channel 1, change it accordingly if needed)
    RC_CHANNEL = 1

    # Connect to the autopilot (change the connection string as per your setup)
    master = mavutil.mavlink_connection('udpin:0.0.0.0:14550')

    # Scale the value from -2 to 2 to 1000 to 2000 PWM
    pwm_value = int((value + 2) * 500) + 1000

    # Clamp PWM value to 1000-2000 range
    pwm_value = max(1000, min(pwm_value, 2000))

    # Set PWM values for all channels
    # Channel 1 is overridden, other channels are set to 1500 PWM
    master.mav.rc_channels_override_send(
        master.target_system,
        master.target_component,
        pwm_value,   # Channel 1 override value
        1500,        # Channel 2 override value
        1500,        # Channel 3 override value
        1500,        # Channel 4 override value
        1500,        # Channel 5 override value
        1500,        # Channel 6 override value
        1500,        # Channel 7 override value
        1500         # Channel 8 override value
    )

    # Close the MAVLink connection
    master.close()

if __name__ == "__main__":
    try:
        # Initialize the ROS node
        rospy.init_node('set_depth_node', anonymous=True)

        # Subscribe to the depth topic
        rospy.Subscriber('/custom_node/z_pixel', Float32, depth_callback)

        # Example loop to continuously send RC override commands
        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            # Send RC override command with the scaled depth value
            send_rc_override_scaled(depth_value)
            rate.sleep()

    except rospy.ROSInterruptException:
        print("ROS node terminated")









'''import rospy
from std_msgs.msg import Float32
from pymavlink import mavutil
from geometry_msgs.msg import PoseStamped
import random

# Rest of the code remains the same


class AUVController:
    def __init__(self):
        rospy.init_node('auv_controller_node', anonymous=True)

        # Set the connection string based on your setup
        connection_string = 'tcp:127.0.0.1:5763'  # Adjust as needed
        self.master = mavutil.mavlink_connection(connection_string)
        print('Heartbeat Received:', self.master.wait_heartbeat())

        # Arm the motors
        self.master.arducopter_arm()
        print('Motors Armed:', self.master.motors_armed_wait())

        # Set depth to -2m
        self.set_depth_target(-2.0)
        print('Depth set to -2m')

        # Create a subscriber for the geometry_msgs/PoseStamped topic
        self.position_sub = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.position_callback)

        # Create a subscribe#r for the custom topic /custom_node/x_pixel
        self.x_pixel_sub = rospy.Subscriber('/custom_node/x_pixel', Float32, self.x_pixel_callback)
        #self.x_pixel_sub = rospy.Subscriber('/custom_node/x_pixel', Float32, self.x_pixel_callback)
        # Initialize target position
        self.target_x = 0.0  # Initialize with a default value

        # Initialize movement parameters
        self.square_side_length = 5.0  # Set the side length of the square in meters
        self.move_forward = True  # Start by moving forward

    def position_callback(self, pose_msg):
        # Extract position information from the PoseStamped message
        x_position = pose_msg.pose.position.x
        y_position = pose_msg.pose.position.y

        # Move in a square pattern (left and right)
        if self.move_forward:
            # Move forward
            self.target_x = x_position + self.square_side_length
        else:
            # Move backward
            self.target_x = x_position - self.square_side_length

        # Update position target message
        self.set_position_target(self.target_x, y_position)

        # Print a message when changing direction
        rospy.loginfo("Changing direction")

        # Sleep to introduce a delay between commands
        rospy.sleep(1.0)

        # Toggle the movement direction
        self.move_forward = not self.move_forward

    def x_pixel_callback(self, x_pixel_msg):
        print('Received x_pixel:', x_pixel_msg.data)
        # Update target_x based on the subscribed x_pixel value
        self.target_x = x_pixel_msg.data

    def set_position_target(self, x, y):
        # Create and send SET_POSITION_TARGET_LOCAL_NED message
        msg = self.master.mav.set_position_target_local_ned_encode(
            0,       # time_boot_ms
            0,       # target system
            0,       # target component
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,  # coordinate frame
            0b0000111111111000,  # type_mask
            x,       # x
            y,       # y
            0.0,     # z
            0.0,     # vx
            0.0,     # vy
            0.0,     # vz
            0.0,     # afx
            0.0,     # afy
            0.0,     # afz
            0.0,     # yaw
            0.0      # yaw_rate
        )
        self.master.mav.send(msg)

    def set_depth_target(self, depth):
        # Create and send SET_POSITION_TARGET_GLOBAL_INT message to set depth
        msg = self.master.mav.set_position_target_global_int_encode(
            0,       # time_boot_ms
            0,       # target system
            0,       # target component
            mavutil.mavlink.MAV_FRAME_GLOBAL_INT,  # coordinate frame
            0b0000111111111000,  # type_mask
            0,       # lat_int (not used)
            0,       # lon_int (not used)
            int(depth * 1e3),  # alt (depth in mm)
            0.0,     # vx
            0.0,     # vy
            0.0,     # vz
            0.0,     # afx
            0.0,     # afy
            0.0,     # afz
            0.0,     # yaw
            0.0      # yaw_rate
        )
        self.master.mav.send(msg)

if __name__ == '__main__':
    try:
        auv_controller = AUVController()
        rospy.spin()

    except rospy.ROSInterruptException:
        pass'''
