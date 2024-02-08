import rospy
from std_msgs.msg import Float32
import csv
import signal
import sys

class AUVRepositioning:
    def __init__(self):
        rospy.init_node('auv_repositioning', anonymous=True)

        # Set up the subscriber for the depth information
        self.depth_sub = rospy.Subscriber('/custom_node/distance', Float32, self.depth_callback)

        # Set up the publisher for random distance_x
        self.distance_x_pub = rospy.Publisher('/custom_node/distance_x', Float32, queue_size=10)
        self.distance_y_pub = rospy.Publisher('/custom_node/distance_y', Float32, queue_size=10)

        # Set the publishing rate to 1 Hz
        self.rate = rospy.Rate(1)
        
        # Define the maximum offsets
        self.x_distance_to_left_side = 64.0
        self.x_distance_to_right_side = -62.6
        self.y_distance_to_top_side = 42.4
        self.y_distance_to_bottom_side = -35.7

        # Create and open the CSV file in write mode
        self.csv_file = open('auv_repositioning_data.csv', 'w')
        # Create a CSV writer object
        self.csv_writer = csv.writer(self.csv_file)
        # Write the header row
        self.csv_writer.writerow(['Depth (m)', 'Distance X (m)', 'Distance Y (m)'])

        # Set up signal handler to ensure proper cleanup
        signal.signal(signal.SIGINT, self.signal_handler)

    def depth_callback(self, depth_msg):
        try:
            # Get the depth value from the message
            depth = depth_msg.data

            # Assuming you have a simple mapping from depth to desired altitude (z-coordinate)
            # You might need a more sophisticated mapping based on your AUV's dynamics
            desired_altitude = depth

            # Calculate distances from the center
            distance_x = (self.x_distance_to_left_side + self.x_distance_to_right_side) / 2
            distance_y = (self.y_distance_to_top_side + self.y_distance_to_bottom_side) / 2

            # Determine the direction to move the camera (front, back, left, or right)
            x_direction = "right" if distance_x >= 0 else "left"
            y_direction = "forward" if distance_y >= 0 else "backward"

            # Print the information
            print(f"Move the camera {abs(distance_x):.2f} meters {x_direction} along the x-axis")
            print(f"Move the camera {abs(distance_y):.2f} meters {y_direction} along the y-axis")
            print(f"Updated Depth: {depth:.2f} meters")

            # Write the data into the CSV file
            self.csv_writer.writerow([depth, distance_x, distance_y])
            self.csv_file.flush()  # Flush the buffer to ensure data is written immediately

            # Publish the calculated distances
            self.publish_distance_x(distance_x)
            self.publish_distance_y(distance_y)

        except Exception as e:
            rospy.logerr(f"Error processing depth information: {e}")

    def publish_distance_x(self, distance_x):
        # Publish the calculated distance_x
        self.distance_x_pub.publish(Float32(distance_x))
    
    def publish_distance_y(self, distance_y):
        # Publish the calculated distance_y
        self.distance_y_pub.publish(Float32(distance_y))

    def run(self):
        while not rospy.is_shutdown():
            self.rate.sleep()

    def signal_handler(self, sig, frame):
        # Close the CSV file before exiting
        self.csv_file.close()
        rospy.loginfo("CSV file closed.")
        sys.exit(0)

if __name__ == '__main__':
    auv_repositioning = AUVRepositioning()
    print("AUV Repositioning Node Initialized")
    auv_repositioning.run()
