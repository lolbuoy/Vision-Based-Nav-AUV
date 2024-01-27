#!/usr/bin/env python

import rospy
from mavros_msgs.srv import CommandBool

def arm_mavros():
    rospy.init_node('arm_mavros_node', anonymous=True)

    # Wait for the service to become available
    rospy.wait_for_service('/mavros/cmd/arming')

    try:
        # Create a handle for the arming service
        arming_service = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)

        # Call the arming service to arm the vehicle
        response = arming_service(True)

        # Check the response
        if response.success:
            rospy.loginfo("MAVROS armed successfully!")
        else:
            rospy.logwarn("Failed to arm MAVROS. Check your setup.")

    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s" % e)

if __name__ == '__main__':
    try:
        arm_mavros()
    except rospy.ROSInterruptException:
        pass

