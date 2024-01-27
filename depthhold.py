#!/usr/bin/env python

import rospy
from mavros_msgs.srv import SetMode

def set_depth_hold_mode():
    rospy.init_node('set_depth_hold_mode_node', anonymous=True)

    # Wait for the service to become available
    rospy.wait_for_service('/mavros/set_mode')

    try:
        # Create a handle for the SetMode service
        set_mode_service = rospy.ServiceProxy('/mavros/set_mode', SetMode)

        # Call the SetMode service to set the flight mode to "depth_hold"
        response = set_mode_service(custom_mode='ALT_HOLD')

        # Check the response
        if response.mode_sent:
            rospy.loginfo("Flight mode set to depth_hold successfully!")
        else:
            rospy.logwarn("Failed to set flight mode to depth_hold. Check your setup.")

    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s" % e)

if __name__ == '__main__':
    try:
        set_depth_hold_mode()
    except rospy.ROSInterruptException:
        pass
