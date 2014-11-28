#!/usr/bin/env python


# ROSPY LOGS
# logdebug
# logerr
# logfatal
# loginfo
# logout
# logwarn

# Import required Python code.
import os
import sys
import rospy

from geometry_msgs.msg import Twist

# Main function.
if __name__ == '__main__':
    # Initialize the node and name it.
    rospy.init_node('turtlepub')
    publicator = rospy.Publisher("/turtle1/cmd_vel", Twist, queue_size=10)
    tw = Twist()
    tw.linear.x = 5.5
    publicator.publish(tw)

