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
from std_msgs.msg import String

# Main function.
if __name__ == '__main__':
    # Initialize the node and name it.
    # rospy.init_node('turtlepub')
    # publicator = rospy.Publisher("turtle1/cmd_vel", String, queue_size=1)
    # rospy.init_node('talker', anonymous=True)


    publicator = rospy.Publisher("turtle1/cmd_vel", Twist, queue_size=1)
    rospy.init_node('talker', anonymous=True)
    # print "ANTES"
    # rospy.sleep(1)
    # print "Despues"

    tw = Twist()
    tw.linear.x = 1.5
    # publicator.publish("HOLA")
    publicator.publish(tw)
