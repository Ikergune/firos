#!/usr/bin/env python

# Import required Python code.
import os
import rospy

from include.constants import CONTEXTBROKER

PROXY_PATH = "/datos/bamboo_ros_ws/proxy/"

# Main function.
if __name__ == '__main__':
    rospy.init_node('proxy', anonymous=True)

    start_command = "node app.js " + CONTEXTBROKER["ADDRESS"] + " " + str(CONTEXTBROKER["PORT"]) + " " + CONTEXTBROKER["PROTOCOL"]
    os.system("cd " + PROXY_PATH + " && npm install && " + start_command)

