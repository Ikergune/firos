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
import signal

from include.libLoader import generateRosDependencies
# generateRosDependencies()

from setup import launchSetup

from include.constants import *
from include.server.firosServer import FirosServer


from include.ros.dependencies import generated
from include.ros.topicHandler import TopicHandler, loadMsgHandlers

# Main function.
if __name__ == '__main__':

    print "Initializing ROS node: " + NODE_NAME
    rospy.init_node(NODE_NAME)
    print "Initialized"

    if sys.argv[1:]:
        port = int(sys.argv[1])
    else:
        port = SERVER["PORT"]
    server = FirosServer(SERVER["ADDRESS"], port)

    def signal_handler(signal, frame):
        print('\nExiting from the application')
        TopicHandler.unregisterAll()
        server.close()
        print('\nExit')
        sys.exit(0)
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    launchSetup()

    print "\nStarting Firos..."
    print "---------------------------------\n"

    loadMsgHandlers()

    # Initialize the node and name it.
    # print "Initializing ROS node: " + NODE_NAME
    # rospy.init_node(NODE_NAME)
    # print "Initialized"

    print "\nPress Ctrl+C to Exit\n"
    server.start()
