#!/usr/bin/env python


# ROSPY LOGS
# logdebug
# logerr
# logfatal
# loginfo
# logout
# logwarn

# Import required Python code.
import sys
import rospy
import signal

from setup import launchSetup

from include.constants import *

from include import confManager
from include.logger import Log
from include.server.firosServer import FirosServer

from include.ros.topicHandler import TopicHandler, loadMsgHandlers, connectionListeners

# Main function.
if __name__ == '__main__':

    Log("INFO", "Initializing ROS node: " + NODE_NAME)
    rospy.init_node(NODE_NAME)
    Log("INFO", "Initialized")

    if sys.argv[1:]:
        port = int(sys.argv[1])
    else:
        port = SERVER["PORT"]
    server = FirosServer(SERVER["ADDRESS"], port)

    def signal_handler(signal, frame):
        Log("INFO",('\nExiting from the application'))
        TopicHandler.unregisterAll()
        server.close()
        Log("INFO",('\nExit'))
        sys.exit(0)
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    launchSetup()

    Log("INFO", "\nStarting Firos...")
    Log("INFO", "---------------------------------\n")

    loadMsgHandlers(confManager.getRobots(True, True))
    connectionListeners()

    # Initialize the node and name it.
    # Log("INFO", "Initializing ROS node: " + NODE_NAME)
    # rospy.init_node(NODE_NAME)
    # Log("INFO", "Initialized")

    Log("INFO", "\nPress Ctrl+C to Exit\n")
    server.start()