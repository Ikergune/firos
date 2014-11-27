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
from include.server.firosServer import FirosServer


from include.ros.topicHandler import TopicHandler, loadMsgHandlers

# Main function.
if __name__ == '__main__':
    # Initialize the node and name it.
    rospy.init_node('firos')

    launchSetup()

    print "\nStarting Firos..."
    print "---------------------------------\n"

    loadMsgHandlers()

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

    print "\nPress Ctrl+C to Exit\n"

    server.start()
