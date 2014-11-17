#!/usr/bin/env python

# Import required Python code.
import sys
import rospy
import signal

import json

from include.constants import *
from include.server import FirosServer
from include.contextbroker.cbSubscriber import *

# Main function.
if __name__ == '__main__':
    # Initialize the node and name it.
    rospy.init_node('firos')
    rospy.loginfo("Starting Firos...")

    if sys.argv[1:]:
        port = int(sys.argv[1])
    else:
        port = SERVER["PORT"]
    server = FirosServer(SERVER["ADDRESS"], port)

    def signal_handler(signal, frame):
        print('Exiting from the application')
        server.close()
        sys.exit(0)
    signal.signal(signal.SIGINT, signal_handler)
    print('Press Ctrl+C to Exit')

    sub = CbSubscriber()

    server.start()
