#!/usr/bin/env python

# MIT License
#
# Copyright (c) <2015> <Ikergune, Etxetar>
#
# Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files
# (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge,
# publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so,
# subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
# MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE
# FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION
# WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.


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
        port = SERVER_PORT
    server = FirosServer("0.0.0.0", port)

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