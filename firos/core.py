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
import json
import os
import sys
import copy
import rospy
import signal
import argparse

from include.constants import Constants as C

# Main function.
if __name__ == '__main__':

    # If launched via roslaunch, we get addtional parameters
    rosLaunch_name = None
    for i in range(len(sys.argv)):
        if "__name:=" in sys.argv[i]:
            rosLaunch_name = sys.argv.pop(i)[8:]
            break
    for i in range(len(sys.argv)):
        if "__log:=" in sys.argv[i]:
            # We can omit this
            sys.argv.pop(i)[7:]
            break


    # Input Parsing
    parser = argparse.ArgumentParser()
    parser.add_argument('-P', action='store', dest='port', help='Set the Port of the Firos-Server')
    parser.add_argument('--conf', action='store', dest='conf_Fold', help='Set the config-Folder for Firos')
    parser.add_argument('--ros-port', action='store', dest='ros_port', help='Set the ROS-Port for Firos')
    parser.add_argument('--ros-node-name', action='store', dest='ros_node_name', help='Set the ROS-Node-Name')
    parser.add_argument('--loglevel', action='store', dest='loglevel', help='Set the LogLevel (INFO, WARNING, ERROR,  CRITICAL)')

                    
    # Get Input
    results = parser.parse_args()
    
    # At first determine the config-Folder location (either in firos/config or customly set)
    current_path = os.path.dirname(os.path.abspath(__file__))
    conf_path = current_path + "/../config"
    if results.conf_Fold is not None:
        current_path = os.getcwd()
        conf_path= current_path + "/" + results.conf_Fold



    # Initialize global variables (Constants.py)
    C.init(conf_path)


    # Importing firos specific scripts
    from include import confManager
    from include.logger import Log, initLog
    from include.server.firosServer import FirosServer
    
    from include.ros.topicHandler import RosTopicHandler, loadMsgHandlers, createConnectionListeners, initPubAndSub

    # Overwrite global variables with command line arguments (iff set)
    if results.port is not None:
        C.MAP_SERVER_PORT = int(results.port)
            
    if results.ros_port is not None:
        C.ROSBRIDGE_PORT = int(results.ros_port)
    
    if rosLaunch_name is not None:
        C.ROS_NODE_NAME = rosLaunch_name
    elif results.ros_node_name is not None:
        C.ROS_NODE_NAME = results.ros_node_name

    if results.loglevel is not None:
        C.LOGLEVEL = results.loglevel

    
    # Starting Up!
    initLog()
    Log("INFO", "Initializing ROS node: " + C.ROS_NODE_NAME)
    rospy.init_node(C.ROS_NODE_NAME)
    Log("INFO", "Initialized")



    try:
        server = FirosServer("0.0.0.0", C.MAP_SERVER_PORT)
    except Exception as ex:
        raise Exception("Unable to create a FirosServer")
    else:
        def signal_handler(signal, frame):
            Log("INFO", ('\nExiting from the application'))
            RosTopicHandler.unregisterAll()
            server.close()
            Log("INFO", ('\nExit'))
            sys.exit(0)
        signal.signal(signal.SIGINT, signal_handler)
        signal.signal(signal.SIGTERM, signal_handler)

        Log("INFO", "\nStarting Firos...")
        Log("INFO", "---------------------------------\n")

        # Topic Handler Routine:
        initPubAndSub()
        loadMsgHandlers(confManager.getRobots(True))
        createConnectionListeners()

        Log("INFO", "\nPress Ctrl+C to Exit\n")
        server.start()

