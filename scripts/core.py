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

current_path = os.path.dirname(os.path.abspath(__file__))
FIROS_CONF_PATH = [current_path + "/../config"]

# Main function.
if __name__ == '__main__':
    # Input Parsing
    parser = argparse.ArgumentParser()
    parser.add_argument('-P', action='store', dest='port', help='Set the Port of the Firos-Server')
    parser.add_argument('--conf', action='store', dest='conf_Fold', help='Set the config-Folder for Firos')
    parser.add_argument('--ros-port', action='store', dest='ros_port', help='Set the ROS-Port for Firos')
    parser.add_argument('--ros-node-name', action='store', dest='ros_node_name', help='Set the ROS-Node-Name')
    parser.add_argument('--loglevel', action='store', dest='loglevel', help='Set the LogLevel (INFO, WARNING, ERROR,  CRITICAL)')

                    
    # Get Input
    results = parser.parse_args()
    


    if results.conf_Fold is not None:
        current_path = os.getcwd()
        FIROS_CONF_PATH[0] = current_path + "/" + results.conf_Fold
        # TODO DL This re-assignment is currently not working!


    # Importing firos specific scripts
    from setup import launchSetup
    from include import constants as c

    from include import confManager
    from include.logger import Log
    from include.mapServer import MapServer
    from include.server.firosServer import FirosServer

    from include.ros.topicHandler import RosTopicHandler, loadMsgHandlers, createConnectionListeners
    from include.rcm import topicManager




    if results.port is not None and type(results.port) is int:
        c.MAP_SERVER_PORT = results.port
            
    if results.ros_port is not None and type(results.ros_port) is int:
        c.ROSBRIDGE_PORT = results.ros_port
    
    if results.ros_node_name is not None and type(results.ros_node_name) is str:
        c.ROS_NODE_NAME = results.ros_node_name

    if results.loglevel is not None and type(results.loglevel) is int:
        c.LOGLEVEL = results.loglevel


    Log("INFO", "Initializing ROS node: " + c.ROS_NODE_NAME)
    rospy.init_node(c.ROS_NODE_NAME)
    Log("INFO", "Initialized")





    try:
        server = FirosServer("0.0.0.0", c.MAP_SERVER_PORT)
    except Exception as ex:
        sys.stderr.write('CB_COMMUNICATION_FAILED')
        exit(1)
    else:
        def signal_handler(signal, frame):
            Log("INFO", ('\nExiting from the application'))
            RosTopicHandler.unregisterAll()
            topicManager.removeListeners()
            server.close()
            Log("INFO", ('\nExit'))
            sys.exit(0)
        signal.signal(signal.SIGINT, signal_handler)
        signal.signal(signal.SIGTERM, signal_handler)

        launchSetup()

        Log("INFO", "\nStarting Firos...")
        Log("INFO", "---------------------------------\n")

        loadMsgHandlers(confManager.getRobots(True, True))
        createConnectionListeners()
        topicManager.setListeners()

        MapServer.load()

        Log("INFO", "\nPress Ctrl+C to Exit\n")
        server.start()

