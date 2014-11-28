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

from std_msgs.msg import String
from geometry_msgs.msg import Twist
from include.ros.rosutils import *

# Main function.
if __name__ == '__main__':
    # Initialize the node and name it.
    rospy.init_node('firos')

    launchSetup()

    print "\nStarting Firos..."
    print "---------------------------------\n"

    # print ros2Obj(String())
    # print ros2Obj(Twist())
    # print ros2Obj(cmd_vel())

    # print "AAAAAAAAAAAAAAAAAAAAAAAA"
    # string_data = "hola Mundo"
    # # string_data = {'x_pose': 1, 'y_pose': 2}
    # twist_data = {'linear': {'y': 2.2, 'x': 1.1, 'z': 3.3}, 'angular': {'y': 5.5, 'x': 4.4, 'z': 6.6}}
    # vel_data = {'x_pose': 1, 'y_pose': 2}

    # strInst = String()
    # twInst = Twist()
    # velInst = cmd_vel()

    # obj2Ros(string_data, strInst)
    # obj2Ros(twist_data, twInst)
    # obj2Ros(vel_data, velInst)

    # print ros2Obj(strInst)
    # print ros2Obj(twInst)
    # print ros2Obj(velInst)


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
