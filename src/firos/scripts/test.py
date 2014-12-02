#!/usr/bin/env python

import rospy
import time

from include.constants import *
from include.pubsub.pubSubFactory import PublisherFactory

from include.ros.rosutils import *

from std_msgs.msg import String

# Main function.
if __name__ == '__main__':
    # Initialize the node and name it.
    rospy.init_node('firos_testing')

    print "\nStarting Firos Testing..."
    print "---------------------------------\n"


    CloudPublisher = PublisherFactory.create()
    # CloudPublisher.publish("talker2", DEFAULT_CONTEXT_TYPE, [{"name": "move", "type": "NotYet", "value": {"x_pose": time.time(), "y_pose": 56, "z_pose": 32}}])
    # CloudPublisher.publish("talker2", DEFAULT_CONTEXT_TYPE, [{"name": "talk", "type": "NotYet", "value": "Hola mundo!!!" + str(time.time())}])

    # print CloudPublisher.createContent("cmd_vel", "NotYet", {"linear": {"x": 5.5, "y": 0.0, "z": 0.0},"angular": {"x": 0.0, "y": 0.0, "z": 0.0}})
    # CloudPublisher.publish("robot2", DEFAULT_CONTEXT_TYPE, [CloudPublisher.createContent("cmd_vel_mux/input/teleop", "NotYet", {"linear": {"x": -0.5, "y": 0.0, "z": 0.0},"angular": {"x": 0.0, "y": 0.0, "z": 0.0}})])
    text = String()
    text.data = "Hello topic"
    CloudPublisher.publish("myrobot", DEFAULT_CONTEXT_TYPE, [CloudPublisher.createContent("topic", "NotYet", ros2Obj(text))])

    # CloudPublisher.publish("robot2", DEFAULT_CONTEXT_TYPE, [CloudPublisher.createContent("move_base_simple/goal", "NotYet", {
    #     "header": {
    #         "seq": 0,
    #         "stamp": "now",
    #         "frame_id": "map"
    #     },
    #     "pose": {
    #         "position": {
    #             "x": 1.0,
    #             "y": 1.0,
    #             "z": 0.0
    #         },
    #         "orientation": {
    #             "x": 0.2,
    #             "y": 0.2,
    #             "z": 0.2,
    #             "w": 0.4
    #         }
    #     }
    # })])

    # time.sleep(1)

    # CloudPublisher.publish("robot1", DEFAULT_CONTEXT_TYPE, [CloudPublisher.createContent("move_base_simple/goal", "NotYet", {
    #     "header": {
    #         "seq": 0,
    #         "stamp": "now",
    #         "frame_id": "map"
    #     },
    #     "pose": {
    #         "position": {
    #             "x": -2.5,
    #             "y": -0.8,
    #             "z": 0.0
    #         },
    #         "orientation": {
    #             "w": 1.0
    #         }
    #     }
    # })])
