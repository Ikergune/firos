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

    initial = False
    wait = True

    if initial:
        initial_pos = "robot1"
        ending_pos = "robot2"
    else:
        initial_pos = "robot2"
        ending_pos = "robot1"


    CloudPublisher = PublisherFactory.create()
    # CloudPublisher.publish("talker2", DEFAULT_CONTEXT_TYPE, [{"name": "move", "type": "NotYet", "value": {"x_pose": time.time(), "y_pose": 56, "z_pose": 32}}])
    # CloudPublisher.publish("talker2", DEFAULT_CONTEXT_TYPE, [{"name": "talk", "type": "NotYet", "value": "Hola mundo!!!" + str(time.time())}])

    # print CloudPublisher.createContent("cmd_vel", "NotYet", {"linear": {"x": 5.5, "y": 0.0, "z": 0.0},"angular": {"x": 0.0, "y": 0.0, "z": 0.0}})
    # CloudPublisher.publish("robot2", DEFAULT_CONTEXT_TYPE, [CloudPublisher.createContent("cmd_vel_mux/input/teleop", "NotYet", {"linear": {"x": -0.5, "y": 0.0, "z": 0.0},"angular": {"x": 0.0, "y": 0.0, "z": 0.0}})])
    CloudPublisher.publish("turtle1", DEFAULT_CONTEXT_TYPE, [CloudPublisher.createContent("cmd_vel", "NotYet", {"linear": {"x": -1.5, "y": 0.0, "z": 0.0},"angular": {"x": 0.0, "y": 0.0, "z": 0.0}})])
    # text = String()
    # text.data = "Hello topic"
    # CloudPublisher.publish("myrobot", DEFAULT_CONTEXT_TYPE, [CloudPublisher.createContent("topic", "NotYet", ros2Obj(text))])


    # CloudPublisher.publish(initial_pos, DEFAULT_CONTEXT_TYPE, [CloudPublisher.createContent("move_base_simple/goal", "NotYet", {
    #     "header": {
    #         "seq": 0,
    #         "stamp": "now",
    #         "frame_id": "map"
    #     },
    #     "pose": {
    #         "position": {
    #             "x": 0.8,
    #             "y": 0.8,
    #             "z": 0.0
    #         },
    #         "orientation": {
    #             "w": 1.0
    #         }
    #     }
    # })])

    # if wait:
    #     time.sleep(1.5)

    # CloudPublisher.publish(ending_pos, DEFAULT_CONTEXT_TYPE, [CloudPublisher.createContent("move_base_simple/goal", "NotYet", {
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

    # if wait:
    #     time.sleep(1)


