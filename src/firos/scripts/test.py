#!/usr/bin/env python

import rospy
import time

from include.constants import *
from include.pubsub.pubSubFactory import PublisherFactory

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
    CloudPublisher.publish("turtle1", DEFAULT_CONTEXT_TYPE, [CloudPublisher.createContent("cmd_vel", "NotYet", {"linear": {"x": -5.5, "y": 0.0, "z": 0.0},"angular": {"x": 0.0, "y": 0.0, "z": 0.0}})])

    # TEst sending string in data and the string alone