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

import os
import json
import rospy


from include.logger import Log
from include.constants import DEFAULT_QUEUE_SIZE, DEFAULT_CONTEXT_TYPE, SEPARATOR_CHAR, IP, MAP_SERVER_PORT, ROSBRIDGE_PORT
from include.libLoader import LibLoader

from include.ros.rosConfigurator import RosConfigurator
from include.ros.rosutils import ros2Obj, obj2Ros, ros2Definition
from include.ros.dependencies.third_party import *

# PubSub Handlers
from include.pubsub.pubSubFactory import PublisherFactory, SubscriberFactory

import std_msgs.msg

CloudSubscriber = SubscriberFactory.create()
CloudPublisher = PublisherFactory.create()

ROBOT_SERVICES = {}


def loadServiceHandler(robot_data):
    msg_types = {}
    for robotName in robot_data:
        robotName = str(robotName)
        robot = robot_data[robotName]
        if robotName not in ROBOT_TOPICS:
            ROBOT_SERVICES[robotName] = []
        if "services" in robot:
            for serviceName in robot['services']:
                service = robot['services'][serviceName]
                serviceName = str(serviceName)
                _final_name = service['msg'].split('.')[-1]
                _service_name = str(service['msg'])
                if _final_name in globals():
                    theclass = globals()[_final_name]
                else:
                    theclass = LibLoader.loadFromSystem(service['msg'])
                msg_types[_service_name] = {
                    "name": _service_name,
                    "type": "rosservice",
                    "value": json.dumps(ros2Definition(theclass())).replace('"', SEPARATOR_CHAR)
                }
                ROBOT_SERVICES[robotName][serviceName] = {
                    "msg": str(topic['msg']),
                    "class": theclass,
                    "service": rospy.ServiceProxy(robotName + "/" + serviceName, theclass)
                }
        CloudSubscriber.subscribe(robotName, DEFAULT_CONTEXT_TYPE, ROBOT_SERVICES[robotName])
    CloudPublisher.publishMsg(msg_types.values())


class ServiceHandler:
    @staticmethod
    def publish(robot, service, data):
        ## \brief Publish data to ROS
        # \param robot name
        # \param topic name
        # \param data to publish
        if robot in ROBOT_TOPICS and topic in ROBOT_TOPICS[robot]["publisher"]:
            instance = ROBOT_TOPICS[robot]["publisher"][topic]
            msg = instance["class"]()
            obj2Ros(data, msg)
            if "publisher" in instance:
                instance["publisher"].publish(msg)

    @staticmethod
    def unregisterAll():
        ## \brief Unregister from all ROS topics
        CloudSubscriber.disconnectAll()
        MapHandler.mapRemover()
        Log("INFO", "Unsubscribing topics...")
        for subscriber in subscribers:
            subscriber.unregister()
        for robot_name in ROBOT_TOPICS:
            for topic in ROBOT_TOPICS[robot_name]["subscriber"]:
                ROBOT_TOPICS[robot_name]["subscriber"][topic]["subscriber"].unregister()
        Log("INFO", "Unsubscribed topics\n")
