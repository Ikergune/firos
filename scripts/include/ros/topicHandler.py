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


TOPIC_BASE_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "topics")
ROBOT_TOPICS = {}
robot_data = {}
subscribers = []


def loadMsgHandlers(robot_data):
    ## \brief Load ROS publishers/subscribers based on robot data
    # \param robot data
    Log("INFO", "Getting configuration data")
    Log("INFO", "Generating topic handlers:")
    msg_types = {}
    for robotName in robot_data:
        robotName = str(robotName)
        robot = robot_data[robotName]
        if robotName not in ROBOT_TOPICS:
            ROBOT_TOPICS[robotName] = {
                "publisher": {},
                "subscriber": {}
            }
        Log("INFO", "  -" + robotName)
        for topicName in robot['topics']:
            topic = robot['topics'][topicName]
            topicName = str(topicName)
            Log("INFO", "    -" + topicName)
            extra = {"robot": robotName, "topic": topicName}
            if type(topic['msg']) is dict:
                _topic_name = robotName + ".msg." + topicName
                module = LibLoader.loadFromFile(os.path.join(TOPIC_BASE_PATH, robotName+topicName+".py"))
                theclass = getattr(module, topicName)
            else:
                _final_name = topic['msg'].split('.')[-1]
                _topic_name = str(topic['msg'])
                if _final_name in globals():
                    theclass = globals()[_final_name]
                else:
                    theclass = LibLoader.loadFromSystem(topic['msg'])
                extra["type"] = _topic_name

            msg_types[_topic_name] = {
                "name": _topic_name,
                "type": "rosmsg",
                "value": json.dumps(ros2Definition(theclass())).replace('"', SEPARATOR_CHAR)
            }

            if topic["type"].lower() == "publisher":
                ROBOT_TOPICS[robotName]["publisher"][topicName] = {
                    "msg": str(topic['msg']),
                    "class": theclass,
                    "publisher": rospy.Publisher(robotName + "/" + topicName, theclass, queue_size=DEFAULT_QUEUE_SIZE)
                }
            elif topic["type"].lower() == "subscriber":
                ROBOT_TOPICS[robotName]["subscriber"][topicName] = {
                    "msg": str(topic['msg']),
                    "class": theclass,
                    "subscriber": rospy.Subscriber(robotName + "/" + topicName, theclass, _callback, extra)
                }
        Log("INFO", "\n")
        CloudSubscriber.subscribe(robotName, DEFAULT_CONTEXT_TYPE, ROBOT_TOPICS[robotName])
        Log("INFO", "Subscribed to " + robotName + "'s topics\n")
    CloudPublisher.publishMsg(msg_types.values())
    MapHandler.mapPublisher()


def connectionListeners():
    ## \brief Create firos listeners for robot creation or removal
    subscribers.append(rospy.Subscriber("firos/disconnect", std_msgs.msg.String, robotDisconnection))
    subscribers.append(rospy.Subscriber("firos/connect", std_msgs.msg.String, _robotConnection))


class MapHandler:
    @staticmethod
    def mapPublisher():
        ## \brief Obtain map topics and publsh their link into context broker
        maps = RosConfigurator.getMapTopics()
        cb_maps = [
            {
                "name": "websocket",
                "type": "connection",
                "value": "ws://{}:{}".format(IP, ROSBRIDGE_PORT)
            }
        ]
        if(MAP_SERVER_PORT):
            cb_maps.append({
                "name": "socketio",
                "type": "connection",
                "value": "http://{}:{}".format(IP, MAP_SERVER_PORT)
            })
        for map_topic in maps:
            CloudPublisher.publishMap(map_topic, cb_maps)

    @staticmethod
    def mapRemover():
        ## \brief Delete map topics from context broker
        maps = RosConfigurator.getMapTopics()
        for map_topic in maps:
            CloudSubscriber.deleteEntity(map_topic, "MAP", False)


class TopicHandler:
    @staticmethod
    def publish(robot, topic, data):
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


def _callback(data, args):
    ## \brief Callback to handle ROS published data and send it to Context Broker
    # \param data
    # \param extra arguments
    robot = str(args['robot'])
    topic = str(args['topic'])
    datatype = ROBOT_TOPICS[robot]["subscriber"][topic]["msg"]
    contextType = DEFAULT_CONTEXT_TYPE
    content = []
    content.append(CloudPublisher.createContent(topic, datatype, ros2Obj(data)))
    CloudPublisher.publish(robot, contextType, content)


def robotDisconnection(data):
    ## \brief Handle robot diconnection
    # \param robot data dict (name)
    robot_name = data.data
    Log("INFO", "Disconnected robot: " + robot_name)
    if robot_name in ROBOT_TOPICS:
        CloudSubscriber.deleteEntity(robot_name, DEFAULT_CONTEXT_TYPE)
        CloudSubscriber.disconnect(robot_name, True)
        for topic in ROBOT_TOPICS[robot_name]["publisher"]:
            ROBOT_TOPICS[robot_name]["publisher"][topic]["publisher"].unregister()
        for topic in ROBOT_TOPICS[robot_name]["subscriber"]:
            ROBOT_TOPICS[robot_name]["subscriber"][topic]["subscriber"].unregister()
        RosConfigurator.removeRobot(robot_name)


def _robotConnection(data):
    ## \brief Handle robot connection
    # \param robot data (Not neccessary)
    robot_name = data.data
    Log("INFO", "Connected robot: " + robot_name)
    loadMsgHandlers(RosConfigurator.systemTopics(True))
