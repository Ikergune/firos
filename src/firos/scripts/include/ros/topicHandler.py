import os
import rospy


from include.logger import Log
from include.constants import DEFAULT_QUEUE_SIZE, DEFAULT_CONTEXT_TYPE
from include.libLoader import LibLoader

from include.ros.rosConfigurator import RosConfigurator
from include.ros.rosutils import ros2Obj, obj2Ros
from include.ros.dependencies.third_party import *

# PubSub Handlers
from include.pubsub.pubSubFactory import PublisherFactory, SubscriberFactory

import std_msgs.msg

CloudSubscriber = SubscriberFactory.create()
CloudPublisher = PublisherFactory.create()


TOPIC_BASE_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "topics")
ROBOT_TOPICS = {}
robot_data   = {}
subscribers  = []

def loadMsgHandlers(robot_data):
    Log("INFO", "Getting configuration data")
    Log("INFO", "Generating topic handlers:")
    for robotName in robot_data:
        robotName = str(robotName)
        robot = robot_data[robotName]
        if robotName not in ROBOT_TOPICS:
            ROBOT_TOPICS[robotName] = {
                "publisher": {},
                "subscriber": {}
            }
        Log("INFO", "  -" + robotName)
        for topic in robot['topics']:
            topicName = str(topic['name'])
            Log("INFO", "    -" + topicName)
            extra = {"robot": robotName, "topic": topicName}
            if type(topic['msg']) is dict:
                module = LibLoader.loadFromFile(os.path.join(TOPIC_BASE_PATH, robotName+topicName+".py"))
                theclass = getattr(module, topicName)
            else:
                _final_name = topic['msg'].split('.')[-1]
                if _final_name in globals():
                    theclass = globals()[_final_name]
                else:
                    theclass = LibLoader.loadFromSystem(topic['msg'])
                extra["type"] = str(topic['msg'])
            if topic["type"].lower() == "publisher":
                ROBOT_TOPICS[robotName]["publisher"][topicName] = {
                    "class": theclass,
                    "publisher": rospy.Publisher(robotName + "/" + topicName, theclass, queue_size=DEFAULT_QUEUE_SIZE)
                }
            elif topic["type"].lower() == "subscriber":
                ROBOT_TOPICS[robotName]["subscriber"][topicName] = {
                    "class": theclass,
                    "subscriber": rospy.Subscriber(robotName + "/" + topicName, theclass, _callback, extra)
                }
        Log("INFO", "\n")
        CloudSubscriber.subscribe(robotName, DEFAULT_CONTEXT_TYPE, ROBOT_TOPICS[robotName])
        Log("INFO", "Subscribed to " + robotName  + "'s topics\n")

def connectionListeners():
    subscribers.append(rospy.Subscriber("firos/disconnect", std_msgs.msg.String, _robotDisconnection))
    subscribers.append(rospy.Subscriber("firos/connect", std_msgs.msg.String, _robotConnection))

class TopicHandler:
    @staticmethod
    def publish(robot, topic, data):
        if robot in ROBOT_TOPICS and topic in ROBOT_TOPICS[robot]["publisher"]:
            instance = ROBOT_TOPICS[robot]["publisher"][topic]
            msg = instance["class"]()
            obj2Ros(data, msg)
            if "publisher" in instance:
                instance["publisher"].publish(msg)

    @staticmethod
    def unregisterAll():
        CloudSubscriber.disconnectAll()
        Log("INFO", "Unsubscribing topics...")
        for subscriber in subscribers:
            subscriber.unregister()
        for robot_name in ROBOT_TOPICS:
            for topic in ROBOT_TOPICS[robot_name]["subscriber"]:
                ROBOT_TOPICS[robot_name]["subscriber"][topic]["subscriber"].unregister()
        Log("INFO", "Unsubscribed topics\n")

def _callback(data, args):
    robot = str(args['robot'])
    topic = str(args['topic'])
    datatype = "NotYet"
    contextType = DEFAULT_CONTEXT_TYPE
    content = []
    content.append(CloudPublisher.createContent(topic, datatype,ros2Obj(data)))
    CloudPublisher.publish(robot, contextType, content)

def _robotDisconnection(data):
    robot_name = data.data
    Log("INFO", "Disconnected robot: " + robot_name)
    CloudSubscriber.deleteEntity(robot_name, DEFAULT_CONTEXT_TYPE)
    CloudSubscriber.disconnect(robot_name, True)

def _robotConnection(data):
    robot_name = data.data
    Log("INFO", "Connected robot: " + robot_name)
    loadMsgHandlers(RosConfigurator.systemTopics(True))
    # CloudSubscriber.subscribe(robot_name, DEFAULT_CONTEXT_TYPE, ROBOT_TOPICS[robotName]["publisher"].keys())