import os
import rospy

from include import confManager
from include.constants import DEFAULT_QUEUE_SIZE, DEFAULT_CONTEXT_TYPE
from include.libLoader import LibLoader
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

def loadMsgHandlers():
    print "Getting configuration data"
    robot_data = confManager.getRobots()
    print "Generating topic handlers:"
    for robotName in robot_data:
        robotName = str(robotName)
        robot = robot_data[robotName]
        ROBOT_TOPICS[robotName] = {}
        print "  -" + robotName
        for topic in robot['topics']:
            topicName = str(topic['name'])
            print "    -" + topicName
            extra = {"robot": robotName, "topic": topicName}
            ROBOT_TOPICS[robotName][topicName] = {}
            if type(topic['msg']) is dict:
                module = LibLoader.loadFromFile(os.path.join(TOPIC_BASE_PATH, robotName+topicName+".py"))
                ROBOT_TOPICS[robotName][topicName]["class"] = getattr(module, topicName)
            else:
                _final_name = topic['msg'].split('.')[-1]
                if _final_name in globals():
                    ROBOT_TOPICS[robotName][topicName]["class"] = globals()[_final_name]
                else:
                    ROBOT_TOPICS[robotName][topicName]["class"] = LibLoader.loadFromSystem(topic['msg'])
                extra["type"] = str(topic['msg'])
            if topic["type"].lower() == "publisher":
                ROBOT_TOPICS[robotName][topicName]["publisher"] = rospy.Publisher(robotName + "/" + topicName, ROBOT_TOPICS[robotName][topicName]["class"], queue_size=DEFAULT_QUEUE_SIZE)
            elif topic["type"].lower() == "subscriber":
                subscribers.append(rospy.Subscriber(robotName + "/" + topicName, ROBOT_TOPICS[robotName][topicName]["class"], _callback, extra))
        print "\n"
        CloudSubscriber.subscribe(robotName, DEFAULT_CONTEXT_TYPE, ROBOT_TOPICS[robotName].keys())
        print "Subscribed to " + robotName  + "'s topics\n"
    subscribers.append(rospy.Subscriber("disconnect", std_msgs.msg.String, _robotDisconnection))
    subscribers.append(rospy.Subscriber("connect", std_msgs.msg.String, _robotConnection))

class TopicHandler:
    @staticmethod
    def publish(robot, topic, data):
        if robot in ROBOT_TOPICS and topic in ROBOT_TOPICS[robot]:
            instance = ROBOT_TOPICS[robot][topic]
            msg = instance["class"]()
            obj2Ros(data, msg)
            if "publisher" in instance:
                instance["publisher"].publish(msg)

    @staticmethod
    def unregisterAll():
        CloudSubscriber.disconnectAll()
        print "Unsubscribing topics..."
        for subscriber in subscribers:
            subscriber.unregister()
        print "Unsubscribed topics\n"

def _callback(data, args):
    robot = str(args['robot'])
    topic = str(args['topic'])
    print "Data received from " + robot + "in topic " + topic
    datatype = "NotYet"
    contextType = DEFAULT_CONTEXT_TYPE
    content = []
    content.append(CloudPublisher.createContent(topic, datatype,ros2Obj(data)))
    CloudPublisher.publish(robot, contextType, content)

def _robotDisconnection(data):
    robot_name = data.data
    print("Disconnected robot: " + robot_name)
    CloudSubscriber.deleteEntity(robot_name, DEFAULT_CONTEXT_TYPE)
    CloudSubscriber.disconnect(robot_name, True)

def _robotConnection(data):
    robot_name = data.data
    print("Connected robot: " + robot_name)
    CloudSubscriber.subscribe(robot_name, DEFAULT_CONTEXT_TYPE, ROBOT_TOPICS[robot_name].keys())