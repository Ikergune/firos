import os
import imp
import rospy

from include import confManager
from std_msgs import msg as MsgTypes

# PubSub Handlers
from include.pubsub.pubSubFactory import PublisherFactory, SubscriberFactory

CloudSubscriber = SubscriberFactory.create()
CloudPublisher = PublisherFactory.create()


TOPIC_BASE_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "topics")
ROBOT_TOPICS = {}
robot_data   = {}
subscribers  = []

def loadMsgHandlers():
    print "Getting configuration data"
    robot_data = confManager.getRobots()
    print "Subscribing to topics:"
    for robotName in robot_data:
        robotName = str(robotName)
        robot = robot_data[robotName]
        ROBOT_TOPICS[robotName] = {}
        print "  -" + robotName
        for topic in robot['topics']:
            topicName = str(topic['name'])
            print "    -" + topicName
            extra = {"robot": robotName, "topic": topicName}
            if type(topic['msg']) is dict:
                module = _loadFromFile(os.path.join(TOPIC_BASE_PATH, robotName+topicName+".py"))
                ROBOT_TOPICS[robotName][topicName] = getattr(module, topicName)
            else:
                ROBOT_TOPICS[robotName][topicName] = getattr(MsgTypes, topic['msg'])
                extra["type"] = str(topic['msg'])
            subscribers.append(rospy.Subscriber(topicName, ROBOT_TOPICS[robotName][topicName], _callback, extra))
        CloudSubscriber.subscribe(robotName, "Robot", ROBOT_TOPICS[robotName].keys())
    print "Subscribed to topics\n"

class TopicHandler:
    @staticmethod
    def publish(robot, topic, data):
        if robot in ROBOT_TOPICS and topic in ROBOT_TOPICS[robot]:
            MsgClass = ROBOT_TOPICS[robot][topic]
            # pub = rospy.Publisher(topic, MsgClass)
            msg = MsgClass()
            for key in data:
                setattr(msg, key, data[key])
    @staticmethod
    def unregisterAll():
        CloudSubscriber.disconnect()
        print "Unsubscribing from topics..."
        for subscriber in subscribers:
            subscriber.unregister()
        print "Unsubscribed from topics\n"

def _loadFromFile(filepath):
    mod_name,file_ext = os.path.splitext(os.path.split(filepath)[-1])

    if file_ext.lower() == '.py':
        py_mod = imp.load_source(mod_name, filepath)

    elif file_ext.lower() == '.pyc':
        py_mod = imp.load_compiled(mod_name, filepath)

    return py_mod

def _callback(data, args):
    robot = str(args['robot'])
    topic = str(args['topic'])
    datatype = "NotYet"
    contextType = "ROBOT"
    content = []
    if "type" in args:
        content.append(Publisher.createContent(topic, datatype, data, True))
    else:
        for index, name in data.__slots__:
            content.append(Publisher.createContent(topic, datatype, data))
    Publisher.publish(robot, contextType, content)