import os
import imp
import rospy

from include import confManager
from include.contextbroker.cbPublisher import CbPublisher
from std_msgs import msg as MsgTypes

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
    print "Subscribed to topics\n"
    # Not needed, the server is listening
    # rospy.spin()

    # print ROBOT_TOPICS
    # print robot_data
    # print "MsgTypes.String"
    # print MsgTypes.String
    # print ROBOT_TOPICS[robotName][topicName]

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
    # Simply print out values in our custom message.
    robot = str(args['robot'])
    topic = str(args['topic'])
    attributes = []
    if "type" in args:
        attributes.append(CbPublisher.createAttribute("uniqueattr", args['type'], data))
    else:
        for index, name in data.__slots__:
            attributes.append(CbPublisher.createAttribute(name, data._slot_types[index], getattr(data, name)))
    # Robot is needed, datatype and attributes (check data, it might be an instance of the class)
    # CbPublisher.publish(robot, topic, attributes)