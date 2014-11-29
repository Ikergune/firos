import os
import rospy

from include import confManager
from include.libLoader import LibLoader
from include.ros.rosutils import ros2Obj, obj2Ros

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
                module = LibLoader.loadFromFile(os.path.join(TOPIC_BASE_PATH, robotName+topicName+".py"))
                ROBOT_TOPICS[robotName][topicName] = getattr(module, topicName)
            else:
                ROBOT_TOPICS[robotName][topicName] = LibLoader.loadFromSystem(topic['msg'])
                extra["type"] = str(topic['msg'])
            subscribers.append(rospy.Subscriber(topicName, ROBOT_TOPICS[robotName][topicName], _callback, extra))
        CloudSubscriber.subscribe(robotName, "ROBOT", ROBOT_TOPICS[robotName].keys())
    print "Subscribed to topics\n"
    print ROBOT_TOPICS

class TopicHandler:
    @staticmethod
    def publish(robot, topic, data):
        if robot in ROBOT_TOPICS and topic in ROBOT_TOPICS[robot]:
            MsgClass = ROBOT_TOPICS[robot][topic]
            print robot + "/" + topic
            publicator = rospy.Publisher(robot + "/" + topic, MsgClass, queue_size=10)
            # print MsgClass
            msg = MsgClass()
            obj2Ros(data, msg)
            # if type(data) is dict:
            #     msg = MsgClass()
            #     for key in data:
            #         setattr(msg, key, data[key])
            # else:
            #     msg = data
            publicator.publish(msg)
            print robot, topic, msg

    @staticmethod
    def unregisterAll():
        CloudSubscriber.disconnect()
        print "Unsubscribing from topics..."
        for subscriber in subscribers:
            subscriber.unregister()
        print "Unsubscribed from topics\n"

def _callback(data, args):
    robot = str(args['robot'])
    topic = str(args['topic'])
    datatype = "NotYet"
    contextType = DEFAULT_CONTEXT_TYPE
    content = []
    content.append(Publisher.createContent(topic, datatype,ros2Obj(data)))
    # if "type" in args:
    #     content.append(Publisher.createContent(topic, datatype, data, True))
    # else:
    #     for index, name in data.__slots__:
    #         content.append(Publisher.createContent(topic, datatype, data))
    Publisher.publish(robot, contextType, content)