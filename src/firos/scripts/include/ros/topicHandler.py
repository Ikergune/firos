import os
import imp
import rospy

from include import confManager
from include.contextbroker.cbPublisher import CbPublisher

TOPIC_BASE_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "topics")
ROBOT_TOPICS = {}

def loadMsgHandlers():
    robot_data = confManager.getRobots()
    for robot in robot_data:
        robotName = str(robot['name'])
        ROBOT_TOPICS[robotName] = {}
        for topic in robot['topics']:
            topicName = str(topic['name'])
            module = _loadFromFile(os.path.join(TOPIC_BASE_PATH, robotName+topicName+".py"))
            ROBOT_TOPICS[robotName][topicName] = getattr(module, topicName)
            rospy.Subscriber('topic', ROBOT_TOPICS[robotName][topicName], _callback)
            # Not needed, the server is listening
            # rospy.spin()

    print ROBOT_TOPICS

class TopicHandler:
    @staticmethod
    def publish(robot, topic, data):
        MsgClass = ROBOT_TOPICS[robot][topic]
        # pub = rospy.Publisher(topic, MsgClass)
        msg = MsgClass()
        for key in data:
            setattr(msg, key, data[key])

def _loadFromFile(filepath):
    mod_name,file_ext = os.path.splitext(os.path.split(filepath)[-1])

    if file_ext.lower() == '.py':
        py_mod = imp.load_source(mod_name, filepath)

    elif file_ext.lower() == '.pyc':
        py_mod = imp.load_compiled(mod_name, filepath)

    return py_mod

def _callback(data):
    # Simply print out values in our custom message.
    robot, topic = getattr(data, "_type").split("/")
    rospy.loginfo(rospy.get_name() + " I heard %s", data.message)
    rospy.loginfo(rospy.get_name() + " a + b = %d", data.a + data.b)
    # Robot is needed, datatype and attributes (check data, it might be an instance of the class)
    # CbPublisher.publish(robot, topic, [])