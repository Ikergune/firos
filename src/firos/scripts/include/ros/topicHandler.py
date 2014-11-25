import imp
import os
import rospy

from include import confManager

TOPIC_BASE_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "topics")
ROBOT_TOPICS = {}


def load_from_file(filepath):
    mod_name,file_ext = os.path.splitext(os.path.split(filepath)[-1])

    if file_ext.lower() == '.py':
        py_mod = imp.load_source(mod_name, filepath)

    elif file_ext.lower() == '.pyc':
        py_mod = imp.load_compiled(mod_name, filepath)

    return py_mod

robot_data = confManager.getRobots()
for robot in robot_data:
    robotName = str(robot['name'])
    ROBOT_TOPICS[robotName] = {}
    for topic in robot['topics']:
        topicName = str(topic['name'])
        module = load_from_file(os.path.join(TOPIC_BASE_PATH, robotName+topicName+".py"))
        ROBOT_TOPICS[robotName][topicName] = getattr(module, topicName)

print ROBOT_TOPICS

class TopicHandler:
    @staticmethod
    def publish(topic, datatype, value):
        print "hola"