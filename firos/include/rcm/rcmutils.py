import re
from include.ros.rosConfigurator import getWhiteLists


def toMsgType(msg):
    return msg.replace("msgs/", "msgs.msg.").replace("/", ".msg.")


def getRobotConfig(robot_name="", topics=[]):
    white_lists = getWhiteLists()
    _topics = {
    }
    for topic in topics:
        _topic = "/" + robot_name + "/" + topic['name']
        pubsub = topic['type']
        if (white_lists[pubsub] is not None and re.search(white_lists[pubsub], _topic) is not None) or (white_lists[pubsub] is None):
            _topics[topic['name']] = {
                "msg": toMsgType(topic["msg"]),
                "type": pubsub
            }
    return {
        robot_name: {
            "topics": _topics
        }
    }
