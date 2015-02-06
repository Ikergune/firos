import os
import re
import json
import socket
import rostopic
import rosgraph

from include.constants import NODE_NAME

regex = re.compile(ur'^\/([\w]+)\/*([\/\-\w]*)$')
robots = None

class RosConfigurator:

    @staticmethod
    def topic_type(t, topic_types):
        matches = [t_type for t_name, t_type in topic_types if t_name == t]
        if matches:
            return matches[0].replace("msgs/", "msgs.msg.").replace("/", ".msg.")
        return 'unknown type'

    @staticmethod
    def setRobot(robots, topic, t_type, pubsub, whiteList):
        matching = re.search(regex, topic)
        robot_topic = matching.group(2)
        if robot_topic != '':
            robot_name = matching.group(1)
            if (whiteList is not None and robot_name in whiteList and robot_topic in whiteList[robot_name]) or (whiteList is None):
                if robot_name not in robots:
                    robots[robot_name] = {"topics": []}
                robots[robot_name]["topics"].append({
                    "name": robot_topic,
                    "msg": t_type,
                    "type": pubsub
                })

    @staticmethod
    def systemTopics(refresh=False):
        global robots
        if robots is None or refresh:
            existing_topics = {
                "publisher": {},
                "subscriber": {}
            }
            whiteList = _getWhiteList()
            print whiteList
            robots = {}
            master = rosgraph.Master('/rostopic')
            try:
                state = master.getSystemState()
                topic_types = rostopic._master_get_topic_types(master)

                pubs, subs, _ = state

                for t, l in pubs:
                    existing_topics["publisher"][t] = l
                for t, l in subs:
                    existing_topics["subscriber"][t] = l

                # ROS publisher --> firos subscribes to listen data published
                for t, l in pubs:
                    subscribing = _isInFiros(t, existing_topics["subscriber"], l)
                    publishing = _isInFiros(t, existing_topics["publisher"], l)
                    if not subscribing and not publishing:
                        _type = RosConfigurator.topic_type(t, topic_types)
                        RosConfigurator.setRobot(robots, t,_type, "subscriber", whiteList)

                # ROS subscriber --> firos publishes data to them
                for t, l in subs:
                    subscribing = _isInFiros(t, existing_topics["subscriber"], l)
                    publishing = _isInFiros(t, existing_topics["publisher"], l)
                    if not subscribing and not publishing:
                        _type = RosConfigurator.topic_type(t, topic_types)
                        RosConfigurator.setRobot(robots, t,_type, "publisher", whiteList)

            except socket.error:
                raise rostopic.ROSTopicIOException("Unable to communicate with master!")

        return robots

def _isInFiros(topic_name, list2Check,nodes):
    using = False
    if topic_name not in list2Check:
        return False
    for node in list2Check[topic_name]:
        if node == "/" + NODE_NAME:
            using = True
            break

    return using

def _getWhiteList():
    try:
        current_path = os.path.dirname(os.path.abspath(__file__))
        json_path = current_path.replace("scripts/include/ros", "config/whitelist.json")
        return json.load(open(json_path))
    except:
        return None