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
import re
import json
import socket
import rostopic
import rosgraph

from include.constants import NODE_NAME

regex = re.compile(ur'^\/([\w]+)\/*([\/\-\w]*)$')
robots = {}
ROBO_TOPIC_REG = {}

class RosConfigurator:

    @staticmethod
    def topic_type(t, topic_types):
        matches = [t_type for t_name, t_type in topic_types if t_name == t]
        if matches:
            return matches[0].replace("msgs/", "msgs.msg.").replace("/", ".msg.")
        return 'unknown type'

    @staticmethod
    def setRobot(_robots, topic, t_type, pubsub, whiteList):
        global ROBO_TOPIC_REG
        global robots
        matching = re.search(regex, topic)
        robot_topic = matching.group(2)
        if robot_topic != '':
            robot_name = matching.group(1)
            if (whiteList is not None and re.search(whiteList, topic) is not None) or (whiteList is None):
                if robot_name not in ROBO_TOPIC_REG:
                    ROBO_TOPIC_REG[robot_name] = {"topics": []}
                if robot_name not in _robots:
                    _robots[robot_name] = {"topics": []}
                _robots[robot_name]["topics"].append({
                    "name": robot_topic,
                    "msg": t_type,
                    "type": pubsub
                })
                if robot_name not in robots:
                    robots[robot_name] = {"topics": []}
                if robot_topic not in ROBO_TOPIC_REG[robot_name]["topics"]:
                    ROBO_TOPIC_REG[robot_name]["topics"].append(robot_topic)
                    robots[robot_name]["topics"].append({
                        "name": robot_topic,
                        "msg": t_type,
                        "type": pubsub
                    })


    @staticmethod
    def removeRobot(robot_name):
        global robots
        if robot_name in robots:
            del robots[robot_name]
            del ROBO_TOPIC_REG[robot_name]

    @staticmethod
    def systemTopics(refresh=False):
        global robots
        if refresh:
            existing_topics = {
                "publisher": {},
                "subscriber": {}
            }
            whiteList = _getWhiteList()
            _robots = {}
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
                        RosConfigurator.setRobot(_robots, t,_type, "subscriber", whiteList)

                # ROS subscriber --> firos publishes data to them
                for t, l in subs:
                    subscribing = _isInFiros(t, existing_topics["subscriber"], l)
                    publishing = _isInFiros(t, existing_topics["publisher"], l)
                    if not subscribing and not publishing:
                        _type = RosConfigurator.topic_type(t, topic_types)
                        RosConfigurator.setRobot(_robots, t,_type, "publisher", whiteList)

            except socket.error:
                raise rostopic.ROSTopicIOException("Unable to communicate with master!")
            return _robots
        else:
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
        data = json.load(open(json_path))
        whiteregex = ur''
        for robot_name in data:
            for topic in data[robot_name]:
                whiteregex += '(/' + robot_name + '/' + topic + ')|'
        whiteregex = whiteregex[:-1]
        whiteregex += "$"
        whiteregex = ur'^' + whiteregex
        return whiteregex
    except:
        return None