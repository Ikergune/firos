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

# map_regex = re.compile(ur'^.*\/(map)[\/]*$')
map_regex = re.compile(ur'^.*\/(map).*$')
topic_regex = re.compile(ur'^\/([\w]+)\/*([\/\-\w]*)$')
substitution_regex = re.compile(ur'^([\/\-\w]*)\/([\w]+)$')
robots = {}
ROBO_TOPIC_REG = {}

CURRENT_TOPIC_REG = {}

mem_whitelist = None


class RosConfigurator:
    ## \brief Tool to get Ros data from system

    @staticmethod
    def getMapTopics():
        ## \brief Get topics that are maps

        master = rosgraph.Master('/rostopic')
        maps = []
        try:
            state = master.getSystemState()

            pubs, subs, _ = state

            for topic_name, l in pubs:
                if re.search(map_regex, topic_name):
                    maps.append(topic_name)
        except socket.error:
            raise rostopic.ROSTopicIOException("Unable to communicate with master!")
        return maps

    @staticmethod
    def topic_type(t, topic_types):
        ## \brief Get topic's data type
        # \param topic
        # \param topic types
        matches = [t_type for t_name, t_type in topic_types if t_name == t]
        if matches:
            text = re.search(substitution_regex, str(matches[0]))
            if text is not None:
                return str(text.group(1)) + ".msg." + str(text.group(2))
            else:
                return matches[0].replace("msgs/", "msgs.msg.").replace("/", ".msg.")

        return 'unknown type'

    @staticmethod
    def setRobot(_robots, topic, t_type, pubsub, whiteLists):
        ## \brief Set robot in robot container based on whitelist and topic lifecycle
        # \param local robot dictionary
        # \param topic name
        # \param topic type
        # \param publisher/subscriber action
        # \param whitelist regular expresions
        global ROBO_TOPIC_REG
        global robots
        matching = re.search(topic_regex, topic)
        robot_topic = matching.group(2)
        if robot_topic != '':
            robot_name = matching.group(1)
            if (whiteLists[pubsub] is not None and re.search(whiteLists[pubsub], topic) is not None) or (whiteLists[pubsub] is None):
                if robot_name not in ROBO_TOPIC_REG:
                    ROBO_TOPIC_REG[robot_name] = {"topics": []}
                if robot_name not in CURRENT_TOPIC_REG:
                    CURRENT_TOPIC_REG[robot_name] = {"topics": []}

                if robot_name not in _robots:
                    _robots[robot_name] = {"topics": {}}

                if robot_topic not in _robots[robot_name]["topics"]:
                    CURRENT_TOPIC_REG[robot_name]["topics"].append(robot_topic)
                    _robots[robot_name]["topics"][robot_topic] = {
                        "msg": t_type,
                        "type": pubsub
                    }

                if robot_name not in robots:
                    robots[robot_name] = {"topics": {}}
                if robot_topic not in robots[robot_name]["topics"]:
                    ROBO_TOPIC_REG[robot_name]["topics"].append(robot_topic)
                    robots[robot_name]["topics"][robot_topic] = {
                        "msg": t_type,
                        "type": pubsub
                    }

    @staticmethod
    def removeRobot(robot_name):
        ## \brief Remove robot from list
        # \param robot name
        global robots
        if robot_name in robots:
            del robots[robot_name]
            del ROBO_TOPIC_REG[robot_name]

    @staticmethod
    def systemTopics(refresh=False):
        ## \brief Get existing topics and return in a map grouped by namespace
        # \param refresh list
        global robots
        global CURRENT_TOPIC_REG
        CURRENT_TOPIC_REG = {}
        if refresh:
            existing_topics = {
                "publisher": {},
                "subscriber": {}
            }
            whiteLists = getWhiteLists()
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
                        RosConfigurator.setRobot(_robots, t, _type, "subscriber", whiteLists)
                        RosConfigurator.setRobot(_robots, t, _type, "publisher", whiteLists)

                # ROS subscriber --> firos publishes data to them
                for t, l in subs:
                    subscribing = _isInFiros(t, existing_topics["subscriber"], l)
                    publishing = _isInFiros(t, existing_topics["publisher"], l)
                    if not subscribing and not publishing:
                        _type = RosConfigurator.topic_type(t, topic_types)
                        RosConfigurator.setRobot(_robots, t, _type, "publisher", whiteLists)
                        RosConfigurator.setRobot(_robots, t, _type, "subscriber", whiteLists)

            except socket.error:
                raise rostopic.ROSTopicIOException("Unable to communicate with master!")

            return _robots
        else:
            return robots


def setWhiteList(additions, deletions, restore=False):
    global mem_whitelist
    if mem_whitelist is None:
        try:
            current_path = os.path.dirname(os.path.abspath(__file__))
            json_path = current_path.replace("scripts/include/ros", "config/whitelist.json")
            mem_whitelist = json.load(open(json_path))
        except:
            mem_whitelist = {}
    if additions:
        for robot_name in additions:
            mem_whitelist[robot_name] = additions[robot_name]
    if deletions:
        for robot_name in deletions:
            if robot_name in mem_whitelist:
                for topic in deletions[robot_name]["publisher"]:
                    if topic in mem_whitelist[robot_name]["publisher"]:
                        mem_whitelist[robot_name]["publisher"].remove(topic)
                for topic in deletions[robot_name]["subscriber"]:
                    if topic in mem_whitelist[robot_name]["subscriber"]:
                        mem_whitelist[robot_name]["subscriber"].remove(topic)
    if restore:
        mem_whitelist = None


def _isInFiros(topic_name, list2Check, nodes):
    using = False
    if topic_name not in list2Check:
        return False
    for node in list2Check[topic_name]:
        if node == "/" + NODE_NAME:
            using = True
            break

    return using


def getWhiteLists():
    return {
        "publisher": _getWhiteList("publisher"),
        "subscriber": _getWhiteList("subscriber")
    }


def _getWhiteList(pubsub):
    try:
        if mem_whitelist is None:
            current_path = os.path.dirname(os.path.abspath(__file__))
            json_path = current_path.replace("scripts/include/ros", "config/whitelist.json")
            data = json.load(open(json_path))
        else:
            data = mem_whitelist

        whiteregex = ur''
        for robot_name in data:
            for topic in data[robot_name][pubsub]:
                whiteregex += '(/' + robot_name + '/' + topic + ')|'
        whiteregex = whiteregex[:-1]
        whiteregex += "$"
        whiteregex = ur'^' + whiteregex
        return whiteregex
    except:
        return None
