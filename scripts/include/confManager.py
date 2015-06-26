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
import json
import copy
import traceback

from include.logger import Log
from include.ros.rosConfigurator import RosConfigurator


def getRobots(refresh=False, withJson=True):
    ## \brief Get robots managed by firos
    # \param Refresh the robot list
    # \param Merge the robots with the configurtation JSON
    try:
        robots = copy.deepcopy(RosConfigurator.systemTopics(refresh))
        if withJson:
            robots_json = getRobotsByJson()
            for robot_name in robots_json:
                robot_name = str(robot_name)
                if robot_name not in robots:
                    robots[robot_name] = {
                        "topics": {}
                    }
                for topic_name in robots_json[robot_name]["topics"]:
                    topic = robots_json[robot_name]["topics"][topic_name]

                    robots[robot_name]["topics"][str(topic_name)] = {
                        "msg": str(topic["msg"]) if type(topic["msg"]) is str else topic["msg"],
                        "type": str(topic["type"])
                    }
        return robots

    except Exception as e:
        traceback.print_exc()
        Log("ERROR", e)
        return {}


def getRobotsByJson():
    ## \brief Get robots in the JSON file
    try:
        current_path = os.path.dirname(os.path.abspath(__file__))
        json_path = current_path.replace("scripts/include", "config/robots.json")
        return json.load(open(json_path))
    except:
        return {}
