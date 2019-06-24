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

import sys
import json
import copy
import traceback

from include.logger import Log
from include.ros.rosConfigurator import RosConfigurator
from include.constants import Constants as C


def getRobots(refresh=False):
    ''' This retrieves the current configuration from FIROS.
        Here we load the `robots.json` and the 'whitelist.json'

        In case the `whitelist.json` is Empty: We do get the current robots 
        from ROS and are adding them as subscribers (by default). In a small
        ROS-World this usually is no problem. But in an environment with many 
        robots we might send a lot of data. 
    '''
    try:
        # Retrieves the whitelist.json. If it does not exists, it returns all topics.
        robots = copy.deepcopy(RosConfigurator.systemTopics(refresh))    
        
        # Retrieves the robots.json.
        robots_json = getRobotsByJson()
        if len(robots_json) == 0: 
            Log("ERROR", "The file 'robots.json' is either empty or does not exist!\n\nExiting")
            sys.exit(1)
        
        # Merge robots.json into whitelist.json (overwrite if neccessary)
        for robot_name in robots_json:
            robot_name = str(robot_name)
            if robot_name not in robots:
                robots[robot_name] = {
                    "topics": {}
                }
            for topic_name in robots_json[robot_name]["topics"]:
                topic = robots_json[robot_name]["topics"][topic_name]

                # Overwrite or add!
                robots[robot_name]["topics"][str(topic_name)] = {
                    "msg": str(topic["msg"]),
                    "type": str(topic["type"])
                }

        return robots

    except Exception as e:
        traceback.print_exc()
        Log("ERROR", e)
        return {}


def getRobotsByJson():
    ''' Load the 'robots.json'-File 
    '''
    try:
        json_path = C.PATH + "/robots.json"
        return json.load(open(json_path))
    except:
        return {}
