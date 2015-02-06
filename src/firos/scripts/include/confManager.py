import os
import json
import copy
import traceback
from include.ros.rosConfigurator import RosConfigurator

def getRobots(refresh=False, withJson=True):
    try:
        robots = copy.deepcopy(RosConfigurator.systemTopics(refresh))
        if withJson:
            robots_json = getRobotsByJson()
            for robot_name in robots_json:
                robot_name = str(robot_name)
                if robot_name not in robots:
                    robots[robot_name] = {
                        "topics": []
                    }
                for topic in robots_json[robot_name]["topics"]:
                    for i, v in enumerate(robots[robot_name]["topics"]):
                        if topic["name"] == v["name"]:
                            del robots[robot_name]["topics"][i]
                            break
                    robots[robot_name]["topics"].append({
                        "name"      : str(topic["name"]),
                        "msg"       : str(topic["msg"]) if type(topic["msg"]) is str else topic["msg"],
                        "type"      : str(topic["type"])
                    })

        return robots

    except Exception as e:
        traceback.print_exc()
        print("ERROR: ",e)
        return {}


def getRobotsByJson():
    try:
        current_path = os.path.dirname(os.path.abspath(__file__))
        json_path = current_path.replace("scripts/include", "config/robots.json")
        return json.load(open(json_path))
    except:
        return {}
