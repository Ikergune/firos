import os
import json
import traceback
from include.ros.rosConfigurator import RosConfigurator

def getRobots():
    try:
        return RosConfigurator.systemTopics()
    except Exception as e:
        traceback.print_exc()
        print("ERROR: ",e)
        return []


def getRobotsByJson():
    try:
        current_path = os.path.dirname(os.path.abspath(__file__))
        json_path = current_path.replace("scripts/include", "config/robots.json")
        return json.load(open(json_path))
    except Exception as e:
        traceback.print_exc()
        print("ERROR: ",e)
        return []
