import os
import json

def setConfiguration():
    try:
        current_path = os.path.dirname(os.path.abspath(__file__))
        json_path = current_path.replace("scripts/include", "config/firosconfig.json")
        return json.load(open(json_path))
    except:
        return {}

configured = False

if not configured:
    configured = True
    configData = setConfiguration()
    INTERFACE = configData["net_interface"] if "net_interface" in configData else "public"
    ENVIRONMENT = configData["environment"] if "environment" in configData else "local"
    LOGLEVEL = configData["log_level"] if "log_level" in configData else "INFO"


environment = ENVIRONMENT

CONFIGURATIONS = {
    "mobile": {
        "SERVER": {
            "ADDRESS" : "0.0.0.0",
            "PORT"    : 10100
        },
        "CONTEXTBROKER": {
            "ADDRESS": "192.168.43.159",
            "PORT"    : 1026,
            "PROTOCOL": "NGSI10"
        }
    },
    "local": {
        "SERVER": {
            "ADDRESS" : "0.0.0.0",
            "PORT"    : 10100
        },
        "CONTEXTBROKER": {
            "ADDRESS": "192.168.4.70",
            "PORT"    : 1026,
            "PROTOCOL": "NGSI10"
        }
    },
    "development": {
        "SERVER": {
            "ADDRESS" : "0.0.0.0",
            "PORT"    : 10100
        },
        "CONTEXTBROKER": {
            "ADDRESS" : "130.206.127.115",
            "PORT"    : 1026,
            "PROTOCOL": "NGSI10"
        }
    },
    "production": {
        "SERVER": {
            "ADDRESS" : "0.0.0.0",
            "PORT"    : 10100
        },
        "CONTEXTBROKER": {
            "ADDRESS" : "130.206.127.115",
            "PORT"    : 1026,
            "PROTOCOL": "NGSI10"
        }
    }
}

SERVER = CONFIGURATIONS[environment]["SERVER"]
CONTEXTBROKER = CONFIGURATIONS[environment]["CONTEXTBROKER"]

# THROTTLING = "PT1S"
THROTTLING = "PT0S"
SUBSCRIPTION_LENGTH = "P1D"
SUBSCRIPTION_REFRESH_DELAY = 20
SEPARATOR_CHAR = "%27"
# SEPARATOR_CHAR = "'"

# ROS CONFIG
NODE_NAME = "firos"
DEFAULT_CONTEXT_TYPE = "ROBOT"
DEFAULT_QUEUE_SIZE = 10
