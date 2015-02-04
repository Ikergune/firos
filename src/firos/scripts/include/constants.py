environment = "local"

CONFIGURATIONS = {
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

# ROS CONFIG
NODE_NAME = "firos"
DEFAULT_CONTEXT_TYPE = "ROBOT"
DEFAULT_QUEUE_SIZE = 10


