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
import urllib2
import netifaces


def setConfiguration():
    try:
        current_path = os.path.dirname(os.path.abspath(__file__))
        json_path = current_path.replace("scripts/include", "config/config.json")
        data = json.load(open(json_path))
        return data[data["environment"]]
    except:
        return {}

configured = False

if not configured:
    configured = True
    configData = setConfiguration()
    INTERFACE = configData["interface"] if "interface" in configData else "public"
    LOGLEVEL = configData["log_level"] if "log_level" in configData else "INFO"


SERVER_PORT = configData["server"]["port"]
if("index" in configData["contextbroker"]):
    INDEX_CONTEXTBROKER = {
        "ADDRESS": configData["contextbroker"]["index"]["address"],
        "PORT": configData["contextbroker"]["index"]["port"],
    }

    DATA_CONTEXTBROKER = {
        "ADDRESS": configData["contextbroker"]["data"]["address"],
        "PORT": configData["contextbroker"]["data"]["port"],
    }
else:
    INDEX_CONTEXTBROKER = {
        "ADDRESS": configData["contextbroker"]["address"],
        "PORT": configData["contextbroker"]["port"],
    }

    DATA_CONTEXTBROKER = {
        "ADDRESS": configData["contextbroker"]["address"],
        "PORT": configData["contextbroker"]["port"],
    }

# THROTTLING = "PT1S"
THROTTLING = configData["contextbroker"]["subscription"]["throttling"]
SUBSCRIPTION_LENGTH = configData["contextbroker"]["subscription"]["subscription_length"]
SUBSCRIPTION_REFRESH_DELAY = configData["contextbroker"]["subscription"]["subscription_refresh_delay"]

SEPARATOR_CHAR = "%27"

# ROS CONFIG
NODE_NAME = "firos"
DEFAULT_CONTEXT_TYPE = "ROBOT"
DEFAULT_QUEUE_SIZE = 10

if INTERFACE == "public":
    IP = urllib2.urlopen('http://ip.42.pl/raw').read()
else:
    netifaces.ifaddresses(INTERFACE)
    IP = netifaces.ifaddresses(INTERFACE)[2][0]['addr']

if "map_server_port" in configData:
    MAP_SERVER_PORT = configData["map_server_port"]
else:
    MAP_SERVER_PORT = None

if "rosbridge_port" in configData:
    ROSBRIDGE_PORT = configData["rosbridge_port"]
else:
    ROSBRIDGE_PORT = 9090
