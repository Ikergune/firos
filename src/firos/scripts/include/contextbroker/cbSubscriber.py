import os
import json
import rospy
import signal
import urllib2

from include.constants import *

class CbSubscriber:
    def __init__(self):
        self.ip = urllib2.urlopen('http://ip.42.pl/raw').read()
        # self.subscribe()

    def subscribe(self):
        subscriber_dict = {
            "SSSSSSSentities": [
                {
                    "type": "Room",
                    "isPattern": "false",
                    "id": "Room1"
                }
            ],
            "SSSSSSSattributes": [
                "temperature"
            ],
            "SSSSSSSreference": "http://{}:{}/firos".format(self.ip, SERVER["PORT"]),
            "SSSSSSSduration": "P1M",
            "SSSSSSSnotifyConditions": [
                {
                    "type": "ONCHANGE",
                    "condValues": [
                        "presssure"
                    ]
                }
            ],
            "SSSSSSSthrottling": "PT5S"
        }
        print "Connecting to context broker..."
        url = "http://{}:{}/{}/subscribeContext".format(CONTEXTBROKER["ADDRESS"], CONTEXTBROKER["PORT"], CONTEXTBROKER["PROTOCOL"])
        subscriber_json = json.dumps(subscriber_dict)
        request = urllib2.Request(url, subscriber_json, {'Content-Type': 'application/json', 'Accept': 'application/json'})
        response = urllib2.urlopen(request)
        response_body = json.loads(response.read())
        response.close()
        if response_body["subscribeError"] is not None:
            rospy.logerr("Error Subscribing to context broker:")
            rospy.logerr(response_body["subscribeError"]["errorCode"]["details"])
            os.kill(os.getpid(), signal.SIGINT)
        else:
            print "Connected to Context Broker"
