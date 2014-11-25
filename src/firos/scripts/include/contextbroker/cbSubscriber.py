import os
import json
import rospy
import signal
import urllib2

from include.constants import *

class CbSubscriber:
    def __init__(self):
        self.ip = urllib2.urlopen('http://ip.42.pl/raw').read()
        self.subscribe()
        # self.disconnect()

    def subscribe(self):
        subscriber_dict = {
            "entities": [
                {
                    "type": "Room",
                    "isPattern": "false",
                    "id": "Room1"
                }
            ],
            "attributes": [
                "cmd_vel"
            ],
            "reference": "http://{}:{}/firos".format(self.ip, SERVER["PORT"]),
            "duration": "P1M",
            "notifyConditions": [
                {
                    "type": "ONCHANGE",
                    "condValues": [
                        "cmd_vel"
                    ]
                }
            ],
            "throttling": "PT1S"
        }
        print "Connecting to context broker..."
        url = "http://{}:{}/{}/subscribeContext".format(CONTEXTBROKER["ADDRESS"], CONTEXTBROKER["PORT"], CONTEXTBROKER["PROTOCOL"])
        subscriber_json = json.dumps(subscriber_dict)
        request = urllib2.Request(url, subscriber_json, {'Content-Type': 'application/json', 'Accept': 'application/json'})
        response = urllib2.urlopen(request)
        response_body = json.loads(response.read())
        response.close()
        if "subscribeError" in response_body:
            rospy.logerr("Error Subscribing to Context Broker:")
            rospy.logerr(response_body["subscribeError"]["errorCode"]["details"])
            os.kill(os.getpid(), signal.SIGINT)
        else:
            self.subscriptionId = response_body["subscribeResponse"]["subscriptionId"]
            print "Connected to Context Broker with id {}".format(self.subscriptionId)

    def disconnect(self):
        print "\nDisconnecting Context Broker subscription {}".format(self.subscriptionId)
        disconnect_dict = {
            "subscriptionId": self.subscriptionId
        }
        url = "http://{}:{}/{}/unsubscribeContext".format(CONTEXTBROKER["ADDRESS"], CONTEXTBROKER["PORT"], CONTEXTBROKER["PROTOCOL"])
        disconnect_json = json.dumps(disconnect_dict)
        request = urllib2.Request(url, disconnect_json, {'Content-Type': 'application/json', 'Accept': 'application/json'})
        response = urllib2.urlopen(request)
        response_body = json.loads(response.read())
        response.close()
        if int(response_body["statusCode"]["code"]) >= 400:
            rospy.logerr("Error Disconnecting from Context Broker (subscription: {}):".format(self.subscriptionId))
            rospy.logerr(response_body["statusCode"]["reasonPhrase"])
            print "\n"
        else:
            print "Disconnected from Context Broker\n"
