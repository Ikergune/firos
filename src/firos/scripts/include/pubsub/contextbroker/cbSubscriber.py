import os
import json
import rospy
import signal
import urllib2

from include.constants import *
from include.pubsub.iPubSub import Isubscriber

IP = urllib2.urlopen('http://ip.42.pl/raw').read()

class CbSubscriber(Isubscriber):
    subscriptionIds = []

    def subscribe(self, namespace, data_type, topics):
        subscriber_dict = self._generateSubscription(namespace, data_type, topics)
        print "Subscribing on context broker to " + data_type + " " + namespace + "..."
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
            self.subscriptionIds.append(response_body["subscribeResponse"]["subscriptionId"])
            print "Connected to Context Broker with id {}".format(self.subscriptionIds[-1])

    def disconnect(self):
        for subscriptionId in self.subscriptionIds:
            print "\nDisconnecting Context Broker subscription {}".format(subscriptionId)
            disconnect_dict = {
                "subscriptionId": subscriptionId
            }
            url = "http://{}:{}/{}/unsubscribeContext".format(CONTEXTBROKER["ADDRESS"], CONTEXTBROKER["PORT"], CONTEXTBROKER["PROTOCOL"])
            disconnect_json = json.dumps(disconnect_dict)
            request = urllib2.Request(url, disconnect_json, {'Content-Type': 'application/json', 'Accept': 'application/json'})
            response = urllib2.urlopen(request)
            response_body = json.loads(response.read())
            response.close()
            if int(response_body["statusCode"]["code"]) >= 400:
                rospy.logerr("Error Disconnecting from Context Broker (subscription: {}):".format(subscriptionId))
                rospy.logerr(response_body["statusCode"]["reasonPhrase"])
                print "\n"
            else:
                print "Disconnected subscription {} from Context Broker ".format(subscriptionId)
        print "\n"

    def _generateSubscription(self, namespace, data_type="Robot", topics=[]):
        return {
            "entities": [
                {
                    "type": data_type,
                    "isPattern": "false",
                    "id": namespace
                }
            ],
            "attributes": topics,
            "reference": "http://{}:{}/firos".format(IP, SERVER["PORT"]),
            "duration": "P1M",
            "notifyConditions": [
                {
                    "type": "ONCHANGE",
                    "condValues": topics
                }
            ],
            "throttling": "PT1S"
        }
