import os
import json
import time
import rospy
import thread
import signal
import urllib2

from include.constants import *
from include.pubsub.iPubSub import Isubscriber

IP = urllib2.urlopen('http://ip.42.pl/raw').read()

class CbSubscriber(Isubscriber):
    subscriptions = []
    refresh_thread = None

    def subscribe(self, namespace, data_type, topics):
        subscriber_dict = self._generateSubscription(namespace, data_type, topics)
        print "Subscribing on context broker to " + data_type + " " + namespace + " and topics: " + str(topics)
        subscription = {
            "namespace" : namespace,
            "data_type" : data_type,
            "topics" : topics
        }
        url = "http://{}:{}/{}/subscribeContext".format(CONTEXTBROKER["ADDRESS"], CONTEXTBROKER["PORT"], CONTEXTBROKER["PROTOCOL"])
        subscriber_json = json.dumps(subscriber_dict)
        request = urllib2.Request(url, subscriber_json, {'Content-Type': 'application/json', 'Accept': 'application/json'})
        response = urllib2.urlopen(request)
        response_body = json.loads(response.read())
        response.close()
        if "subscribeError" in response_body:
            print "Error Subscribing to Context Broker:"
            print response_body["subscribeError"]["errorCode"]["details"]
            os.kill(os.getpid(), signal.SIGINT)
        else:
            subscription["id"] = response_body["subscribeResponse"]["subscriptionId"]
            self.subscriptions.append(subscription)
            print "Connected to Context Broker with id {}".format(subscription["id"])
        if self.refresh_thread is None:
            self.refresh_thread = thread.start_new_thread( self._refreshSubscriptions, ("CBSub-Refresh", 2, ) )

    def disconnect(self):
        for subscription in self.subscriptions:
            subscriptionId = subscription["id"]
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
                print "Error Disconnecting from Context Broker (subscription: {}):".format(subscriptionId)
                print response_body["statusCode"]["reasonPhrase"]
                print "\n"
            else:
                print "Disconnected subscription {} from Context Broker ".format(subscriptionId)
        print "\n"

    def refreshSubscriptions(self):
        for subscription in self.subscriptions:
            subscriber_dict = self._generateSubscription(subscription["namespace"], subscription["data_type"], subscription["topics"], subscription["id"])
            subscriber_dict.pop("entities", None)
            subscriber_dict.pop("reference", None)
            url = "http://{}:{}/{}/contextSubscriptions/{}".format(CONTEXTBROKER["ADDRESS"], CONTEXTBROKER["PORT"], CONTEXTBROKER["PROTOCOL"], subscription["id"])
            subscriber_json = json.dumps(subscriber_dict)
            request = urllib2.Request(url, subscriber_json, {'Content-Type': 'application/json', 'Accept': 'application/json'})
            request.get_method = lambda: 'PUT'
            response = urllib2.urlopen(request)
            response_body = json.loads(response.read())
            response.close()
            if "subscribeError" in response_body:
                print "Error Refreshing subscription"
                print response_body["subscribeError"]["errorCode"]["details"]
            elif "orionError" in response_body:
                print "Error Refreshing subscription"
                print response_body["orionError"]["details"]
            else:
                print "Refreshed Connection to Context Broker with id {}".format(subscription["id"])

    def parseData(self, data):
        return json.loads(data.replace("'", '"'))

    def _generateSubscription(self, namespace, data_type=DEFAULT_CONTEXT_TYPE, topics=[], subscriptionId=None):
        data = {
            "entities": [
                {
                    "type": data_type,
                    "isPattern": "false",
                    "id": namespace
                }
            ],
            # "attributes": topics,
            "reference": "http://{}:{}/firos".format(IP, SERVER["PORT"]),
            "duration": SUBSCRIPTION_LENGTH,
            "notifyConditions": [
                {
                    "type": "ONCHANGE",
                    "condValues": topics
                }
            ],
            "throttling": THROTTLING
        }
        if subscriptionId is not None:
            data["subscriptionId"] = str(subscriptionId)
        return data

    def _refreshSubscriptions(self, threadName, delay):
        # Seconds to days
        total_delay = SUBSCRIPTION_REFRESH_DELAY * 60 *60 * 24
        while True:
            time.sleep(total_delay)
            self.refreshSubscriptions()

