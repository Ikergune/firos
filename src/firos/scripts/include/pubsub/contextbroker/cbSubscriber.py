import os
import json
import time
import thread
import signal
import urllib2

from include.constants import *
from include.pubsub.iPubSub import Isubscriber

IP = urllib2.urlopen('http://ip.42.pl/raw').read()
# IP = "10.8.0.6"

class CbSubscriber(Isubscriber):
    subscriptions = {}
    refresh_thread = None

    def subscribe(self, namespace, data_type, robot):
        if namespace not in self.subscriptions:
            topics = robot["publisher"].keys()
            print "Subscribing on context broker to " + data_type + " " + namespace + " and topics: " + str(topics)
            subscription = {
                "namespace" : namespace,
                "data_type" : data_type,
                "topics" : topics
            }
            url = "http://{}:{}/{}/subscribeContext".format(CONTEXTBROKER["ADDRESS"], CONTEXTBROKER["PORT"], CONTEXTBROKER["PROTOCOL"])
            subscriber_json = json.dumps(self._generateSubscription(namespace, data_type, topics))
            # print subscriber_json
            response_body = self._sendRequest(url, subscriber_json)
            if "subscribeError" in response_body:
                print "Error Subscribing to Context Broker:"
                print response_body["subscribeError"]["errorCode"]["details"]
                os.kill(os.getpid(), signal.SIGINT)
            else:
                subscription["id"] = response_body["subscribeResponse"]["subscriptionId"]
                self.subscriptions[namespace] = subscription
                print "Connected to Context Broker with id {}".format(subscription["id"])
            if self.refresh_thread is None:
                self.refresh_thread = thread.start_new_thread( self._refreshSubscriptions, ("CBSub-Refresh", 2, ) )

    def disconnect(self, namespace, delete=False):
        if namespace in self.subscriptions:
            subscription = self.subscriptions[namespace]
            subscriptionId = subscription["id"]
            print "\nDisconnecting Context Broker subscription {}".format(subscriptionId)
            url = "http://{}:{}/{}/unsubscribeContext".format(CONTEXTBROKER["ADDRESS"], CONTEXTBROKER["PORT"], CONTEXTBROKER["PROTOCOL"])
            disconnect_json = json.dumps({
                "subscriptionId": subscriptionId
            })
            response_body = self._sendRequest(url, disconnect_json)
            if int(response_body["statusCode"]["code"]) >= 400:
                print "Error Disconnecting from Context Broker (subscription: {}):".format(subscriptionId)
                print response_body["statusCode"]["reasonPhrase"]
                print "\n"
            else:
                print "Disconnected subscription {} from Context Broker ".format(subscriptionId)

            print "Deleting entity"
            self.deleteEntity(subscription["namespace"], subscription["data_type"])
            print "\n"
            if delete:
                del self.subscriptions[namespace]

    def disconnectAll(self):
        for subscription in self.subscriptions:
            self.disconnect(subscription)

    def refreshSubscriptions(self):
        for subscription in self.subscriptions.values():
            subscriber_dict = self._generateSubscription(subscription["namespace"], subscription["data_type"], subscription["topics"], subscription["id"])
            subscriber_dict.pop("entities", None)
            subscriber_dict.pop("reference", None)
            url = "http://{}:{}/{}/contextSubscriptions/{}".format(CONTEXTBROKER["ADDRESS"], CONTEXTBROKER["PORT"], CONTEXTBROKER["PROTOCOL"], subscription["id"])
            subscriber_json = json.dumps(subscriber_dict)
            response_body = self._sendRequest(url, subscriber_json, 'PUT')
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

    def deleteEntity(self, namespace, data_type):
        print "DELETING: ", namespace, data_type
        operation_json = json.dumps({
            "contextElements": [
                {
                    "type": data_type,
                    "isPattern": "false",
                    "id": namespace
                }
            ],
            "updateAction": "DELETE"
        })
        url = "http://{}:{}/{}/updateContext".format(CONTEXTBROKER["ADDRESS"], CONTEXTBROKER["PORT"], CONTEXTBROKER["PROTOCOL"])
        response_body = self._sendRequest(url, operation_json)
        if "errorCode" in response_body:
            print "Error deleting entity"
            print response_body["errorCode"]["details"]
        elif "orionError" in response_body:
            print "Error deleting entity"
            print response_body["orionError"]["details"]
        else:
            print "Deleted entity " + namespace


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

    def _sendRequest(self, url, data, method=None):
        request = urllib2.Request(url, data, {'Content-Type': 'application/json', 'Accept': 'application/json'})
        if method is not None:
            request.get_method = lambda: method
        response = urllib2.urlopen(request)
        response_body = json.loads(response.read())
        response.close()
        return response_body

