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
import time
import thread
import signal
import urllib2

from include.constants import *
from include.logger import Log
from include.pubsub.iPubSub import Isubscriber
from include.pubsub.contextbroker.ngsi9 import registerContext, deleteContext, deleteAllContexts, refreshAllContexts


class CbSubscriber(Isubscriber):
    ## \brief Context broker subscription handler
    subscriptions = {}
    refresh_thread = None

    def subscribe(self, namespace, data_type, robot):
        ## \brief Subscribe to entities' changes
        # \param entity name
        # \param entity type
        # \param robot object
        if namespace not in self.subscriptions:
            registerContext(namespace, data_type, robot)
            topics = robot["publisher"].keys()
            Log("INFO", "Subscribing on context broker to " + data_type + " " + namespace + " and topics: " + str(topics))
            subscription = {
                "namespace": namespace,
                "data_type": data_type,
                "topics": topics
            }
            url = "http://{}:{}/NGSI10/subscribeContext".format(DATA_CONTEXTBROKER["ADDRESS"], DATA_CONTEXTBROKER["PORT"])
            subscriber_json = json.dumps(self._generateSubscription(namespace, data_type, topics))
            response_body = self._sendRequest(url, subscriber_json)
            if response_body is not None:
                if "subscribeError" in response_body:
                    Log("ERROR", "Error Subscribing to Context Broker:")
                    Log("ERROR", response_body["subscribeError"]["errorCode"]["details"])
                    os.kill(os.getpid(), signal.SIGINT)
                else:
                    subscription["id"] = response_body["subscribeResponse"]["subscriptionId"]
                    self.subscriptions[namespace] = subscription
                    Log("INFO", "Connected to Context Broker with id {}".format(subscription["id"]))
            if self.refresh_thread is None:
                self.refresh_thread = thread.start_new_thread(self._refreshSubscriptions, ("CBSub-Refresh", 2, ))

    def disconnect(self, namespace, delete=False):
        ## \brief Delete subscription by namespace
        # \param entity name
        # \param flag to indicate if the subscription must be deleted locally (False by default)
        if namespace in self.subscriptions:
            deleteContext(namespace, True)
            subscription = self.subscriptions[namespace]
            subscriptionId = subscription["id"]
            Log("INFO", "\nDisconnecting Context Broker subscription {}".format(subscriptionId))
            url = "http://{}:{}/NGSI10/unsubscribeContext".format(DATA_CONTEXTBROKER["ADDRESS"], DATA_CONTEXTBROKER["PORT"])
            disconnect_json = json.dumps({
                "subscriptionId": subscriptionId
            })
            response_body = self._sendRequest(url, disconnect_json)
            if response_body is not None:
                if int(response_body["statusCode"]["code"]) >= 400:
                    Log("ERROR", "Error Disconnecting from Context Broker (subscription: {}):".format(subscriptionId))
                    Log("ERROR", response_body["statusCode"]["reasonPhrase"])
                    Log("INFO", "\n")
                else:
                    Log("INFO", "Disconnected subscription {} from Context Broker ".format(subscriptionId))

            Log("INFO", "Deleting entity")
            self.deleteEntity(subscription["namespace"], subscription["data_type"])
            Log("INFO", "\n")
            if delete:
                del self.subscriptions[namespace]

    def disconnectAll(self):
        ## \brief Delete all subscriptions
        deleteAllContexts()
        for subscription in self.subscriptions:
            self.disconnect(subscription)

    def refreshSubscriptions(self):
        ## \brief Refresh exisiting subscriptions on context broker
        refreshAllContexts()
        for subscription in self.subscriptions.values():
            subscriber_dict = self._generateSubscription(subscription["namespace"], subscription["data_type"], subscription["topics"], subscription["id"])
            subscriber_dict.pop("entities", None)
            subscriber_dict.pop("reference", None)
            url = "http://{}:{}/NGSI10/contextSubscriptions/{}".format(DATA_CONTEXTBROKER["ADDRESS"], DATA_CONTEXTBROKER["PORT"], subscription["id"])
            subscriber_json = json.dumps(subscriber_dict)
            response_body = self._sendRequest(url, subscriber_json, 'PUT')
            if response_body is not None:
                if "subscribeError" in response_body:
                    Log("ERROR", "Error Refreshing subscription")
                    Log("ERROR", response_body["subscribeError"]["errorCode"]["details"])
                elif "orionError" in response_body:
                    Log("ERROR", "Error Refreshing subscription")
                    Log("ERROR", response_body["orionError"]["details"])
                else:
                    Log("INFO", "Refreshed Connection to Context Broker with id {}".format(subscription["id"]))

    def parseData(self, data):
        ## \brief Parse the received data
        # \param data
        # print data
        return json.loads(data.replace(SEPARATOR_CHAR, '"'))

    def deleteEntity(self, namespace, data_type, removeContext=True):
        ## \brief Delete entity from context broker
        # \param entity name
        # \param entity type
        Log("INFO", "DELETING: ", namespace, data_type)
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
        url = "http://{}:{}/NGSI10/updateContext".format(DATA_CONTEXTBROKER["ADDRESS"], DATA_CONTEXTBROKER["PORT"])
        response_body = self._sendRequest(url, operation_json)
        if response_body is not None:
            if "errorCode" in response_body:
                Log("ERROR", "Error deleting entity")
                Log("ERROR", response_body["errorCode"]["details"])
            elif "orionError" in response_body:
                Log("ERROR", "Error deleting entity")
                Log("ERROR", response_body["orionError"]["details"])
            else:
                Log("INFO", "Deleted entity " + namespace)

        if removeContext:
            deleteContext(namespace, True)

    def _generateSubscription(self, namespace, data_type=DEFAULT_CONTEXT_TYPE, topics=[], subscriptionId=None):
        ## \brief Generate subscription message
        # \param entity name
        # \param entity type
        # \param entity's topics
        data = {
            "entities": [
                {
                    "type": data_type,
                    "isPattern": "false",
                    "id": namespace
                }
            ],
            # "attributes": topics,
            "reference": "http://{}:{}/firos".format(IP, SERVER_PORT),
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
        ## \brief Thread handler for subscripiton refresh
        # \param theradname
        # \param delay time

        # Seconds to days
        total_delay = SUBSCRIPTION_REFRESH_DELAY * 60 * 60 * 24
        while True:
            time.sleep(total_delay)
            self.refreshSubscriptions()

    def _sendRequest(self, url, data, method=None):
        ## \brief Send request to context broker
        # \param url to request to
        # \param data to send
        # \param HTTP method (GET by default)
        try:
            request = urllib2.Request(url, data, {'Content-Type': 'application/json', 'Accept': 'application/json'})
            if method is not None:
                request.get_method = lambda: method
            response = urllib2.urlopen(request)
            response_body = json.loads(response.read())
            response.close()
            return response_body
        except Exception as ex:
            Log("ERROR", ex.reason)
            return None
