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

__author__ = "Dominik Lux"
__credits__ = ["Peter Detzner"]
__maintainer__ = "Dominik Lux"
__version__ = "0.0.1a"
__status__ = "Developement"

import time
import thread
import requests

from include.FiwareObjectConverter.objectFiwareConverter import ObjectFiwareConverter

from include.constants import *
from include.logger import Log


class CbSubscriber(object):
    ## \brief Context broker subscription handler
    subscriptions = {} # TODO  DL remove?
    subscriptionThreads = {}
    subscriptionIds = {}
    refresh_thread = None

    def subscribeToCB(self, robotID, topic):
        # If not already subscribed, start a new thread which handles the subscription
        # And only If the topic list is not empty!
        if robotID not in self.subscriptions and topic:
            Log("INFO", "Subscribing on context broker to " + robotID + " and topics: " + str(topic))
            self.subscriptionThreads[robotID] = thread.start_new_thread(self.subscribeThread, (robotID, topic))
            #Start Thread via subscription


    def unsubscribeALLFromCB(self):
        suburl = "http://{}:{}".format(DATA_CONTEXTBROKER["ADDRESS"], DATA_CONTEXTBROKER["PORT"])
        for robotID in self.subscriptionIds:
            response = requests.delete(suburl + self.subscriptionIds[robotID])
        # TODO DL Error Handling!


    def subscribeThread(self, robotID, topicList):
        # Subscribe New, Unsubscribe old and sleep. If time has passed re-do!
        suburl = "http://{}:{}".format(DATA_CONTEXTBROKER["ADDRESS"], DATA_CONTEXTBROKER["PORT"])
        
        while True:
            # Subscribe
            jsonData = self.subscribeJSONGenerator(robotID, topicList)
            response = requests.post(suburl + "/v2/subscriptions", data=jsonData, headers={'Content-Type': 'application/json'})
            newSubID = response.headers['Location'] # <- get subscription-ID

            # Unsubscribe
            if robotID in self.subscriptionIds:
                response = requests.delete(suburl + self.subscriptionIds[robotID])
            self.subscriptionIds[robotID] = newSubID

            #Sleep
            time.sleep(290) # sleep TODO DL from config loaded seconds
            Log("INFO", "Refreshing Subscription for " + robotID + " and topics: " + str(topicList))
            
            
            # TODO DL Error Handling!



    def subscribeJSONGenerator(self, robotID, topicList):
        # This method generates the JSON for the 'v2/subscriptions'-API of Orion CB
        struct =  {
            "subject": {
                "entities": [
                    {
                    "id": str(robotID),
                    "type": "ROBOT"  # Type is currently always a robot! #TODO DL load from cfg 
                    }
                ],
                "condition": {
                    "attrs": topicList
                }
            },
            "notification": {
            "http": {
                "url": "http://{}:{}/firos".format(IP, SERVER_PORT) # TODO DL HTTP?
            },
            "attrs": topicList
            },
            "expires": time.strftime("%Y-%m-%dT%H:%M:%S.00Z", time.gmtime(time.time() + 300)) # TODO DL set expire in Config, ISO 8601,  300 secs
            # "throttling": 5  # TODO DL Maybe throttle?
            }
        return json.dumps(struct)



    def receivedData(self, robotID, topic, jsonData):
    # parse the Input and let the TopicHandler (TODO DL remove from there)
    # publish the data back into ROS
        kv = self.TypeValue()
        ObjectFiwareConverter.fiware2Obj(jsonData, kv, setAttr=True, useMetadata=False)
        return kv

    ## DL - Back Parsing Stub Class
    class TypeValue(object):
        def __init__(self):
            pass

    # TODO DL remove wrapper!
    def subscribe(self, namespace, data_type, robot):
        # unused -> data_type
        self.subscribeToCB(namespace, robot["publisher"].keys())
