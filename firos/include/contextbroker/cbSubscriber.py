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
import requests
import json
try:
    # Python 3
    import _thread as thread
except ImportError:
    # Pyrhon 2
    import thread

from include.constants import Constants as C
from include.logger import Log




class CbSubscriber(object):
    ''' The CbSubscriber handles the subscriptions on the ContextBroker.
        Only the url CONTEXT_BROKER / v2 / subcriptions  is used here!
        As on CbPublisher on shutdown all subscriptions are deleted form 
        ContextBroker. 

        this Objects also converts the received data from ContextBroker
        back into a Python-Object. 

        Here each topic of an robot is subscribed seperately.

        THIS IS THE ONLY FILE WHICH OPERATES ON /v2/subscriptions
    '''

    # Saves the subscriptions IDs returned from ContextBroker.
    # Follwoing Structure: subscriptionIds[ROBOT_ID][TOPIC] returns a sub-Id in String
    subscriptionIds = {}
    CB_BASE_URL = None
    FIROS_NOTIFY_URL = None
    def __init__(self):
        ''' Lazy Initialization of CB_BASE_URL and FIROS_NOTIFY_URL
        '''
        self.CB_BASE_URL = "http://{}:{}".format(C.CONTEXTBROKER_ADRESS, C.CONTEXTBROKER_PORT)
        self.FIROS_NOTIFY_URL = "http://{}:{}/firos".format(C.MAP_SERVER_ADRESS, C.MAP_SERVER_PORT)


    def subscribeToCB(self, robotID, topicList):
        ''' This method starts for each topic an own thread, which handles the subscription

            robotID: The string of the robotID
            topicList: A list of topics, corresponding to the robotID
        '''
        # If not already subscribed, start a new thread which handles the subscription for each topic for an robot.
        # And only If the topic list is not empty!
        if robotID not in self.subscriptionIds and topicList:
            Log("INFO", "Subscribing on Context-Broker to " + robotID + " and topics: " + str(list(topicList)))
            self.subscriptionIds[robotID] = {}
            for topic in topicList:
                thread.start_new_thread(self.subscribeThread, (robotID, topic)) #Start Thread via subscription         


    def unsubscribeALLFromCB(self):
        ''' Simply unsubscribed from all tracked subscriptions
        '''
        for robotID in self.subscriptionIds:
            for topic in self.subscriptionIds[robotID]:
                response = requests.delete(self.CB_BASE_URL + self.subscriptionIds[robotID][topic])
                self._checkResponse(response, subID=self.subscriptionIds[robotID][topic])


    def subscribeThread(self, robotID, topic):
        ''' A Subscription-Thread. It Life-Cycle is as follows:
            -> Subscribe -> Delete old Subs-ID -> Save new Subs-ID -> Wait ->

            robotID: A string corresponding to the robotID
            topic: The Topic (string) to subscribe to.
        '''
        while True:
            # Subscribe
            jsonData = self.subscribeJSONGenerator(robotID, topic)
            response = requests.post(self.CB_BASE_URL + "/v2/subscriptions", data=jsonData, headers={'Content-Type': 'application/json'})
            self._checkResponse(response, created=True, robTop=(robotID, topic))

            if 'Location' in response.headers:
                newSubID = response.headers['Location'] # <- get subscription-ID
            else:
                Log("WARNING",  "Firos was not able to subscribe to topic: {} for robot {}".format(topic, robotID))

            # Unsubscribe
            if robotID in self.subscriptionIds and topic in self.subscriptionIds[robotID]:
                response = requests.delete(self.CB_BASE_URL + self.subscriptionIds[robotID][topic])
                self._checkResponse(response, subID=self.subscriptionIds[robotID][topic])
                
            # Save new ID
            self.subscriptionIds[robotID][topic] = newSubID

            # Wait
            time.sleep(int(C.CB_SUB_LENGTH * C.CB_SUB_REFRESH)) # sleep Length * Refresh-Rate (where 0 < Refresh-Rate < 1)
            Log("INFO", "Refreshing Subscription for " + robotID + " and topic: " + str(topic))


    def subscribeJSONGenerator(self, robotID, topic):
        ''' This method returns the correct JSON-format to subscribe to the ContextBroker. 
            The Expiration-Date/Throttle and Type of robots is retreived here via configuration

            robotID: The String of the Robot-Id.
            topic: The actual topic to subscribe to.
        '''
        # This struct correspondes to following JSON-format:
        # https://fiware-orion.readthedocs.io/en/master/user/walkthrough_apiv2/index.html#subscriptions
        struct =  {
            "subject": {
                "entities": [
                    {
                    "id": str(robotID),
                    "type": C.CONTEXT_TYPE
                    }
                ],
                "condition": {
                    "attrs": [str(topic)]
                }
            },
            "notification": {
            "http": {
                "url": self.FIROS_NOTIFY_URL 
            },
            "attrs": [str(topic)]
            },
            "expires": time.strftime("%Y-%m-%dT%H:%M:%S.00Z", time.gmtime(time.time() + C.CB_SUB_LENGTH)), # ISO 8601
            "throttling": C.CB_THROTTLING  
            }
        return json.dumps(struct)


    def _checkResponse(self, response, robTop=None, subID=None, created=False):
        ''' If a not good response from ContextBroker is received, the error will be printed.
    
            response: The response from ContextBroker
            robTop:   A string Tuple (robotId, topic), for the curretn robot/topic
            subID:    The Subscription ID string, which should get deleted
            created:  Creation or Deletion of a subscription (bool)
        '''
        if not response.ok:
            if created:
                Log("ERROR", "Could not create subscription for Robot {} and topic {} in Context-Broker :".format(robTop[0], robTop[1]))
                Log("ERROR", response.content)
            else:
                Log("WARNING", "Could not delete subscription {} from Context-Broker :".format(subID))
                Log("WARNING", response.content)
