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

import os
import copy

import json
import rospy
import requests

from include.logger import Log
from include.constants import DATA_CONTEXTBROKER
from include.FiwareObjectConverter.objectFiwareConverter import ObjectFiwareConverter

class CbPublisher(object):
    posted_history = {}

    def publishToCB(self, robotID, topic, datatype, rawMsg, msgDefintionDict):
        #TODO DL set publish frequency!
        # if struct not initilized, intitilize it even on ContextBroker!
        if robotID not in self.posted_history:
            self.posted_history[robotID] = {}
            self.posted_history[robotID]['type'] = 'ROBOT'
            self.posted_history[robotID]['id'] = robotID
            jsonStr = ObjectFiwareConverter.obj2Fiware(self.posted_history[robotID], ind=0,  ignorePythonMetaData=True) 
            # TODO Error Checking!
            url = "http://{}:{}/v2/entities".format(DATA_CONTEXTBROKER["ADDRESS"], DATA_CONTEXTBROKER["PORT"])
            response = requests.post(url, data=jsonStr, headers={'Content-Type': 'application/json'})

        if topic not in self.posted_history[robotID]:
            self.posted_history[robotID][topic] = {}

        # Check if descriptions are already added
        if 'descriptions' not in self.posted_history[robotID]:
            self.posted_history[robotID]['descriptions'] = self.loadDescriptions(robotID)

        # check if previous posted topic type is the same, iff not, we do not post it to the context broker
        if self.posted_history[robotID][topic] != {} and rawMsg._type != self.posted_history[robotID][topic]._type:
            Log("WARNING",  "Received Msg-Type '{}' but expected '{}' on Topic '{}'".format(rawMsg._type, self.posted_history[robotID][topic]._type, topic))
            return

        
        # Replace previous rawMsg with current one
        self.posted_history[robotID][topic] = rawMsg
        
        # Set Definition-Dict TODO DL do not rebuild every definition Dict!
        definitionDict = {}
        definitionDict[topic] = msgDefintionDict
        completeJsonStr = ObjectFiwareConverter.obj2Fiware(self.posted_history[robotID], ind=0, dataTypeDict=definitionDict,  ignorePythonMetaData=True) 

        # format json, so that the contextbroker accept it.
        partJsonStr =  json.dumps({
            topic: json.loads(completeJsonStr)[topic]
            })


        # TODO Error Checking!
        url = "http://{}:{}/v2/entities/{}/attrs".format(DATA_CONTEXTBROKER["ADDRESS"], DATA_CONTEXTBROKER["PORT"], robotID)
        response = requests.post(url, data=partJsonStr, headers={'Content-Type': 'application/json'})


    def unpublishALLFromCB(self):
        # Removes all on start-up created entities on shutdown
        puburl = "http://{}:{}/v2/entities/".format(DATA_CONTEXTBROKER["ADDRESS"], DATA_CONTEXTBROKER["PORT"])
        for robotID in self.posted_history:
            response = requests.delete(puburl + robotID)
            # TODO DL ERRORCHecking!
        
        
    def loadDescriptions(self, robotID):
        # TODO DL refactor and no hardcoded reference to file!
        current_path = os.path.dirname(os.path.abspath(__file__))
        json_path = current_path.replace("scripts/include/contextbroker", "config/robotdescriptions.json")
        description_data = json.load(open(json_path))
        
        if robotID in description_data:
            if 'descriptions' in description_data[robotID]:
                return description_data[robotID]['descriptions']

        return None