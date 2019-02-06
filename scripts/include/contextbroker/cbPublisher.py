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

import json
import requests

from include.logger import Log
from include.constants import Constants as C
from include.FiwareObjectConverter.objectFiwareConverter import ObjectFiwareConverter




class CbPublisher(object):
    ''' The CbPublisher handles the Enities on CONTEXT_BROKER / v2 / entities .
        It creates not creaed Entities and updates their attributes via 'publishToCB'.
        On Shutdown the tracked Entities are deleted. 

        Also the rawMsg is converted here via the Object Converter

        THIS IS THE ONLY FILE WHICH OPERATES ON /v2/entities 
    '''

    # Keeps track of the posted Content on the ContextBroker
    # via posted_history[ROBOT_ID][TOPIC] 
    posted_history = {}
    CB_HEADER = {'Content-Type': 'application/json'}
    CB_BASE_URL = None

    def __init__(self):
        ''' Lazy Initialization of CB_BASE_URL
        '''
        self.CB_BASE_URL = "http://{}:{}/v2/entities/".format(C.CONTEXTBROKER_ADRESS, C.CONTEXTBROKER_PORT)

    def publishToCB(self, robotID, topic, rawMsg, msgDefintionDict):
        ''' This is the actual publish-Routine which updates and creates Entities on the
            ContextBroker. It also keeps track via posted_history on already posted entities and topics

            robotID: A string corresponding to the Robot-Id
            topic:   Also a string, corresponding to the topic of the robot
            rawMsg:  the raw data directly obtained from rospy
            msgDefintionDict: The Definition as obtained FROM ros2Definition TODO DL 

            TODO DL During Runtime an Entitiy might get deleted, check it here!
            TODO DL set publish frequency
        '''
        # if struct not initilized, intitilize it even on ContextBroker!
        if robotID not in self.posted_history:
            self.posted_history[robotID] = {}
            self.posted_history[robotID]['type'] = 'ROBOT'
            self.posted_history[robotID]['id'] = robotID
            # Intitialize Entitiy/Robot-Construct on ContextBroker
            jsonStr = ObjectFiwareConverter.obj2Fiware(self.posted_history[robotID], ind=0,  ignorePythonMetaData=True) 
            response = requests.post(self.CB_BASE_URL, data=jsonStr, headers=self.CB_HEADER)
            self._responseCheck(response, attrAction=0, topEnt=robotID)

        if topic not in self.posted_history[robotID]:
            self.posted_history[robotID][topic] = {}

        # Check if descriptions are already added, if not execute again with descriptions!
        if 'descriptions' not in self.posted_history[robotID]:
            self.posted_history[robotID]['descriptions'] = self._loadDescriptions(robotID)
            if self.posted_history[robotID]['descriptions'] is not None:
                self.publishToCB(robotID, 'descriptions', self.posted_history[robotID]['descriptions'], None)

        # check if previous posted topic type is the same, iff not, we do not post it to the context broker
        if (self.posted_history[robotID][topic] != {} and topic != "descriptions"  
                and rawMsg._type != self.posted_history[robotID][topic]._type):
            Log("ERROR",  "Received Msg-Type '{}' but expected '{}' on Topic '{}'".format(rawMsg._type, self.posted_history[robotID][topic]._type, topic))
            return

        
        # Replace previous rawMsg with current one
        self.posted_history[robotID][topic] = rawMsg
        
        # Set Definition-Dict
        definitionDict = {}
        definitionDict[topic] = msgDefintionDict
        completeJsonStr = ObjectFiwareConverter.obj2Fiware(self.posted_history[robotID], ind=0, dataTypeDict=definitionDict,  ignorePythonMetaData=True) 

        # format json, so that the contextbroker accepts it.
        partJsonStr =  json.dumps({
            topic: json.loads(completeJsonStr)[topic]
            })


        # Update attribute on ContextBroker
        response = requests.post(self.CB_BASE_URL + robotID + "/attrs", data=partJsonStr, headers=self.CB_HEADER)
        self._responseCheck(response, attrAction=1, topEnt=topic)


    def unpublishALLFromCB(self):
        ''' Removes all previously tracked Entities/Robots on ContextBroker
        '''
        for robotID in self.posted_history:
            response = requests.delete(self.CB_BASE_URL + robotID)
            self._responseCheck(response, attrAction=2, topEnt=robotID)
        
        
    def _loadDescriptions(self, robotID):
        ''' Simply load the descriptions from the 'robotdescriptions.json'-file and 
            return its value

            robotID: The Robot-Id-String
        '''
        json_path = C.PATH + "/robotdescriptions.json"
        description_data = json.load(open(json_path))
        
        # Check if a robotID has descriptions
        if robotID in description_data:
            if 'descriptions' in description_data[robotID]:
                return description_data[robotID]['descriptions']

        return None


    def _responseCheck(self, response, attrAction=0, topEnt=None):
        ''' Check if Response is ok (2XX and some 3XX). If not print an individual Error.
            
            response: the actual response
            attrAction: One of [0, 1, 2]  which maps to -> [Creation, Update, Deletion]
            topEnt: the String of an Entity or a topic, which was used
        '''
        if not response.ok:
            if attrAction == 0:
                Log("WARNING", "Could not create Entitiy/Robot {} in Contextbroker :".format(topEnt))
                Log("WARNING", response.content)
            elif attrAction == 1:
                Log("ERROR", "Cannot update attributes in Contextbroker for topic: {} :".format(topEnt))
                Log("ERROR", response.content)
            else:
                Log("WARNING", "Could not delete Entitiy/Robot {} in Contextbroker :".format(topEnt))
                Log("WARNING", response.content)