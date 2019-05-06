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

import re
import cgi
import json
import requests
try:
    # Python 3
    from urllib.parse import urlparse, parse_qs
    from http.server import BaseHTTPRequestHandler
    isPython3 = True

except ImportError:
    # Python 2
    from BaseHTTPServer import BaseHTTPRequestHandler
    from urlparse import urlparse, parse_qs
    isPython3 = False

from include.logger import Log
from include.confManager import getRobots
from include.ros.rosConfigurator import RosConfigurator, setWhiteList
from include.ros.topicHandler import RosTopicHandler, loadMsgHandlers, ROS_PUBLISHER, ROS_SUBSCRIBER, ROS_TOPIC_AS_DICT
from include.contextbroker.cbSubscriber import CbSubscriber
from include.constants import Constants as C 
from include.FiwareObjectConverter.objectFiwareConverter import ObjectFiwareConverter


class RequestHandler(BaseHTTPRequestHandler):
    ''' This is the FIROS-HTTP-Request-Handler. It is needed,
        because the ContextBroker sends Information about the
        subscriptions via HTTP. This Class just handles incoming 
        Requests and deligates them further to the specific. Firos
        allows some extra operations here like Connect and Disconnect.
    '''
    def do_GET(self):
        ''' Case: only a GET Request
        '''
        path = urlparse("http://localhost" + self.path).path
        action = getAction(path, "GET")
        if action is not None:
            action["action"](self, action)
        else:
            end_request(self, ('Content-type', 'text/html'), 200, "Firos is running!")
        return

    def do_POST(self):
        ''' Case: only a POST Requst
        '''
        path = urlparse("http://localhost" + self.path).path
        action = getAction(path, "POST")
        if action is not None:
            action["action"](self, action)
        else:
            end_request(self, ('Content-type', 'text/html'), 200, "Firos is running!")
        return


def getPostParams(request):
    ''' Returns from the given request its parameters which were 
        posted prior. 
    '''
    ctype, pdict = cgi.parse_header(request.headers.get('content-type'))
    if ctype == 'multipart/form-data':
        return cgi.parse_multipart(request.rfile, pdict)
    elif ctype == 'application/x-www-form-urlencoded':
        length = int(request.headers.getheader('content-length'))
        return cgi.parse_qs(request.rfile.read(length), keep_blank_values=1)
    elif ctype == 'application/json':
        json_data = request.rfile.read(int(request.headers['Content-Length']))

        return json.loads(json_data)
    else:
        return {}


def getAction(path, method):
    ## \brief URL checker to find what action to execute
    # \param url string
    # \param HTTP method
    for route in MAPPER[method]:
        if re.search(route['regexp'], path):
            return route
    return None


###############################################################################
#############################   Request Mapping   #############################
###############################################################################


def requestFromCB(request, action):
    ''' The ContextBroker is informing us via one of our subscriptions.
        We convert the received content back with the cbSubscriber and publish 
        it in ROS.

        request: The request from Context-Broker
        action: Here it is unused
    '''
    # retreive Data and get the updated information
    receivedData = getPostParams(request)
    data = receivedData['data'][0] # Specific to NGSIv2 
    jsonData = json.dumps(data)
    topics = data.keys() # Convention Topic-Names are the attributes by an JSON Object, except: type, id

    # iterate through every 'topic', since we only receive updates from one topic
    # Only id, type and 'topicname' are present
    for topic in topics:
        if topic != 'id' and topic != 'type':
            dataStruct = buildTypeStruct(data[topic])
            obj = convertReceivedDataFromCB(jsonData)
            # Publish in ROS
            RosTopicHandler.publish(data['id'], topic, getattr(obj, topic), dataStruct)

    # Send OK!
    request.send_response(204)


def listRobots(request, action):
    ''' Generates a list of all robots (depending on RosConfigurator, confManager)
        and returns them back as json

        TODO DL, currently a List of containing 'topics' with a list of topics is returned
        Better would be a list of robotIds with their corersponding topics and types
    '''
    robots = getRobots(False)
    data = []
    for robot_name in robots.keys():
        robot_data = {"name": robot_name, "topics": []}
        robot = robots[robot_name]
        for topic_name in robot["topics"]:
            topic = robot["topics"][topic_name]
            topic_data = {
                "name": topic_name,
                "pubsub": topic["type"]
            }
            if type(topic["msg"]) is dict:
                topic_data["type"] = "Custom"
                topic_data["structure"] = topic["msg"]
            else:
                topic_data["type"] = topic["msg"]
                topic_data["structure"] = ROS_TOPIC_AS_DICT[topic_name]
            robot_data["topics"].append(topic_data)
        data.append(robot_data)

    # Return data and success
    end_request(request, ('Content-Type', 'application/json'), 200, json.dumps(data))


def onRobotData(request, action):
    ''' Returns the actual Content of the Context-Broker onto
        the page. Here we only query the ContextBroker, No Manipulation
        is done here.

        Depending what is written after 'robot', we just map it to /v2/entities/XXXXX
        and also return the same status_code
    '''
    partURL =request.path[7:] # Depends on prefix '/robot/'

    # Only used to query information from Context-Broker
    cb_base_url = "http://{}:{}/v2/entities/".format(C.CONTEXTBROKER_ADRESS, C.CONTEXTBROKER_PORT)
    response = requests.get(cb_base_url + partURL)

    # Return the Information provided by the Context-Broker
    end_request(request, ('Content-Type', 'application/json'), response.status_code, response.text)


def onConnect(request, action):
    ''' This resets firos into its original state

        TODO DL reset, instead of connect?
        TODO DL Add real connect for only one Robot?
    '''
    Log("INFO", "Connecting robots")
    loadMsgHandlers(RosConfigurator.systemTopics(True))

    # Return Success
    end_request(request, None, 200, "")


def onDisConnect(request, action):
    ''' Removes the robot specified via url like
        '/disconnect/ROBOT_ID' from ROS-Publisher and 
        Ros-Subscriber

        We only are here when the URl is like:
        '/robot/disconnect/ROBOT_ID'

        TODO DL RosConfigurator is here used?
    '''
    partURL = request.path
    # If at the end is a slash we remove it simply
    if "/" is partURL[-1]:
        partURL = partURL[:-1]

    # Get ROBOT_ID, which is the last element
    robotID = partURL.split("/")[-1]


    Log("INFO", "Disconnecting robot '{}'".format(robotID))
    # Iterate through every topic and unregister, then delete it
    if robotID in ROS_PUBLISHER:
        for topic in ROS_PUBLISHER[robotID]:
            ROS_PUBLISHER[robotID][topic].unregister()
        del ROS_PUBLISHER[robotID]
        RosConfigurator.removeRobot(robotID)
    
    if robotID in ROS_SUBSCRIBER:
        for topic in ROS_SUBSCRIBER[robotID]:
            ROS_SUBSCRIBER[robotID][topic].unregister()
        del ROS_SUBSCRIBER[robotID]
        RosConfigurator.removeRobot(robotID)
    
    # Return success
    end_request(request, None, 200, "")



### The below Operations are no longer maintained.
def onWhitelistWrite(request, action):
    data = getPostParams(request)
    setWhiteList(data, None)
    end_request(request, None, 200, "")


def onWhitelistRemove(request, action):
    data = getPostParams(request)
    setWhiteList(None, data)
    end_request(request, None, 200, "")


def onWhitelistRestore(request, action):
    setWhiteList(None, None, True)
    end_request(request, None, 200, "")
### The above Operations are no longer maintained



# Mapper to the methods 
MAPPER = {
    "GET": [
        {"regexp": "^/robots/*$", "action": listRobots},
        {"regexp": "^/robot/.*$", "action": onRobotData}],
    "POST": [
        {"regexp": "^/firos/*$", "action": requestFromCB},
        {"regexp": "^/robot/connect/*$", "action": onConnect},
        {"regexp": "^/robot/disconnect/(\w+)/*$", "action": onDisConnect},
        {"regexp": "^/whitelist/write/*$", "action": onWhitelistWrite},
        {"regexp": "^/whitelist/remove/*$", "action": onWhitelistRemove},
        {"regexp": "^/whitelist/restore/*$", "action": onWhitelistRestore}]
}


def end_request(request, header, status, content):
    '''
        Ends the request via the statuscode, one header, end_headers and its content
    '''
    request.send_response(status)
    if header is not None:
        request.send_header(header[0], header[1])
    request.end_headers()
    if isPython3:
        request.wfile.write(bytes(content, "utf-8"))
    else:
        request.wfile.write(bytes(content))


### Back Conversion From Entity-JSON into Python-Object
def buildTypeStruct(obj):
    ''' This generates a struct containing a type (the actual ROS-Message-Type) and 
        its value (either empty or more ROS-Message-Types).

        This struct is used later to recursivley load needed Messages and fill them with 
        content before they are posted back to ROS.

        obj:    The received update from Context-Broker
    '''
    s = {}

    # Searching for a point to get ROS-Message-Types from the obj, see Fiware-Object-Converter
    if 'value' in obj and 'type' in obj and "." in obj['type'] : 
        s['type'] = obj['type']
        objval = obj['value'] 
        s['value'] = {}

        # For each value in Object repeat!
        for k in objval:
            if 'type' in objval[k] and 'value' in objval[k] and objval[k]['type'] == 'array': # Check if we got an Array-Type value
                l = []
                for klist in objval[k]['value']:
                    l.append(buildTypeStruct(klist))
                s['value'][k] = l
            else:
                s['value'][k] = buildTypeStruct(objval[k])

    return s


def convertReceivedDataFromCB(jsonData):
    ''' This parses the Input Back into a TypeValue-object via the 
        Object-Converter. This method is here to uniform the Obejct-Conversions 
        in CbPublisher and CbSubscriber

        topic:    The topic, which should be converted. 
                    the topic should have "id", "type" and "TOPIC" in it
    '''
    kv = TypeValue()
    ObjectFiwareConverter.fiware2Obj(jsonData, kv, setAttr=True, useMetaData=False)

    return kv

class TypeValue(object):
    ''' A Stub-Object to parse the received data
    '''