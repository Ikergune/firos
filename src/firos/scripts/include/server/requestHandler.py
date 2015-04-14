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

import re
import cgi
import json
from urlparse import urlparse, parse_qs
from BaseHTTPServer import BaseHTTPRequestHandler

from include.logger import Log
from include.constants import SEPARATOR_CHAR, DEFAULT_CONTEXT_TYPE
from include.confManager import getRobots
from include.ros.rosutils import ros2Definition
from include.ros.rosConfigurator import RosConfigurator, setWhiteList
from include.ros.topicHandler import TopicHandler, loadMsgHandlers, ROBOT_TOPICS
from include.pubsub.pubSubFactory import SubscriberFactory, QueryBuilderFactory

CloudSubscriber = SubscriberFactory.create()
CloudQueryBulder = QueryBuilderFactory.create()

TOPIC_TIMESTAMPS = {}


class RequestHandler(BaseHTTPRequestHandler):
    ## \brief FIROS http request handler
    def do_GET(self):
        ## \brief GET request handler
        # \param self
        path = urlparse("http://localhost" + self.path).path
        action = getAction(path, "GET")
        if action is not None:
            action["action"](self, action)
        else:
            self.send_response(200)
            self.send_header('Content-type', 'text/html')
            self.end_headers()
            self.wfile.write("GENERIC PAGE")
        return

    def do_POST(self):
        ## \brief POST request handler
        # \param self
        path = urlparse("http://localhost" + self.path).path
        action = getAction(path, "POST")
        if action is not None:
            action["action"](self, action)
        else:
            self.send_response(200)
            self.send_header('Content-type', 'text/html')
            self.end_headers()
            self.wfile.write("GENERIC PAGE")
        return


def pathParams(request, regexp):
    ## \brief Get url path params
    # \param client request to parse
    # \param regexp to use
    return list(re.match(regexp, urlparse("http://localhost" + request.path).path).groups())


def getParams(request):
    ## \brief GET requests' param parser
    # \param client request
    return parse_qs(urlparse("http://localhost" + request.path).query)


def postParams(request):
    ## \brief POST requests' param parser
    # \param client request
    ctype, pdict = cgi.parse_header(request.headers.getheader('content-type'))
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


def onTopic(request, action):
    ## \brief Handle topic reception
    # \param client request
    # \param action
    try:
        contexts = postParams(request)
        contexts = contexts['contextResponses']
        for context in contexts:
            if context['statusCode']['code'] == "200":
                robot = context['contextElement']
                robotName = robot['id']
                if robotName not in TOPIC_TIMESTAMPS:
                    TOPIC_TIMESTAMPS[robotName] = {}
                for topic in robot['attributes']:
                    if topic["name"] == "COMMAND":
                        commands = topic["value"]
                        robot['attributes'].remove(topic)
                        break
                for topic in robot['attributes']:
                    if topic["name"] != "descriptions" and topic["name"] in commands:
                        value = CloudSubscriber.parseData(topic['value'])
                        if (topic["name"] not in TOPIC_TIMESTAMPS[robotName]) or ("firosstamp" in value and TOPIC_TIMESTAMPS[robotName][topic["name"]] != value["firosstamp"]) or ("firosstamp" not in value):
                            if "firosstamp" in value:
                                TOPIC_TIMESTAMPS[robotName][topic["name"]] = value["firosstamp"]
                                # value.remove(value["firosstamp"])
                            TopicHandler.publish(robotName, topic['name'], value)
    except Exception as e:
        Log("ERROR", e)
    request.send_response(200)
    request.send_header('Content-type', 'text/plain')
    request.end_headers()
    request.wfile.write("Received by firos")


def onRobots(request, action):
    ## \brief Handle robot list request
    # \param client request
    # \param action
    robots = getRobots(False, True)
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
                topic_data["structure"] = ros2Definition(ROBOT_TOPICS[robot_name][topic["type"]][topic_name]["class"]())
            robot_data["topics"].append(topic_data)
        data.append(robot_data)
    request.send_response(200)
    request.send_header('Content-type', 'application/json')
    setCors(request)
    request.end_headers()
    request.wfile.write(json.dumps(data))


def onRobotData(request, action):
    ## \brief Handle robot info request
    # \param client request
    # \param action
    robot_name = pathParams(request, action["regexp"])[0]
    data = CloudQueryBulder.findById(robot_name, "ROBOT", False)
    if "errorCode" in data:
        request.send_response(int(data["errorCode"]["code"]))
    else:
        request.send_response(200)
        robot_list = []
        for context in data["contextResponses"]:
            for attribute in context["contextElement"]["attributes"]:
                if attribute["name"] != "COMMAND" and attribute["name"] != "descriptions":
                    print attribute["name"]
                    print attribute["value"]
                    attribute["value"] = json.loads(attribute["value"].replace(SEPARATOR_CHAR, '"'))
            context["contextElement"].pop("isPattern", None)
            robot_list.append(context["contextElement"])
        data = robot_list

    request.send_header('Content-type', 'application/json')
    setCors(request)

    request.end_headers()
    request.wfile.write(json.dumps(data))


def onConnect(request, action):
    ## \brief Launch robot search and connection
    # \param client request
    # \param action
    Log("INFO", "Connecting robots")
    loadMsgHandlers(RosConfigurator.systemTopics(True))
    request.send_response(200)
    request.end_headers()
    request.wfile.write("")


def onDisConnect(request, action):
    ## \brief Disconnect robot
    # \param client request
    # \param action
    robot_name = pathParams(request, action["regexp"])[0]
    Log("INFO", "Disconnecting robot" + robot_name)
    if robot_name in ROBOT_TOPICS:
        CloudSubscriber.deleteEntity(robot_name, DEFAULT_CONTEXT_TYPE)
        CloudSubscriber.disconnect(robot_name, True)
        for topic in ROBOT_TOPICS[robot_name]["publisher"]:
            ROBOT_TOPICS[robot_name]["publisher"][topic]["publisher"].unregister()
        for topic in ROBOT_TOPICS[robot_name]["subscriber"]:
            ROBOT_TOPICS[robot_name]["subscriber"][topic]["subscriber"].unregister()
        RosConfigurator.removeRobot(robot_name)
    request.send_response(200)
    request.end_headers()
    request.wfile.write("")


def onWhitelistWrite(request, action):
    ## \brief Handle robot info request
    # \param client request
    # \param action
    data = postParams(request)
    setWhiteList(data, None)
    request.send_response(200)
    request.end_headers()
    request.wfile.write("")


def onWhitelistRemove(request, action):
    ## \brief Handle robot info request
    # \param client request
    # \param action
    data = postParams(request)
    setWhiteList(None, data)
    request.send_response(200)
    request.end_headers()
    request.wfile.write("")


def onWhitelistRestore(request, action):
    ## \brief Handle robot info request
    # \param client request
    # \param action
    setWhiteList(None, None, True)
    request.send_response(200)
    request.end_headers()
    request.wfile.write("")


# URL structure
# ^/firos/(\w+)/update/*$
# ^/TEXT/whatever_is_inside/TEXT/+$

MAPPER = {
    "GET": [
        {"regexp": "^/robots/*$", "action": onRobots},
        {"regexp": "^/robot/(\w+)/*$", "action": onRobotData}],
    "POST": [
        {"regexp": "^/firos/*$", "action": onTopic},
        {"regexp": "^/robot/connect/*$", "action": onConnect},
        {"regexp": "^/robot/disconnect/(\w+)/*$", "action": onDisConnect},
        {"regexp": "^/whitelist/write/*$", "action": onWhitelistWrite},
        {"regexp": "^/whitelist/remove/*$", "action": onWhitelistRemove},
        {"regexp": "^/whitelist/restore/*$", "action": onWhitelistRestore}],
    "PUT": [],
    "DELETE": [],
}


def setCors(request):
    # \brief Set CORS headers in request
    # \param client request
    request.send_header("Access-Control-Allow-Credentials", True)
    request.send_header("Access-Control-Allow-Headers", "api-version, content-length, content-md5, content-type, date, request-id, response-time")
    request.send_header("Access-Control-Allow-Methods", "GET,POST,PUT,DELETE")
    request.send_header("Access-Control-Expose-Headers", "api-version, content-length, content-md5, content-type, date, request-id, response-time")
    request.send_header("Access-Control-Allow-Origin", "*")
