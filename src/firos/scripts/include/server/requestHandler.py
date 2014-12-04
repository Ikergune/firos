import re
import cgi
import json
from urlparse import urlparse, parse_qs
from BaseHTTPServer import BaseHTTPRequestHandler

from include.ros.topicHandler import TopicHandler
from include.pubsub.pubSubFactory import SubscriberFactory

CloudSubscriber = SubscriberFactory.create()

TOPIC_TIMESTAMPS = {}

class RequestHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        path = urlparse("http://localhost" + self.path).path
        action = getAction(path, "GET")
        if action is not None:
            action["action"](self, action)
        else:
            self.send_response(200)
            self.send_header('Content-type','text/html')
            self.end_headers()
            self.wfile.write("GENERIC PAGE")
        return

    def do_POST(self):
        path = urlparse("http://localhost" + self.path).path
        action = getAction(path, "POST")
        if action is not None:
            action["action"](self, action)
        else:
            self.send_response(200)
            self.send_header('Content-type','text/html')
            self.end_headers()
            self.wfile.write("GENERIC PAGE")
        return

def pathParams(request, regexp):
    return list(re.match(regexp, urlparse("http://localhost" + request.path).path).groups())

def getParams(request):
    return parse_qs(urlparse("http://localhost" + request.path).query)

def postParams(request):
    ctype, pdict = cgi.parse_header(request.headers.getheader('content-type'))
    if ctype == 'multipart/form-data':
        return cgi.parse_multipart(request.rfile, pdict)
    elif ctype == 'application/x-www-form-urlencoded':
        length = int(request.headers.getheader('content-length'))
        return cgi.parse_qs(request.rfile.read(length), keep_blank_values=1)
    elif ctype == 'application/json':
        json_data = request.rfile.read()
        # print json_data
        return json.loads(json_data)
    else:
        return {}

def getAction(path, method):
    for route in MAPPER[method]:
        if re.search(route['regexp'], path):
            return route
    return None



###############################################################################
#############################   Request Mapping   #############################
###############################################################################

def onTopic(request, action):
    print "Context Broker Notification"
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
                if topic["name"] in commands:
                    value = CloudSubscriber.parseData(topic['value'])
                    if topic["name"] not in TOPIC_TIMESTAMPS[robotName] or TOPIC_TIMESTAMPS[robotName][topic["name"]] != value["firosstamp"]:
                        TopicHandler.publish(robotName, topic['name'], value)
                    TOPIC_TIMESTAMPS[robotName][topic["name"]] = value["firosstamp"]

    request.send_response(200)
    request.send_header('Content-type','text/plain')
    request.end_headers()
    request.wfile.write("Received by firos")

# URL structure
# ^/firos/(\w+)/update/*$
# ^/TEXT/whatever_is_inside/TEXT/+$

MAPPER = {
    "GET": [],
    "POST": [{"regexp": "^/firos/*$", "action": onTopic}],
    "PUT": [],
    "DELETE": [],
}