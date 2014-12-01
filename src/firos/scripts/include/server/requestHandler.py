import cgi
import json
from urlparse import urlparse
from BaseHTTPServer import BaseHTTPRequestHandler

from include.ros.topicHandler import TopicHandler
from include.pubsub.pubSubFactory import SubscriberFactory

CloudSubscriber = SubscriberFactory.create()

class RequestHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        path = urlparse("http://localhost" + self.path).path
        if path in MAPPER["GET"]:
            MAPPER["GET"][path](self)
        else:
            self.send_response(200)
            self.send_header('Content-type','text/html')
            self.end_headers()
            self.wfile.write("GENERIC PAGE")
        return

    def do_POST(self):
        path = urlparse("http://localhost" + self.path).path
        if path in MAPPER["POST"]:
            MAPPER["POST"][path](self)
        else:
            self.send_response(200)
            self.send_header('Content-type','text/html')
            self.end_headers()
            self.wfile.write("GENERIC PAGE")
        return

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


###############################################################################
#############################   Request Mapping   #############################
###############################################################################

def onTopic(request):
    print "Context Broker Notification"
    contexts = postParams(request)
    contexts = contexts['contextResponses']
    for context in contexts:
        if context['statusCode']['code'] == "200":
            robot = context['contextElement']
            robotName = robot['id']
            for topic in robot['attributes']:
                if topic["name"] == "COMMAND":
                    commands = topic["value"]
                    robot['attributes'].remove(topic)
                    break
            for topic in robot['attributes']:
                if topic["name"] in commands:
                    value = CloudSubscriber.parseData(topic['value'])
                    TopicHandler.publish(robotName, topic['name'], value)

    request.send_response(200)
    request.send_header('Content-type','text/plain')
    request.end_headers()
    request.wfile.write("Received by firos")

MAPPER = {
    "GET": {
    },
    "POST": {
        "/firos": onTopic
    },
    "PUT": {
    },
    "DELETE": {
    },
}