from urlparse import urlparse
from BaseHTTPServer import BaseHTTPRequestHandler

class RequestHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        print "Empieza"
        path = urlparse("http://localhost" + self.path).path
        print path
        if path in MAPPER["GET"]:
            MAPPER["GET"][path](self)
        else:
            self.send_response(200)
            self.send_header('Content-type','text/html')
            self.end_headers()
            self.wfile.write("GENERIC PAGE")
        return

def onTopic(request):
    print "Context Broker Notification"
    request.send_response(200)
    request.send_header('Content-type','text/html')
    request.end_headers()
    request.wfile.write("FIROS")

MAPPER = {
    "GET": {
        "/firos": onTopic
    },
    "POST": {
    },
    "PUT": {
    },
    "DELETE": {
    },
}