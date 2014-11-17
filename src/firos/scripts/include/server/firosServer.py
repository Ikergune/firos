from BaseHTTPServer import HTTPServer

from include.server.requestHandler import RequestHandler

class FirosServer:
    def __init__(self, address="127.0.0.1", port=8000):
        self.address = address
        self.port = port
        self.stopped = False

        Protocol     = "HTTP/1.0"

        server_address = (self.address, self.port)

        RequestHandler.protocol_version = Protocol
        self.httpd = HTTPServer(server_address, RequestHandler)

    def start(self):
        sa = self.httpd.socket.getsockname()
        print "Serving HTTP on", sa[0], "port", sa[1], "..."
        while not self.stopped:
            self.httpd.handle_request()

    def close(self):
        self.stopped = True