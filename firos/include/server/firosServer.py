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

from BaseHTTPServer import HTTPServer

from include.logger import Log

from include.server.requestHandler import RequestHandler


class FirosServer:
    ## \brief FIROS http server
    # \param self
    # \param ip address
    # \param port to listen to
    def __init__(self, address="0.0.0.0", port=8000):
        self.address = address
        self.port = port
        self.stopped = False

        Protocol = "HTTP/1.0"

        server_address = (self.address, self.port)

        RequestHandler.protocol_version = Protocol
        self.httpd = HTTPServer(server_address, RequestHandler)

    def start(self):
        ## \brief start FIROS http server
        # \param self
        sa = self.httpd.socket.getsockname()
        Log("INFO", "\nServing HTTP on", sa[0], "port", sa[1], "...")
        while not self.stopped:
            self.httpd.handle_request()

    def close(self):
        ## \brief stop FIROS http server
        # \param self
        self.stopped = True
