from BaseHTTPServer import BaseHTTPRequestHandler,HTTPServer

class CbHandler(BaseHTTPRequestHandler):
    t1 = "HOLA!!"
    def do_GET(self):
        print "PETICION"
        self.send_response(200)
        self.send_header('Content-type','text/html')
        self.end_headers()
        self.wfile.write(self.t1)
        return