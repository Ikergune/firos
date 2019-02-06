#!/usr/bin/env python

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

# Import required Python code.
import os
import getpass
import subprocess
from threading import Thread

from include.constants import Constants as C

MAP_SERVER_PROCESS = None

class MapServer:
    def __init__(self):
        ## \brief Starts Map server thread
        self.mapserver_path = os.path.dirname(os.path.abspath(__file__)).replace("include", "")
        self.start_command = "node mapserver.js " + str(C.MAP_SERVER_PORT) + " " + str(C.ROSBRIDGE_PORT)

        MapThread = Thread(target=self._launchMapServer)
        MapThread.daemon = True
        MapThread.start()


    def _launchMapServer(self):
        # TODO DL can this be omitted?
        ## \brief If map_server is configured launches it
        if(C.MAP_SERVER_PORT):
            # if not os.path.exists(os.path.join(self.mapserver_path, 'node_modules')):
            #     text = "---------------------------------------------------------------------------------------\n"
            #     text += "---------------------------------------------------------------------------------------\n"
            #     text += "FIROS is going to install mapserver's dependencies, to do this it will need root access\n"
            #     text += "---------------------------------------------------------------------------------------\n"
            #     text += "---------------------------------------------------------------------------------------"
            #     print text
            #     os.system("cd {} && sudo npm install && sudo chown -R {} node_modules".format(self.mapserver_path, getpass.getuser()))
            # subprocess.Popen(["node", self.mapserver_path + "mapserver.js", str(C.MAP_SERVER_PORT), str(C.ROSBRIDGE_PORT)])
            pass
