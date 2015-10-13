import os
import subprocess
from threading import Thread
from include.constants import *

BASE_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__)))
COAP_AGENT_DIR = os.path.join(BASE_DIR, 'lightweightm2m-iotagent')
CONFIG_FILE_DIR = os.path.join(COAP_AGENT_DIR, 'new_config.js')

class CoapManager(object):
    def __init__(self):
        self.generateConfigFile()
        self.launchAgent()

    def generateConfigData(self):
        return """var config = {};

config.lwm2m = {
    logLevel: 'DEBUG',
    port: 60001,
    defaultType: 'Device',
    ipProtocol: 'udp4',
    serverProtocol: 'udp4',
    formats: [
        {
            name: 'application-vnd-oma-lwm2m/text',
            value: 1541
        },
        {
            name: 'application-vnd-oma-lwm2m/tlv',
            value: 1542
        },
        {
            name: 'application-vnd-oma-lwm2m/json',
            value: 1543
        },
        {
            name: 'application-vnd-oma-lwm2m/opaque',
            value: 1544
        }
    ],
    writeFormat: 'application-vnd-oma-lwm2m/text',
    types: [ ]
};

config.ngsi = {
    logLevel: 'DEBUG',
    contextBroker: {
        host: 'CB_HOST',
        port: 'CB_PORT'
    },
    server: {
        port: AGENT_PORT
    },
    deviceRegistry: {
        type: 'mongodb',
        host: 'localhost'
    },
    types: { },
    service: 'smartGondor',
    subservice: '/gardens',
    providerUrl: 'http://FIROS_HOST:AGENT_PORT',
    deviceRegistrationDuration: 'P1M'
};

module.exports = config;""".replace('CB_HOST', str(DATA_CONTEXTBROKER['ADDRESS']))\
            .replace('CB_PORT', str(DATA_CONTEXTBROKER['PORT']))\
            .replace('AGENT_PORT', str(AGENT_PORT)).replace('FIROS_HOST', str(IP))

    def generateConfigFile(self):
        file = open(CONFIG_FILE_DIR, 'w')
        file.write(self.generateConfigData())
        file.close()

    def launchAgent(self):
        ## \brief Starts Map server thread
        CoapThread = Thread(target=self._launchAgent)
        CoapThread.daemon = True
        CoapThread.start()

    def _launchAgent(self):
        if AGENT_PORT:
            if not os.path.exists(os.path.join(COAP_AGENT_DIR, 'node_modules')):
                text = "---------------------------------------------------------------------------------------\n"
                text += "---------------------------------------------------------------------------------------\n"
                text += "FIROS is going to install coap agent's dependencies\n"
                text += "---------------------------------------------------------------------------------------\n"
                text += "---------------------------------------------------------------------------------------"
                print text
                os.system("cd {} && npm install".format(COAP_AGENT_DIR))
            subprocess.Popen(["node", COAP_AGENT_DIR + '/bin/lwm2mAgent.js', 'new_config.js'])
