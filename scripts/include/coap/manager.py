import os
from include.constants import *

BASE_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__)))
CONFIG_FILE_DIR = os.path.join(BASE_DIR, 'lightweightm2m-iotagent/new_config.js')

class CoapManager(object):
    def __init__(self):
        self.generateConfigFile()

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
