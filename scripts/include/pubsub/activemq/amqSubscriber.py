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

import os
import json
import time
import thread
import signal
import urllib2

from include.constants import *
from include.logger import Log
from include.pubsub.iPubSub import Isubscriber
from include.pubsub.activemq.connector import Connector


# from include.pubsub.contextbroker.ngsi9 import registerContext, deleteContext, deleteAllContexts, refreshAllContexts


class AMqSubscriber(Isubscriber):
    ## \brief Context broker subscription handler
    subscriptions = {}

    def subscribe(self, namespace, entity):
        ## \brief Subscribe to entities' changes
        # \param entity name
        # \param entity type
        # \param entity object
        if namespace not in self.subscriptions:
            self.subscriptions[namespace] = Connector.subscribe(namespace)

    def disconnect(self, namespace, delete=False):
        ## \brief Delete subscription by namespace
        # \param entity name
        # \param flag to indicate if the subscription must be deleted locally (False by default)
        if namespace in self.subscriptions:
            Connector.unsubscribe(namespace)
            if delete:
                del self.subscriptions[namespace]

    def disconnectAll(self):
        ## \brief Delete all subscriptions
        for subscription in self.subscriptions:
            Connector.unsubscribe(subscription)

    def parseData(self, data):
        ## \brief Parse the received data
        # \param data
        # print data
        return json.loads(data.replace(SEPARATOR_CHAR, '"'))
