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

from include.pubsub.iPubSub import Isubscriber

from include.constants import SEPARATOR_CHAR
from include.pubsub.kiara.kiaraGateway import KiaraGateway

kiara = KiaraGateway()


class KiaraSubscriber(Isubscriber):
    ## \brief Interface for content listener
    def subscribe(self, namespace, data_type, robot):
        ## \brief Subscribe to entities' changes
        # \param entity name
        # \param entity type
        # \param robot object
        pass

    def disconnect(self, namespace):
        ## \brief Delete subscription by namespace
        # \param entity name
        pass

    def disconnectAll(self):
        ## \brief Delete all subscriptions
        # kiara.
        pass

    def deleteEntity(self, namespace, data_type):
        ## \brief Delete entity from content publisher
        # \param entity name
        # \param entity type
        pass

    def parseData(self, data):
        ## \brief Parse the received data
        # \param data
        pass

    def _generateSubscription(self, namespace, data_type="Robot", topics=[]):
        ## \brief Generate subscription message
        # \param entity name
        # \param entity type
        # \param entity's topics
        pass
