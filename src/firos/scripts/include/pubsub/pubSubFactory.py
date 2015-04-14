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

from include.pubsub.iPubSub import Ipublisher, Isubscriber, IqueryBuilder

from include.pubsub.contextbroker.cbPublisher import CbPublisher
from include.pubsub.contextbroker.cbSubscriber import CbSubscriber
from include.pubsub.contextbroker.cbQueryBuilder import CbQueryBuilder

PUBSUB_TYPE = "ContextBroker"


class PublisherFactory:
    @staticmethod
    def create():
        if PUBSUB_TYPE is "ContextBroker":
            return CbPublisher()
        else:
            return Ipublisher()

    @staticmethod
    def getClass():
        if PUBSUB_TYPE is "ContextBroker":
            return CbPublisher
        else:
            return Ipublisher


class SubscriberFactory:
    @staticmethod
    def create():
        if PUBSUB_TYPE is "ContextBroker":
            return CbSubscriber()
        else:
            return Isubscriber()

    @staticmethod
    def getClass():
        if PUBSUB_TYPE is "ContextBroker":
            return CbSubscriber
        else:
            return Isubscriber


class QueryBuilderFactory:
    @staticmethod
    def create():
        if PUBSUB_TYPE is "ContextBroker":
            return CbQueryBuilder()
        else:
            return IqueryBuilder()

    @staticmethod
    def getClass():
        if PUBSUB_TYPE is "ContextBroker":
            return CbQueryBuilder
        else:
            return IqueryBuilder
