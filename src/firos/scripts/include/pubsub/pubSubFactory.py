from include.pubsub.iPubSub import Ipublisher, Isubscriber

from include.pubsub.contextbroker.cbPublisher import CbPublisher
from include.pubsub.contextbroker.cbSubscriber import CbSubscriber

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
