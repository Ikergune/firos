class Ipublisher:
    def createContent(topic, datatype, data, isPrimitive=False):
        raise Exception('Method not implemented')

    def publish(contex_id, datatype, attributes=[]):
        raise Exception('Method not implemented')

class Isubscriber:
    def subscribe(self, namespace, data_type, topics):
        raise Exception('Method not implemented')

    def disconnect(self):
        raise Exception('Method not implemented')

    def deleteEntity(self, namespace, data_type):
        raise Exception('Method not implemented')

    def parseData(self):
        raise Exception('Method not implemented')

    def _generateSubscription(self, namespace, data_type="Robot", topics=[]):
        raise Exception('Method not implemented')

class IqueryBuilder:
    def findById(self, id):
        raise Exception('Method not implemented')