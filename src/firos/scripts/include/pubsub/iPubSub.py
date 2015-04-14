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


class Ipublisher:
    ## \brief Interface for content publisher
    def createContent(topic, datatype, data, isPrimitive=False):
        ## \brief Format the data into FIROS format
        # \param topic name
        # \param topic type
        # \param topic value
        raise Exception('Method not implemented')

    def publish(contex_id, datatype, attributes=[]):
        ## \brief Publish data of an entity
        # \param entity name
        # \param entity type
        # \param entity attributes
        raise Exception('Method not implemented')

    def publishMap(self, context_id, attributes=[]):
        ## \brief Publish data of an entity in context broker
        # \param map topic name
        # \param map connections
        raise Exception('Method not implemented')

    def publishMsg(attributes=[]):
        ## \brief Publish message structures
        # \param entity attributes
        raise Exception('Method not implemented')


class Isubscriber:
    ## \brief Interface for content listener
    def subscribe(self, namespace, data_type, robot):
        ## \brief Subscribe to entities' changes
        # \param entity name
        # \param entity type
        # \param robot object
        raise Exception('Method not implemented')

    def disconnect(self, namespace):
        ## \brief Delete subscription by namespace
        # \param entity name
        raise Exception('Method not implemented')

    def disconnectAll(self):
        ## \brief Delete all subscriptions
        raise Exception('Method not implemented')

    def deleteEntity(self, namespace, data_type):
        ## \brief Delete entity from content publisher
        # \param entity name
        # \param entity type
        raise Exception('Method not implemented')

    def parseData(self, data):
        ## \brief Parse the received data
        # \param data
        raise Exception('Method not implemented')

    def _generateSubscription(self, namespace, data_type="Robot", topics=[]):
        ## \brief Generate subscription message
        # \param entity name
        # \param entity type
        # \param entity's topics
        raise Exception('Method not implemented')


class IqueryBuilder:
    ## \brief Content publisher query builder
    def findById(self, id):
        ## \brief Search entity data by its name
        # \param entity name
        raise Exception('Method not implemented')
