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

import json
import time
import rospy
import urllib2

from include.logger import Log
from include.constants import INDEX_CONTEXTBROKER, DATA_CONTEXTBROKER, SEPARATOR_CHAR
from include.pubsub.iPubSub import Ipublisher

PUBLISH_FREQUENCY = 250
posted_history = {}


class CbPublisher(Ipublisher):
    ## \brief Context broker publisher class
    def createContent(self, topic, datatype, data):
        ## \brief Format the data into FIROS format
        # \param topic name
        # \param topic type
        # \param topic value
        data["firosstamp"] = time.time()
        return {
            "name": topic,
            "type": datatype,
            "value": json.dumps(data).replace('"', SEPARATOR_CHAR)
        }

    def publish(self, context_id, datatype, attributes=[]):
        ## \brief Publish data of an entity in context broker
        # \param entity name
        # \param entity type
        # \param entity attributes
        _publish(context_id, datatype, attributes, DATA_CONTEXTBROKER)

    def publishMap(self, context_id, attributes=[]):
        ## \brief Publish data of an entity in context broker
        # \param map topic name
        # \param map connections
        _publish(context_id, "MAP", attributes, DATA_CONTEXTBROKER, False)

    def publishMsg(self, attributes=[]):
        ## \brief Publish message structures in context broker
        # \param entity attributes
        _publish("rosmsg", "ROSDEFINITION", attributes, INDEX_CONTEXTBROKER, False)


def _publish(context_id, datatype, attributes, connection, sendCommand=True):
    ## \brief Publish data of an entity in context broker
    # \param entity name
    # \param entity type
    # \param entity attributes
    # \param context broker to send to
    if context_id not in posted_history:
        posted_history[context_id] = {}
    commands = []
    attr2Send = []
    current = time.time() * 1000
    for attribute in attributes:
        if attribute["name"] not in posted_history[context_id]:
            posted_history[context_id][attribute["name"]] = 0
        if (current - posted_history[context_id][attribute["name"]]) > PUBLISH_FREQUENCY:
            commands.append(attribute["name"])
            attr2Send.append(attribute)
            posted_history[context_id][attribute["name"]] = current

    if len(commands) > 0:
        if(sendCommand):
            attr2Send.insert(0, {
                "name": "COMMAND",
                "type": "COMMAND",
                "value": commands
            })
        data = {
            "contextElements": [
                {
                    "id": context_id,
                    "type": datatype,
                    "isPattern": "false",
                    "attributes": attr2Send
                }
            ],
            "updateAction": "APPEND"
        }

        url = "http://{}:{}/NGSI10/updateContext".format(connection["ADDRESS"], connection["PORT"])
        data_json = json.dumps(data)
        try:
            request = urllib2.Request(url, data_json, {'Content-Type': 'application/json', 'Accept': 'application/json'})
            response = urllib2.urlopen(request)
            response_body = json.loads(response.read())
            response.close()

            if "errorCode" in response_body:
                rospy.logerr("Error sending data to Context Broker:")
                rospy.logerr(response_body["errorCode"]["details"])
        except Exception as ex:
            Log("ERROR", ex.reason)
            return None
