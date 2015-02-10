import json
import time
import rospy
import urllib2

from include.logger import Log
from include.constants import CONTEXTBROKER, SEPARATOR_CHAR
from include.pubsub.iPubSub import Ipublisher

PUBLISH_FREQUENCY = 250
posted_history = {}

class CbPublisher(Ipublisher):
    def createContent(self, topic, datatype, data):
        data["firosstamp"] = time.time()
        return {
            "name": topic,
            "type": datatype,
            "value": json.dumps(data).replace('"', SEPARATOR_CHAR)
        }

    def publish(self, context_id, datatype, attributes=[]):
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

            url = "http://{}:{}/{}/updateContext".format(CONTEXTBROKER["ADDRESS"], CONTEXTBROKER["PORT"], CONTEXTBROKER["PROTOCOL"])
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