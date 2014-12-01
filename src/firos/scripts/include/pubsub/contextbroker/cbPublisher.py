import json
import rospy
import urllib2

from include.constants import CONTEXTBROKER
from include.pubsub.iPubSub import Ipublisher

class CbPublisher(Ipublisher):
    def createContent(self, topic, datatype, data):
        return {
            "name": topic,
            "type": datatype,
            "value": json.dumps(data).replace('"', "'")
        }

    def publish(self, contex_id, datatype, attributes=[]):
        data = {
            "contextElements": [
                {
                    "id": contex_id,
                    "type": datatype,
                    "isPattern": "false",
                    "attributes": attributes
                }
            ],
            "updateAction": "APPEND"
        }

        url = "http://{}:{}/{}/updateContext".format(CONTEXTBROKER["ADDRESS"], CONTEXTBROKER["PORT"], CONTEXTBROKER["PROTOCOL"])
        data_json = json.dumps(data)
        print data_json
        request = urllib2.Request(url, data_json, {'Content-Type': 'application/json', 'Accept': 'application/json'})
        response = urllib2.urlopen(request)
        response_body = json.loads(response.read())
        print response_body
        response.close()

        if "errorCode" in response_body:
            rospy.logerr("Error sending data to Context Broker:")
            rospy.logerr(response_body["errorCode"]["details"])
        else:
            print "Success sending"