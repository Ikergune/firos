import json
import urllib2
import time

from include.constants import CONTEXTBROKER

class CbPublisher:
    @staticmethod
    def createAttribute(name, datatype, value):
        return {
            "name": name,
            "type": datatype,
            "value": value
        }

    @staticmethod
    def publish(contex_id, datatype, attributes=[]):
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