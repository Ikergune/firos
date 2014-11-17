import json
import urllib2

CONTEXTBROKER = {
    "ADDRESS" : "130.206.83.196",
    "PORT"    : 1026,
    "PROTOCOL": "NGSI10"
}

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
                    "type": datatype,
                    "isPattern": "false",
                    "id": contex_id,
                    "attributes": attributes
                }
            ],
            "updateAction": "APPEND"
        }
        url = "http://{}:{}/{}/updateContext".format(CONTEXTBROKER["ADDRESS"], CONTEXTBROKER["PORT"], CONTEXTBROKER["PROTOCOL"])
        data_json = json.dumps(data)
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

#Main code, for testing the publishing in context broker. Launch with python cbPublisher.py
if __name__ == '__main__':
    # Initialize the node and name it.
    print "Enviando"
    CbPublisher.publish("Room1", "Room",[CbPublisher.createAttribute("cmd_vel", "integer", 25)])