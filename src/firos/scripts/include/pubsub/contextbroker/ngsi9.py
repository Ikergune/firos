import copy
import json
import urllib2

from include.constants import CONTEXTBROKER, SUBSCRIPTION_LENGTH

contexts = {}

def registerContext(entity_id, data_type, robot,  isPattern=False):
    url = "http://{}:{}/NGSI9/registerContext".format(CONTEXTBROKER["ADDRESS"], CONTEXTBROKER["PORT"])
    data = {
        "contextRegistrations": [
            {
                "entities": [
                    {
                        "type": data_type,
                        "isPattern": "false",
                        "id": entity_id
                    }
                ],
                "attributes": topics2NGSI9(robot),
                "providingApplication": "http://{}:{}/{}".format(CONTEXTBROKER["ADDRESS"], CONTEXTBROKER["PORT"], CONTEXTBROKER["PROTOCOL"])
            }
        ],
        "duration": SUBSCRIPTION_LENGTH
    }
    response = _sendRequest(url, json.dumps(data))
    if "registrationId" in response:
        contexts[entity_id] = {
            "data": data,
            "registrationId": response["registrationId"]
        }

def deleteAllContexts():
    for key in contexts:
        deleteContext(key)

def deleteContext(entity_id, delete=False):
    url = "http://{}:{}/NGSI9/registerContext".format(CONTEXTBROKER["ADDRESS"], CONTEXTBROKER["PORT"])
    data = {
        "contextRegistrations": [
            {
                "providingApplication": "http://{}:{}/{}".format(CONTEXTBROKER["ADDRESS"], CONTEXTBROKER["PORT"], CONTEXTBROKER["PROTOCOL"])
            }
        ],
        "duration": "P0D",
        "registrationId": contexts[entity_id]["registrationId"]
    }
    response = _sendRequest(url, json.dumps(data))
    if delete:
        del contexts[entity_id]

def refreshAllContexts():
    for key in contexts:
        refreshContext(key)

def refreshContext(entity_id):
    url = "http://{}:{}/NGSI9/registerContext".format(CONTEXTBROKER["ADDRESS"], CONTEXTBROKER["PORT"])
    data = copy.deepcopy(contexts[entity_id]["data"])
    data["registrationId"] = contexts[entity_id]["registrationId"]
    response = _sendRequest(url, json.dumps(data))

def topics2NGSI9(robot):
    return iterateTopics(robot["publisher"], "publisher") + iterateTopics(robot["subscriber"], "subscriber")

def iterateTopics(topics, topic_type):
    elems = []
    for key in topics:
        elems.append( {
            "name": key,
            "type": topic_type + ":" + topics[key]["class"]._type.replace("/", ".msg."),
            "isDomain": "false"
        })
    return elems


def _sendRequest(url, data, method=None):
    request = urllib2.Request(url, data, {'Content-Type': 'application/json', 'Accept': 'application/json'})
    if method is not None:
        request.get_method = lambda: method
    response = urllib2.urlopen(request)
    data = response.read()
    response_body = json.loads(data)
    response.close()
    return response_body

