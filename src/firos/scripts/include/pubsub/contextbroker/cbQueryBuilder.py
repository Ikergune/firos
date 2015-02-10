import json
import urllib2

from include.logger import Log
from include.constants import CONTEXTBROKER
from include.pubsub.iPubSub import IqueryBuilder

class CbQueryBuilder(IqueryBuilder):
    def findById(self, entity_id, data_type="ROBOT", isPattern=False):
        url = "http://{}:{}/{}/queryContext".format(CONTEXTBROKER["ADDRESS"], CONTEXTBROKER["PORT"], CONTEXTBROKER["PROTOCOL"])
        data = {
            "entities": [
                {
                    "type": data_type,
                    "isPattern": "true" if isPattern else "false",
                    "id": entity_id
                }
            ]
        }
        return self._sendRequest(url, json.dumps(data))

    def _sendRequest(self, url, data, method=None):
        try:
            request = urllib2.Request(url, data, {'Content-Type': 'application/json', 'Accept': 'application/json'})
            if method is not None:
                request.get_method = lambda: method
            response = urllib2.urlopen(request)
            data = response.read()
            response_body = json.loads(data)
            response.close()
            return response_body
        except Exception as ex:
            Log("ERROR", ex.reason)
            return None