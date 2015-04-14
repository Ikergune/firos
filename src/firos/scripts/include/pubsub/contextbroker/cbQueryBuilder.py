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
import urllib2

from include.logger import Log
from include.constants import DATA_CONTEXTBROKER
from include.pubsub.iPubSub import IqueryBuilder


class CbQueryBuilder(IqueryBuilder):
    ## \brief Query data to context broker
    def findById(self, entity_id, data_type="ROBOT", isPattern=False):
        ## \brief Get entity data from context broker
        # \param entity name (can be regular expression)
        # \param entity type
        # \param if the entity name is a pattern or not (false by default)
        url = "http://{}:{}/NGSI10/queryContext".format(DATA_CONTEXTBROKER["ADDRESS"], DATA_CONTEXTBROKER["PORT"])
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
        ## \brief Send request to context broker
        # \param url to request to
        # \param data to send
        # \param HTTP method (GET by default)
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
