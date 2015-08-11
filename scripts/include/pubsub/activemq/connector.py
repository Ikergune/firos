import time
import sys

import stomp

class MyListener(stomp.ConnectionListener):
    def on_error(self, headers, message):
        print('received an error %s' % message)
    def on_message(self, headers, message):
        print('received a message %s' % message)



class ActiveMq:
    class __impl:
        QUEUE = '/queue/firos'

        def __init__(self):
            self.conn = stomp.Connection([('localhost', 61613)])
            self.conn.set_listener('', MyListener())
            self.conn.start()
            self.conn.connect()
            self.conn.subscribe(destination=self.QUEUE, id=1, ack='auto')

        def sendData(self, data):
            self.conn.send(body=data, destination=self.QUEUE)

        def close(self):
            self.conn.disconnect()

    __instance = None

    def __init__(self):
        if ActiveMq.__instance is None:
            ActiveMq.__instance = ActiveMq.__impl()

        self.__dict__['_ActiveMq__instance'] = ActiveMq.__instance

    def __getattr__(self, attr):
        return getattr(self.__instance, attr)

    def __setattr__(self, attr, value):
        return setattr(self.__instance, attr, value)


# s1 = ActiveMq()
# print id(s1), s1.spam()

# s2 = ActiveMq()
# print id(s2), s2.spam()