from py4j.java_gateway import JavaGateway


class KiaraGateway:
    class __impl:
        def __init__(self):
        # self.gateway = JavaGateway(start_callback_server=True)
            self.gateway = JavaGateway()
            # self.gateway.entry_point.setCallback(self)
            self.entry_point = self.gateway.entry_point

        def sendData(self, data):
            print "SENDING IN KIARA"
            print data
            print self.entry_point
            self.entry_point.sendMessage(data)

        def onData(self, data):
            print "Data received"
            print data

        class Java:
            implements = ['com.firos.kiara.interfaces.Firos']

    __instance = None

    def __init__(self):
        if KiaraGateway.__instance is None:
            KiaraGateway.__instance = KiaraGateway.__impl()

        self.__dict__['_KiaraGateway__instance'] = KiaraGateway.__instance

    def __getattr__(self, attr):
        return getattr(self.__instance, attr)

    def __setattr__(self, attr, value):
        return setattr(self.__instance, attr, value)


# s1 = KiaraGateway()
# print id(s1), s1.spam()

# s2 = KiaraGateway()
# print id(s2), s2.spam()
