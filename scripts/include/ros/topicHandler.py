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

__author__ = "Dominik Lux"
__credits__ = ["Peter Detzner"]
__maintainer__ = "Dominik Lux"
__version__ = "0.0.1a"
__status__ = "Developement"

import os
import rospy
import importlib

from include.logger import Log
from include.constants import DEFAULT_QUEUE_SIZE
from include.libLoader import LibLoader
from include.ros.rosConfigurator import RosConfigurator
from include.ros.dependencies.third_party import *

# PubSub Handlers
from include.contextbroker.cbPublisher import CbPublisher
from include.contextbroker.cbSubscriber import CbSubscriber

# this Message is needed, for the Listeners on connect on disconnect
import std_msgs.msg


CloudSubscriber = CbSubscriber()
CloudPublisher = CbPublisher()

# TODO DL see loadMsgHandlers
TOPIC_BASE_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "topics")

# Structs whith robotID and each robotID with a topic: 
# ROS_PUBSUB[robotID][topic] --> returns  rospy Publisher/Subsriber
ROS_PUBLISHER = {}
ROS_SUBSCRIBER = {}

# Topics in ROS do only have one data-type! 
# (There might be an Exception to this if the topic gets deregistered, this is currently ignored)
# We use this here to load the type of an topic and the dictionary rep. only once!
ROS_TOPIC_TYPE = {}  
ROS_TOPIC_AS_DICT ={}

# ROS-Node Subscribers  (connect/disconnect)
subscribers = []

# Actual ROS-Classes are used in instantiateROSMessage and loadMsgHandlers. 
# An entry is the ._type-Attribute of the ROS-Messages ('/' is replaced by '.')
ROS_MESSAGE_CLASSES = {}

# If shutdown is signaled, do stop posting ROS-Messages to the ContextBroker
SHUTDOWN_SIGNAL = False


def loadMsgHandlers(robot_data):
    ''' This method initializes The Publisher and Subscriber for ROS and 
        the Subscribers for the Context-broker (based on ROS-Publishers).
        It also initializes the structs for ROS messages (types, dicts and classes)

        For each robotID with its topic, the ROS message-structs are initialized.
        Then (depending on Publisher or Subscriber) the corrsponding rospy Publisher/Subscriber
        is generated and added in its struct.

        robot_data: The data, as in robots.json specified

        TODO DL maybe change the message loading to something simpler?
    '''


    Log("INFO", "Getting configuration data")
    Log("INFO", "Generating topic handlers:")

    for robotID in robot_data.keys(): 
        for topic in robot_data[robotID]['topics'].keys():
            # for each robotID and topic in robot_data:

            # Load specific message from robot_data
            # TODO DL maybe change this in config to ._type values? Refactor!
            msg = str(robot_data[robotID]['topics'][topic]['msg'])
            if type(msg) is dict:
                # TODO DL Dead Code?
                _topic_name = robotID + ".msg." + robotID
                module = LibLoader.loadFromFile(os.path.join(TOPIC_BASE_PATH, robotID+topic+".py"))
                theclass = getattr(module, topic)
            else:
                _final_name = msg.split('.')[-1]
                _topic_name = str(msg)
                if _final_name in globals():
                    theclass = globals()[_final_name] 
                else:
                    theclass = LibLoader.loadFromSystem(msg)
            
            # Add specific message in struct to not load it again later.
            if theclass._type not in ROS_MESSAGE_CLASSES:
                ROS_MESSAGE_CLASSES[theclass._type.replace("/", ".")] = theclass  # Replacing '/' with '.' see Obejct-Converter

            # Create, if not already, a dictionary from the corresponding message-type
            if topic not in ROS_TOPIC_AS_DICT:
                ROS_TOPIC_AS_DICT[topic] = rosMsg2Dict(theclass())
            
            # Set the topic class-type, which is for each topic always the same
            ROS_TOPIC_TYPE[topic] = theclass._type

            # Create Publisher or Subscriber
            if robot_data[robotID]['topics'][topic]['type'].lower() == "subscriber":
                # Case it is a subscriber, add it in subscribers
                if robotID not in ROS_SUBSCRIBER:
                    ROS_SUBSCRIBER[robotID] = {}
                    
                additionalArgsCallback = {"robot": robotID, "topic": topic} # Add addtional Infos about topic and robotID 
                ROS_SUBSCRIBER[robotID][topic] = rospy.Subscriber(robotID + "/" + topic, theclass, _publishToCBRoutine, additionalArgsCallback)
            else:
                # Case it is a publisher, add it in publishers
                if robotID not in ROS_PUBLISHER:
                    ROS_PUBLISHER[robotID] = {}

                ROS_PUBLISHER[robotID][topic] = rospy.Publisher(robotID + "/" + topic, theclass, queue_size=DEFAULT_QUEUE_SIZE)
        
        # After initializing ROS-PUB/SUBs, intitialize ContextBroker-Subscriber based on ROS-Publishers for each robot
        if robotID in ROS_PUBLISHER:
            CloudSubscriber.subscribeToCB(str(robotID), ROS_PUBLISHER[robotID].keys())
            Log("INFO", "\n")
            Log("INFO", "Subscribed to " + robotID + "'s topics\n")


def _publishToCBRoutine(data, args):
    ''' This routine is executed on every received (subscribed) message on ROS.
        It just wraps it content and publishes the data via the cbPublisher.publishToCB
        Here we explicitly check at the SHUTDOWN_SIGNAL. if it is set, we stop publishing

        data: data received from ROS
        args: additional arguments we set prior
    '''
    if not SHUTDOWN_SIGNAL:
        robot = args['robot']
        topic = args['topic'] # Retreiving additional Infos, which were set on initialization
        CloudPublisher.publishToCB(robot, topic, data, ROS_TOPIC_AS_DICT[topic])



class RosTopicHandler:
    ''' The Class RosTopicHandler is a Wrapper-Class which 
        just maps the publish Routine to the cbPublisher (for the requestHandler) and
        by shutdown removes and deletes all Subscriptions/created Entities (for core)
    '''


    
    @staticmethod
    def publish(robotID, topic, convertedData, dataStruct):
        ''' This method publishes the receive data from the 
            ContextBroker to ROS

            robotID: The Robot-Id
            topic: The topic to be published
            convertedData: the converted data from the cbSubscriber
            dataStruct: The struct of convertedData, specified by their types
        '''
        if robotID in ROS_PUBLISHER and topic in ROS_PUBLISHER[robotID]:
            if topic in ROS_TOPIC_TYPE and ROS_TOPIC_TYPE[topic].replace("/", ".") == dataStruct['type']: 
                # check if a publisher to this robotID and topic is set 
                # then check the received and expected type to be equal
                # Iff, then publish received message to ROS
                newMsg = instantiateROSMessage(convertedData, dataStruct)     
                ROS_PUBLISHER[robotID][topic].publish(newMsg)


    @staticmethod
    def unregisterAll():
        global SHUTDOWN_SIGNAL
        ''' First set the SHUTDOWN_SIGNAL, then
            unregister all subscriptions on ContextBroker,
            delete all created Entities on Context Broker and
            unregister subscriptions from ROS
        '''
        SHUTDOWN_SIGNAL = True

        CloudSubscriber.unsubscribeALLFromCB()
        CloudPublisher.unpublishALLFromCB()

        Log("INFO", "Unsubscribing topics...")
        for subscriber in subscribers:
            subscriber.unregister()
        for robotID in ROS_SUBSCRIBER:
            for topic in ROS_SUBSCRIBER[robotID]:
                ROS_SUBSCRIBER[robotID][topic].unregister()
        Log("INFO", "Unsubscribed topics\n")



def instantiateROSMessage(obj, dataStruct):
    ''' This method instantiates via obj and dataStruct the actual ROS-Message like
        "geometry_msgs.Twist". Explicitly it loads the ROS-Message-class (if not already done)
        with the dataStruct["type"] if given and recursively sets all attributes of the Message. 

        obj: The Object to instantiate
        dataStruct: The corresponding dataStruct, which helps by setting the explicit ROS-Message
    '''
    # Check if type and value in datastruct, if not we default to a primitive value
    if 'type' in dataStruct and 'value' in dataStruct:
        
        # Load Message-Class only if not already loaded!
        if dataStruct['type'] not in ROS_MESSAGE_CLASSES:
            msgType = dataStruct['type'].split(".") # see Fiware-Object-Converter, explicit Types of ROS-Messages are retreived from there 
            moduleLoader = importlib.import_module(msgType[0] + ".msg")
            msgClass = getattr(moduleLoader, msgType[1])
            ROS_MESSAGE_CLASSES[dataStruct['type']] = msgClass
        #instantiate Message
        instance = ROS_MESSAGE_CLASSES[dataStruct['type']]()

        for attr in ROS_MESSAGE_CLASSES[dataStruct['type']].__slots__:
            # For each attribute in Message
            if attr in obj and attr in dataStruct['value']:
                # Check iff obj AND dataStruct contains attr
                if type(dataStruct['value'][attr]) is list:
                    l =[]
                    for it in range(len(dataStruct['value'][attr])):
                        l.append(instantiateROSMessage(obj[attr][it], dataStruct['value'][attr][it]))
                    setattr(instance, attr, l)
                else:
                    setattr(instance, attr, instantiateROSMessage(obj[attr], dataStruct['value'][attr]))
        return instance
    else:
        # Struct is {}:
        if type(obj) is dict:
            # if it is still a dict, convert into an Object with attributes
            t = Temp()
            for k in obj:
                setattr(t, k, obj[k])
            return t
        else:
            # something more simple (int, long, float), return it
            return obj

            
class Temp(object):
    ''' A Temp-object, to convert from a Dictionary into an object with attributes.
    '''
    pass


def rosMsg2Dict(rosClassInstance):
    ''' Generating a dictionary out of the instance of a
        ROS-Message

        rosClassInstance: an actual instance of the ROS-Message (vales will be omitted)
    '''
    obj = {}
    for key, t in zip(rosClassInstance.__slots__, rosClassInstance._slot_types):
        attr = getattr(rosClassInstance, key)
        if hasattr(attr, '__slots__'):
            obj[key] = rosMsg2Dict(attr)
        else:
            obj[key] = t
    return obj



###############################################################################
#######################   Connect/Disconnect Mapping   ########################
###############################################################################

def createConnectionListeners():
    ''' This creates the following listeners for firos in ROS for robot-creation 
        and -removal and maps them to the methods below:

        /firos/connect    --> std_msgs/String
        /firos/disconnect --> std_msgs/String
    '''
    subscribers.append(rospy.Subscriber("firos/disconnect", std_msgs.msg.String, robotDisconnection))
    subscribers.append(rospy.Subscriber("firos/connect", std_msgs.msg.String, _robotConnection))


def robotDisconnection(data):
    ''' Unregisters from a given robotID by a ROBOT

        data: The String which was sent to firos
    '''
    robotID = str(data.data)

    Log("INFO", "Disconnected robot: " + robotID)
    if robotID in ROS_PUBLISHER:
        for topic in ROS_PUBLISHER[robotID]:
            ROS_PUBLISHER[robotID][topic].unregister()
        del ROS_PUBLISHER[robotID]

    if robotID in ROS_SUBSCRIBER:
        for topic in ROS_SUBSCRIBER[robotID]:
            ROS_SUBSCRIBER[robotID][topic].unregister()
        del ROS_SUBSCRIBER[robotID]


def _robotConnection(data):
    ''' This resets firos into its original state

        TODO DL reset, instead of connect?
        TODO DL Add real connect for only one Robot?
    '''
    robot_name = data.data
    Log("INFO", "Connected robot: " + robot_name)
    loadMsgHandlers(RosConfigurator.systemTopics(True))
