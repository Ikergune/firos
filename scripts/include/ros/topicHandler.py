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
from include.constants import DEFAULT_QUEUE_SIZE, DEFAULT_CONTEXT_TYPE
from include.libLoader import LibLoader

from include.ros.rosConfigurator import RosConfigurator
from include.ros.dependencies.third_party import *

# PubSub Handlers
from include.contextbroker.cbPublisher import CbPublisher
from include.contextbroker.cbSubscriber import CbSubscriber


import std_msgs.msg


CloudSubscriber = CbSubscriber()
CloudPublisher = CbPublisher()


TOPIC_BASE_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "topics")
ROBOT_TOPICS = {}
robot_data = {}
subscribers = []
topic_DataTypeDefinition = {} # filled in loadMsgHandlers with infos about the concrete Datatype, 

# Actual ROS-classes used for instanciateROSMessage
ROS_MESSAGE_CLASSES = {}


def loadMsgHandlers(robot_data):
    ## \brief Load ROS publishers/subscribers based on robot data
    # \param robot data
    #TODO DL refactor. ROBOT_TOPICS needs to be easier!
    Log("INFO", "Getting configuration data")
    Log("INFO", "Generating topic handlers:")
    for robotName in robot_data:  # for each robot 
        robotName = str(robotName)
        robot = robot_data[robotName]

        # Add robot-Entry iff not already inserted 
        if robotName not in ROBOT_TOPICS:
            ROBOT_TOPICS[robotName] = {
                "publisher": {},
                "subscriber": {}
            }
        Log("INFO", "  -" + robotName)

        for topicName in robot['topics']:  # for each topic of an robot
            topic = robot['topics'][topicName]
            topicName = str(topicName)
            Log("INFO", "    -" + topicName)
            extra = {"robot": robotName, "topic": topicName}

            # TODO DL 'If' Might be Dead Code?
            if type(topic['msg']) is dict:
                _topic_name = robotName + ".msg." + topicName
                module = LibLoader.loadFromFile(os.path.join(TOPIC_BASE_PATH, robotName+topicName+".py"))
                theclass = getattr(module, topicName)
            else:
                _final_name = topic['msg'].split('.')[-1]
                _topic_name = str(topic['msg'])
                if _final_name in globals():
                    theclass = globals()[_final_name] 
                else:
                    theclass = LibLoader.loadFromSystem(topic['msg'])
                extra["type"] = _topic_name

            topic_DataTypeDefinition[_topic_name] = ros2Definition(theclass())

            if topic["type"].lower() == "publisher":
                ROBOT_TOPICS[robotName]["publisher"][topicName] = {
                    "msg": str(topic['msg']),
                    "class": theclass,
                    "publisher": rospy.Publisher(robotName + "/" + topicName, theclass, queue_size=DEFAULT_QUEUE_SIZE)
                }
            elif topic["type"].lower() == "subscriber":
                ROBOT_TOPICS[robotName]["subscriber"][topicName] = {
                    "msg": str(topic['msg']),
                    "class": theclass,
                    "subscriber": rospy.Subscriber(robotName + "/" + topicName, theclass, _publishToCBRoutine, extra)
                }

        Log("INFO", "\n")
        # CloudSubscriber.subscribe(robotName, DEFAULT_CONTEXT_TYPE, ROBOT_TOPICS[robotName])
        CloudSubscriber.subscribeToCB(str(robotName), ROBOT_TOPICS[robotName]["publisher"].keys())
        Log("INFO", "Subscribed to " + robotName + "'s topics\n")


def connectionListeners():
    ## \brief Create firos listeners for robot creation or removal
    #TODO DL this might not work!
    subscribers.append(rospy.Subscriber("firos/disconnect", std_msgs.msg.String, robotDisconnection))
    subscribers.append(rospy.Subscriber("firos/connect", std_msgs.msg.String, _robotConnection))


class TopicHandler:
    
    # Send back to ROS
    @staticmethod
    def publishDirect(robotID, topic, obj, dataStruct):
        if robotID in ROBOT_TOPICS and topic in ROBOT_TOPICS[robotID]["publisher"]:
            instance = ROBOT_TOPICS[robotID]["publisher"][topic]
            if instance["class"]._type.replace("/", ".") == dataStruct['type']: # check if received and expected type are equal!
                newMsg = instanciateROSMessage(obj, dataStruct)  # TODO DL  Refactor ROBOT_TOPICS       
                if "publisher" in instance:
                    instance["publisher"].publish(newMsg)

    @staticmethod
    def unregisterAll():
        CloudSubscriber.unsubscribeALLFromCB()
        CloudPublisher.unpublishALLFromCB()

        Log("INFO", "Unsubscribing topics...")
        for subscriber in subscribers:
            subscriber.unregister()
        for robot_name in ROBOT_TOPICS:
            for topic in ROBOT_TOPICS[robot_name]["subscriber"]:
                ROBOT_TOPICS[robot_name]["subscriber"][topic]["subscriber"].unregister()
        Log("INFO", "Unsubscribed topics\n")


def _publishToCBRoutine(data, args):
    # this Routine is executed on every received ROS-Message
    # TODO DL topic_DataTypeDefinition can be omitted via ROS_MESSAGE_CLASSES
    robot = str(args['robot'])
    topic = str(args['topic'])
    datatype = ROBOT_TOPICS[robot]["subscriber"][topic]["msg"]

    CloudPublisher.publishToCB(robot, topic, data, topic_DataTypeDefinition[datatype])





#TODO Testing
def instanciateROSMessage(obj, dataStruct):
    ''' This method instanciates via obj and dataStruct the actual ROS-Message like
        "geometry_msgs.Twist". Explicitly it loads the ROS-Message-class (if not already done)
        with the dataStruct["type"] if given and recursively sets all attributes of the Message. 
    '''
    # Check if type and value in datastruct, if not we default to a priimitive value
    if 'type' in dataStruct and 'value' in dataStruct:
        
        # Load Message-Class only once!
        if dataStruct['type'] not in ROS_MESSAGE_CLASSES:
            msgType = dataStruct['type'].split(".") # see Fiware-Object-Coverter, explicit Types of ROS-Messages are retreived from there 
            moduleLoader = importlib.import_module(msgType[0] + ".msg")
            msgClass = getattr(moduleLoader, msgType[1])
            ROS_MESSAGE_CLASSES[dataStruct['type']] = msgClass
        #Instanciate Message
        instance = ROS_MESSAGE_CLASSES[dataStruct['type']]()


        for attr in ROS_MESSAGE_CLASSES[dataStruct['type']].__slots__:
            # For each attribute in Message
            if attr in obj and attr in dataStruct['value']:
                # Check iff obj AND dataStruct contains attr
                if type(dataStruct['value'][attr]) is list:
                    l =[]
                    for it in range(len(dataStruct['value'][attr])):
                        l.append(instanciateROSMessage(obj[attr][it], dataStruct['value'][attr][it]))
                    setattr(instance, attr, l)
                else:
                    setattr(instance, attr, instanciateROSMessage(obj[attr], dataStruct['value'][attr]))
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


# TODO DL are Definitions still somewhere used?
def ros2Definition(msgInstance):
    ## \brief Generate Ros object definition
    # \param ROS Object instance
    obj = {}
    for key, t in zip(msgInstance.__slots__, msgInstance._slot_types):
        attr = getattr(msgInstance, key)
        if hasattr(attr, '__slots__'):
            obj[key] = ros2Definition(attr)
        else:
            obj[key] = t
    return obj










## TODO DL are they somewhere used?

def robotDisconnection(data):
    ## \brief Handle robot diconnection
    # \param robot data dict (name)
    robot_name = data.data
    Log("INFO", "Disconnected robot: " + robot_name)
    if robot_name in ROBOT_TOPICS:
        for topic in ROBOT_TOPICS[robot_name]["publisher"]:
            ROBOT_TOPICS[robot_name]["publisher"][topic]["publisher"].unregister()
        for topic in ROBOT_TOPICS[robot_name]["subscriber"]:
            ROBOT_TOPICS[robot_name]["subscriber"][topic]["subscriber"].unregister()
        RosConfigurator.removeRobot(robot_name)


def _robotConnection(data):
    ## \brief Handle robot connection
    # \param robot data (Not neccessary)
    robot_name = data.data
    Log("INFO", "Connected robot: " + robot_name)
    loadMsgHandlers(RosConfigurator.systemTopics(True))
