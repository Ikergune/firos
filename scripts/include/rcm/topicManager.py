import json
import rospy
import std_msgs.msg

from firos.srv import FIROS_Info
from firos.msg import Robot_Event, CB_Event
from include.constants import Constants as C #ROS_SUB_QUEUE_SIZE, ROS_NODE_NAME
from include.ros.topicHandler import robotDisconnection, loadMsgHandlers
from include.rcm.rcmutils import getRobotConfig


class TopicManager(object):
    rcm_listener = None
    firos_connect_listener = None
    firos_disconnect_listener = None
    robot_topics_service = None


    def __init__(self):
        ''' Lazy Initialization of all inner class attrs
        '''
        self.rcm_listener = rospy.Subscriber("/rcm/robot_event", Robot_Event, self.onRcmEvent, {})
        self.firos_disconnect_listener = rospy.Subscriber(C.ROS_NODE_NAME + "/disconnect", std_msgs.msg.String, self.onDisconnect)
        self.firos_connect_listener = rospy.Subscriber(C.ROS_NODE_NAME + "/connect", std_msgs.msg.String, self.onConnect)
        self.cb_publisher = rospy.Publisher("/" + C.ROS_NODE_NAME +  "/cb_event", CB_Event, queue_size=C.ROS_SUB_QUEUE_SIZE)
        self.robot_topics_service = rospy.ServiceProxy('/' + C.ROS_NODE_NAME + '_info', FIROS_Info)

    def getRobotTopics(self, robot_name):
        robot_json = self.robot_topics_service(robot_name)
        parsed = json.loads(robot_json.json_format)
        robot_data = getRobotConfig(robot_name, parsed[robot_name]["topics"])
        return robot_data


    def onRcmEvent(self, data, args=None):
        ## \brief Callback to handle ROS published data and send it to Context Broker
        # \param data
        # \param extra arguments

        msg = CB_Event()
        msg.entity_name = data.instance_name

        # DISCONNECTION
        if data.instance_status == 0:
            robot_data = std_msgs.msg.String()
            robot_data.data = data.instance_name
            robotDisconnection(robot_data)
            msg.entity_status = 0
            self.cb_publisher.publish(msg)

        # CONNECTION
        elif data.instance_status == 1:
            robot_data = self.getRobotTopics(data.instance_name)
            loadMsgHandlers(robot_data)
            msg.entity_status = 1
            self.cb_publisher.publish(msg)


    def onDisconnect(self, data):
        self.sendConnection(data.data, 0)


    def onConnect(self, data):
        self.sendConnection(data.data, 1)


    def sendConnection(self, robot_name, status):
        msg = CB_Event()
        msg.entity_name = robot_name
        msg.entity_status = status
        self.cb_publisher.publish(msg)


    def removeListeners(self):
        self.rcm_listener.unregister()
        self.firos_disconnect_listener.unregister()
        self.firos_connect_listener.unregister()
        self.cb_publisher.unregister()
