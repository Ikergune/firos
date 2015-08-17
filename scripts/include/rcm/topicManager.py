import json
import rospy
import std_msgs.msg

from firos.srv import FIROS_Info
from firos.msg import Robot_Event, CB_Event
from include.constants import DEFAULT_QUEUE_SIZE
from include.ros.topicHandler import robotDisconnection, loadMsgHandlers
from include.rcm.rcmutils import getRobotConfig

rcm_listener = None
firos_connect_listener = None
firos_disconnect_listener = None
robot_topics_service = rospy.ServiceProxy('/firos_info', FIROS_Info)
cb_publisher = rospy.Publisher("/firos/cb_event", CB_Event, queue_size=DEFAULT_QUEUE_SIZE)


def getRobotTopics(robot_name):
    robot_json = robot_topics_service(robot_name)
    parsed = json.loads(robot_json.json_format)
    robot_data = getRobotConfig(robot_name, parsed[robot_name]["topics"])
    return robot_data


def onRcmEvent(data, args=None):
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
        cb_publisher.publish(msg)

    # CONNECTION
    elif data.instance_status == 1:
        robot_data = getRobotTopics(data.instance_name)
        loadMsgHandlers(robot_data)
        msg.entity_status = 1
        cb_publisher.publish(msg)


def onDisconnect(data):
    sendConnection(data.data, 0)


def onConnect(data):
    sendConnection(data.data, 1)


def sendConnection(robot_name, status):
    msg = CB_Event()
    msg.entity_name = robot_name
    msg.entity_status = status
    cb_publisher.publish(msg)


def setListeners():
    global rcm_listener
    global firos_disconnect_listener
    global firos_connect_listener
    rcm_listener = rospy.Subscriber("/rcm/robot_event", Robot_Event, onRcmEvent, {})
    firos_disconnect_listener = rospy.Subscriber("firos/disconnect", std_msgs.msg.String, onDisconnect)
    firos_connect_listener = rospy.Subscriber("firos/connect", std_msgs.msg.String, onConnect)


def removeListeners():
    global rcm_listener
    global firos_disconnect_listener
    global firos_connect_listener
    global cb_publisher
    rcm_listener.unregister()
    firos_disconnect_listener.unregister()
    firos_connect_listener.unregister()
    cb_publisher.unregister()
