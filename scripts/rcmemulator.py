#!/usr/bin/env python
import json
import rospy
from firos.srv import FIROS_Info, FIROS_InfoResponse


def get_robot_info(req):
    ret = FIROS_InfoResponse()
    robot = {
        req.instance_name: {
            "topics": [
                {
                    "name": "cmd_vel_mux/input/teleop",
                    "msg": "geometry_msgs/Twist",
                    "type": "publisher"
                },
                {
                    "name": "invented_topic_not_allowed",
                    "msg": "std_msgs/String",
                    "type": "publisher"
                },
                {
                    "name": "move_base/goal",
                    "msg": "move_base_msgs/MoveBaseActionGoal",
                    "type": "publisher"
                },
                {
                    "name": "move_base/result",
                    "msg": "move_base_msgs/MoveBaseActionResult",
                    "type": "subscriber"
                }
            ]
        }
    }
    ret.json_format = json.dumps(robot)
    return ret


if __name__ == "__main__":
    ROS_NODE = 'rcm_emulator'
    ROS_SERVICE = 'firos_info'
    print 'Node ' + ROS_NODE + ' up with service ' + ROS_SERVICE
    rospy.init_node(ROS_NODE)
    rospy.Service(ROS_SERVICE, FIROS_Info, get_robot_info)
    rospy.spin()
