FIROS
=====

FIROS is a tool that helps connecting robots to the cloud. For this purpose it uses the Robot Operating System (ROS, <http://www.ros.org/>) and the FIWARE  Context Broker (http://catalogue.fiware.org/enablers/publishsubscribe-context-broker-orion-context-broker) as a way to publish and listen robot's data.

FIROS works as a translator between the robotics field and the cloud world, transforming ROS messages into NGSI to publish them in the cloud, and vice versa.

Installing FIROS
================

Requirements
------------

-   Ubuntu
-   Python 2.7 or greater
-   ROS Hydro or greater <http://wiki.ros.org/es/ROS/Installation>

Installation
------------

1.  Make sure you have set your working space (http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment)
2.  Open a Terminal and navigate to the ROS workspace you want to use. If you just followed the ROS environment tutorial, it will be /home/catkin_ws.
	`cd /home/catkin_ws`
3.  Clone the FIROS git repository into your ROS workspace.
	`git clone https://github.com/ikergune/firos`
4.  Build the FIROS package with the following command. This will create a devel and build folder under your workspace.
	`catkin_make`
5.  For convenience, add the FIROS source to your *.bashrc*" in order to allow any new command line instance to use FIROS.
	`echo source /FIROS_PATH/devel/setup.bash>>-/.bashrc`
	Just replace *FIROS_PATH* with the actual path to your FIROS package. For example:
	`echo source /home/catkin_ws/firos/devel/setup.bash>>-/.bashrc`
6.  Execute "source setup.bash" to allow the current command line instance to use FIROS

FIROS is now installed in your robot! 


Configuring FIROS
=================

FIROS has several configuration files located at *src/firos/config*.

config.json
-----------

This file contains the configuration related to FIROS launching environment. Here is a description of each parameter:

-   *environment*: These parameters take care of the *local*, *development* and *production* environment configurations by setting up the `*context broker's IP*, *port* and *FIROS rest apis' port*. 
FIROS will use an environment's configuration based this value, but there can be as many environments as you want.
-   *server*: It contains information related to the FIROS server
	- *port*: The port in which FIROS is listening. If you want to let access outside of your local network, you might want to redirect that port on your router.
-   *contextbroker*: Contains information related to the Context broker configuration
    -   *address*: Context broker's IP address
    -   *port*: Context broker's port
    -   *subscription*: Context broker's subscription information
        -   *throttling*: The update frequency at which the Context Broker sends updates to the robot.
        -   *subscription_length*: The subscription expiration time
        -   *subscription_refresh_delay*: The subscription refresh rate in days in order to avoid its expiration
-   *interface*: Network configuration of the card in use
    -   *public*: Public IP. Do not forget to redirect the proper ports in your network
    -   *wlan0, et0, tun0*, etc: Different network interface configuration.
-   *log_level*: It represents the verbosity of the logging system for FIROS. Available options are as follows: *"NONE", "INFO", "DEBUG" ,"WARNING", "ERROR"* and *"CRITICAL"*

Here is an example of a *config.json* file for a *local* environment:
``` javascript
{
  "environment": "local",

  "local": {
    "server": {
        "port": 10100
    },
    "contextbroker": {
        "address"   : "192.168.43.159",
        "port"      : 1026,
        "subscription": {
          "throttling": "PT0S",
          "subscription_length": "P1M",
          "subscription_refresh_delay": 20
        }
    },
    "log_level": "INFO",
    "interface": "public"
  }
}
```


robotdescriptions.json
----------------------

The robots can have some public files that can be useful for users to understand or use the robot. This files' references can be published in context broker, to do this att the descriptions under the robot name like in this example:

``` javascript
"turtle1": {
    "descriptions": [
        "http://wiki.ros.org/ROS/Tutorials/UsingRxconsoleRoslaunch",
        "http://wiki.ros.org/ROS/Tutorials/UnderstandingNodes"
    ]
},"youbot": {
    "descriptions": [
        "http://wiki.ros.org/ROS/Tutorials/UnderstandingServicesParams",
        "http://wiki.ros.org/ROS/Tutorials/UsingRqtconsoleRoslaunch"
    ]
}
```

whitelist.json
--------------

When FIROS is launched or is notified that a new robot has been connected, it searchs for the new robots and topics. this file contains a whitelist that represents the robots that are allowed to connect to FIROS, the topics that will be used and their role (publisher or subscriber), if a topic is publisher it means that FIROS will publish on it and if it is subscriber FIROS will listen to it. The robot and the topic names can be regular expressions (without the "^" at the beginning and "$" at the end). Here is an example:

``` javascript
"turtle\\w+": {
    "publisher": ["cmd_vel"],
    "subscriber": ["pose"]
},
"robot\\w+": {
    "publisher": ["cmd_vel.*teleop", ".*move_base/goal", ".*move_base/cancel"],
    "subscriber": [".*move_base/result"]
}
```

robots.json
-----------

Despite of connecting to new robots you can force some connections, to do this you must add the robot name and topics with their role to this file. The role is the same as in whitelist (publisher to publish on it and subscriber to listen to it). Each topic should need also to define its type. Here is an example:

``` javascript
"robot1":{
    "topics": {
        "cmd_vel_mux/input/teleop": {
            "msg": "geometry_msgs.msg.Twist",
            "type": "publisher"
        },
        "move_base/goal": {
            "msg": "move_base_msgs.msg.MoveBaseActionGoal",
            "type": "publisher"
        },
        "move_base/result": {
            "msg": "move_base_msgs.msg.MoveBaseActionResult",
            "type": "subscriber"
        }
    }
},"turtle1":{
    "topics": {
        "cmd_vel": {
            "msg": "geometry_msgs.msg.Twist",
            "type": "publisher"
        },
        "pose": {
            "msg": "turtlesim.msg.Pose",
            "type": "subscriber"
        }
    }
 }
```

Getting Topic Types
===================

To get the topic type to add it to robots.json these are the steps you need to do (example using the turtlesim <http://wiki.ros.org/turtlesim>):

1.  Launch command "rostopic list" it will show you all the topics that are registered
2.  Select the topic you want and launch the command "rostopic info THE_TOPIC". It will show something like this:

<!-- -->

    rostopic info /turtle1/cmd_vel
        Type: geometry_msgs/Twist

        Publishers: None

        Subscribers:
            * /turtlesim (http://192.168.4.42 :45825/)

This means that the robot is listening to data published on /turtle1/cmd_vel so FIROS should publish on it (publisher). The type is "geometry_msgs/Twist" but FIROS needs it package, the package used to be the same name replacing "/" with ".msg.", so the type is "geometry_msgs.msg.Twist"

Another example:

    rostopic info /turtle1/pose
        Type: turtlesim/Pose

        Publishers:
            * /turtlesim (http://192.168.4.42 :45825/)

        Subscribers: None

This means that the robot is publishing data on /turtle1/pose so FIROS should listen to it (subscriber). The type is "turtlesim/Pose" but FIROS needs it package, the package used to be the same name replacing "/" with ".msg.", so the type is "turtlesim.msg.Pose"

So the robots.json should have this content (it can have more robots and topics):

``` javascript
"turtle1":{
    "topics": {
        "cmd_vel": {
            "msg": "geometry_msgs.msg.Twist",
            "type": "publisher"
        },
        "pose": {
            "msg": "turtlesim.msg.Pose",
            "type": "subscriber"
        }
    }
}
```

FIROS Topics
============

FIROS is listening to 2 topics to handle robot connections.

/FIROS/connect
--------------

When this topic is called (with an empty string) it will search for new robots and topics and will connect with any robot and topic that matches with the whitelist.

/FIROS/disconnect
-----------------

This topic must be called with a String message with the name of the robot that has to be disconnected.

API
===

FIROS has several REST entry points that are used to connect with the context broker or get data from FIROS.

GET /robots
-----------

Get robots handled by FIROS with their topics, each topic contains name, type, role and structure:

``` javascript
[
    {
        "name": "turtle1",
        "topics": [
            {
                "type": "turtlesim.msg.Pose",
                "name": "pose",
                "structure": {
                    "y": "float32",
                    "x": "float32",
                    "linear_velocity": "float32",
                    "angular_velocity": "float32",
                    "theta": "float32"
                },
                "pubsub": "subscriber"
            },
            {
                "type": "geometry_msgs.msg.Twist",
                "name": "cmd_vel",
                "structure": {
                    "linear": {
                        "y": "float64",
                        "x": "float64",
                        "z": "float64"
                    },
                    "angular": {
                        "y": "float64",
                        "x": "float64",
                        "z": "float64"
                    }
                },
                "pubsub": "publisher"
            }
        ],
    }
]
```

GET /robot/NAME
---------------

Get a robot's data published on context broker. What it does is to build query using the NAME that must be send in the path and it will return data using this structure:

``` javascript
[
    {
        "id": "turtle1",
        "type": "ROBOT",
        "attributes": [
            {
                "type": "COMMAND",
                "name": "COMMAND",
                "value": [
                    "pose"
                ]
            },
            {
                "type": "DescriptionData",
                "name": "descriptions",
                "value": "http://wiki.ros.org/ROS/Tutorials/UsingRxconsoleRoslaunch||http://wiki.ros.org/ROS/Tutorials/UnderstandingNodes"
            },
            {
                "type": "turtlesim.msg.Pose",
                "name": "pose",
                "value": {
                    "FIROSstamp": 1424423606.27683,
                    "linear_velocity": 0,
                    "theta": 0,
                    "y": 5.544444561004639,
                    "x": 5.885244369506836,
                    "angular_velocity": 0
                }
            },
            {
                "type": "geometry_msgs.msg.Twist",
                "name": "cmd_vel",
                "value": {
                    "FIROSstamp": 1424423604309,
                    "linear": {
                        "y": 0,
                        "x": 0,
                        "z": 0
                    },
                    "angular": {
                        "y": 0,
                        "x": 0,
                        "z": 0
                    }
                }
            }
        ]
    }
]
```

POST /FIROS
-----------

This is the api that will handle the context broker's subscription data. So the user doesn't have to use it.

POST /robot/connect
-------------------

This API will make FIROS to search for new robots and topics that are allowed in the configuration whitelist and will connect to them

POST /robot/diconnect/NAME
--------------------------

This API will make FIROS to disconnect from the robot specified in the path parameter NAME. It will delete any connection an delete the entity from the context broker.

POST /whitelist/write
---------------------

This api overwrites or creates entries in the robot whitelist described. To do this you must send the data in the next fomat:

``` javascript
{
    "turtle\\w+": {
        "publisher": ["cmd_vel"],
        "subscriber": ["pose"]
    },
    "robot\\w+": {
        "publisher": ["cmd_vel.*teleop", ".*move_base/goal"],
        "subscriber": [".*move_base/result"]
    }
}
```

NOTE: The elements you want to overwrite should be sent with the elements yo want to keep.

EXAMPLE:

if we have this whitelist:

``` javascript
{
    "turtle\\w+": {
        "publisher": ["cmd_vel"],
        "subscriber": ["pose"]
    }
}
```

and you send:

``` javascript
{
    "turtle\\w+": {
        "publisher": ["cmd_vel2"],
        "subscriber": []
    }
}
```

the result will be:

``` javascript
{
    "turtle\\w+": {
        "publisher": ["cmd_vel2"],
        "subscriber": []
    }
}
```

POST /whitelist/remove
----------------------

This api removes whitelist elements the format is the next one:

``` javascript
{
    "turtle\\w+": {
        "publisher": [],
        "subscriber": ["pose"]
    },
    "robot\\w+": {
        "publisher": ["cmd_vel.*teleop", ".*move_base/goal"],
        "subscriber": []
    }
}
```

EXAMPLE:

If we have this whitelist:

``` javascript
{
    "turtle\\w+": {
        "publisher": ["cmd_vel"],
        "subscriber": ["pose"]
    },
    "robot\\w+": {
        "publisher": ["cmd_vel.*teleop", ".*move_base/goal"],
        "subscriber": [".*move_base/result"]
    }
}
```

sending this json:

``` javascript
{
    "turtle\\w+": {
        "publisher": [],
        "subscriber": ["pose"]
    },
    "robot\\w+": {
        "publisher": ["cmd_vel.*teleop", ".*move_base/goal"],
        "subscriber": []
    }
}
```

The result will be:

``` javascript
{
    "turtle\\w+": {
        "publisher": ["cmd_vel"],
        "subscriber": []
    },
    "robot\\w+": {
        "publisher": [],
        "subscriber": [".*move_base/result"]
    }
}
```

POST /whitelist/restore
-----------------------

This api restores whitelist to its initial state


======================================================
=============================================
=============================================

After that you can use FIROS by executing

``` bash
rosrun FIROS core.py
```

NOTE: Before connecting to any robot its libraries must be intalled in the system, for example turtlebot installation: <http://wiki.ros.org/Robots/TurtleBot#turtlebot.2BAC8-Tutorials.2BAC8-indigo.Installation>
