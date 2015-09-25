FIROS
=====

FIROS is a tool that helps connecting robots to the cloud. For this purpose it uses the Robot Operating System (ROS, <http://www.ros.org/>) and the FIWARE  Context Broker (http://catalogue.fiware.org/enablers/publishsubscribe-context-broker-orion-context-broker) as a way to publish and listen robot's data.

FIROS works as a translator between the robotics field and the cloud world, transforming ROS messages into NGSI to publish them in the cloud, and vice versa.

This project is part of [FIWARE](http://www.fiware.org).

Installing FIROS
================

Requirements
------------

-   Ubuntu
-   Python 2.7 or greater
-   Nodejs v0.10 or greater
-   ROS Hydro or greater <http://wiki.ros.org/es/ROS/Installation>

Installation
------------

1.  Make sure you have set your working space (http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment)
2.  Open a Terminal and navigate to the ROS workspace you want to use. If you just followed the ROS environment tutorial, it will be ~/catkin_ws.

   `cd ~/catkin_ws/src`

3.  Clone the FIROS git repository into your ROS workspace.

   `git clone https://github.com/ikergune/firos`

4.  Build the FIROS package with the following commands. This will create a devel and build folder under your workspace.

   `cd ~/catkin_ws`  
   `catkin_make`

5.  For convenience, you may wish to source your setup.sh script from your .bashrc so that your environment is ready as soon as you log in. e.g. 

   `echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc`

6.  Execute "source devel/setup.bash" to allow the current command line instance to use FIROS

FIROS is now installed in your robot! 

**Important Note:** The first time FIROS is launched it will ask you for root permissions to install it's dependencies


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

Robots may have some public files so users can understand some characteristics or even use their devices. All the references contained in this file can be published on the Context Broker; to do so, just follow the next example:

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

Everytime FIROS is launched or whenever it gets a notification about a new robot being connected, it looks the available *topics* on the robot. this configuration file contains a list of allowed robots and topics to be connected to this particular instance of FIROS. It also defines whether the topic is a *publisher*, when FIROS transmits data to it, or a *subscriber* in case FIROS should be listening to any incoming information on that topic.
Names corresponding to both *robots* and *topics* can also be *regular expressions* avoiding the '^' at the beginning and '$' at the end. Here is an example:

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

It is also possible to force some robot connections. This is done by adding the robot name, its topics and roles to the *robots.json* file. The role parameter must be the same as the on in the *whitelist.json* file and each topic must also contain a *type* parameter to define its role. The next file is an example of this configuration:

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

There is also a way to request the topic type, which is usefull in order to add them to the previous *robots.json* file. The following example is based on the *Turtlesim* example of ROS available here <http://wiki.ros.org/turtlesim>. Here are the steps to follow:

1.  Launch the `rostopic list` command. This will show all the registered topics.
2.  Now that you know the name of the topic, just execute the following command: `rostopic info TOPIC_NAME`. It will result on something like this:

<!-- -->
`
    rostopic info /turtle1/cmd_vel
        Type: geometry_msgs/Twist

        Publishers: None

        Subscribers:
            * /turtlesim (http://192.168.4.42 :45825/)`

This means that the robot `turtle1` is listening to data published on `/turtle1/cmd_vel` so FIROS should be publishing on it. 
The *type* of the `/turtle1/cmd_vel` is `geometry_msgs/Twist` what internally corresponds to the `geometry_msgs.msg.Twist` package

Here is another example:
`
    rostopic info /turtle1/pose
        Type: turtlesim/Pose

        Publishers:
            * /turtlesim (http://192.168.4.42 :45825/)

        Subscribers: None`

In this case, `turtle1` is publishing data on `/turtle1/pose`, so FIROS can listen to it. As seen before, the *type* `turtlesim/Pose` corresponds to the internal package `turtlesim.msg.Pose`

So we can deduce the slice of *robots.json* related to `turtle1`:

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

FIROS is listening to 2 topics in order to handle robot connections.

/FIROS/connect
--------------

Calling this topic with an empty string will make FIROS connect to new robots in case their names and topics match the ones allowed on the *whitelist.json*


/FIROS/disconnect
-----------------

Disconnecting robots from FIROS is possible by simply calling this topic with the robot name.
`/FIROS/disconnect turtle1`

API
===

FIROS has several REST entry points that are used for connecting with the context broker or getting data from FIROS.

You can find FIROS api at http://docs.firos.apiary.io/#

GET /robots
-----------

Get robots handled by FIROS with their corresponding *topics*. Each *topic* contains the `name`, `type`, `role` and `structure`:

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

Get the data published by the robot on Context Broker. It builds a query using the *NAME*, what will return data using the following structure:

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

This API handles the subscription data of the context broker. 

POST /robot/connect
-------------------

This API makes FIROS connecting to new robots in case their names and topics match the ones allowed on the *whitelist.json*

POST /robot/diconnect/NAME
--------------------------

This API forces FIROS to disconnect from the robot specified by the *NAME* parameter. It will also delete any connection and entity associated to the particular robot on the Context Broker.

POST /whitelist/write
---------------------

This API overwrites or creates entries in the robot *whitelist*. This can be done by sending the following data:

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

NOTE: In case you want to keep any element, it must be sent along with the ones to be replaced.

EXAMPLE:

Take this *whitelist.json* as a starting point:

``` javascript
{
    "turtle\\w+": {
        "publisher": ["cmd_vel"],
        "subscriber": ["pose"]
    }
}
```

Now, the following command is sent:

``` javascript
POST /whitelist/write
{
    "turtle\\w+": {
        "publisher": ["cmd_vel2"],
        "subscriber": []
    }
}
```

The resulting *whitelist* will be as follows:

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

This API removes elements from the *whitelist*. The format is as follows:

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

Take this *whitelist.json* as a starting point:

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

Now, the following json is sent:

``` javascript
POST /whitelist/remove
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

The resulting *whitelist* will look as follows:

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

This API restores the *whitelist* file to its initial state.

Ent-to-end tests
================

In order to test if firos is publishing into ContextBroker you can run the following command:

``` bash
~$ rostopic pub -1 s1 std_msgs/String "data: 'test'"  __ns:=end_end_test
```

And then:

``` bash
~$ (curl contextbroker_ip:1026/v1/queryContext -s -S --header 'Content-Type: application/json' --header 'Accept: application/json' -d @- | python -mjson.tool) <<EOF
{
        "entities": [
                {
                        "type": "ROBOT",
                        "isPattern": "true",
                        "id": "end_end_test"
                }
        ],
        "attributes": [
                "s1"
        ]
}
EOF
```

If everything went right you'll get something like this:

``` javascript
{
        "contextResponses": [
                {
                        "contextElement": {
                                "attributes": [
                                        {
                                                "name": "s1",
                                                "type": "std_msgs.msg.String",
                                                "value": "{%27firosstamp%27: 1443020619.58971, %27data%27: %27test%27}"
                                        }
                                ],
                                "id": "end_end_test",
                                "isPattern": "false",
                                "type": "ROBOT"
                        },
                        "statusCode": {
                                "code": "200",
                                "reasonPhrase": "OK"
                        }
                }
        ]
}
```

Notifications from ContextBroker to firos can be tested by running the following command in one terminal...

``` bash
rostopic echo /end_end_test/p1
```

... and the following command in another terminal:

``` bash
~$ (curl contextbroker_ip:1026/v1/updateContext -s -S --header 'Content-Type: application/json' --header 'Accept: application/json' -d @- | python -mjson.tool) <<EOF
{
          "contextElements": [
                  {
                          "type": "ROBOT",
                          "isPattern": "false",
                          "id": "end_end_test",
                          "attributes": [
                          {
                                  "name": "p1",
                                  "type": "std_msgs.msg.String",
                                  "value": "{%27data%27: %27`echo $RANDOM`%27}"
                          },
                          {
                                  "name": "COMMAND",
                                  "type": "COMMAND",
                                  "value": ["p1"]
                          }
                          ]
                  }
          ],
          "updateAction": "APPEND"
}
EOF
```

If everything went ok, in the first terminal you'll see something like this:

``` bash
data: random_number
---
```

License
=======

Firos is licensed under [MIT License](https://opensource.org/licenses/MIT).
