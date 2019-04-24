# API

FIROS has several REST entry points that are used for connecting with the context broker or getting data from FIROS.

You can find the old FIROS API [here](http://docs.FIROS.apiary.io/#) (OLD)

## GET /robots

Get robots handled by FIROS with their corresponding _topics_. Each _topic_ contains the `name`, `type`, `pubsub`-Role
and `structure` as follows:

```json
[
    {
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
        "name": "turtle1"
    }
]
```

## GET /robot/NAME

Gets the data which is published/subscribed by the robot in the Context-Broker. Here the contents of the Context Broker
is shown.

Here as an example: the content of `turtlesim` with its publishing topic `pose`:

```json
{
    "id": "turtle1",
    "type": "MyROBOT",
    "descriptions": {
        "type": "object",
        "value": {
            "MySanatiyValue": {
                "type": "number",
                "value": 1
            },
            "SomeReferenceLink": {
                "type": "string",
                "value": "http://wiki.ros.org/ROS/Tutorials/UnderstandingNodes"
            }
        },
        "metadata": {}
    },
    "pose": {
        "type": "turtlesim.Pose",
        "value": {
            "y": {
                "type": "number",
                "value": 5.544444561
            },
            "x": {
                "type": "number",
                "value": 5.544444561
            },
            "linear_velocity": {
                "type": "number",
                "value": 0
            },
            "theta": {
                "type": "number",
                "value": 0
            },
            "angular_velocity": {
                "type": "number",
                "value": 0
            }
        },
        "metadata": {
            "dataType": {
                "type": "dataType",
                "value": {
                    "y": "float32",
                    "x": "float32",
                    "linear_velocity": "float32",
                    "theta": "float32",
                    "angular_velocity": "float32"
                }
            }
        }
    }
}
```

## POST /firos

This API handles the subscription data of the context broker.

## POST /robot/connect

This call restores the configuration of FIROS. Disconnected robots are connected again.

## POST /robot/disconnect/NAME

This call forces FIROS to disconnect from the robot specified by the **NAME** parameter. If Publisher, FIROS will no
longer publish its data. If Subscriber, FIROS will not push the Information into the ROS-World

## POST /whitelist/write

This API overwrites or creates entries in the robot _whitelist_. This can be done by sending the following data:

```json
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

Take this _whitelist.json_ as a starting point:

```json
{
    "turtle\\w+": {
        "publisher": ["cmd_vel"],
        "subscriber": ["pose"]
    }
}
```

Now, the following command is sent:

```json
POST /whitelist/write
{
    "turtle\\w+": {
        "publisher": ["cmd_vel2"],
        "subscriber": []
    }
}
```

The resulting _whitelist_ will be as follows:

```json
{
    "turtle\\w+": {
        "publisher": ["cmd_vel2"],
        "subscriber": []
    }
}
```

## POST /whitelist/remove

This API removes elements from the _whitelist_. The format is as follows:

```json
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

Take this _whitelist.json_ as a starting point:

```json
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

```json
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

The resulting _whitelist_ will look as follows:

```json
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

## POST /whitelist/restore

This API restores the _whitelist_ file to its initial state.
