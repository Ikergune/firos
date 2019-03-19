# Installion From Scratch With ROS and catkin

To Install Firos you first need to follow this [Installaion Instuctions](http://wiki.ros.org/ROS/Installation). ROS is needed for FIROS, since it imports `ROS-messages`. You also need to [create a catkin-workspace](http://wiki.ros.org/catkin/Tutorials/create_a_workspace) to be able to create a ROS-Node out of FIROS.

You might also consider to set up a [contextbroker](https://fiware-orion.readthedocs.io/en/master/), so that FIROS can publish and subscribe on it. If a contextbroker is not available you can quickly set one up via [Docker](https://docs.docker.com/install/overview/) and use a `docker-compose.yml` as [here](https://hub.docker.com/r/fiware/orion/) to start a contextbroker.

## Cloning this Project
After you have set up ROS and created a catkin-workspace you can finally clone this repository and create the FIROS-Node as follows:

```shell
cd "catkin_workspace_base_directory"/src
git clone --recursive https://github.com/iml130/firos.git
cd "catkin_workspace_base_directory"
catkin_make
```


**Note**:
*   FIROS is using a git submodule (which is required to run properly). Newer versions of git can clone submodules via the `--recursive` option
*   Also check wheather your local submodule-folder (currently in `firos/include`) contains files to be sure that everything was cloned.


## Basic Configuration of FIROS
FIROS won't start if you just run the node. Some basic configuration need to be set prior. You can find an example-configuration-folder in `config`.   The `config.json`-file should contain something like: 

```json
{
  "environment": "local",

  "mobile": {
    "server": {
        "port": 10100
    },
    "contextbroker": {
        "address"   : "192.168.43.159",
        "port"      : 1026,
        "subscription": {
          "throttling": 0,
          "subscription_length": 300,
          "subscription_refresh_delay": 0.5
        }
    },
    "log_level": "INFO",
    "interface": "wlan0"
  },    

  ...
}
```

You need to specifiy, which environment you want to use. In this example the environment-configuration `"mobile"` is shown but the environment-configuration `"local"` (also somewhere in this file) is used. Specify your own environment-configuration, or edit one to your needs. The values for `"contextbroker->adress"`, `"contextbroker->port"` and `"interface"` need to be set in every environment-configuration. The Information from the contextbroker can be retrieved by its configuration. If you are running a local instance, `"contextbroker->adress"` can also be set to `"localhost"`. You can retrieve information about your network interface via:

> ip link

Just set it appropriately.

This is the absolute minimum configuration you need to do in order to be able to start up FIROS. To actually publish and subscribe to ROS-Topics you should checkout [Configuration-Files](configuration-files.md) or the [Turtlesim-Example](turtlesim-example.md).



## Run FIROS
**Optionally:** to execute FIROS with PYTHON3 you need to add:
>PYTHONPATH+=:/FULL_PATH_TO_THIS_REPO/firos/include/genpy/

**NOTE:** We try to omit this appending in the future, for an easier access to FIROS with python3


Then just execute:
>rosrun firos core.py 

or
>python firos/core.py

to execute FIROS with Python2



## Troubleshooting

### Dependency XY is missing
FIROS uses e.g. `requests` which is not a standard python package ([ref](http://docs.python-requests.org/en/master/dev/philosophy/#standard-library)). In this case you might already have it installed. If not use your package-manager like `apt`, `pacman`, `pip` , ... to add it to your machine.


# Installation via Docker

## WIP


---
# Old Installation-Information


Cloning This Project
================
This project uses a submodule. Depending on your git version you might need to do the following:

Clone this project via: 
```sh
git clone --recursive https://github.com/iml130/firos.git
# If thretre Folder firos/include/FiwareObjectConverter is still empty do:
git submodule update --init --recursive
```
or
```sh
git clone https://github.com/iml130/firos.git
git submodule update --init --recursive
```


The `FiwareObejctConverter` is our own `Python-Object<->Fiware-JSON`-Converter. 

Also, make sure you are using the correct version of the submodule.
You can read more about git submodules [here](https://git-scm.com/book/en/v2/Git-Tools-Submodules).

Installing FIROS via `catkin_make`
================

Requirements
------------

-   Ubuntu
-   Python 2.7 or greater
-   ROS Hydro or greater <http://wiki.ros.org/es/ROS/Installation>

Installation
------------

1.  Make sure you have set [your working space](http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment)
2.  Open a Terminal and navigate to the ROS workspace you want to use. If you just followed the ROS environment tutorial, it will be ~/catkin_ws.

   >cd ~/catkin_ws/src

3.  Clone the FIROS git repository into your ROS workspace.

   >git clone --recursive git@github.com:iml130/firos.git

4.  Build the FIROS package with the following commands. This will create a devel and build folder under your workspace.

   >cd ~/catkin_ws
   >catkin_make

5.  For convenience, you may wish to source your setup.sh script from your .bashrc so that your environment is ready as soon as you log in. e.g. 

   >echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc

   If you don't want to edit you `.bashrc`, you can also just execute:

   >source ~/catkin_ws/devel/setup.bash

6.  Execute "source devel/setup.bash" to allow the current command-line instance to setup the sources you have inside yout `catkin_ws`

FIROS is now ready to be used!

Configuring FIROS
=================

FIROS has several configuration files located at *src/FIROS/config*.

config.json
-----------

This file contains the configuration related to FIROS launching environment. Here is a description of each parameter:

-   *environment*: This parameter distinguishes which configuration set inside this json should be used. FIROS will use an environment's configuration based this value, but there can be as many environments as you want.
-   *server*: This contains the Port of the FirosServer (The Port, where the `GET`- and `POST`-Operations can be executed)
-   *contextbroker*: Contains information related to the Context broker configuration
    -   *address*: Context broker's (IP-) address
    -   *port*: Context broker's port
    -   *subscription*: Context broker's subscription information
        -   *throttling*: The throttling in seconds. The contextbroker sends another update of the entity after the `throttling`-seconds have passed
        -   *subscription_length*: The subscription expiration time (in seconds)
        -   *subscription_refresh_delay*: The subscription refresh delay (between 0 and 1) to re-subscribe to the contextbroker -> After *subscription_length* * *subscription_refresh_delay* seconds the subscription is refreshed.
-   *interface*: Network configuration of the card in use
    -   *public*: Public IP. Do not forget to redirect the proper ports in your network
    -   *wlan0, et0, tun0*, etc: Different network interface configuration.
    -   If you experience problems with the public interface, feel free to set the interface-name directly, like *wlan0*, *enp0*, *eth0*. etc.. 
-   *log_level*: It represents the verbosity of the logging system for FIROS. Available options are as follows: *"NONE", "INFO", "DEBUG" ,"WARNING", "ERROR"* and *"CRITICAL"*

Here is an example of a *config.json* file for a *local* environment:
``` json
{
  "environment": "local",

  "local": {
    "server": {
        "port": 10100
    },
    "contextbroker": {
        "address"   : "192.168.0.101",
        "port"      : 1026,
        "subscription": {
          "throttling": 3, 
          "subscription_length": 300,
          "subscription_refresh_delay": 0.9
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

``` json
{
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
}
```

whitelist.json
--------------

Every time FIROS is launched or whenever it gets a notification about a new robot being connected, it looks the available *topics* on the robot. this configuration file contains a list of allowed robots and topics to be connected to this particular instance of FIROS. It also defines whether the topic is a *publisher*, when FIROS transmits data to it, or a *subscriber* in case FIROS should be listening to any incoming information on that topic.
Names corresponding to both *robots* and *topics* can also be *regular expressions* avoiding the '^' at the beginning and '$' at the end. Here is an example:

``` json
{
	"turtle\\w+": {
		"publisher": ["cmd_vel"],
		"subscriber": ["pose"]
	},
	"robot\\w+": {
		"publisher": ["cmd_vel.*teleop", ".*move_base/goal", ".*move_base/cancel"],
		"subscriber": [".*move_base/result"]
	}
}
```

robots.json
-----------

It is also possible to force some robot connections. This is done by adding the robot name, its topics and roles to the *robots.json* file. The role parameter must be the same as the on in the *whitelist.json* file and each topic must also contain a *type* parameter to define its role. The next file is an example of this configuration:

``` json
{
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
}
```

The `type`'s Publish and Subscribe are always from Context Broker's point of view. If subscribed, the framework pushes ROS-Information to the Context-Broker. ROS-Messages to the robots are sent to the contextbroker by the `type` publish. Thus a communication betweeen robots can be established through the Context-Broker. Even Other Non-ROS-Application can control robots via the Context-Broker.

Here is an Example-Configuration between the communication of `turtlesim`
on *Machine1* and `teleop_twist_keyboard` on *Machine2*:

```json
Machine1:
{
	"turtle1": {
		"topics": {
			"cmd_vel": {
				"msg": "geometry_msgs.msg.Twist",
				"type": "publisher"
			}
}
Machine2:
{
	"turtle1": {
		"topics": {
			"cmd_vel": {
				"msg": "geometry_msgs.msg.Twist",
				"type": "Subscriber"
			}
}

```


Getting Topic Types
===================

There is also a way to request the topic type, which is useful in order to add them to the previous *robots.json* file. The following example is based on the *Turtlesim* example of ROS available here <http://wiki.ros.org/turtlesim>. Here are the steps to follow:

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

``` json
{
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
}
```
