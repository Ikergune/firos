# Installion From Scratch With ROS and catkin

To Install Firos you first need to follow this [Installaion Instuctions](http://wiki.ros.org/ROS/Installation). ROS is needed for FIROS, since it imports `ROS-messages`. You also need to [create a catkin-workspace](http://wiki.ros.org/catkin/Tutorials/create_a_workspace) to be able to create a ROS-Node out of FIROS.

You might also consider to set up a [contextbroker](https://fiware-orion.readthedocs.io/en/master/), so that FIROS can publish and subscribe on it. If a contextbroker is not available you can quickly set one up via [Docker](https://docs.docker.com/install/overview/) and use a `docker-compose.yml` as [here](https://hub.docker.com/r/fiware/orion/) to start a contextbroker.

## Cloning this Project
After you have set up ROS and created a catkin-workspace you can finally clone this repository and create the FIROS-Node as follows:

> cd "catkin_workspace_base_directory"/src

> git clone --recursive https://github.com/iml130/firos.git

> cd "catkin_workspace_base_directory"

> catkin_make

**Note**:
* FIROS is using a git submodule (which is required to run properly). Newer versions of git can clone submodules via the `--recursive` option
* Also check wheather your local submodule-folder (currently in `firos/include`) contains files to be sure that everything was cloned.


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

You need to specifiy, which environment you want to use. In this example the environment-configuration `"mobile"` is shown but the environment-configuration `"local"` (also somewhere in this file) is used. Specify your own environment-configuration, or edit one to your needs. The values for `"contextbroker->adress"`, `"contextbroker->port"` and `"interface"` need to be set in every environment-configuration. The Information from the contextbroker can be retrieved by its configuration. If you are running a local instance, `"contextbroker->adress"` can also be set to `"localhost"`. You can retreive information about your network interface via:

> ip link

Just set it appropriately.

This is the absolute minimum configuration you need to do in order to be able to start up FIROS. To actually publish and subscribe to ROS-Topics you should checkout [Configuration-Files](../user/configuration-files.md) or the [Turtlesim-Example](../user/turtlesim-example.md).



## Run FIROS
To run the FIROS-Node you first need to append the `PYTHONPATH`-Environment-Variable as follows:

>PYTHONPATH+=:/FULL_PATH_TO_THIS_REPO/firos/include/FiwareObjectConverter/

Optionally to execute FIROS with PYTHON3 you need to add:
>PYTHONPATH+=:/FULL_PATH_TO_THIS_REPO/firos/include/genpy/

**NOTE:** We try to omit this appending in the future, for an easier access to FIROS


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

