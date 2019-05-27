# Installion From Scratch With ROS and catkin

To Install Firos you first need to follow this [Installaion Instuctions](http://wiki.ros.org/ROS/Installation). ROS is
needed for FIROS, since it imports `ROS-messages` and uses other specific `ROS-Executables` like `rospy` or `rostopic`.
You need to [create a catkin-workspace](http://wiki.ros.org/catkin/Tutorials/create_a_workspace) to be able to create a
ROS-Node out of FIROS.

You might also consider to set up a [contextbroker](https://fiware-orion.readthedocs.io/en/master/), so that FIROS can
publish and subscribe on it. If a contextbroker is not available you can quickly set one up via
[Docker](https://docs.docker.com/install/overview/) and use a `docker-compose.yml` as
[here](https://hub.docker.com/r/fiware/orion/) to start one.

## Cloning this Project

After you have set up ROS and created a catkin-workspace you can finally clone this repository, install its dependencies
and create the FIROS-Node as follows:

```shell
# Clone Repository
cd "catkin_workspace_base_directory"/src
git clone --recursive https://github.com/iml130/firos.git
cd "catkin_workspace_base_directory"/src/firos

# Install Dependencies
-pip install -r requirements.txt

# Make Node
cd "catkin_workspace_base_directory"
catkin_make
```

**Note**:

-   FIROS uses git submodules (which is required to run properly). Newer versions of git can clone submodules via the
    `--recursive` option
-   Also check whether your local submodule-folder (currently in `firos/include/FiwareObjectConverter` and
    `firos/include/genpy`) contains files to be sure that everything was cloned.

## Basic Configuration of FIROS

FIROS won't start if you just run the node. Some basic configuration need to be set prior. You can find an
example-configuration-folder in `config`. The `config.json`-file should contain something like:

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

You need to specifiy, which environment you want to use. In this example the environment-configuration `"mobile"` is
shown but the environment-configuration `"local"` (also somewhere in this file) is used. Specify your own
environment-configuration, or edit one to your needs. The values for `"contextbroker->adress"`, `"contextbroker->port"`
and `"interface"` need to be set in every environment-configuration. The Information from the contextbroker can be
retrieved by its configuration. If you are running a local instance, `"contextbroker->adress"` can also be set to
`"localhost"`. You can retrieve information about your network interface via:

> ip link

Just set it appropriately.

This is the absolute minimum configuration you need to do in order to be able to start up FIROS. To actually publish and
subscribe to ROS-Topics you should checkout [Configuration-Files](configuration-files.md) or the
[Turtlesim-Example](turtlesim-example.md).

## Run FIROS

Just execute:

> rosrun firos core.py

or

> python firos/core.py

to execute FIROS with Python2

Firos should function via Python3. You can try it via:

> python3 firos/core.py

## Troubleshooting

### Dependency XY is missing

FIROS uses e.g. `requests` which is not a standard python package
([ref](http://docs.python-requests.org/en/master/dev/philosophy/#standard-library)). In this case you might already have
it installed. If not use your package-manager like `apt`, `pacman`, `pip` , `...` to add it to your machine. Usually all
needed packages are inside `requirements.txt`

# Installation via Docker

There exists a FIROS-Docker-Version which currently can be build locally. This installation only requires
[Docker](https://docs.docker.com/install/).

## Cloning this Project

During or after the Docker-Installation you need to clone this repository via:

```shell
git clone --recursive https://github.com/iml130/firos.git
```

Please check whether the folders `firos/include/FiwareObjectConverter` and `firos/include/genpy` contains any content.
If not, the submodules were not initialized successfully and you might need to take a look at
[this](https://git-scm.com/docs/git-submodule)

After you cloned this repository you have two options to start up FIROS:

### Using `docker build`

Beginning from the base of this repository, FIROS can be built via docker using:

> docker build -f ./docker/Dockerfile --tag firos:localbuild .

This will create an image with a pre-configured `config.json` which requires the Orion-ContextBroker. Before running
this image, you need to specify a `robots.json`. Information on how to create the configuration-files can be found in
[Configuration-Files](configuration-files.md) or in the [Turtlesim-Example](turtlesim-example.md). An
example-pre-configured configuration for docker can be found in `firos/docker/docker-config`

Assuming you have a network `finet` (`-> "firos-net"`): You need to start a roscore, MongoDB, the Orion-ContextBroker
and afterwards FIROS like this:

```shell
# Starting roscore
docker run -it --net finet --name rosmaster ros:melodic-ros-core roscore

# Starting mongodb
docker run --net finet --name mongodb mongo:3.4

# Starting Orion-ContextBroker and link to mongodb
docker run -it --rm --net finet --name orion --link mongodb -p 1026:1026 fiware/orion -dbhost mongodb

# Starting firos (Set the paths for the needed Configuration-Files here!)
docker run -it --net finet --name firos \
    -p 10100:10100 \
    --env ROS_MASTER_URI=http://rosmaster:11311 \
    -v CONFIG_FILE_ROBOTS:/catkin_ws/src/firos/config/robots.json \
    -v CONFIG_FILE_WHITELIST:/catkin_ws/src/firos/config/whitelist.json \
    firos:localbuild
```

After this FIROS is ready to publish data and subscribe onto the local Orion-ContextBroker.

### Using `docker-compose`

The `docker-compose.yml` can be located inside the `docker`-folder at the base of this repository. Before executing the
compose-file you need to configure the configurations-files, which this docker-image uses in
`firos/docker/docker-config`. Please have a look at [Configuration-Files](configuration-files.md) or the
[Turtlesim-Example](turtlesim-example.md). The folder contains a basic example with `turtlesim` and can be used as is.

If everything is set up, execute inside the `docker`-folder:

> docker-compose up

This launches the Orion-Context-Broker (named `orion`), a `roscore`-Instance (named `rosmaster`) and FIROS (named
`firos`) with its specific configuration inside `docker-config` with a netowrk (like `docker_default`). The Ports:
`10100` and `1026` are also exposed to the host-machine.

### Adding another ROS-Application into this Environment

In order to add another ROS-Application into this environment you can either write another `docker-compose.yml` which
includes the environment-variable `"ROS_MASTER_URI=http://rosmaster:11311"` with its correspoding network
`net: "docker_default"` or call the correspoding `docker run` command:

```shell
docker run --net docker_default --name YOUR_NAME --env ROS_MASTER_URI=http://rosmaster:11311 YOUR_IMAGE:NAME_HERE
```
