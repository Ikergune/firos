FROM ros:melodic-ros-core-bionic

RUN mkdir catkin_ws \
    && mkdir catkin_ws/src \
    && mkdir catkin_ws/src/firos \
    && mkdir catkin_ws/devel \
    && mkdir catkin_ws/build

COPY . /catkin_ws/src/firos
WORKDIR /catkin_ws

# TODO Set up configuration files!


# install ros packages
RUN apt-get update && apt-get install -y ros-melodic-ros-base python python-pip
RUN pip install -r ./src/firos/requirements.txt
RUN /bin/bash -c '. /opt/ros/melodic/setup.bash; catkin_make'

CMD /bin/bash -c '. /opt/ros/melodic/setup.bash; . /catkin_ws/devel/setup.bash; rosrun firos core.py'





