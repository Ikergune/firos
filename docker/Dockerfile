FROM ros:melodic-ros-core-bionic

RUN mkdir catkin_ws \
    && mkdir catkin_ws/src \
    && mkdir catkin_ws/src/firos \
    && mkdir catkin_ws/devel \
    && mkdir catkin_ws/build

COPY . /catkin_ws/src/firos
WORKDIR /catkin_ws

# Standard 'config.json'-Configuration for Docker 
RUN echo '{\n\
   "environment":"docker",\n\
   "docker":{\n\
      "server":{\n\
         "port":10100\n\
      },\n\
      "contextbroker":{\n\
         "address":"orion",\n\
         "port":1026,\n\
         "subscription":{\n\
            "throttling":0,\n\
            "subscription_length":300,\n\
            "subscription_refresh_delay":0.5\n\
         }\n\
      },\n\
      "log_level":"INFO",\n\
      "interface":"eth0"\n\
   }\n\
}\n' > /catkin_ws/src/firos/config/config.json

# remove descriptions/whitelist and robots configuration, since those need to be added by the user
RUN rm -f /catkin_ws/src/firos/config/robots.json && \
    rm -f /catkin_ws/src/firos/config/robotdescriptions.json && \
    rm -f /catkin_ws/src/firos/config/whitelist.json



# install ros packages
RUN apt-get update && apt-get install -y ros-melodic-ros-base python python-pip
RUN pip install -r ./src/firos/requirements.txt
RUN /bin/bash -c '. /opt/ros/melodic/setup.bash; catkin_make'


EXPOSE 10100
CMD /bin/bash -c '. /opt/ros/melodic/setup.bash; . /catkin_ws/devel/setup.bash; rosrun firos core.py'





