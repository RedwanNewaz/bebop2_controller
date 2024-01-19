FROM osrf/ros:noetic-desktop

RUN apt update && apt install -y \
    libavahi-client-dev python-is-python3

RUN mkdir -p /home/airlab/catkin_ws/src

WORKDIR /home/airlab/catkin_ws

COPY ./src/parrot_arsdk /home/airlab/catkin_ws/src/parrot_arsdk

RUN /usr/bin/bash -c "source /opt/ros/noetic/setup.bash && catkin_make"

COPY ./src/bebop_autonomy /home/airlab/catkin_ws/src/bebop_autonomy

RUN export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/home/airlab/catkin_ws/devel/lib/parrot_arsdk

RUN rosdep install --from-paths src --ignore-src -r -y

RUN /usr/bin/bash -c "source /home/airlab/catkin_ws/devel/setup.bash && catkin_make; exit 0"

RUN /usr/bin/bash -c "source /home/airlab/catkin_ws/devel/setup.bash && catkin_make"

# ENTRYPOINT [ "/home/airlab/catkin_ws/devel/setup.bash" ]