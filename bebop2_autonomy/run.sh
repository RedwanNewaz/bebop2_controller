docker run -it --rm  -v $(pwd)/src:/home/airlab/catkin_ws/src -w /home/airlab/catkin_ws --net host osrf/ros:noetic-desktop bash 
# https://github.com/antonellabarisic/parrot_arsdk/tree/noetic_dev

# libavahi-client-dev python-is-python3
# /home/airlab/catkin_ws/build/parrot_arsdk/arsdk
# export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/home/airlab/catkin_ws/devel/lib/parrot_arsdk
# rosdep install --from-paths src --ignore-src -r -y