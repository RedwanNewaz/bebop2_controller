FROM osrf/ros:noetic-desktop

RUN apt update && apt install -y \
    ros-noetic-joy \
    ros-noetic-robot-localization \
    ros-noetic-apriltag-ros \
    python3-catkin-tools

WORKDIR /home/airlab/drone_ws


# nvidia-container-runtime
ENV NVIDIA_VISIBLE_DEVICES \
    ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES \
    ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics


ENTRYPOINT [ "/ros_entrypoint.sh" ]
CMD ["/bin/bash"]
