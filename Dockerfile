# ROS 2 Jazzy + Gazebo Harmonic with GUI support (WSLg / D3D12 GPU acceleration)
FROM osrf/ros:jazzy-desktop-full

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
        mesa-utils \
        ros-jazzy-ros-gz \
        ros-jazzy-gz-ros2-control \
        ros-jazzy-ros-gz-sim-demos \
        python3-colcon-common-extensions \
        python3-rosdep \
        git nano sudo bash-completion \
    && rm -rf /var/lib/apt/lists/*

# Reuse the image's 'ubuntu' user (UID/GID 1000, same as the WSL host user)
ARG USERNAME=ubuntu
RUN echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/${USERNAME} \
    && chmod 0440 /etc/sudoers.d/${USERNAME} \
    && (getent group render || groupadd render) \
    && usermod -aG video,render ${USERNAME}

USER ${USERNAME}
RUN mkdir -p /home/${USERNAME}/ws/src \
    && { echo 'source /opt/ros/jazzy/setup.bash'; \
         echo '[ -f ~/ws/install/setup.bash ] && source ~/ws/install/setup.bash'; \
         echo 'source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash 2>/dev/null'; \
       } >> /home/${USERNAME}/.bashrc

WORKDIR /home/${USERNAME}/ws
CMD ["bash"]
