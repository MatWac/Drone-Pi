FROM ros:humble
ARG USERNAME=matwac
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Create the user
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

# Update and install necessary packages
RUN apt-get update && apt-get upgrade -y \
    && apt-get install -y python3-pip x11-apps \
    && pip install icm20948 pyserial

# Install ROS and rqt tools
RUN apt-get install -y ros-${ROS_DISTRO}-rqt* ros-${ROS_DISTRO}-robot-localization

# Environment setup
RUN echo "source /opt/ros/humble/setup.bash" >> /home/${USERNAME}/.bashrc \
    && echo "source /home/ws/install/setup.bash" >> /home/${USERNAME}/.bashrc

ENV SHELL /bin/bash
ENV DISPLAY=:0

# Set the default user
USER $USERNAME
CMD ["/bin/bash"]
