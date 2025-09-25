FROM osrf/ros:galactic-desktop

# Add vscode user with same UID and GID as your host system
# (copied from https://code.visualstudio.com/remote/advancedcontainers/add-nonroot-user#_creating-a-nonroot-user)
ARG USERNAME=vscode
ARG USER_UID=1000
ARG USER_GID=$USER_UID
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME
# Switch from root to user
USER $USERNAME

# Add user to video group to allow access to webcam
RUN sudo usermod --append --groups video $USERNAME

# Install Git
RUN sudo apt install -y git

# Rosdep update
RUN rosdep update

# Update all packages
RUN sudo apt update && sudo apt upgrade -y

# Source the ROS setup file
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc

# Additional dependency to fix zed camera not showing up in rviz
RUN sudo apt install -y python3-tk

RUN sudo apt update && sudo apt upgrade -y

# Install EUFS dependencies (because rosdep doesn't always work for some reason)
RUN sudo apt install -y ros-galactic-xacro \
        ros-galactic-joint-state-publisher python3-scipy \
        ros-galactic-gazebo-plugins libyaml-cpp-dev \
        ros-galactic-ackermann-msgs gazebo11 ros-galactic-gazebo-dev ros-galactic-gazebo-ros
        

# Install pip, wget and zstd
RUN sudo apt install -y python3-pip 
# wget zstd

# # Install ZED SDK
# RUN wget -q --show-progress --progress=bar:force:noscroll https://download.stereolabs.com/zedsdk/4.2/cu12/ubuntu20 -O ZED_SDK_Ubuntu20.run && \
#         chmod +x ZED_SDK_Ubuntu20.run && \
#         ./ZED_SDK_Ubuntu20.run -- silent && \
#         rm ZED_SDK_Ubuntu20.run