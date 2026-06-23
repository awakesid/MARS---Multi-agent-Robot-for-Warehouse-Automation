FROM osrf/ros:jazzy-desktop

# Install dependencies
RUN apt-get update && apt-get install -y \
    python3-pip \
    ros-jazzy-cv-bridge \
    ros-jazzy-image-transport \
    ros-jazzy-rqt-image-view \
    python3-colcon-common-extensions \
    tmux \
    v4l-utils \
    && rm -rf /var/lib/apt/lists/*

RUN pip3 install opencv-contrib-python --no-deps --break-system-packages

# Auto-source ROS in every terminal
RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc
RUN echo "source /root/mars_ws/install/setup.bash 2>/dev/null || true" >> /root/.bashrc

WORKDIR /root/mars_ws
