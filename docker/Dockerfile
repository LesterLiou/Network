FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive
ENV DISPLAY=:0
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8
ENV ROS_DISTRO=humble

RUN apt-get update && apt-get install -y --no-install-recommends \
    sudo nano vim udev \
    build-essential \
    netperf iproute2 iputils-ping net-tools iperf3 \
    python3 python3-pip python3-gi \
    xauth x11-utils x11-xserver-utils \
    tcpdump traceroute dnsutils \
    git curl unzip \
    tzdata \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

RUN pip3 install --no-cache-dir flent matplotlib numpy pandas

# ======================== time zone ========================
RUN ln -sf /usr/share/zoneinfo/Asia/Taipei /etc/localtime && \
    echo "Asia/Taipei" > /etc/timezone

##################### ROS2 Key #####################
RUN apt-key del F42ED6FBAB17C654 || true && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o \
        /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
        http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
        > /etc/apt/sources.list.d/ros2.list

##################### ROS 2 Humble ＋ colcon ＋ rosdep #####################
RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-humble-desktop-full \
        python3-rosdep \
        python3-colcon-common-extensions \
        python3-argcomplete && \
    apt-get clean && rm -rf /var/lib/apt/lists/*

RUN rosdep init || true && rosdep update

##################### 安裝 cv_bridge #####################
RUN apt-get update && \
    apt-get install -y --no-install-recommends ros-humble-cv-bridge && \
    apt-get clean && rm -rf /var/lib/apt/lists/*

##################### 建立工作空間 #####################
WORKDIR /root/NETWORK/ros2_ws
RUN mkdir -p src
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc && \
    echo "source /root/NETWORK/ros2_ws/install/setup.bash" >> ~/.bashrc && \
    echo 'alias r="cd /root/NETWORK/ros2_ws && colcon build && source install/setup.bash"' >> /root/.bashrc


CMD ["bash"]
