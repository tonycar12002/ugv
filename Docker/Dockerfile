FROM nvidia/cudagl:9.0-devel-ubuntu16.04

# ros
# install bootstrap tools
RUN apt-get update -y
RUN apt-get install -y bash git make wget sudo build-essential \
    cmake net-tools iputils-ping network-manager \
    vim python-pip python-dev \
    libjpeg-dev libtiff-dev libpng-dev libfltk1.3-dev fluid \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install --no-install-recommends -y \
    python-rosdep \
    python-rosinstall \
    python-vcstools \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -q -y \
    dirmngr \
    gnupg2 \
    lsb-release \
    && rm -rf /var/lib/apt/lists/*


# setup environment
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8

# setup keys
RUN apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys 421C365BD9FF1F717815A3895523BAEEB01FA116

# setup sources.list
RUN echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -sc` main" > /etc/apt/sources.list.d/ros-latest.list

# bootstrap rosdep

RUN rosdep init \
    && rosdep update

# install ros packages
ENV ROS_DISTRO kinetic
RUN apt-get update && apt-get install -y --fix-missing\
    ros-kinetic-ros-core=1.3.2-0* \
    ros-kinetic-desktop-full \
    ros-kinetic-robot \
    ros-kinetic-perception \
    ros-kinetic-navigation \
    ros-kinetic-robot-localization \
    ros-kinetic-roslint \
    ros-kinetic-hector-trajectory-server \
    ros-kinetic-joystick-drivers \
    ros-kinetic-rqt \
    ros-kinetic-rqt-common-plugins \
    ros-kinetic-web-video-server \
    ros-kinetic-rosserial-arduino \
    ros-kinetic-rosserial \
    ros-kinetic-geodesy \
    && rm -rf /var/lib/apt/lists/*

# library
RUN apt-get update -y
RUN apt-get install -y --fix-missing \
    libboost-all-dev \
    cmake\
    git \
    bash-completion \
    libglu1-mesa-dev \
    freeglut3-dev \
    mesa-common-dev

# gstam
WORKDIR /root
RUN git clone https://bitbucket.org/gtborg/gtsam.git && \
    cd gtsam && mkdir build && cd build && cmake .. && make install


# ros library
RUN apt-get update -y
RUN apt-get update && apt-get install -y --fix-missing \
    ros-kinetic-controller-manager \
    ros-kinetic-tf2* \
    ros-kinetic-rtabmap* \
    ros-kinetic-lms1xx \
    ros-kinetic-twist* \
    ros-kinetic-interactive-markers \
    ros-kinetic-interactive-marker-twist-server \
    ros-kinetic-velodyne* \
    ros-kinetic-gazebo-* \
    ros-kinetic-joint-state* \
    ros-kinetic-diff-drive-controller 
