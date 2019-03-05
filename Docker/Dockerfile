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

RUN /bin/bash -c "source /opt/ros/kinetic/setup.bash "

ENV HOME /root
WORKDIR ${HOME}
# GTSAM
RUN git clone https://bitbucket.org/gtborg/gtsam.git
RUN cd gtsam && \
    git checkout 4.0.0-alpha2 && \
    mkdir build && \
    cd build && \
    cmake -DMETIS_SHARED=ON -DGTSAM_BUILD_STATIC_LIBRARY=OFF -DGTSAM_BUILD_TESTS=OFF -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF -DCMAKE_BUILD_TYPE=Release .. && \
    make -j$(nproc) && \
    make install && \
    cd && \
    rm -r gtsam

# g2o
WORKDIR ${HOME}
RUN git clone https://github.com/RainerKuemmerle/g2o.git
RUN cd g2o && \
    git checkout 20170730_git && \
    mkdir build && \
    cd build && \
    cmake -DBUILD_LGPL_SHARED_LIBS=ON -DG2O_BUILD_APPS=OFF -DG2O_BUILD_EXAMPLES=OFF -DBUILD_SHARED_LIBS=ON -DCMAKE_BUILD_TYPE=Release .. && \
    make -j$(nproc) && \
    make install && \
    cd && \
    rm -r g2o

# libpointmatcher 
WORKDIR ${HOME}
RUN git clone https://github.com/ethz-asl/libnabo.git
#commit Apr 25 2018
RUN cd libnabo && \
    git checkout 7e378f6765393462357b8b74d8dc8c5554542ae6 && \
    mkdir build && \
    cd build && \
    cmake -DCMAKE_BUILD_TYPE=Release .. && \
    make -j$(nproc) && \
    make install && \
    cd && \
    rm -r libnabo

WORKDIR ${HOME}
RUN git clone https://github.com/ethz-asl/libpointmatcher.git
#commit Jan 19 2018
RUN cd libpointmatcher && \
    git checkout 00004bd41e44a1cf8de24ad87e4914760717cbcc && \
    mkdir build && \
    cd build && \
    cmake -DCMAKE_BUILD_TYPE=Release .. && \
    make -j$(nproc) && \
    make install && \
    cd && \
    rm -r libpointmatcher

# Clone source code
WORKDIR ${HOME}
ARG CACHE_DATE=2016-01-01
RUN git clone https://github.com/tonycar12002/rtabmap.git 
RUN apt-get update && apt-get install -y --fix-missing \
    ros-kinetic-cv-bridge


# ros library
RUN apt-get update -y
RUN apt-get update && apt-get install -y --fix-missing \
    ros-kinetic-controller-manager \
    ros-kinetic-tf2* \
    ros-kinetic-lms1xx \
    ros-kinetic-twist* \
    ros-kinetic-interactive-markers \
    ros-kinetic-interactive-marker-twist-server \
    ros-kinetic-velodyne* \
    ros-kinetic-gazebo-* \
    ros-kinetic-joint-state* \
    ros-kinetic-diff-drive-controller \
    ros-kinetic-octomap-* \
    ros-kinetic-openni2-camera \
    ros-kinetic-openni2-launch 

RUN apt-get update && apt-get install -y --fix-missing \
    ros-kinetic-rtabmap \
    ros-kinetic-rtabmap-ros \
    ros-kinetic-openslam-gmapping \
    ros-kinetic-frontier-exploration

RUN pip install scipy
    
# Setup the ZED SDK
RUN apt-get update --fix-missing
RUN apt-get install lsb-release wget less udev sudo apt-transport-https apt-utils -y
RUN wget -O ZED_SDK_Linux_Ubuntu16.run https://download.stereolabs.com/zedsdk/2.4/ubuntu_cuda9 
RUN echo ttf-mscorefonts-installer msttcorefonts/accepted-mscorefonts-eula select true | sudo debconf-set-selections
RUN chmod +x ZED_SDK_Linux_Ubuntu16.run ; ./ZED_SDK_Linux_Ubuntu16.run silent

