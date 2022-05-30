FROM ros:noetic-perception

MAINTAINER duyongquan <quandy2020@126.com>

# sources
RUN echo "deb https://mirrors.ustc.edu.cn/ubuntu/ bionic main restricted universe multiverse" >> /etc/apt/sources.list
RUN echo "deb-src https://mirrors.ustc.edu.cn/ubuntu/ bionic main restricted universe multiverse" >> /etc/apt/sources.list
RUN echo "deb https://mirrors.ustc.edu.cn/ubuntu/ bionic-updates main restricted universe multiverse" >> /etc/apt/sources.list
RUN echo "deb-src https://mirrors.ustc.edu.cn/ubuntu/ bionic-updates main restricted universe multiverse" >> /etc/apt/sources.list
RUN echo "deb https://mirrors.ustc.edu.cn/ubuntu/ bionic-backports main restricted universe multiverse" >> /etc/apt/sources.list
RUN echo "deb-src https://mirrors.ustc.edu.cn/ubuntu/ bionic-backports main restricted universe multiverse" >> /etc/apt/sources.list
RUN echo "deb https://mirrors.ustc.edu.cn/ubuntu/ bionic-security main restricted universe multiverse" >> /etc/apt/sources.list
RUN echo "deb-src https://mirrors.ustc.edu.cn/ubuntu/ bionic-security main restricted universe multiverse" >> /etc/apt/sources.list
RUN echo "deb https://mirrors.ustc.edu.cn/ubuntu/ bionic-proposed main restricted universe multiverse" >> /etc/apt/sources.list
RUN echo "deb-src https://mirrors.ustc.edu.cn/ubuntu/ bionic-proposed main restricted universe multiverse" >> /etc/apt/sources.list

RUN apt-get update -y && \
    apt-get install -y --no-install-recommends apt-utils \
    build-essential \
    git  \
    wget \
    cmake \
    libssl-dev \
    cppcheck \
    curl \
    doxygen \
    libboost-all-dev \
    libatlas-base-dev \
    libsuitesparse-dev \
    libglew-dev \
    stow \
    gdb \
    google-perftools \
    google-mock \
    libcairo2-dev \
    libgoogle-glog-dev \
    liblua5.3-dev \
    lsb-release \
    graphviz \
    libblas-dev \
    libcurl4-openssl-dev \
    libfreetype6-dev \
    liblapack-dev \
    libpcap-dev \
    software-properties-common \
    unzip \
    vim \
    locate \
    libfmt-dev \
    python3-pip \
    yasm \
    pkg-config \
    libjpeg-dev \
    libtiff-dev \
    libpng-dev \
    libavcodec-dev \
    libavformat-dev \
    libswscale-dev \
    libv4l-dev \
    gfortran \
    libtbb2 \
    libtbb-dev \
    libpq-dev \
    ros-noetic-tf2-eigen \
    ros-noetic-urdf \
    zip && \
    apt-get clean && rm -rf /var/lib/apt/lists/* && \
    echo '\n\n\n' 

WORKDIR /x-slam

# Installer
COPY ./install /tmp/install
COPY ./LTSLAM /x-slam

# RUN bash /tmp/installers/install_cmake.sh
RUN bash /tmp/install/install_python_modules.sh
RUN bash /tmp/install/install_eigen3.sh
RUN bash /tmp/install/install_sophus.sh
RUN bash /tmp/install/install_gflags.sh
RUN bash /tmp/install/install_google_test.sh
RUN bash /tmp/install/install_ceres_solver.sh
RUN bash /tmp/install/install_abseil-cpp.sh
RUN bash /tmp/install/install_g2o.sh
RUN bash /tmp/install/install_opencv.sh
RUN bash /tmp/install/install_protobuf.sh
RUN bash /tmp/install/install_pangolin.sh
RUN bash /tmp/install/install_dbow3.sh
RUN bash /tmp/install/install_osqp.sh

# Build X-SLAM
ENV TERM xterm
ENV PYTHONIOENCODING UTF-8
