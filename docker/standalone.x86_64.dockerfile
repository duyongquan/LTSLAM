FROM ubuntu:20.04

ENV DEBIAN_FRONTEND=noninteractive

MAINTAINER duyongquan <quandy2020@126.com>

RUN apt-get update -y && \
    apt-get install -y \
    build-essential \
    git  \
    wget \
    cmake \
    libssl-dev \
    cppcheck \
    curl \
    doxygen \
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
    libsuitesparse-dev \
    lsb-release \
    graphviz \
    libblas-dev \
    libboost-all-dev \
    libcurl4-openssl-dev \
    libfreetype6-dev \
    liblapack-dev \
    libpcap-dev \
    locate \
    shellcheck \
    software-properties-common \
    unzip \
    vim \
    zip && \
    apt-get clean && rm -rf /var/lib/apt/lists/* && \
    echo '\n\n\n'

WORKDIR /xslam

# Installer
COPY 3rdparty /tmp/3rdparty
COPY ./install/installers /tmp/installers

RUN bash /tmp/installers/install_cmake.sh 
RUN bash /tmp/installers/install_gflags.sh
RUN bash /tmp/installers/install_google_test.sh
RUN bash /tmp/installers/install_ceres_solver.sh
RUN bash /tmp/installers/install_g2o.sh
RUN bash /tmp/installers/install_pangolin.sh
RUN bash /tmp/installers/install_dbow3.sh
RUN bash /tmp/installers/install_eigen3.sh

# RUN bash /tmp/installers/install_opencv.sh
# RUN bash /tmp/installers/install_python_modules.sh
