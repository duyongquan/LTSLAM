FROM ubuntu:20.04

ENV DEBIAN_FRONTEND=noninteractive

MAINTAINER duyongquan <quandy2020@126.com>

RUN apt-get update -y && \
    apt-get install -y \
    build-essential \
    git 
#     cmake \
#     cppcheck \
#     curl \
#     doxygen \
#     gdb \
#     google-perftools \
#     graphviz \
#     libblas-dev \
#     libboost-all-dev \
#     libcurl4-openssl-dev \
#     libfreetype6-dev \
#     liblapack-dev \
#     libpcap-dev \
#     locate \
#     lua5.3-dev \
#     nfs-common \
#     shellcheck \
#     software-properties-common \
#     sshfs \
#     subversion \
#     unzip \
#     vim \
#     wget \
#     zip && \
#     apt-get clean && rm -rf /var/lib/apt/lists/* && \
#     echo '\n\n\n'

WORKDIR /xslam

# Installer
COPY 3rdparty /tmp/3rdparty

COPY ./install/installers /tmp/installers

# RUN bash /tmp/installers/install_bazel.sh 
# RUN bash /tmp/installers/install_ceres_solver.sh
# RUN bash /tmp/installers/install_dbow3.sh
# RUN bash /tmp/installers/install_g2o.sh
# RUN bash /tmp/installers/install_gflags.sh
# RUN bash /tmp/installers/install_glog.sh
# RUN bash /tmp/installers/install_google_test.sh
# RUN bash /tmp/installers/install_gtsam.sh
# RUN bash /tmp/installers/install_opencv.sh
# RUN bash /tmp/installers/install_osqp.sh
# RUN bash /tmp/installers/install_pangolin.sh
# RUN bash /tmp/installers/install_pcl.sh
# RUN bash /tmp/installers/install_protobuf.sh
# RUN bash /tmp/installers/install_python_modules.sh
# RUN bash /tmp/installers/install_sophus.sh