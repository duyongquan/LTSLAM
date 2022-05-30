#!/usr/bin/env bash

set -e

CURR_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"
. ${CURR_DIR}/installer_base.sh

wget https://github.com/opencv/opencv/archive/refs/tags/4.5.5.zip -O opencv.zip &&
unzip opencv.zip && \
wget https://github.com/opencv/opencv_contrib/archive/refs/tags/4.5.5.zip -O opencv_contrib.zip && \
unzip opencv_contrib.zip && \
cd opencv-4.5.5 && \
mkdir build && \
cd build && \
cmake -DOPENCV_EXTRA_MODULES_PATH=../../opencv_contrib-4.5.5/modules \
    -DBUILD_TIFF=ON \
    -DBUILD_opencv_java=OFF \
    -DWITH_CUDA=OFF \
    -DENABLE_AVX=ON \
    -DWITH_OPENGL=ON \
    -DWITH_OPENCL=ON \
    -DWITH_IPP=ON \
    -DWITH_TBB=ON \
    -DWITH_EIGEN=ON \
    -DWITH_V4L=ON \
    -DBUILD_TESTS=OFF \
    -DBUILD_PERF_TESTS=OFF \
    -DCMAKE_BUILD_TYPE=RELEASE \
    -DBUILD_opencv_python3=OFF \
    -DCMAKE_INSTALL_PREFIX=$(python3.8 -c "import sys; print(sys.prefix)") \
    -DPYTHON_EXECUTABLE=$(which python3.8) \
    -DPYTHON_INCLUDE_DIR=$(python3.8 -c "from distutils.sysconfig import get_python_inc; print(get_python_inc())") \
    -DPYTHON_PACKAGES_PATH=$(python3.8 -c "from distutils.sysconfig import get_python_lib; print(get_python_lib())") .. \
&& make -j6 && make install \
&& rm ../../opencv.zip \
&& rm ../../opencv_contrib.zip \
&& rm -r ../../opencv-4.5.5 \
&& rm -r ../../opencv_contrib-4.5.5

ldconfig
ok "Successfully installed OpenCV opencv-4.5.5."
