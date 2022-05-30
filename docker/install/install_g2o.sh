#!/usr/bin/env bash

# Fail on first error.
set -e

cd /x-slam/3rdparty/g2o
mkdir build && cd build && cmake ..
make -j6
make install

# Clean up.
rm -rf build
