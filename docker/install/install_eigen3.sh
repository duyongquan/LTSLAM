#!/usr/bin/env bash

# Fail on first error.
set -e

cd /x-slam/3rdparty/eigen-3.3.7
mkdir build && cd build && cmake ..
make -j6
make install

# Clean up.
rm -rf build
