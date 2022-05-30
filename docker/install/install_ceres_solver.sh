#!/usr/bin/env bash

# Fail on first error.
set -e

# Clean up.
rm -rf build

cd /x-slam/3rdparty/ceres-solver
mkdir build && cd build && cmake ..
make -j6
make install

# Clean up.
rm -rf build
