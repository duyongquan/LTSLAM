#!/usr/bin/env bash

# Fail on first error.
set -e

wget https://github.com/Kitware/CMake/releases/download/v3.22.4/cmake-3.22.4.tar.gz

tar -xf cmake-3.22.4.tar.gz
cd cmake-3.22.4
mkdir build && cd build && cmake ..
make -j6
make install

# Clean up.
rm -rf cmake-3.22.4-linux-x86_64.sh
