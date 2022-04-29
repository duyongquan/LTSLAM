#!/usr/bin/env bash

# Fail on first error.
set -e

cd "$(dirname "${BASH_SOURCE[0]}")"

cd eigen-3.3.7 
mkdir build && cd build && cmake ..
make -j4
make install

# Clean up.
rm -fr build