#!/usr/bin/env bash

set -e

cd "$(dirname "${BASH_SOURCE[0]}")"
. ./installer_base.sh

# Note(storypku):
# libgeos-dev for shapely
# libhdf5-dev for h5py

apt_get_update_and_install \
    libgeos-dev \
    libhdf5-dev

# libc6-dev
[[ -f /usr/include/xlocale.h ]] || ln -s /usr/include/locale.h /usr/include/xlocale.h

pip3_install -r py3_requirements.txt

# Since pypcd installed via `pip install` only works with python2.7,
# we can only install it this way
git clone https://github.com/DanielPollithy/pypcd

pushd pypcd >/dev/null
    make install
popd >/dev/null
rm -rf pypcd

if [[ -n "${CLEAN_DEPS}" ]]; then
    apt_get_remove libhdf5-dev
    apt_get_update_and_install \
        libhdf5-100
fi

# Clean up cache to reduce layer size.
apt-get clean && \
    rm -rf /var/lib/apt/lists/*