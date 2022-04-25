!/usr/bin/env bash

# Fail on first error.
set -e

cd "$(dirname "${BASH_SOURCE[0]}")"
. ./installer_base.sh

VERSION="3.19.6"
TARGET_ARCH="$(uname -m)"

function symlink_if_not_exist() {
    local dest="/usr/local/bin/cmake"
    if [[ ! -e "${dest}" ]]; then
        info "Created symlink ${dest} for convenience."
        ln -s ${SYSROOT_DIR}/bin/cmake /usr/local/bin/cmake
    fi
}

CMAKE_SH=
CHECKSUM=
if [[ "${TARGET_ARCH}" == "x86_64" ]]; then
    CMAKE_SH="cmake-${VERSION}-Linux-x86_64.sh"
    CHECKSUM="d94155cef56ff88977306653a33e50123bb49885cd085edd471d70dfdfc4f859"
elif [[ "${TARGET_ARCH}" == "aarch64" ]]; then
    CMAKE_SH="cmake-${VERSION}-Linux-aarch64.sh"
    CHECKSUM="f383c2ef96e5de47c0a55957e9af0bdfcf99d3988c17103767c9ef1b3cd8c0a9"
fi

DOWNLOAD_LINK="https://github.com/Kitware/CMake/releases/download/v${VERSION}/${CMAKE_SH}"
download_if_not_cached "${CMAKE_SH}" "${CHECKSUM}" "${DOWNLOAD_LINK}"

chmod a+x ${CMAKE_SH}
./${CMAKE_SH} --skip-license --prefix="${SYSROOT_DIR}"
symlink_if_not_exist
rm -fr ${CMAKE_SH}