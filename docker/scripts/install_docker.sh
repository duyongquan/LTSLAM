#!/usr/bin/env bash


function install_filesystem_support() {
  MACHINE_VERSION=$(uname -r)

  MAIN_KERNEL_VERSION=${MACHINE_VERSION:0:1}
  if [ ${MAIN_KERNEL_VERSION} -gt 3 \
      -a -f /lib/modules/$MACHINE_VERSION/kernel/fs/overlayfs/overlay.ko ]; then
    echo "the kernel version 4 or higher;"
    echo "it has support overlay2"
    sudo modprobe overlay
  else
    echo "the kernel version is lower than 4"
    echo "try to install aufs"
    sudo apt-get update
    sudo apt-get install -y \
        linux-image-extra-${MACHINE_VERSION} \
        linux-image-extra-virtual
  fi

  sudo apt-get update
  sudo apt-get install -y \
      apt-transport-https \
      ca-certificates \
      curl \
      software-properties-common
  curl -fsSL "https://download.docker.com/linux/ubuntu/gpg" | sudo apt-key add -
}

function install_docker_x86() {
  sudo add-apt-repository \
     "deb [arch=amd64] https://download.docker.com/linux/ubuntu $(lsb_release -cs) stable"
  sudo apt-get update
  sudo apt-get install -y docker-ce
  sudo groupadd docker
  sudo gpasswd -a $USER docker
  newgrp docker
}


function install() {
  # The machine type, currently support x86_64
  install_filesystem_support
  MACHINE_ARCH=$(uname -m)
  if [ "$MACHINE_ARCH" == 'x86_64' ]; then
    install_docker_x86
  else
    echo "Unknown machine architecture $MACHINE_ARCH"
    exit 1
  fi
}

case $1 in
  install)
    install
    ;;
  uninstall)
    sudo apt-get remove docker docker-engine docker.io
    sudo apt-get purge docker-ce
    ;;
  *)
    install
    ;;
esac