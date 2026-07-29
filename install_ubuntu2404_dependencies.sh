#!/usr/bin/env bash
set -euo pipefail

if [[ ${EUID} -ne 0 ]]; then
  echo "Run this script with sudo: sudo bash $0" >&2
  exit 1
fi

export DEBIAN_FRONTEND=noninteractive

apt-get update
apt-get install -y software-properties-common curl ca-certificates
add-apt-repository -y universe

ros_apt_version="$(curl -fsSL https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest \
  | sed -n 's/.*"tag_name": "\([^"]*\)".*/\1/p')"
ubuntu_codename="$(. /etc/os-release && printf '%s' "${UBUNTU_CODENAME:-${VERSION_CODENAME}}")"
ros_apt_deb="/tmp/ros2-apt-source_${ros_apt_version}.${ubuntu_codename}_all.deb"

curl -fL \
  "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ros_apt_version}/ros2-apt-source_${ros_apt_version}.${ubuntu_codename}_all.deb" \
  -o "${ros_apt_deb}"
dpkg -i "${ros_apt_deb}"

apt-get update
apt-get install -y \
  ros-jazzy-desktop \
  ros-dev-tools \
  python3-colcon-meson \
  build-essential \
  cmake \
  meson \
  ninja-build \
  pkg-config \
  python3-jinja2 \
  python3-yaml \
  python3-ply \
  libboost-dev \
  libgnutls28-dev \
  openssl \
  libtiff-dev \
  pybind11-dev \
  libevent-dev \
  libdrm-dev \
  libjpeg-dev \
  libsdl2-dev \
  libyaml-dev \
  libudev-dev \
  libdw-dev \
  libx11-dev \
  python3-numpy \
  python3-tk \
  python3-smbus \
  python3-rpi.gpio \
  python3-serial \
  i2c-tools

if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
  rosdep init
fi

echo "System dependencies installed successfully."
