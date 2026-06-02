#!/bin/bash
set -e

docker build -t ur10e_force_control:humble \
  --build-arg ROS_DISTRO=humble \
  "$(dirname "$(realpath "$0")")"
