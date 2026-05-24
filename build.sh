#!/bin/bash
set -e

docker build -t ur10e_force_control:humble \
  --build-arg USER_UID="$(id -u)" \
  --build-arg USER_GID="$(id -g)" \
  "$(dirname "$(realpath "$0")")"
