#!/bin/bash
# Start the container if not already running, then open an interactive bash shell.
set -e

CONTAINER="ur10e_force_control"
IMAGE="ur10e_force_control:humble"
REPO_ROOT="$(cd "$(dirname "$0")/../.." && pwd)"

if [ -z "$(docker ps -q -f name="^${CONTAINER}$")" ]; then
  echo "Starting container ${CONTAINER}..."
  xhost +local:docker 2>/dev/null || true
  docker run -it -d \
    --name "${CONTAINER}" \
    --network host \
    -e DISPLAY="${DISPLAY}" \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    -v "${REPO_ROOT}/workspace:/workspace" \
    "${IMAGE}"
fi

docker exec -it "${CONTAINER}" bash
