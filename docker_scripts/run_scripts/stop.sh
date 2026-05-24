#!/bin/bash
# Stop and remove the ur10e_force_control container.
CONTAINER="ur10e_force_control"

if [ -n "$(docker ps -q -f name="^${CONTAINER}$")" ]; then
  echo "Stopping ${CONTAINER}..."
  docker stop "${CONTAINER}"
  docker rm "${CONTAINER}"
  echo "Container removed."
else
  echo "Container ${CONTAINER} is not running."
fi
