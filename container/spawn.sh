#!/bin/bash

DIR_NAME=$(cd "$(dirname "$0")" || exit; pwd)
SOURCE_DIR=$(realpath "${DIR_NAME}"/../)

IMAGE_ID=personal-dev
LATEST_CONTAINER_ID=$(docker ps --all --filter ancestor=${IMAGE_ID} --latest --quiet)

Mount() {
    if [[ -d $1 ]]; then
        MOUNT_ARGS="${MOUNT_ARGS} --volume $1:$1"
    fi
}

CreateContainer() {
    echo "Creating new container"

    MOUNT_ARGS="--volume ${SOURCE_DIR}:${SOURCE_DIR}"

    echo "Mount arguments:"
    echo "${MOUNT_ARGS}"

    docker run                       \
    --interactive                    \
    --tty                            \
    --privileged                     \
    --entrypoint /bin/bash           \
    --hostname container             \
    --network host                   \
    --env DISPLAY=$DISPLAY           \
    -v $XAUTH:/root/.Xauthority      \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    ${MOUNT_ARGS}                    \
    ${IMAGE_ID}
}

StartContainer() {
    echo "Starting container $1"
    docker start --interactive $1
}

if [ -z "${LATEST_CONTAINER_ID}" ]; then
    CreateContainer
else
    read -rp "Should reuse existing container ${LATEST_CONTAINER_ID}:${IMAGE_ID} ? (y/n): " REUSE

    if [[ $REUSE == [yY] || $REUSE == [yY][eE][sS] ]]; then
        StartContainer "${LATEST_CONTAINER_ID}"
    else
        CreateContainer
    fi
fi
