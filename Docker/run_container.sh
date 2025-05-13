#!/bin/bash

# X11 izinlerini aç (GUI uygulamaları için)
xhost +local:docker

# Docker konteyneri başlat
docker run -it --rm \
    -v $SSH_AUTH_SOCK:/ssh-agent \
    -e SSH_AUTH_SOCK=/ssh-agent \
    --gpus all \
    --env="DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --env="NVIDIA_DRIVER_CAPABILITIES=all" \
    --env="NVIDIA_VISIBLE_DEVICES=all" \
    --env="GAZEBO_MODEL_PATH=/uc-miis-master-project-ws/models" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --volume="$HOME/uc-miis-master-project-ws:/uc-miis-master-project-ws" \
    --volume="$HOME/uc-miis-master-project-ws/docker_vscode_user_data:/vscode-user-data" \
    --volume="$HOME/uc-miis-master-project-ws/docker_vscode_extensions:/vscode-extensions" \
    my-ros-jazzy-gazebo-vscode
