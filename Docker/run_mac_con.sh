#!/bin/bash

xhost + 127.0.0.1

docker run -it --rm \
  -e DISPLAY=host.docker.internal:0 \
  -v $HOME/uc-miis-master-project-ws:/uc-miis-master-project-ws \
  -v $HOME/uc-miis-master-project-ws/docker-vscode-user-data:/vscode-user-data \
  -v $HOME/uc-miis-master-project-ws/docker-vscode-extensions:/vscode-extensions \
  my-ros-jazzy-gazebo-vscode