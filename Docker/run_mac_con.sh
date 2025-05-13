#!/bin/bash

open -a XQuartz
sleep 2

IP=$(ifconfig | grep "inet " | grep -v 127.0.0.1 | awk '{print $2}')
export DISPLAY=$IP:0
xhost + $IP


docker run -it --rm \
  -e DISPLAY=host.docker.internal:0 \
  -v $HOME/uc-miis-master-project-ws:/uc-miis-master-project-ws \
  -v $HOME/uc-miis-master-project-ws/docker-vscode-user-data:/vscode-user-data \
  -v $HOME/uc-miis-master-project-ws/docker-vscode-extensions:/vscode-extensions \
  my-ros-jazzy-gazebo-vscode
