#!/bin/bash
SCRIPT_PATH=$(dirname $(realpath $0))

COLOR_OFF='\033[0m'
COLOR_LIGHT_BLUE='\033[1;36m'

log()
{
    printf "\n${COLOR_LIGHT_BLUE}\n"
    printf " ------------------------------------------------------------------\n"
    printf "   $1\n"
    printf " ------------------------------------------------------------------\n"
    printf "${COLOR_OFF}\n\n"
}

# -----------------------------------------------------

log "Building docker image ..."

cd $SCRIPT_PATH/docker
docker build -t condor .

# -----------------------------------------------------

log "Building frontend ..."
cd $SCRIPT_PATH/frontend/aaa_hmi
docker run --rm -u $(id -u ${USER}):$(id -g ${USER}) -v $PWD:/workspace condor bash -c "cd /workspace && npm install && ng build" 

# -----------------------------------------------------

log "Building ROS workspace ..."
cd $SCRIPT_PATH/backend
docker run --rm -u $(id -u ${USER}):$(id -g ${USER}) -v $PWD:/workspace condor bash -c "cd /workspace/ros_ws && source /opt/ros/jazzy/setup.bash && colcon build" 

# -----------------------------------------------------

log "Installing ..."

# Creating deploy folder
mkdir -p $SCRIPT_PATH/install
rm -rf $SCRIPT_PATH/install/*

# Installing entrypoint and docker-compose.yaml
cp $SCRIPT_PATH/docker/entrypoint.sh $SCRIPT_PATH/install/
cp $SCRIPT_PATH/docker/docker-compose.yaml $SCRIPT_PATH/install/

# Installing frontend dist
mkdir -p $SCRIPT_PATH/install/dist
cp -r $SCRIPT_PATH/frontend/aaa_hmi/dist/aaa_hmi/browser/* $SCRIPT_PATH/install/dist

# Installing ROS ws
mkdir -p $SCRIPT_PATH/install/ros
cp -r $SCRIPT_PATH/backend/ros_ws/install/* $SCRIPT_PATH/install/ros