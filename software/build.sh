#!/bin/bash

set -e

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

log "Building docker build image ..."

cd $SCRIPT_PATH/docker
docker build -t autoplane-build -f DockerfileBuild .

# -----------------------------------------------------

log "Building autoplane-hmi ..."
cd $SCRIPT_PATH/autoplane-hmi
docker run -t --rm -u $(id -u ${USER}):$(id -g ${USER}) -v $PWD:/workspace autoplane-build bash -c "cd /workspace && npm install --legacy-peer-deps && npm run tauri build" 

# -----------------------------------------------------

log "Building ROS workspace ..."
cd $SCRIPT_PATH/ros_ws
docker run --rm -u $(id -u ${USER}):$(id -g ${USER}) -v $PWD:/workspace autoplane-build bash -c "cd /workspace && source /opt/ros/jazzy/setup.bash && colcon build" 

# -----------------------------------------------------

log "Building docker run image ..."

rm -rf $SCRIPT_PATH/docker/assets/firmware/ros
cp -r $SCRIPT_PATH/ros_ws/install $SCRIPT_PATH/docker/assets/firmware/ros
cd $SCRIPT_PATH/docker
docker build -t autoplane-run -f DockerfileRun .

# -----------------------------------------------------

log "Deploying build results ..."

cd $SCRIPT_PATH/deploy
rm -f ./*.tar.gz
rm -f ./*.deb

docker save autoplane-run:latest -o autoplane-run.tar.gz
cp $SCRIPT_PATH/autoplane-hmi/src-tauri/target/release/bundle/deb/*.deb $SCRIPT_PATH/deploy/