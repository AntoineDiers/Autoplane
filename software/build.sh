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

log "Installing QEMU ..."
docker run --privileged --rm tonistiigi/binfmt --install all

# -----------------------------------------------------

log "Building ROS docker build image ..."

cd $SCRIPT_PATH/docker
docker build --platform linux/arm64/v8 -t autoplane-build-ros -f DockerfileBuildRos .

# -----------------------------------------------------

log "Building HMI docker build image ..."

cd $SCRIPT_PATH/docker
docker build -t autoplane-build-hmi -f DockerfileBuildHmi .

# -----------------------------------------------------

log "Building autoplane-hmi ..."
cd $SCRIPT_PATH/autoplane-hmi
docker run -t --rm -u $(id -u ${USER}):$(id -g ${USER}) -v $PWD:/workspace autoplane-build-hmi bash -c "cd /workspace && npm install --legacy-peer-deps && npm run tauri build" 

# -----------------------------------------------------

log "Building ROS workspace ..."
cd $SCRIPT_PATH
rm -rf ros_ws/build ros_ws/install ros_ws/log
docker run --rm --platform linux/arm64/v8 -u $(id -u ${USER}):$(id -g ${USER}) -v $PWD:/workspace autoplane-build-ros bash -c "cd /workspace/ros_ws && source /opt/ros/jazzy/setup.bash && colcon build" 

# -----------------------------------------------------

log "Building docker run image ..."

rm -rf $SCRIPT_PATH/docker/assets/firmware/ros
cp -r $SCRIPT_PATH/ros_ws/install $SCRIPT_PATH/docker/assets/firmware/ros
cd $SCRIPT_PATH/docker
docker build --platform linux/arm64/v8 -t autoplane-run -f DockerfileRun .

# -----------------------------------------------------

log "Deploying build results ..."

cd $SCRIPT_PATH/deploy
rm -f ./*.deb
rm -f ./*.tar

cp $SCRIPT_PATH/autoplane-hmi/src-tauri/target/release/bundle/deb/*.deb $SCRIPT_PATH/deploy/

docker save -o autoplane-run.tar autoplane-run

docker tag autoplane-run localhost:5000/autoplane-run
docker push localhost:5000/autoplane-run