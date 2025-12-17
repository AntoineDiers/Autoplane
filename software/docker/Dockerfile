FROM ubuntu:24.04

# Install ROS
COPY assets/install_ros.sh /tmp/install_ros.sh
RUN /tmp/install_ros.sh

# Install angular
ENV NVM_DIR="/usr/local/nvm"
ENV NODE_VERSION="24.11.1"
COPY assets/install_angular.sh /tmp/install_angular.sh
RUN /tmp/install_angular.sh
ENV PATH="/usr/local/nvm/versions/node/v${NODE_VERSION}/bin:${PATH}"
ENV NG_CLI_ANALYTICS="false"

# Install http-server 
RUN npm install -g http-server