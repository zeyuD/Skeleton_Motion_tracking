#!/bin/zsh

# Source directory
SRC_DIR='/usr/share/doc/ultraleap-hand-tracking-service/samples'

# Build configuration
BUILD_TYPE='Release'
REPOS_BUILD_ROOT=/home/zeyu/leapmotion/build
REPOS_INSTALL_ROOT=/home/zeyu/leapmotion

# Create the build directory if it doesn't exist
mkdir -p ${REPOS_BUILD_ROOT}/${BUILD_TYPE}/leapc_example

# Configure the project
cmake -S ${SRC_DIR} -B ${REPOS_BUILD_ROOT}/${BUILD_TYPE}/leapc_example `       
        -DCMAKE_INSTALL_PREFIX="${REPOS_INSTALL_ROOT}/leapc_example" `    
        -DCMAKE_BUILD_TYPE="${BUILD_TYPE}"    
    
cmake --build ${REPOS_BUILD_ROOT}/${BUILD_TYPE}/leapc_example -j --config ${BUILD_TYPE}