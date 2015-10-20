#!/bin/bash

# Script for pulling in the external project that builds Java data
# structures from a ROS package.

set -euo pipefail
IFS=$'\n\t'

git clone https://github.com/rosjava/rosjava_bootstrap

git -C rosjava_bootstrap fetch origin indigo:indigo
git -C rosjava_bootstrap checkout indigo
mkdir output_dir

# Deploy to the local Maven repo.
ROS_MAVEN_DEPLOYMENT_REPOSITORY=$HOME/.m2/repository
export ROS_MAVEN_DEPLOYMENT_REPOSITORY

cd rosjava_bootstrap/

# Build the jar files and deploy to Maven repo.
./gradlew

ROS_PACKAGE_PATH=$(pwd)/../../ros-pkg
export ROS_PACKAGE_PATH

# Generate the .java files from the ros-pkg source.
./gradlew run
