#!/bin/bash
set -e
PROJECT_SOURCE_DIR=$1
CMAKE_BINARY_DIR=$2
echo "PROJECT_SOURCE_DIR=${PROJECT_SOURCE_DIR}"
echo "CMAKE_BINARY_DIR=${CMAKE_BINARY_DIR}"

set -v

mkdir -p ${CMAKE_BINARY_DIR}/deb
cp -r ${PROJECT_SOURCE_DIR}/deb/mayfly_0.0.1-1_arm64 ${CMAKE_BINARY_DIR}/deb/.
mkdir -p ${CMAKE_BINARY_DIR}/deb/mayfly_0.0.1-1_arm64/usr/bin/
cp ${CMAKE_BINARY_DIR}/bin/mayfly ${CMAKE_BINARY_DIR}/deb/mayfly_0.0.1-1_arm64/usr/bin/.
chmod -R 755 ${CMAKE_BINARY_DIR}/deb/mayfly_0.0.1-1_arm64 #needed for buildbot
dpkg -b ${CMAKE_BINARY_DIR}/deb/mayfly_0.0.1-1_arm64


