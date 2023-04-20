#!/bin/bash
#获得lib目录下两个so

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
PLUGIN_DIR="$( cd "${SCRIPT_DIR}/hesai-nvidia-driveworks_plugin_v1.0.1/lib" &> /dev/null && pwd )"

SOX86_DIR="/usr/local/driveworks-4.0/samples/build/src/sensors/lidar/plugin_at128/libplugin_lidar_hesai.so"
SOARM_DIR="/usr/local/driveworks-4.0/samples/build_target/src/sensors/lidar/plugin_at128/libplugin_lidar_hesai.so"

SOX86_NAME="libplugin_lidar_hesai_x86.so"
SOARM_NAME="libplugin_lidar_hesai_arm.so"

cd ${PLUGIN_DIR}
echo "path=$PWD"

ln ${SOX86_DIR} ./${SOX86_NAME}
ln ${SOARM_DIR} ./${SOARM_NAME}