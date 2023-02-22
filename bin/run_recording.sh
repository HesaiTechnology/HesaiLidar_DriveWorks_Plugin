#!/bin/bash

# this script demonstrates how to play back a recording using the nvidia driveworks 
# sample_lidar_replay application. the single parameter is the name of the recording session,
# and it is used to locate the recording data files and sensor config file from the 
# recording in the ../data directory

if [ -z "$1" ]
  then
    echo "Usage: ./run.sh SESSION_NAME "
    echo "  Where: "
    echo "  SESSION_NAME is the base name of the recording "
    echo "  Data and sensor config files stored in ../data, "
    exit 1
fi

# store the current script directory in a variable
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
DATA_DIR="$( cd "${SCRIPT_DIR}/../data" &> /dev/null && pwd )"
PLUGIN_DIR="$( cd "${SCRIPT_DIR}/../lib" &> /dev/null && pwd )"

ARCH=$( dpkg --print-architecture )

if [ "${ARCH}" = "amd64" ]; then
    PLUGIN_FILE="libplugin_lidar_hesai_x86.so"
else
    PLUGIN_FILE="libplugin_lidar_hesai_arm.so"
fi

PLUGIN_PATH="${PLUGIN_DIR}/${PLUGIN_FILE}"

# create a session_id to use for recording and config filenames
SESSION_ID=$1

SENSOR_TYPE="CUSTOM_EX"

LIDAR_TYPE="AT128E2X"

# too long real path might cause trouble
CORRECTION_FILE="../share/correction_at128.dat"

LOG_LEVEL="WARN"
SENSOR_UDP_PORT=2368
SENSOR_PTC_PORT=9347

DW_CMD=/usr/local/driveworks/bin/sample_lidar_replay

${DW_CMD} --protocol=lidar.virtual --params=file="${DATA_DIR}/${SESSION_ID}.bin",device=${SENSOR_TYPE},\
log_level="${LOG_LEVEL}",decoder-path="${PLUGIN_PATH}",correction_file=${CORRECTION_FILE},lidar_type=${LIDAR_TYPE}
