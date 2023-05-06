#!/bin/bash

# this script demonstrates how to make a recording using the nvidia driveworks 
# recording tool. it uses a template rig file and replaces the template elements
# with the parameter values. the first parameter is the name of the session, and will 
# be used to name the sensor config file that will be stored for the session in the ../data
# directory. the second parameter is the ip or hostname of the sensor, and the third is the 
# ip or hostname of the computer

if [ -z "$3" ]
  then
    echo "Usage: ./record.sh SESSION_NAME SENSOR_ADDR HOST_ADDR"
    echo "  Where: "
    echo "    SESSION_NAME is used to name the recording data 
              and sensor config file that will be stored in ../data, "
    echo "    SENSOR_ADDR is the sensor's ip or hostname, and "
    echo "    HOST_ADDR is the ip or hostname of this computer "
    exit 1
fi

# store the current bin directory in a variable
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
PLUGIN_DIR="$( cd "${SCRIPT_DIR}/../lib" &> /dev/null && pwd )"

# create data folder to store recordings
if [ ! -d "${SCRIPT_DIR}/../data" ]; then
  mkdir ../data
fi
DATA_DIR="$( cd "${SCRIPT_DIR}/../data" &> /dev/null && pwd )"

ARCH=$( dpkg --print-architecture )

if [ "${ARCH}" = "amd64" ]; then
    PLUGIN_FILE="libplugin_lidar_hesai_x86_64.so"
else
    PLUGIN_FILE="libplugin_lidar_hesai_aarch64.so"
fi

PLUGIN_PATH="${PLUGIN_DIR}/${PLUGIN_FILE}"

# create a session_id to use for recording and config filenames
SESSION_ID=$1

# must be CUSTOM_EX or recording program elapses
SENSOR_TYPE="CUSTOM_EX"

# support hesai lidar AT128 - "AT128E2X", QT128 - "QT128C2X", P128 - "Pandar128E3X"
LIDAR_TYPE="AT128E2X"

# set your host and sensor ip/hostname and ports here
SENSOR_ADDR=$2
UDP_PORT=2368
PTC_PORT=9347

HOST_ADDR=$3
LOG_LEVEL="WARN"

# Set your multicast IP address of connected Lidar, optional, e.g. "239.0.0.1"
MULTCAST_IP='239.0.0.1'

# set the correction file path of your specific lidar AT128
CORRECTION_FILE="../share/correction_at128.dat"

# replace the parameters in the template file
cat ${SCRIPT_DIR}/hesai_lidar_rig_template.json \
    | sed 's|<SESSION_ID>|'"${SESSION_ID}"'|g' \
    | sed 's|<SENSOR_TYPE>|'"${SENSOR_TYPE}"'|g' \
    | sed 's|<LIDAR_TYPE>|'"${LIDAR_TYPE}"'|g' \
    | sed 's|<LOG_LEVEL>|'"${LOG_LEVEL}"'|g' \
    | sed 's|<SENSOR_ADDR>|'"${SENSOR_ADDR}"'|g' \
    | sed 's|<HOST_ADDR>|'"${HOST_ADDR}"'|g' \
    | sed 's|<UDP_PORT>|'"${UDP_PORT}"'|g' \
    | sed 's|<PTC_PORT>|'"${PTC_PORT}"'|g' \
    | sed 's|<MULTCAST_IP>|'"${MULTCAST_IP}"'|g' \
    | sed 's|<PLUGIN_PATH>|'"${PLUGIN_PATH}"'|g' \
    | sed 's|<DATA_DIR>|'"${DATA_DIR}"'|g' \
    | sed 's|<CORRECTION_FILE>|'"${CORRECTION_FILE}"'|g' \
    > "${DATA_DIR}/${SESSION_ID}_hesai_lidar_rig.json"

# available only when drivework SDK is installed in your computer
DW_CMD=/usr/local/driveworks/tools/capture/recorder

${DW_CMD} "${DATA_DIR}/${SESSION_ID}_hesai_lidar_rig.json"

# nvidia recorder puts files in bin directory, move them to data directory
mv ${SCRIPT_DIR}/${SESSION_ID}.* ${DATA_DIR}