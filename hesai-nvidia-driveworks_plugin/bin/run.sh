#!/bin/bash

# this script demonstrates how to play data from a live sensor using the nvidia driveworks 
# sample_lidar_replay application. the first parameter the ip or hostname of the sensor, and the second is the 
# ip or hostname of the computer

if [ -z "$2" ]
  then
    echo "Usage: ./run.sh SENSOR_ADDR HOST_ADDR"
    echo "  Where: "
    echo "    SENSOR_ADDR is the sensor's ip or hostname, and "
    echo "    HOST_ADDR is the ip or hostname of this computer "
    exit 1
fi

# store the current script directory in a variable
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
PLUGIN_DIR="$( cd "${SCRIPT_DIR}/../lib" &> /dev/null && pwd )"

ARCH=$( dpkg --print-architecture )

if [ "${ARCH}" = "amd64" ]; then
    PLUGIN_FILE="libplugin_lidar_hesai_x86.so"
else
    PLUGIN_FILE="libplugin_lidar_hesai_arm.so"
fi

PLUGIN_PATH="${PLUGIN_DIR}/${PLUGIN_FILE}"

#  CUSTOM_EX can be displayed in replay sample
SENSOR_TYPE="CUSTOM_EX"

# Support hesai lidar AT128 - "AT128E2X", QT128 - "QT128C2X", P128 - "Pandar128E3X"
LIDAR_TYPE="AT128E2X"

# Set path of your correction file, loading local correction file would start if no file from lidar was found
CORRECTION_FILE="../share/correction_at128.dat"

# Set your host and sensor ip/hostname and ports here
SENSOR_ADDR=$1
SENSOR_UDP_PORT=2368
SENSOR_PTC_PORT=9347

# Set your multicast IP address of connected Lidar, optional, e.g. "239.0.0.1"
MULTCAST_IP="239.0.0.1"
HOST_ADDR=$2
LOG_LEVEL="WARN"

DW_CMD=/usr/local/driveworks/bin/sample_lidar_replay

${DW_CMD} --protocol=lidar.custom --params=device=${SENSOR_TYPE},log_level="${LOG_LEVEL}",\
ip=${SENSOR_ADDR},host_ip=${HOST_ADDR},udp_port=${SENSOR_UDP_PORT},ptc_port=${SENSOR_PTC_PORT},\
decoder-path="${PLUGIN_PATH}",multcast_ip="${MULTCAST_IP}",lidar_type=${LIDAR_TYPE},correction_file=${CORRECTION_FILE}