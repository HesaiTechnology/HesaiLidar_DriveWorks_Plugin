#!/bin/bash

sudo ln -s /path/to/where/download/HesaiLidar_DriveWorks_Plugin plugin_lidar_hesai

# Drivework platform must be installed at first in /usr/local/*
cd /usr/local/driveworks/samples/

mkdir build-x86 build-linux-aarch64

# Create soft link in drivework compile package
cd build-x86

# Add add_subdirectory(HesaiLidar_DriveWorks_Plugin)
# ln -s /usr/local/driveworks-5.8/samples/build-x86/src/sensors/lidar/plugin_lidar_hesai/libplugin_lidar_hesai.so libplugin_lidar_hesai_x86.so

# Make
cmake ../

make -j