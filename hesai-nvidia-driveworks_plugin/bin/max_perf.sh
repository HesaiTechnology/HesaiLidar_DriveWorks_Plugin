#!/bin/bash

# Increase UDP buffer size limits, here to 16MB.  
# See https://docs.nvidia.com/drive/driveworks-3.5/lidar_usecase3.html

sudo sysctl -w net.core.rmem_default=16777216
sudo sysctl -w net.core.rmem_max=16777216

