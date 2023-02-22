# HESAI Lidar AT128 Plugin for NVIDIA DRIVEWORKS 

## Overview
Contained here is the HESAI Lidar AT128 Plugin for NVIDIA DRIVEWORKS 5.8. The plugin is 
an implementation of NVIDIA's interface for Custom Lidars, described here: 
- https://developer.nvidia.com/docs/drive/driveworks/latest/nvsdk_dw_html/sensorplugins_lidarsensor.html

The plugin itself consists of a bin directory, lib directory and a data directory
to store the lidar recordings bin files

Depending on your platform, an x86 Ubuntu 20.04 PC or DRIVE 
system, the plugin shared library can be found in the lib directory: 

x86 Ubuntu 20.04 PC: 
- `./lib/libplugin_lidar_hesai_x86.so`

DRIVE arm platform:
- `./lib/libplugin_lidar_hesai_arm.so`


## Prerequisites

### DRIVEWORKS OS installation
Ensure you have DRIVEWORKS OS installed by NVIDIA SDK Manager in your PC or DRIVE system
Described here:
https://developer.nvidia.com/docs/drive/drive-os/6.0.5/public/drive-os-installation/index.html

### DRIVEWORKS SDK installation
Ensure you have DRIVEWORKS SDK installed and have tested by running
`/usr/local/driveworks/bin/sample_lidar_replay`

### Prepare the correction file for each HESAI AT128 lidar

@note This is only essential when playing user-recorded sensor data, correction file was obtained automatically in Live Stream Sensor

- `./share/correction_at128.dat` is the example correction file to play back the example recorded sensor data    `lidar_hesai_at128.bin`
For each AT128 lidar, you can pass your correcion file path to `correction_file` in scripts. The correcion files for any AT128 are not the same!

The unique correction file of each Lidar AT128 was provided by your vendors, also acquired from the software HESAI PandarView2 
For Download: https://www.hesaitech.com/en/Download


## Scripts

With an HESAI Lidar sensor connected to you system through ethernet, you can get started quickly with the provided example scripts which can be found in the bin directories:

### Increase the system's UDP buffer maximums

You may need to increase the buffer limits of the UDP buffer on the system.
A convenient script for this can be run using the following command: 
- `./max_perf.sh`

### Live stream HESAI sensor

To live stream data from the sensor use the following command:
- `./run.sh <DEVICE-ADDR> <HOST-ADDR> `
    - `DEVICE-ADDR` is the IP address of the sensor
    - `HOST-ADDR` is the IP address of network adapter on this system to which the sensor is connected. 
Example: 
- `./run.sh 192.168.1.201 192.168.1.100`

### Record sessions

To record sessions, the NVIDIA recorder tool must be used.

1. Create a RIG file that references the plugin's shared library file for the platform. 
    - An example rig file with the necessary parameters  for the plugin is provided in the `../data` directory. 
    - NVIDIA's documentation of the RIG file can be found here: 
   https://developer.nvidia.com/docs/drive/driveworks/latest/nvsdk_dw_html/rigconfiguration_usecase0.html
2. With the RIG file properly configured, recordings are executed with the recorder tool as follows: 
- `cd /usr/local/driveworks/tools/capture`
- `./recorder <path to RIG json file>` 
3. Once prompted, press "s" to start recording and press 'q' to stop recording.

Alternatively, to accelerate getting started with recording, an example script is provided that takes the same parameters as the live stream script above, automatically creates a sample RIG file from a template, and then calls the NVIDIA recorder tool. 
- `./record.sh <SESSION-ID> <DEVICE-ADDR> <HOST-ADDR> `
    - `SESSION-ID` is a unique identifier for the session which is used to name the recording and config files, that reside in `/data` directory 
    - `DEVICE-ADDR` is the IP address or hostname of the sensor
    - `HOST-ADDR` is the IP address or hostname of network adapter on this system to which the sensor is connected. 
Example:
- `./record.sh session1 192.168.1.201 192.168.1.100`

### Playback recorded sessions

To playback recorded sensor data, use the following command, with the recording's.bin, .bin.seek, and .cfg, files in the `/data` directory:
- `./run_recording.sh <SESSION-ID> ` 
Example:
- `./run_recording.sh session1`
Make sure `session1.bin` file exist in the `/data` directory

Display the recorded example sensor data by using `./run_recording.sh lidar_hesai_at128`, it works both in x86 and arm machine, enjoy!


## Plugin Parameters

The above scripts are provided as examples of how to set up the parameters for NVIDIA DRIVEWORKS applications to properly use the HESAI plugin.

The parameters required for operation of the plugin are detailed below:

- `device`: Must be `CUSTOM_EX`
- `ip`: The sensor's hostname or IP address
- `host_ip`: The computer or DRIVE system's hostname or IP address
- `udp_port`:  The UDP port the sensor's LIDAR messages are sent to
- `ptc_port`: The TCP port Pandar TCP Commands (PTC) are sent to
- `data_dir` : The directory where the plugin will store (in live mode) or look for (in recording mode) the config file associated with the session
- `session_id`: This will be used as the name for the config file and recording files, i.e. if `session_id = TEST_1`, then the config file will be 
`TEST_1.json` and the recording data files will be `TEST_1.bin` and 
`TEST_1.bin.seek` 
- `decoder-path`: The full path to the plugin shared library
- `file`: The full path to the `.bin` file containing the recording, this is only required if playing back from a recording
- `multcast_ip`: The multicast IP address of connected Lidar, will be used to get udp packets from multicast ip address
- `lidar_type`: The lidar type here is `AT128E2X`
- `correction_file`: The correction file for the sensor

These parameters are provided to the NVIDIA DRIVEWORKS sample apps in the `--params` command-line parameter, as shown in the example scripts, and in the JSON element `"parameter"` in the example RIG file.

With `log_level` set to `WARN`, sufficient output to the console should happen to indicate if parameters are missing or incorrect.

## Simulate live sensor

You can replay the pcap file to simulate the live sensor, and then use our scripts to play, record, and run recording.
A feasible UDP pacakge sender can be created by Python with the following link:
https://gist.github.com/ninedraft/7c47282f8b53ac015c1e326fffb664b5

Make sure `python` and `scapy`are installed in your computer.

@note differ from live sensor, the program ends when replaying the pcap file comes to end.
