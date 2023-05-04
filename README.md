# HESAI LIDAR PLUGIN for NVIDIA DRIVEWORKS

## About the project

Contained here is the HESAI Lidar AT128 Plugin for NVIDIA DRIVEWORKS 5.8. The plugin is 
an implementation of NVIDIA's interface for Custom Lidars, described here: 
- https://developer.nvidia.com/docs/drive/driveworks/latest/nvsdk_dw_html/sensorplugins_lidarsensor.html

The plugin itself consists of the source codes folder to be built, and the released hesai library folder 'hesai-nvidia-driveworks_plugin' which can be used without compiling the source code.

## hesai-nvidia-driveworks_plugin

Contained here are the scripts, dynamic library,and examples explain how to playing the real-time point cloud using the compiled hesai plugin.

For details please read README.md in this folder.

## Environment and Dependencies

### System environment requirement: Linux + NVIDIA DriveWork SDK

Ubuntu 18.04 - with nvidia driveworks SDK 4.0 installed.

Ubuntu 20.04 - with nvidia driveworks SDK 5.8 installed.

Make sure the samples provided by nvidia can be complied in your computer by following the procedures of
- https://docs.nvidia.com/drive/drive-os-5.2.6.0L/drive-qsg-dz/download-run-sdkm/index.html#download-devzone

## Download and Build

The plugin can not be built independently, because several nvidia driveworks SDK libraries are required as the prerequisite while compiling.

The current methods is to build the lidar plugin when building the drivework sample. The drivework sample can be found at 
- /usr/local/driveworks/samples/ 

1. Make sure the sample can be built in your computer at first

2. Create a soft link to this plugin folder

- sudo ln -s /path/to/where/downloaded/HesaiLidar_DriveWorks_Plugin plugin_lidar_hesai

3. Add the modular cmake command to the upper CMakeLists.txt, namely the following sentence

- add_subdirectory(plugin_lidar_hesai)

4. Create build folder

- cd /usr/local/driveworks/samples/
- mkdir build-x86
- cd build-x86

5. Configure and make

- cmake ../
- make -j

6. Find the customized hesai library

- cd /usr/local/driveworks/samples/build-x86/src/sensors/lidar/plugin_lidar_hesai
- ls

## Configuration

To use the library compiled by yourself, you need to 

1. Modify the name of the previous so at first

- cd /path/to/download/this/folder/hesai-nvidia-driveworks_plugin/lib
- mv libplugin_lidar_hesai_x86.so libplugin_lidar_hesai_x86_old.so

2. Create a new soft link to your so file

- ln -s /usr/local/driveworks-5.8/samples/build-x86/src/sensors/lidar/plugin_lidar_hesai/libplugin_lidar_hesai.so libplugin_lidar_hesai_x86.so

Notice that files contained in hesai-nvidia-driveworks_plugin/lib should not be update by developer.

## Run

Follow the instruction in the folder of hesai-nvidia-driveworks_plugin

## Contact

For support of the Hesai Lidar Plugin for Nvidia Drivework, please use Github Issues in this repo.

For support of Hesai products outside of the SDK, please use Hesai customer support.

## License

The Apache License, version 2.0. [https://www.apache.org/licenses/LICENSE-2.0]