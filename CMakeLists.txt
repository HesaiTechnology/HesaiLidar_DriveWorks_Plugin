# Copyright (c) 2022 HESAI Corporation. All rights reserved.
set(CODEOWNERS @skadle)

project(plugin_lidar_hesai C CXX)

#-------------------------------------------------------------------------------
# Project files
#-------------------------------------------------------------------------------
set(PUBLIC_DOCS
    README.md
)

include_directories(
    UdpParser/include
    ./include
    ./UdpProtocol
)
set(SOURCES
    at128decoder.cpp
    ./src/HesaiLidar.cpp
    ./src/InputSocket.cpp
    ./src/tcp_command_client.c
    ./src/util.c
    ./src/PlatUtils.cpp
    ./UdpParser/src/Udp4_3_Parser.cpp
    ./UdpParser/src/Udp3_2_Parser.cpp
    ./UdpParser/src/GeneralParser.cpp
)

set(LIBRARIES
    samples_framework
    ${Driveworks_LIBRARIES}
    sample_sensors_plugin_common
)

#-------------------------------------------------------------------------------
# Final target
#-------------------------------------------------------------------------------
add_library(${PROJECT_NAME} SHARED ${SOURCES})
target_link_libraries(${PROJECT_NAME} PRIVATE ${LIBRARIES})
set_property(TARGET ${PROJECT_NAME} PROPERTY FOLDER "Samples")

# ------------------------------------------------------------------------------
# Install target
# ------------------------------------------------------------------------------
sdk_install_sample_library(${PROJECT_NAME})