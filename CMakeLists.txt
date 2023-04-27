# Copyright (c) 2022 HESAI Technology Corporation. All rights reserved.
set(CODEOWNERS @skadle)

project(plugin_lidar_hesai C CXX)

#-------------------------------------------------------------------------------
# Project files
#-------------------------------------------------------------------------------
set(PUBLIC_DOCS
    ${CMAKE_CURRENT_SOURCE_DIR}/README.md
)

set(SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/HSSensorPlugin.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/HesaiLidar.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/InputSocket.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/TcpCommandClient.c
    ${CMAKE_CURRENT_SOURCE_DIR}/src/util.c
    ${CMAKE_CURRENT_SOURCE_DIR}/src/PlatUtils.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/UdpParser/src/Udp1_4_Parser.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/UdpParser/src/Udp4_3_Parser.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/UdpParser/src/Udp3_2_Parser.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/UdpParser/src/GeneralParser.cpp
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

target_include_directories(${PROJECT_NAME} PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/UdpParser/include
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/UdpProtocol
)

target_link_libraries(${PROJECT_NAME} PRIVATE ${LIBRARIES})

set_property(TARGET ${PROJECT_NAME} PROPERTY FOLDER "Samples")

# ------------------------------------------------------------------------------
# Install target
# ------------------------------------------------------------------------------
sdk_install_sample_library(${PROJECT_NAME})