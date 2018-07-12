# *  @Copyright (c) 2016-2017 DJI
# *
# * Permission is hereby granted, free of charge, to any person obtaining a copy
# * of this software and associated documentation files (the "Software"), to deal
# * in the Software without restriction, including without limitation the rights
# * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# * copies of the Software, and to permit persons to whom the Software is
# * furnished to do so, subject to the following conditions:
# *
# * The above copyright notice and this permission notice shall be included in
# * all copies or substantial portions of the Software.
# *
# * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# * SOFTWARE.
# *
# *

cmake_minimum_required(VERSION 2.8)
project(matrice210)

if(NOT ONBOARDSDK_SOURCE)
    set(ONBOARDSDK_SOURCE "${CMAKE_CURRENT_SOURCE_DIR}/../../Onboard-SDK/osdk-core")
endif()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -pthread -g -O0")
set(CMAKE_VERBOSE_MAKEFILE OFF)

set(DJIOSDK 0)
set(DJIOSDK_MINOR_VERSION 1)
set(DJIOSDK_PATCH_VERSION 0)
set(DJIOSDK_VERSION
  ${DJIOSDK_MAJOR_VERSION}.${DJIOSDK_MINOR_VERSION}.${DJIOSDK_PATCH_VERSION})

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/bin)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/libs)

add_subdirectory(${ONBOARDSDK_SOURCE} ${PROJECT_BINARY_DIR}/bin/osdk-core)

# TODO Remove : Used to auto-completion
if (WIN32)
    include_directories(PIGPIO)
endif (WIN32)

include_directories(${ONBOARDSDK_SOURCE}/api/inc)
include_directories(${ONBOARDSDK_SOURCE}/utility/inc)
include_directories(${ONBOARDSDK_SOURCE}/hal/inc)
include_directories(${ONBOARDSDK_SOURCE}/protocol/inc)
include_directories(${ONBOARDSDK_SOURCE}/platform/linux/inc)

set(HELPER_FUNCTIONS_DIR ${ONBOARDSDK_SOURCE}/../sample/linux/common)

include_directories(${HELPER_FUNCTIONS_DIR})
FILE(GLOB SOURCE_FILES *.hpp *.cpp
        ${HELPER_FUNCTIONS_DIR}/dji_linux_environment.cpp
        ${HELPER_FUNCTIONS_DIR}/dji_linux_helpers.cpp
        )

add_executable(${PROJECT_NAME} ${SOURCE_FILES}
        Mobile/Mobile.cpp Mobile/Mobile.h
        timer.cpp timer.h
        PackageManager/PackageManager.cpp PackageManager/PackageManager.h
        FlightController.cpp FlightController.h
        Console/Console.cpp Console/Console.h
        Mission/PositionMission.cpp Mission/PositionMission.h
        Mission/PositionOffsetMission.cpp Mission/PositionOffsetMission.h
        Mission/VelocityMission.cpp Mission/VelocityMission.h
        I2C/I2C_Communication.cpp I2C/I2C_Communication.h)
target_link_libraries(${PROJECT_NAME} djiosdk-core)
if(UNIX)
    target_link_libraries(${PROJECT_NAME} pigpio)
endif(UNIX)