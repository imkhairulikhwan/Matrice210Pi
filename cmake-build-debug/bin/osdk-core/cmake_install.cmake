# Install script for directory: C:/Users/jonathan.michel/Desktop/Matrice210/Onboard-SDK/osdk-core

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "C:/Program Files (x86)/matrice210")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Debug")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xdevx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "C:/Program Files (x86)/matrice210/lib/cmake/djiosdk/DJIOSDKConfig.cmake;C:/Program Files (x86)/matrice210/lib/cmake/djiosdk/DJIOSDKConfigVersion.cmake")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "C:/Program Files (x86)/matrice210/lib/cmake/djiosdk" TYPE FILE FILES
    "C:/Users/jonathan.michel/Desktop/Matrice210/Onboard-SDK/osdk-core/cmake-modules/DJIOSDKConfig.cmake"
    "C:/Users/jonathan.michel/Desktop/Matrice210/Onboard-SDK/osdk-core/cmake-modules/DJIOSDKConfigVersion.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xshlibx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "C:/Program Files (x86)/matrice210/lib/libdjiosdk-core.a")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "C:/Program Files (x86)/matrice210/lib" TYPE STATIC_LIBRARY FILES "C:/Users/jonathan.michel/Desktop/Matrice210/code/Matrice210Pi3/cmake-build-debug/libs/libdjiosdk-core.a")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xdevx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "C:/Program Files (x86)/matrice210/include/djiosdk/dji_ack.hpp;C:/Program Files (x86)/matrice210/include/djiosdk/dji_broadcast.hpp;C:/Program Files (x86)/matrice210/include/djiosdk/dji_camera.hpp;C:/Program Files (x86)/matrice210/include/djiosdk/dji_command.hpp;C:/Program Files (x86)/matrice210/include/djiosdk/dji_control.hpp;C:/Program Files (x86)/matrice210/include/djiosdk/dji_error.hpp;C:/Program Files (x86)/matrice210/include/djiosdk/dji_gimbal.hpp;C:/Program Files (x86)/matrice210/include/djiosdk/dji_hardware_sync.hpp;C:/Program Files (x86)/matrice210/include/djiosdk/dji_hotpoint.hpp;C:/Program Files (x86)/matrice210/include/djiosdk/dji_mfio.hpp;C:/Program Files (x86)/matrice210/include/djiosdk/dji_mission_base.hpp;C:/Program Files (x86)/matrice210/include/djiosdk/dji_mission_manager.hpp;C:/Program Files (x86)/matrice210/include/djiosdk/dji_mission_type.hpp;C:/Program Files (x86)/matrice210/include/djiosdk/dji_mobile_communication.hpp;C:/Program Files (x86)/matrice210/include/djiosdk/dji_status.hpp;C:/Program Files (x86)/matrice210/include/djiosdk/dji_subscription.hpp;C:/Program Files (x86)/matrice210/include/djiosdk/dji_telemetry.hpp;C:/Program Files (x86)/matrice210/include/djiosdk/dji_type.hpp;C:/Program Files (x86)/matrice210/include/djiosdk/dji_vehicle.hpp;C:/Program Files (x86)/matrice210/include/djiosdk/dji_vehicle_callback.hpp;C:/Program Files (x86)/matrice210/include/djiosdk/dji_version.hpp;C:/Program Files (x86)/matrice210/include/djiosdk/dji_virtual_rc.hpp;C:/Program Files (x86)/matrice210/include/djiosdk/dji_waypoint.hpp;C:/Program Files (x86)/matrice210/include/djiosdk/dji_aes.hpp;C:/Program Files (x86)/matrice210/include/djiosdk/dji_crc.hpp;C:/Program Files (x86)/matrice210/include/djiosdk/dji_open_protocol.hpp;C:/Program Files (x86)/matrice210/include/djiosdk/dji_protocol_base.hpp;C:/Program Files (x86)/matrice210/include/djiosdk/dji_hard_driver.hpp;C:/Program Files (x86)/matrice210/include/djiosdk/dji_log.hpp;C:/Program Files (x86)/matrice210/include/djiosdk/dji_memory.hpp;C:/Program Files (x86)/matrice210/include/djiosdk/dji_platform_manager.hpp;C:/Program Files (x86)/matrice210/include/djiosdk/dji_thread_manager.hpp;C:/Program Files (x86)/matrice210/include/djiosdk/dji_circular_buffer.hpp;C:/Program Files (x86)/matrice210/include/djiosdk/dji_singleton.hpp;C:/Program Files (x86)/matrice210/include/djiosdk/linux_serial_device.hpp;C:/Program Files (x86)/matrice210/include/djiosdk/posix_thread.hpp;C:/Program Files (x86)/matrice210/include/djiosdk/posix_thread_manager.hpp")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "C:/Program Files (x86)/matrice210/include/djiosdk" TYPE FILE FILES
    "C:/Users/jonathan.michel/Desktop/Matrice210/Onboard-SDK/osdk-core/api/inc/dji_ack.hpp"
    "C:/Users/jonathan.michel/Desktop/Matrice210/Onboard-SDK/osdk-core/api/inc/dji_broadcast.hpp"
    "C:/Users/jonathan.michel/Desktop/Matrice210/Onboard-SDK/osdk-core/api/inc/dji_camera.hpp"
    "C:/Users/jonathan.michel/Desktop/Matrice210/Onboard-SDK/osdk-core/api/inc/dji_command.hpp"
    "C:/Users/jonathan.michel/Desktop/Matrice210/Onboard-SDK/osdk-core/api/inc/dji_control.hpp"
    "C:/Users/jonathan.michel/Desktop/Matrice210/Onboard-SDK/osdk-core/api/inc/dji_error.hpp"
    "C:/Users/jonathan.michel/Desktop/Matrice210/Onboard-SDK/osdk-core/api/inc/dji_gimbal.hpp"
    "C:/Users/jonathan.michel/Desktop/Matrice210/Onboard-SDK/osdk-core/api/inc/dji_hardware_sync.hpp"
    "C:/Users/jonathan.michel/Desktop/Matrice210/Onboard-SDK/osdk-core/api/inc/dji_hotpoint.hpp"
    "C:/Users/jonathan.michel/Desktop/Matrice210/Onboard-SDK/osdk-core/api/inc/dji_mfio.hpp"
    "C:/Users/jonathan.michel/Desktop/Matrice210/Onboard-SDK/osdk-core/api/inc/dji_mission_base.hpp"
    "C:/Users/jonathan.michel/Desktop/Matrice210/Onboard-SDK/osdk-core/api/inc/dji_mission_manager.hpp"
    "C:/Users/jonathan.michel/Desktop/Matrice210/Onboard-SDK/osdk-core/api/inc/dji_mission_type.hpp"
    "C:/Users/jonathan.michel/Desktop/Matrice210/Onboard-SDK/osdk-core/api/inc/dji_mobile_communication.hpp"
    "C:/Users/jonathan.michel/Desktop/Matrice210/Onboard-SDK/osdk-core/api/inc/dji_status.hpp"
    "C:/Users/jonathan.michel/Desktop/Matrice210/Onboard-SDK/osdk-core/api/inc/dji_subscription.hpp"
    "C:/Users/jonathan.michel/Desktop/Matrice210/Onboard-SDK/osdk-core/api/inc/dji_telemetry.hpp"
    "C:/Users/jonathan.michel/Desktop/Matrice210/Onboard-SDK/osdk-core/api/inc/dji_type.hpp"
    "C:/Users/jonathan.michel/Desktop/Matrice210/Onboard-SDK/osdk-core/api/inc/dji_vehicle.hpp"
    "C:/Users/jonathan.michel/Desktop/Matrice210/Onboard-SDK/osdk-core/api/inc/dji_vehicle_callback.hpp"
    "C:/Users/jonathan.michel/Desktop/Matrice210/Onboard-SDK/osdk-core/api/inc/dji_version.hpp"
    "C:/Users/jonathan.michel/Desktop/Matrice210/Onboard-SDK/osdk-core/api/inc/dji_virtual_rc.hpp"
    "C:/Users/jonathan.michel/Desktop/Matrice210/Onboard-SDK/osdk-core/api/inc/dji_waypoint.hpp"
    "C:/Users/jonathan.michel/Desktop/Matrice210/Onboard-SDK/osdk-core/protocol/inc/dji_aes.hpp"
    "C:/Users/jonathan.michel/Desktop/Matrice210/Onboard-SDK/osdk-core/protocol/inc/dji_crc.hpp"
    "C:/Users/jonathan.michel/Desktop/Matrice210/Onboard-SDK/osdk-core/protocol/inc/dji_open_protocol.hpp"
    "C:/Users/jonathan.michel/Desktop/Matrice210/Onboard-SDK/osdk-core/protocol/inc/dji_protocol_base.hpp"
    "C:/Users/jonathan.michel/Desktop/Matrice210/Onboard-SDK/osdk-core/hal/inc/dji_hard_driver.hpp"
    "C:/Users/jonathan.michel/Desktop/Matrice210/Onboard-SDK/osdk-core/hal/inc/dji_log.hpp"
    "C:/Users/jonathan.michel/Desktop/Matrice210/Onboard-SDK/osdk-core/hal/inc/dji_memory.hpp"
    "C:/Users/jonathan.michel/Desktop/Matrice210/Onboard-SDK/osdk-core/hal/inc/dji_platform_manager.hpp"
    "C:/Users/jonathan.michel/Desktop/Matrice210/Onboard-SDK/osdk-core/hal/inc/dji_thread_manager.hpp"
    "C:/Users/jonathan.michel/Desktop/Matrice210/Onboard-SDK/osdk-core/utility/inc/dji_circular_buffer.hpp"
    "C:/Users/jonathan.michel/Desktop/Matrice210/Onboard-SDK/osdk-core/utility/inc/dji_singleton.hpp"
    "C:/Users/jonathan.michel/Desktop/Matrice210/Onboard-SDK/osdk-core/platform/linux/inc/linux_serial_device.hpp"
    "C:/Users/jonathan.michel/Desktop/Matrice210/Onboard-SDK/osdk-core/platform/linux/inc/posix_thread.hpp"
    "C:/Users/jonathan.michel/Desktop/Matrice210/Onboard-SDK/osdk-core/platform/linux/inc/posix_thread_manager.hpp"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xdevx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}C:/Program Files (x86)/matrice210/lib/cmake/djiosdk/djiosdkTargets.cmake")
    file(DIFFERENT EXPORT_FILE_CHANGED FILES
         "$ENV{DESTDIR}C:/Program Files (x86)/matrice210/lib/cmake/djiosdk/djiosdkTargets.cmake"
         "C:/Users/jonathan.michel/Desktop/Matrice210/code/Matrice210Pi3/cmake-build-debug/bin/osdk-core/CMakeFiles/Export/C_/Program_Files_(x86)/matrice210/lib/cmake/djiosdk/djiosdkTargets.cmake")
    if(EXPORT_FILE_CHANGED)
      file(GLOB OLD_CONFIG_FILES "$ENV{DESTDIR}C:/Program Files (x86)/matrice210/lib/cmake/djiosdk/djiosdkTargets-*.cmake")
      if(OLD_CONFIG_FILES)
        message(STATUS "Old export file \"$ENV{DESTDIR}C:/Program Files (x86)/matrice210/lib/cmake/djiosdk/djiosdkTargets.cmake\" will be replaced.  Removing files [${OLD_CONFIG_FILES}].")
        file(REMOVE ${OLD_CONFIG_FILES})
      endif()
    endif()
  endif()
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "C:/Program Files (x86)/matrice210/lib/cmake/djiosdk/djiosdkTargets.cmake")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "C:/Program Files (x86)/matrice210/lib/cmake/djiosdk" TYPE FILE FILES "C:/Users/jonathan.michel/Desktop/Matrice210/code/Matrice210Pi3/cmake-build-debug/bin/osdk-core/CMakeFiles/Export/C_/Program_Files_(x86)/matrice210/lib/cmake/djiosdk/djiosdkTargets.cmake")
  if("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Dd][Ee][Bb][Uu][Gg])$")
    list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
     "C:/Program Files (x86)/matrice210/lib/cmake/djiosdk/djiosdkTargets-debug.cmake")
    if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
        message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
    endif()
    if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
        message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
    endif()
file(INSTALL DESTINATION "C:/Program Files (x86)/matrice210/lib/cmake/djiosdk" TYPE FILE FILES "C:/Users/jonathan.michel/Desktop/Matrice210/code/Matrice210Pi3/cmake-build-debug/bin/osdk-core/CMakeFiles/Export/C_/Program_Files_(x86)/matrice210/lib/cmake/djiosdk/djiosdkTargets-debug.cmake")
  endif()
endif()

