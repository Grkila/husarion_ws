# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "/husarion_ws/build/husarion_ugv_hardware_interfaces/ep_libgpiod/src/ep_libgpiod"
  "/husarion_ws/build/husarion_ugv_hardware_interfaces/ep_libgpiod/src/ep_libgpiod-build"
  "/husarion_ws/install/husarion_ugv_hardware_interfaces"
  "/husarion_ws/build/husarion_ugv_hardware_interfaces/ep_libgpiod/tmp"
  "/husarion_ws/build/husarion_ugv_hardware_interfaces/ep_libgpiod/src/ep_libgpiod-stamp"
  "/husarion_ws/build/husarion_ugv_hardware_interfaces/ep_libgpiod/src"
  "/husarion_ws/build/husarion_ugv_hardware_interfaces/ep_libgpiod/src/ep_libgpiod-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/husarion_ws/build/husarion_ugv_hardware_interfaces/ep_libgpiod/src/ep_libgpiod-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/husarion_ws/build/husarion_ugv_hardware_interfaces/ep_libgpiod/src/ep_libgpiod-stamp${cfgdir}") # cfgdir has leading slash
endif()
