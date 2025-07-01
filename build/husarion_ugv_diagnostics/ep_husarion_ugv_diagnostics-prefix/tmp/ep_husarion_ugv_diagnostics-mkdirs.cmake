# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "/husarion_ws/src/husarion_ugv_ros/husarion_ugv_diagnostics"
  "/husarion_ws/build/husarion_ugv_diagnostics"
  "/husarion_ws/build/husarion_ugv_diagnostics/ep_husarion_ugv_diagnostics-prefix"
  "/husarion_ws/build/husarion_ugv_diagnostics/ep_husarion_ugv_diagnostics-prefix/tmp"
  "/husarion_ws/build/husarion_ugv_diagnostics/ep_husarion_ugv_diagnostics-prefix/src/ep_husarion_ugv_diagnostics-stamp"
  "/husarion_ws/build/husarion_ugv_diagnostics/ep_husarion_ugv_diagnostics-prefix/src"
  "/husarion_ws/build/husarion_ugv_diagnostics/ep_husarion_ugv_diagnostics-prefix/src/ep_husarion_ugv_diagnostics-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/husarion_ws/build/husarion_ugv_diagnostics/ep_husarion_ugv_diagnostics-prefix/src/ep_husarion_ugv_diagnostics-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/husarion_ws/build/husarion_ugv_diagnostics/ep_husarion_ugv_diagnostics-prefix/src/ep_husarion_ugv_diagnostics-stamp${cfgdir}") # cfgdir has leading slash
endif()
