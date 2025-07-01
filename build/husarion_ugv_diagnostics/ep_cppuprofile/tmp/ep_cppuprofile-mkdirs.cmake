# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "/husarion_ws/build/husarion_ugv_diagnostics/ep_cppuprofile/src/ep_cppuprofile"
  "/husarion_ws/build/husarion_ugv_diagnostics/ep_cppuprofile/src/ep_cppuprofile-build"
  "/husarion_ws/install/husarion_ugv_diagnostics"
  "/husarion_ws/build/husarion_ugv_diagnostics/ep_cppuprofile/tmp"
  "/husarion_ws/build/husarion_ugv_diagnostics/ep_cppuprofile/src/ep_cppuprofile-stamp"
  "/husarion_ws/build/husarion_ugv_diagnostics/ep_cppuprofile/src"
  "/husarion_ws/build/husarion_ugv_diagnostics/ep_cppuprofile/src/ep_cppuprofile-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/husarion_ws/build/husarion_ugv_diagnostics/ep_cppuprofile/src/ep_cppuprofile-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/husarion_ws/build/husarion_ugv_diagnostics/ep_cppuprofile/src/ep_cppuprofile-stamp${cfgdir}") # cfgdir has leading slash
endif()
