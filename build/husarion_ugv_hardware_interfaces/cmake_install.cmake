# Install script for directory: /husarion_ws/src/husarion_ugv_ros/husarion_ugv_hardware_interfaces

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/husarion_ws/install/husarion_ugv_hardware_interfaces")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
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

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

# Set default install directory permissions.
if(NOT DEFINED CMAKE_OBJDUMP)
  set(CMAKE_OBJDUMP "/usr/bin/objdump")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/husarion_ws/build/husarion_ugv_hardware_interfaces/include/husarion_ugv_hardware_interfaces")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libhusarion_ugv_hardware_interfaces.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libhusarion_ugv_hardware_interfaces.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libhusarion_ugv_hardware_interfaces.so"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES "/husarion_ws/build/husarion_ugv_hardware_interfaces/libhusarion_ugv_hardware_interfaces.so")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libhusarion_ugv_hardware_interfaces.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libhusarion_ugv_hardware_interfaces.so")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libhusarion_ugv_hardware_interfaces.so"
         OLD_RPATH "/opt/ros/jazzy/lib:/husarion_ws/install/husarion_ugv_msgs/lib:/opt/ros/jazzy/share/libphidget22/cmake/../../../opt/libphidget22/lib:/opt/ros/jazzy/lib/x86_64-linux-gnu:/husarion_ws/install/husarion_ugv_hardware_interfaces/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libhusarion_ugv_hardware_interfaces.so")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/husarion_ugv_hardware_interfaces" TYPE DIRECTORY FILES "/husarion_ws/src/husarion_ugv_ros/husarion_ugv_hardware_interfaces/include/")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/husarion_ugv_hardware_interfaces" TYPE DIRECTORY FILES "/husarion_ws/src/husarion_ugv_ros/husarion_ugv_hardware_interfaces/config")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/husarion_ugv_hardware_interfaces/test" TYPE DIRECTORY FILES "/husarion_ws/src/husarion_ugv_ros/husarion_ugv_hardware_interfaces/test/config")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/husarion_ugv_hardware_interfaces/environment" TYPE FILE FILES "/opt/ros/jazzy/lib/python3.12/site-packages/ament_package/template/environment_hook/library_path.sh")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/husarion_ugv_hardware_interfaces/environment" TYPE FILE FILES "/husarion_ws/build/husarion_ugv_hardware_interfaces/ament_cmake_environment_hooks/library_path.dsv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/husarion_ugv_hardware_interfaces" TYPE FILE FILES "/husarion_ws/src/husarion_ugv_ros/husarion_ugv_hardware_interfaces/husarion_ugv_hardware_interfaces.xml")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/package_run_dependencies" TYPE FILE FILES "/husarion_ws/build/husarion_ugv_hardware_interfaces/ament_cmake_index/share/ament_index/resource_index/package_run_dependencies/husarion_ugv_hardware_interfaces")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/parent_prefix_path" TYPE FILE FILES "/husarion_ws/build/husarion_ugv_hardware_interfaces/ament_cmake_index/share/ament_index/resource_index/parent_prefix_path/husarion_ugv_hardware_interfaces")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/husarion_ugv_hardware_interfaces/environment" TYPE FILE FILES "/opt/ros/jazzy/share/ament_cmake_core/cmake/environment_hooks/environment/ament_prefix_path.sh")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/husarion_ugv_hardware_interfaces/environment" TYPE FILE FILES "/husarion_ws/build/husarion_ugv_hardware_interfaces/ament_cmake_environment_hooks/ament_prefix_path.dsv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/husarion_ugv_hardware_interfaces/environment" TYPE FILE FILES "/opt/ros/jazzy/share/ament_cmake_core/cmake/environment_hooks/environment/path.sh")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/husarion_ugv_hardware_interfaces/environment" TYPE FILE FILES "/husarion_ws/build/husarion_ugv_hardware_interfaces/ament_cmake_environment_hooks/path.dsv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/husarion_ugv_hardware_interfaces" TYPE FILE FILES "/husarion_ws/build/husarion_ugv_hardware_interfaces/ament_cmake_environment_hooks/local_setup.bash")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/husarion_ugv_hardware_interfaces" TYPE FILE FILES "/husarion_ws/build/husarion_ugv_hardware_interfaces/ament_cmake_environment_hooks/local_setup.sh")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/husarion_ugv_hardware_interfaces" TYPE FILE FILES "/husarion_ws/build/husarion_ugv_hardware_interfaces/ament_cmake_environment_hooks/local_setup.zsh")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/husarion_ugv_hardware_interfaces" TYPE FILE FILES "/husarion_ws/build/husarion_ugv_hardware_interfaces/ament_cmake_environment_hooks/local_setup.dsv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/husarion_ugv_hardware_interfaces" TYPE FILE FILES "/husarion_ws/build/husarion_ugv_hardware_interfaces/ament_cmake_environment_hooks/package.dsv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/packages" TYPE FILE FILES "/husarion_ws/build/husarion_ugv_hardware_interfaces/ament_cmake_index/share/ament_index/resource_index/packages/husarion_ugv_hardware_interfaces")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/hardware_interface__pluginlib__plugin" TYPE FILE FILES "/husarion_ws/build/husarion_ugv_hardware_interfaces/ament_cmake_index/share/ament_index/resource_index/hardware_interface__pluginlib__plugin/husarion_ugv_hardware_interfaces")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/husarion_ugv_hardware_interfaces/cmake/export_husarion_ugv_hardware_interfacesExport.cmake")
    file(DIFFERENT _cmake_export_file_changed FILES
         "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/husarion_ugv_hardware_interfaces/cmake/export_husarion_ugv_hardware_interfacesExport.cmake"
         "/husarion_ws/build/husarion_ugv_hardware_interfaces/CMakeFiles/Export/48fb3829075ef4ac6babace8f1ea246d/export_husarion_ugv_hardware_interfacesExport.cmake")
    if(_cmake_export_file_changed)
      file(GLOB _cmake_old_config_files "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/husarion_ugv_hardware_interfaces/cmake/export_husarion_ugv_hardware_interfacesExport-*.cmake")
      if(_cmake_old_config_files)
        string(REPLACE ";" ", " _cmake_old_config_files_text "${_cmake_old_config_files}")
        message(STATUS "Old export file \"$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/husarion_ugv_hardware_interfaces/cmake/export_husarion_ugv_hardware_interfacesExport.cmake\" will be replaced.  Removing files [${_cmake_old_config_files_text}].")
        unset(_cmake_old_config_files_text)
        file(REMOVE ${_cmake_old_config_files})
      endif()
      unset(_cmake_old_config_files)
    endif()
    unset(_cmake_export_file_changed)
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/husarion_ugv_hardware_interfaces/cmake" TYPE FILE FILES "/husarion_ws/build/husarion_ugv_hardware_interfaces/CMakeFiles/Export/48fb3829075ef4ac6babace8f1ea246d/export_husarion_ugv_hardware_interfacesExport.cmake")
  if(CMAKE_INSTALL_CONFIG_NAME MATCHES "^()$")
    file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/husarion_ugv_hardware_interfaces/cmake" TYPE FILE FILES "/husarion_ws/build/husarion_ugv_hardware_interfaces/CMakeFiles/Export/48fb3829075ef4ac6babace8f1ea246d/export_husarion_ugv_hardware_interfacesExport-noconfig.cmake")
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/husarion_ugv_hardware_interfaces/cmake" TYPE FILE FILES "/husarion_ws/build/husarion_ugv_hardware_interfaces/ament_cmake_export_dependencies/ament_cmake_export_dependencies-extras.cmake")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/husarion_ugv_hardware_interfaces/cmake" TYPE FILE FILES "/husarion_ws/build/husarion_ugv_hardware_interfaces/ament_cmake_export_include_directories/ament_cmake_export_include_directories-extras.cmake")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/husarion_ugv_hardware_interfaces/cmake" TYPE FILE FILES "/husarion_ws/build/husarion_ugv_hardware_interfaces/ament_cmake_export_targets/ament_cmake_export_targets-extras.cmake")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/husarion_ugv_hardware_interfaces/cmake" TYPE FILE FILES
    "/husarion_ws/build/husarion_ugv_hardware_interfaces/ament_cmake_core/husarion_ugv_hardware_interfacesConfig.cmake"
    "/husarion_ws/build/husarion_ugv_hardware_interfaces/ament_cmake_core/husarion_ugv_hardware_interfacesConfig-version.cmake"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/husarion_ugv_hardware_interfaces" TYPE FILE FILES "/husarion_ws/src/husarion_ugv_ros/husarion_ugv_hardware_interfaces/package.xml")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/husarion_ws/build/husarion_ugv_hardware_interfaces/gmock/cmake_install.cmake")
  include("/husarion_ws/build/husarion_ugv_hardware_interfaces/gtest/cmake_install.cmake")

endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
file(WRITE "/husarion_ws/build/husarion_ugv_hardware_interfaces/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
