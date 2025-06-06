# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_test_robot_plugin_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED test_robot_plugin_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(test_robot_plugin_FOUND FALSE)
  elseif(NOT test_robot_plugin_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(test_robot_plugin_FOUND FALSE)
  endif()
  return()
endif()
set(_test_robot_plugin_CONFIG_INCLUDED TRUE)

# output package information
if(NOT test_robot_plugin_FIND_QUIETLY)
  message(STATUS "Found test_robot_plugin: 0.0.0 (${test_robot_plugin_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'test_robot_plugin' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${test_robot_plugin_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(test_robot_plugin_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${test_robot_plugin_DIR}/${_extra}")
endforeach()
