# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_robotx_ekf_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED robotx_ekf_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(robotx_ekf_FOUND FALSE)
  elseif(NOT robotx_ekf_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(robotx_ekf_FOUND FALSE)
  endif()
  return()
endif()
set(_robotx_ekf_CONFIG_INCLUDED TRUE)

# output package information
if(NOT robotx_ekf_FIND_QUIETLY)
  message(STATUS "Found robotx_ekf: 1.0.0 (${robotx_ekf_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'robotx_ekf' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${robotx_ekf_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(robotx_ekf_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "ament_cmake_export_libraries-extras.cmake;ament_cmake_export_include_directories-extras.cmake")
foreach(_extra ${_extras})
  include("${robotx_ekf_DIR}/${_extra}")
endforeach()
