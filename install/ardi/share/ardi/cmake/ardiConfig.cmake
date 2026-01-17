# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_ardi_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED ardi_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(ardi_FOUND FALSE)
  elseif(NOT ardi_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(ardi_FOUND FALSE)
  endif()
  return()
endif()
set(_ardi_CONFIG_INCLUDED TRUE)

# output package information
if(NOT ardi_FIND_QUIETLY)
  message(STATUS "Found ardi: 1.0.0 (${ardi_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'ardi' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ardi_DEPRECATED_QUIET)
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(ardi_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${ardi_DIR}/${_extra}")
endforeach()
