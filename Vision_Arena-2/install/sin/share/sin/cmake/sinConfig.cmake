# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_sin_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED sin_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(sin_FOUND FALSE)
  elseif(NOT sin_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(sin_FOUND FALSE)
  endif()
  return()
endif()
set(_sin_CONFIG_INCLUDED TRUE)

# output package information
if(NOT sin_FIND_QUIETLY)
  message(STATUS "Found sin: 0.0.0 (${sin_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'sin' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${sin_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(sin_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${sin_DIR}/${_extra}")
endforeach()
