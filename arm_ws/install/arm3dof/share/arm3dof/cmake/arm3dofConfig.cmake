# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_arm3dof_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED arm3dof_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(arm3dof_FOUND FALSE)
  elseif(NOT arm3dof_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(arm3dof_FOUND FALSE)
  endif()
  return()
endif()
set(_arm3dof_CONFIG_INCLUDED TRUE)

# output package information
if(NOT arm3dof_FIND_QUIETLY)
  message(STATUS "Found arm3dof: 0.0.0 (${arm3dof_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'arm3dof' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT arm3dof_DEPRECATED_QUIET)
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(arm3dof_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${arm3dof_DIR}/${_extra}")
endforeach()
