# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_kasva_sekli_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED kasva_sekli_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(kasva_sekli_FOUND FALSE)
  elseif(NOT kasva_sekli_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(kasva_sekli_FOUND FALSE)
  endif()
  return()
endif()
set(_kasva_sekli_CONFIG_INCLUDED TRUE)

# output package information
if(NOT kasva_sekli_FIND_QUIETLY)
  message(STATUS "Found kasva_sekli: 0.0.0 (${kasva_sekli_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'kasva_sekli' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT kasva_sekli_DEPRECATED_QUIET)
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(kasva_sekli_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${kasva_sekli_DIR}/${_extra}")
endforeach()
