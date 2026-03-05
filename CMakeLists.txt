cmake_minimum_required(VERSION 3.20)

# ------------------------------------------------------------------------------
# Project
# ------------------------------------------------------------------------------
project(crx_kinematics LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# ------------------------------------------------------------------------------
# Options
# ------------------------------------------------------------------------------
option(ROBODK_DEPLOY "Copy built library to RoboDK robotextensions after build" OFF)
set(ROBODK_ROOT "" CACHE PATH "RoboDK root path (e.g. C:/RoboDK). If empty, will attempt discovery on Windows.")
option(CRXKIN_USE_VENDORED_EIGEN "Prefer vendored Eigen in third_party/eigen if present" ON)

# ------------------------------------------------------------------------------
# Output dirs (MSVC-like): <repo>/build/{Debug,Release}
# Works for both single-config (Ninja/Make) and multi-config (VS/MSBuild)
# ------------------------------------------------------------------------------
set(_OUT_ROOT "${CMAKE_SOURCE_DIR}/build")
foreach(_CFG DEBUG RELEASE RELWITHDEBINFO MINSIZEREL)
  string(TOUPPER "${_CFG}" _CFG_U)
  if(_CFG_U STREQUAL "DEBUG")
    set(_CFG_DIR "Debug")
  else()
    # Treat all non-Debug configs as "Release" to match the batch expectation,
    # or change this mapping if you want RelWithDebInfo to have its own dir.
    set(_CFG_DIR "Release")
  endif()

  set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_${_CFG_U} "${_OUT_ROOT}/${_CFG_DIR}")
  set(CMAKE_LIBRARY_OUTPUT_DIRECTORY_${_CFG_U} "${_OUT_ROOT}/${_CFG_DIR}")
  set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY_${_CFG_U} "${_OUT_ROOT}/${_CFG_DIR}")
endforeach()

# Also set generic fallbacks (single-config generators use these)
if(CMAKE_BUILD_TYPE STREQUAL "Debug")
  set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${_OUT_ROOT}/Debug")
  set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${_OUT_ROOT}/Debug")
  set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${_OUT_ROOT}/Debug")
else()
  set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${_OUT_ROOT}/Release")
  set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${_OUT_ROOT}/Release")
  set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${_OUT_ROOT}/Release")
endif()

message(STATUS "Build output root: ${_OUT_ROOT}")

# ------------------------------------------------------------------------------
# Eigen: prefer vendored, otherwise FetchContent (Eigen 5.0.1)
# ------------------------------------------------------------------------------
set(_VENDORED_EIGEN_DIR "${CMAKE_SOURCE_DIR}/third_party/eigen")

if(CRXKIN_USE_VENDORED_EIGEN AND EXISTS "${_VENDORED_EIGEN_DIR}/Eigen/Core")
  message(STATUS "Using vendored Eigen include path: ${_VENDORED_EIGEN_DIR}")
  add_library(Eigen3::Eigen INTERFACE IMPORTED)
  target_include_directories(Eigen3::Eigen INTERFACE "${_VENDORED_EIGEN_DIR}")
else()
  message(STATUS "Vendored Eigen not found (or disabled). Fetching Eigen 5.0.1 via FetchContent...")

  include(FetchContent)
  set(EIGEN_BUILD_TESTING OFF CACHE BOOL "" FORCE)
  set(EIGEN_BUILD_DOC OFF CACHE BOOL "" FORCE)
  set(EIGEN_BUILD_PKGCONFIG OFF CACHE BOOL "" FORCE)
  set(EIGEN_MPL2_ONLY ON CACHE BOOL "" FORCE)

  FetchContent_Declare(
    Eigen
    GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
    GIT_TAG        5.0.1
    GIT_SHALLOW    TRUE
    GIT_PROGRESS   TRUE
  )
  FetchContent_MakeAvailable(Eigen)
endif()

# ------------------------------------------------------------------------------
# Library target (src/)
# ------------------------------------------------------------------------------
add_library(crx_kinematics SHARED
  "${CMAKE_SOURCE_DIR}/src/crx_kinematics.cpp"
  "${CMAKE_SOURCE_DIR}/src/crx_solver.cpp"
  "${CMAKE_SOURCE_DIR}/src/crx_robodk_adapter.cpp"
)

set_target_properties(crx_kinematics PROPERTIES
  OUTPUT_NAME "crx_kinematics"
)

target_include_directories(crx_kinematics PRIVATE
  "${CMAKE_SOURCE_DIR}/include"   
  "${CMAKE_SOURCE_DIR}/src"       
)

target_link_libraries(crx_kinematics PRIVATE Eigen3::Eigen)

if(NOT MSVC)
  target_compile_options(crx_kinematics PRIVATE -fvisibility=hidden)
endif()

# ------------------------------------------------------------------------------
# RoboDK deployment (optional post-build copy)
# - Windows: tries registry HKLM\SOFTWARE\RoboDK\INSTDIR if ROBODK_ROOT not set
# ------------------------------------------------------------------------------
function(_crxkin_discover_robodk_root out_var)
  if(ROBODK_ROOT AND IS_DIRECTORY "${ROBODK_ROOT}")
    set(${out_var} "${ROBODK_ROOT}" PARENT_SCOPE)
    return()
  endif()

  if(WIN32)
    # Query registry. Prefer CMake's 'reg' to avoid PowerShell policy issues.
    execute_process(
      COMMAND reg query "HKLM\\SOFTWARE\\RoboDK" /v INSTDIR
      OUTPUT_VARIABLE _reg_out
      ERROR_VARIABLE  _reg_err
      RESULT_VARIABLE _reg_rc
      OUTPUT_STRIP_TRAILING_WHITESPACE
    )
    if(_reg_rc EQUAL 0)
      # Parse: "...  INSTDIR    REG_SZ    C:\Program Files\RoboDK"
      string(REGEX REPLACE ".*INSTDIR[ \t]+REG_SZ[ \t]+([^\r\n]+).*" "\\1" _inst "${_reg_out}")
      if(IS_DIRECTORY "${_inst}")
        file(TO_CMAKE_PATH "${_inst}" _inst_norm)
        set(${out_var} "${_inst_norm}" PARENT_SCOPE)
        return()
      endif()
    endif()

    # Fallback
    if(IS_DIRECTORY "C:/RoboDK")
      set(${out_var} "C:/RoboDK" PARENT_SCOPE)
      return()
    endif()
  endif()

  # Non-windows fallbacks (match your qmake defaults loosely)
  if(APPLE)
    set(_mac "${CMAKE_HOME_DIRECTORY}/RoboDK/RoboDK.app/Contents/MacOS")
    if(IS_DIRECTORY "${_mac}")
      set(${out_var} "${_mac}" PARENT_SCOPE)
      return()
    endif()
  elseif(UNIX)
    set(_lin "$ENV{HOME}/RoboDK/bin")
    if(IS_DIRECTORY "${_lin}")
      set(${out_var} "${_lin}" PARENT_SCOPE)
      return()
    endif()
  endif()

  set(${out_var} "" PARENT_SCOPE)
endfunction()

if(ROBODK_DEPLOY)
  _crxkin_discover_robodk_root(_RDK_ROOT)

  if(WIN32)
    if(_RDK_ROOT)
      set(_RDK_DST "${_RDK_ROOT}/bin/robotextensions")
    else()
      set(_RDK_DST "")
    endif()
  else()
    if(_RDK_ROOT)
      set(_RDK_DST "${_RDK_ROOT}/robotextensions")
    else()
      set(_RDK_DST "")
    endif()
  endif()

  if(NOT _RDK_DST)
    message(WARNING "ROBODK_DEPLOY=ON but RoboDK root could not be discovered. Set -DROBODK_ROOT=... to enable deploy.")
  else()
    message(STATUS "RoboDK deploy enabled. Destination: ${_RDK_DST}")

    add_custom_command(TARGET crx_kinematics POST_BUILD
      COMMAND ${CMAKE_COMMAND} -E make_directory "${_RDK_DST}"
      COMMAND ${CMAKE_COMMAND} -E copy_if_different
              "$<TARGET_FILE:crx_kinematics>"
              "${_RDK_DST}/$<TARGET_FILE_NAME:crx_kinematics>"
      COMMENT "Deploying crx_kinematics to RoboDK robotextensions"
    )
  endif()
endif()
