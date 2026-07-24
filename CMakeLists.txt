cmake_minimum_required(VERSION 3.20)

# ------------------------------------------------------------------------------
# Project
# ------------------------------------------------------------------------------
project(crx_kinematics VERSION 0.3.1 LANGUAGES CXX)

include(CTest)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON CACHE BOOL
    "Generate compile_commands.json for C++ tooling" FORCE)

# Reject Eigen modules that are not available under MPL 2.0. This definition
# must be active before any Eigen header is parsed.
add_compile_definitions(EIGEN_MPL2_ONLY)

# ------------------------------------------------------------------------------
# Options
# ------------------------------------------------------------------------------
option(ROBODK_DEPLOY "Copy built library to RoboDK robotextensions after build" OFF)
set(ROBODK_ROOT "" CACHE PATH "RoboDK root path (e.g. C:/RoboDK). If empty, will attempt discovery on Windows.")
option(CRXKIN_USE_VENDORED_EIGEN "Prefer vendored Eigen in third_party/eigen if present" ON)
option(CRXKIN_ENABLE_CLANG_TIDY "Run clang-tidy while compiling crx_kinematics" OFF)

# ------------------------------------------------------------------------------
# C++ quality tools
# - Prefer the LLVM tools bundled with Visual Studio, discovered through vswhere.
# - Fall back to PATH so the same targets remain useful outside Visual Studio.
# ------------------------------------------------------------------------------
set(_CRXKIN_VS_LLVM_BIN "")
if(WIN32)
  find_program(CRXKIN_VSWHERE_EXECUTABLE
    NAMES vswhere
    HINTS
      "C:/Program Files (x86)/Microsoft Visual Studio/Installer"
      "$ENV{ProgramFiles}/Microsoft Visual Studio/Installer"
    DOC "Visual Studio installation locator"
  )

  if(CRXKIN_VSWHERE_EXECUTABLE)
    execute_process(
      COMMAND "${CRXKIN_VSWHERE_EXECUTABLE}"
              -latest -products *
              -requires Microsoft.VisualStudio.Component.VC.Llvm.Clang
              -property installationPath
      OUTPUT_VARIABLE _CRXKIN_VS_INSTALL_PATH
      RESULT_VARIABLE _CRXKIN_VSWHERE_RESULT
      OUTPUT_STRIP_TRAILING_WHITESPACE
    )
    if(_CRXKIN_VSWHERE_RESULT EQUAL 0 AND _CRXKIN_VS_INSTALL_PATH)
      file(TO_CMAKE_PATH "${_CRXKIN_VS_INSTALL_PATH}/VC/Tools/Llvm/x64/bin"
           _CRXKIN_VS_LLVM_BIN)
      message(STATUS "Visual Studio LLVM tools: ${_CRXKIN_VS_LLVM_BIN}")
    endif()
  endif()
endif()

find_program(CRXKIN_CLANG_TIDY_EXECUTABLE
  NAMES clang-tidy
  HINTS "${_CRXKIN_VS_LLVM_BIN}"
  DOC "clang-tidy executable"
)
find_program(CRXKIN_CLANG_FORMAT_EXECUTABLE
  NAMES clang-format
  HINTS "${_CRXKIN_VS_LLVM_BIN}"
  DOC "clang-format executable"
)

if(CRXKIN_ENABLE_CLANG_TIDY)
  if(NOT CRXKIN_CLANG_TIDY_EXECUTABLE)
    message(FATAL_ERROR
      "CRXKIN_ENABLE_CLANG_TIDY=ON, but clang-tidy was not found. "
      "Install the Visual Studio C++ Clang tools component or put clang-tidy on PATH.")
  endif()
  set(CMAKE_CXX_CLANG_TIDY "${CRXKIN_CLANG_TIDY_EXECUTABLE}")
  message(STATUS "clang-tidy enabled: ${CRXKIN_CLANG_TIDY_EXECUTABLE}")
endif()

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
  set(_CRXKIN_EIGEN_SOURCE_DIR "${_VENDORED_EIGEN_DIR}")
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
  set(_CRXKIN_EIGEN_SOURCE_DIR "${eigen_SOURCE_DIR}")
endif()

set(_CRXKIN_EIGEN_LICENSE "${_CRXKIN_EIGEN_SOURCE_DIR}/LICENSE")
set(_CRXKIN_EIGEN_LEGAL_FILES
  COPYING.APACHE
  COPYING.BSD
  COPYING.MINPACK
  COPYING.MPL2
  COPYING.README
  LICENSE
)
set(_CRXKIN_THIRD_PARTY_NOTICES "${CMAKE_SOURCE_DIR}/THIRD_PARTY_NOTICES.md")
set(_CRXKIN_PROJECT_LICENSE "${CMAKE_SOURCE_DIR}/LICENSE")
set(_CRXKIN_REQUIRED_LEGAL_FILES
  "${_CRXKIN_PROJECT_LICENSE}"
  "${_CRXKIN_THIRD_PARTY_NOTICES}"
)
foreach(_EIGEN_LEGAL_FILE IN LISTS _CRXKIN_EIGEN_LEGAL_FILES)
  list(APPEND _CRXKIN_REQUIRED_LEGAL_FILES
       "${_CRXKIN_EIGEN_SOURCE_DIR}/${_EIGEN_LEGAL_FILE}")
endforeach()
foreach(_LEGAL_FILE IN LISTS _CRXKIN_REQUIRED_LEGAL_FILES)
  if(NOT EXISTS "${_LEGAL_FILE}")
    message(FATAL_ERROR "Required third-party legal file not found: ${_LEGAL_FILE}")
  endif()
endforeach()

# ------------------------------------------------------------------------------
# Library target (src/)
# ------------------------------------------------------------------------------
set(CRXKIN_CPP_SOURCES
  "${CMAKE_SOURCE_DIR}/src/crx_kinematics.cpp"
  "${CMAKE_SOURCE_DIR}/src/crx_math_compat.cpp"
  "${CMAKE_SOURCE_DIR}/src/crx_solver.cpp"
  "${CMAKE_SOURCE_DIR}/src/crx_robodk_adapter.cpp"
)

file(GLOB CRXKIN_FORMAT_SOURCES CONFIGURE_DEPENDS
  "${CMAKE_SOURCE_DIR}/src/*.cpp"
  "${CMAKE_SOURCE_DIR}/include/*.h"
  "${CMAKE_SOURCE_DIR}/include/*.hpp"
  "${CMAKE_SOURCE_DIR}/tests/*.cpp"
  "${CMAKE_SOURCE_DIR}/tests/*.h"
  "${CMAKE_SOURCE_DIR}/tests/*.hpp"
)

add_library(crx_kinematics SHARED ${CRXKIN_CPP_SOURCES})

set_target_properties(crx_kinematics PROPERTIES
  OUTPUT_NAME "crx_kinematics"
  VERSION "${PROJECT_VERSION}"
)

target_include_directories(crx_kinematics PRIVATE
  "${CMAKE_SOURCE_DIR}/include"   
  "${CMAKE_SOURCE_DIR}/src"       
)

target_link_libraries(crx_kinematics PRIVATE Eigen3::Eigen)

# CMake supplies the platform-appropriate Release optimization flags.
target_compile_definitions(crx_kinematics PRIVATE
  "$<$<CONFIG:Release>:NDEBUG>"
)

if(WIN32 AND CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  # LLVM 22 can combine adjacent sin/cos calls into the POSIX `sincos` symbol,
  # which is not supplied by the MSVC runtime. Keep the compatibility function
  # itself unoptimized so the same transform cannot be applied recursively.
  set_source_files_properties(
    "${CMAKE_SOURCE_DIR}/src/crx_math_compat.cpp"
    PROPERTIES COMPILE_OPTIONS "/Od"
  )
endif()

if(CMAKE_CXX_COMPILER_ID MATCHES "Clang|GNU" AND
   NOT (MSVC OR CMAKE_CXX_COMPILER_FRONTEND_VARIANT STREQUAL "MSVC"))
  target_compile_options(crx_kinematics PRIVATE
    "$<$<CONFIG:Release>:-O3>"
    "$<$<CONFIG:Release>:-march=native>"
  )
endif()

# Keep the notice, exact Eigen header source, and licensing files beside every
# binary. The vendored tree may contain local changes, so an upstream URL alone
# is not sufficient corresponding-source access.
add_custom_command(TARGET crx_kinematics POST_BUILD
  COMMAND ${CMAKE_COMMAND} -E make_directory
          "$<TARGET_FILE_DIR:crx_kinematics>/licenses/Eigen"
  COMMAND ${CMAKE_COMMAND} -E copy_if_different
          "${_CRXKIN_PROJECT_LICENSE}"
          "$<TARGET_FILE_DIR:crx_kinematics>/LICENSE"
  COMMAND ${CMAKE_COMMAND} -E copy_if_different
          "${_CRXKIN_THIRD_PARTY_NOTICES}"
          "$<TARGET_FILE_DIR:crx_kinematics>/THIRD_PARTY_NOTICES.md"
  COMMAND ${CMAKE_COMMAND} -E copy_if_different
          "${_CRXKIN_EIGEN_LICENSE}"
          "$<TARGET_FILE_DIR:crx_kinematics>/licenses/Eigen/LICENSE"
  COMMAND ${CMAKE_COMMAND} -E copy_if_different
          "${_CRXKIN_EIGEN_SOURCE_DIR}/COPYING.APACHE"
          "${_CRXKIN_EIGEN_SOURCE_DIR}/COPYING.BSD"
          "${_CRXKIN_EIGEN_SOURCE_DIR}/COPYING.MINPACK"
          "${_CRXKIN_EIGEN_SOURCE_DIR}/COPYING.MPL2"
          "${_CRXKIN_EIGEN_SOURCE_DIR}/COPYING.README"
          "$<TARGET_FILE_DIR:crx_kinematics>/licenses/Eigen"
  COMMAND ${CMAKE_COMMAND} -E copy_directory
          "${_CRXKIN_EIGEN_SOURCE_DIR}/Eigen"
          "$<TARGET_FILE_DIR:crx_kinematics>/licenses/Eigen/source/Eigen"
  COMMENT "Copying project and third-party licenses and notices"
  VERBATIM
)

if(MSVC OR CMAKE_CXX_COMPILER_FRONTEND_VARIANT STREQUAL "MSVC")
  target_compile_options(crx_kinematics PRIVATE /W4 /permissive-)
else()
  target_compile_options(crx_kinematics PRIVATE -Wall -Wextra -Wpedantic)
endif()

if(NOT WIN32)
  target_compile_options(crx_kinematics PRIVATE -fvisibility=hidden)
endif()

if(CRXKIN_CLANG_FORMAT_EXECUTABLE)
  add_custom_target(format
    COMMAND "${CRXKIN_CLANG_FORMAT_EXECUTABLE}" -i ${CRXKIN_FORMAT_SOURCES}
    WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}"
    COMMENT "Formatting C++ sources with clang-format"
    VERBATIM
  )
  add_custom_target(format-check
    COMMAND "${CRXKIN_CLANG_FORMAT_EXECUTABLE}" --dry-run --Werror
            ${CRXKIN_FORMAT_SOURCES}
    WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}"
    COMMENT "Checking C++ formatting with clang-format"
    VERBATIM
  )
endif()

if(CRXKIN_CLANG_TIDY_EXECUTABLE)
  set(_CRXKIN_CLANG_TIDY_DRIVER_ARGS)
  if(MSVC)
    list(APPEND _CRXKIN_CLANG_TIDY_DRIVER_ARGS
         "--extra-arg-before=--driver-mode=cl")
  endif()

  add_custom_target(tidy
    COMMAND "${CRXKIN_CLANG_TIDY_EXECUTABLE}" -p "${CMAKE_BINARY_DIR}"
            ${_CRXKIN_CLANG_TIDY_DRIVER_ARGS}
            ${CRXKIN_CPP_SOURCES}
    WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}"
    COMMENT "Linting C++ sources with clang-tidy"
    VERBATIM
  )
  add_custom_target(tidy-fix
    COMMAND "${CRXKIN_CLANG_TIDY_EXECUTABLE}" -p "${CMAKE_BINARY_DIR}" --fix
            ${_CRXKIN_CLANG_TIDY_DRIVER_ARGS}
            ${CRXKIN_CPP_SOURCES}
    WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}"
    COMMENT "Applying clang-tidy fixes"
    VERBATIM
  )
endif()

# ------------------------------------------------------------------------------
# Tests and verification utility
# ------------------------------------------------------------------------------
if(BUILD_TESTING)
  find_program(CRXKIN_UV_EXECUTABLE
    NAMES uv
    DOC "uv executable used to run the Python regression suite"
  )

  if(CRXKIN_UV_EXECUTABLE)
    add_test(
      NAME crx.python-regression
      COMMAND "${CRXKIN_UV_EXECUTABLE}" run pytest
      WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}"
    )
    set_tests_properties(crx.python-regression PROPERTIES
      ENVIRONMENT "CRXKIN_LIBRARY_PATH=$<TARGET_FILE:crx_kinematics>"
    )

    add_custom_target(check
      COMMAND "${CMAKE_CTEST_COMMAND}" --test-dir "${CMAKE_BINARY_DIR}"
              --output-on-failure
      DEPENDS crx_kinematics
      COMMENT "Running CMake/CTest regression checks"
      USES_TERMINAL
      VERBATIM
    )
  else()
    message(WARNING
      "BUILD_TESTING=ON, but uv was not found. CTest integration is disabled."
    )
  endif()
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
      COMMAND ${CMAKE_COMMAND} -E make_directory "${_RDK_DST}/licenses/Eigen"
      COMMAND ${CMAKE_COMMAND} -E copy_if_different
              "$<TARGET_FILE:crx_kinematics>"
              "${_RDK_DST}/$<TARGET_FILE_NAME:crx_kinematics>"
      COMMAND ${CMAKE_COMMAND} -E copy_if_different
              "${_CRXKIN_PROJECT_LICENSE}"
              "${_RDK_DST}/LICENSE"
      COMMAND ${CMAKE_COMMAND} -E copy_if_different
              "${_CRXKIN_THIRD_PARTY_NOTICES}"
              "${_RDK_DST}/THIRD_PARTY_NOTICES.md"
      COMMAND ${CMAKE_COMMAND} -E copy_if_different
              "${_CRXKIN_EIGEN_LICENSE}"
              "${_RDK_DST}/licenses/Eigen/LICENSE"
      COMMAND ${CMAKE_COMMAND} -E copy_if_different
              "${_CRXKIN_EIGEN_SOURCE_DIR}/COPYING.APACHE"
              "${_CRXKIN_EIGEN_SOURCE_DIR}/COPYING.BSD"
              "${_CRXKIN_EIGEN_SOURCE_DIR}/COPYING.MINPACK"
              "${_CRXKIN_EIGEN_SOURCE_DIR}/COPYING.MPL2"
              "${_CRXKIN_EIGEN_SOURCE_DIR}/COPYING.README"
              "${_RDK_DST}/licenses/Eigen"
      COMMAND ${CMAKE_COMMAND} -E copy_directory
              "${_CRXKIN_EIGEN_SOURCE_DIR}/Eigen"
              "${_RDK_DST}/licenses/Eigen/source/Eigen"
      COMMENT "Deploying crx_kinematics to RoboDK robotextensions"
      VERBATIM
    )
  endif()
endif()
