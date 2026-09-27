cmake_minimum_required(VERSION 3.20)

project(vg_core
  VERSION 0.1.0
  DESCRIPTION "Platform-agnostic core for Victory Garden: mapping, entity tracking and sun analysis"
  LANGUAGES CXX)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Only build tests by default when vg_core is the top-level project, so a
# consuming app (e.g. vg_android via add_subdirectory) doesn't pull them in.
if(PROJECT_IS_TOP_LEVEL)
  set(_vg_core_tests_default ON)
else()
  set(_vg_core_tests_default OFF)
endif()
option(VG_CORE_BUILD_TESTS "Build vg_core unit tests" ${_vg_core_tests_default})
option(VG_CORE_WARNINGS_AS_ERRORS "Treat compiler warnings as errors" OFF)

include(cmake/CompilerWarnings.cmake)

add_library(vg_core
  src/version.cpp)
add_library(vg_core::vg_core ALIAS vg_core)

target_compile_features(vg_core PUBLIC cxx_std_17)
set_target_properties(vg_core PROPERTIES CXX_EXTENSIONS OFF)
target_include_directories(vg_core
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
vg_core_set_warnings(vg_core)

configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/src/version_config.hpp.in
  ${CMAKE_CURRENT_BINARY_DIR}/generated/vg_core/version_config.hpp)
target_include_directories(vg_core PRIVATE ${CMAKE_CURRENT_BINARY_DIR}/generated)

if(VG_CORE_BUILD_TESTS)
  enable_testing()
  add_subdirectory(tests)
endif()

include(GNUInstallDirs)
install(TARGETS vg_core EXPORT vg_coreTargets
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR})
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(EXPORT vg_coreTargets
  NAMESPACE vg_core::
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/vg_core)
