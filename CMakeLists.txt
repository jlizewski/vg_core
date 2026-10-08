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
option(VG_CORE_BUILD_IO "Build vg_core_io (capture session recording and playback)" ON)
option(VG_CORE_BUILD_APPS "Build command-line apps such as vg_replay" ${_vg_core_tests_default})

include(cmake/CompilerWarnings.cmake)
include(cmake/Dependencies.cmake)

add_library(vg_core
  src/depth_spool.cpp
  src/geo_reference.cpp
  src/ground_segmentation.cpp
  src/height_map.cpp
  src/height_map_io.cpp
  src/live_height_map.cpp
  src/map_builder.cpp
  src/solar.cpp
  src/sun_map.cpp
  src/tsdf_volume.cpp
  src/voxel_map_io.cpp
  src/version.cpp)
add_library(vg_core::vg_core ALIAS vg_core)

target_compile_features(vg_core PUBLIC cxx_std_17)
set_target_properties(vg_core PROPERTIES CXX_EXTENSIONS OFF)
target_include_directories(vg_core
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
find_package(Threads REQUIRED)
target_link_libraries(vg_core PUBLIC Eigen3::Eigen Threads::Threads)
vg_core_set_warnings(vg_core)

configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/src/version_config.hpp.in
  ${CMAKE_CURRENT_BINARY_DIR}/generated/vg_core/version_config.hpp)
target_include_directories(vg_core PRIVATE ${CMAKE_CURRENT_BINARY_DIR}/generated)

# Capture session I/O (MCAP + protobuf). Kept out of vg_core so the mapping
# library has no file-format dependencies.
if(VG_CORE_BUILD_IO)
  include(cmake/IoDependencies.cmake)
  add_library(vg_core_io
    src/io/mcap_impl.cpp
    src/io/mcap_repair.cpp
    src/io/messages.cpp
    src/io/protobuf.cpp
    src/io/schema_descriptors.cpp
    src/io/session.cpp
    src/io/session_player.cpp
    src/io/session_reader.cpp
    src/io/session_writer.cpp)
  add_library(vg_core::vg_core_io ALIAS vg_core_io)
  target_link_libraries(vg_core_io PUBLIC vg_core PRIVATE vg_mcap)
  set_target_properties(vg_core_io PROPERTIES CXX_EXTENSIONS OFF)
  vg_core_set_warnings(vg_core_io)

  # Each apps/vg_<name>.cpp is a target vg_<name>, which the Python tools run
  # as `vg <name>` (underscores become subcommand words: vg_mcap_repair is
  # `vg mcap repair`). See tools/vg_tools/apps.py.
  if(VG_CORE_BUILD_APPS)
    file(GLOB _vg_core_apps CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/apps/vg_*.cpp)
    foreach(source ${_vg_core_apps})
      get_filename_component(app ${source} NAME_WE)
      add_executable(${app} ${source})
      target_link_libraries(${app} PRIVATE vg_core_io)
      vg_core_set_warnings(${app})
    endforeach()
  endif()
endif()

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
