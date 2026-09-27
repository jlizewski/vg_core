include(FetchContent)

# Prefer a system GoogleTest; fall back to fetching a pinned release.
find_package(GTest 1.14 QUIET)
if(NOT GTest_FOUND)
  FetchContent_Declare(googletest
    GIT_REPOSITORY https://github.com/google/googletest.git
    GIT_TAG b514bdc898e2951020cbdca1304b75f5950d1f59 # v1.15.2
    GIT_SHALLOW FALSE)
  set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
  set(INSTALL_GTEST OFF CACHE BOOL "" FORCE)
  set(BUILD_GMOCK OFF CACHE BOOL "" FORCE)
  FetchContent_MakeAvailable(googletest)
endif()

include(GoogleTest)

add_executable(vg_core_tests
  height_map_io_test.cpp
  tsdf_volume_test.cpp
  version_test.cpp)
target_link_libraries(vg_core_tests PRIVATE vg_core::vg_core GTest::gtest_main)
vg_core_set_warnings(vg_core_tests)

gtest_discover_tests(vg_core_tests)

if(VG_CORE_BUILD_IO)
  add_executable(vg_core_io_tests
    protobuf_test.cpp
    session_io_test.cpp)
  target_link_libraries(vg_core_io_tests PRIVATE vg_core::vg_core_io GTest::gtest_main)
  vg_core_set_warnings(vg_core_io_tests)
  gtest_discover_tests(vg_core_io_tests)
endif()
