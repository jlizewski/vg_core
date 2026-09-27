include(FetchContent)

# Dependencies of vg_core_io only; the vg_core mapping library doesn't use them.

# zstd: chunk compression for MCAP. Static library, no programs or tests.
if(NOT TARGET libzstd_static)
  FetchContent_Declare(zstd
    GIT_REPOSITORY https://github.com/facebook/zstd.git
    GIT_TAG v1.5.7 # f8745da6ff1ad1e7bab384bd1f9d742439278e99
    GIT_SHALLOW TRUE
    SOURCE_SUBDIR build/cmake)
  set(ZSTD_BUILD_PROGRAMS OFF CACHE BOOL "" FORCE)
  set(ZSTD_BUILD_TESTS OFF CACHE BOOL "" FORCE)
  set(ZSTD_BUILD_SHARED OFF CACHE BOOL "" FORCE)
  set(ZSTD_BUILD_STATIC ON CACHE BOOL "" FORCE)
  set(ZSTD_LEGACY_SUPPORT OFF CACHE BOOL "" FORCE)
  set(ZSTD_MULTITHREAD_SUPPORT OFF CACHE BOOL "" FORCE)
  FetchContent_MakeAvailable(zstd)
  set(VG_ZSTD_INCLUDE_DIR ${zstd_SOURCE_DIR}/lib)
endif()

# MCAP C++ library (header-only). Its CMake isn't used; SOURCE_SUBDIR points
# nowhere so only the sources are fetched.
FetchContent_Declare(mcap
  GIT_REPOSITORY https://github.com/foxglove/mcap.git
  GIT_TAG releases/cpp/v2.1.3 # 1420296ffcfdcde4b6894c0c1aba0ad083f93dde
  GIT_SHALLOW TRUE
  SOURCE_SUBDIR do-not-add)
FetchContent_MakeAvailable(mcap)
add_library(vg_mcap INTERFACE)
target_include_directories(vg_mcap SYSTEM INTERFACE ${mcap_SOURCE_DIR}/cpp/mcap/include
  ${VG_ZSTD_INCLUDE_DIR})
# Only zstd is linked; LZ4-compressed files are not supported. MCAP_PUBLIC is
# emptied because the library is compiled into vg_core_io statically; otherwise
# Windows builds would dllimport symbols that live in the same binary.
target_compile_definitions(vg_mcap INTERFACE MCAP_COMPRESSION_NO_LZ4 MCAP_PUBLIC=)
target_link_libraries(vg_mcap INTERFACE libzstd_static)
