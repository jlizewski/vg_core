include(FetchContent)

# Dependencies of vg_core_config (YAML settings files) only; the vg_core
# mapping library doesn't use them, so it stays free of them on phones.

# yaml-cpp: prefer a system install; otherwise fetch a pinned release.
find_package(yaml-cpp 0.8 QUIET)
if(NOT TARGET yaml-cpp::yaml-cpp AND NOT TARGET yaml-cpp)
  FetchContent_Declare(yaml-cpp
    GIT_REPOSITORY https://github.com/jbeder/yaml-cpp.git
    GIT_TAG 0.8.0 # f7320141120f720aecc4c32be25586e7da9eb978
    GIT_SHALLOW TRUE)
  set(YAML_CPP_BUILD_TESTS OFF CACHE BOOL "" FORCE)
  set(YAML_CPP_BUILD_TOOLS OFF CACHE BOOL "" FORCE)
  set(YAML_CPP_BUILD_CONTRIB OFF CACHE BOOL "" FORCE)
  set(YAML_CPP_INSTALL OFF CACHE BOOL "" FORCE)
  set(YAML_BUILD_SHARED_LIBS OFF CACHE BOOL "" FORCE)
  # yaml-cpp 0.8.0 asks for CMake 3.4, which CMake 4 no longer accepts.
  set(CMAKE_POLICY_VERSION_MINIMUM 3.5)
  FetchContent_MakeAvailable(yaml-cpp)
  unset(CMAKE_POLICY_VERSION_MINIMUM)
endif()
if(NOT TARGET yaml-cpp::yaml-cpp)
  add_library(yaml-cpp::yaml-cpp ALIAS yaml-cpp)
endif()
