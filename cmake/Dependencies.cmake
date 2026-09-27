include(FetchContent)

# Eigen (header-only). Prefer a system install; otherwise fetch a pinned
# release. Only the headers are used, so the fetched tree is not added as a
# subdirectory (SOURCE_SUBDIR points nowhere) to skip Eigen's own CMake setup.
find_package(Eigen3 3.4 QUIET NO_MODULE)
if(NOT TARGET Eigen3::Eigen)
  FetchContent_Declare(eigen
    GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
    GIT_TAG 3147391d946bb4b6c68edd901f2add6ac1f31f8c # 3.4.0
    GIT_SHALLOW FALSE
    SOURCE_SUBDIR do-not-add)
  FetchContent_MakeAvailable(eigen)
  add_library(Eigen3::Eigen INTERFACE IMPORTED GLOBAL)
  target_include_directories(Eigen3::Eigen SYSTEM INTERFACE ${eigen_SOURCE_DIR})
endif()
