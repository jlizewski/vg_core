include(FetchContent)

# ONNX Runtime, for vg_core_onnx: running segmentation models on the desktop
# (Windows, Linux, macOS). Prebuilt releases, pinned by checksum; phones use
# their own runtime (see include/vg_core/segmentation.hpp).
set(VG_ONNXRUNTIME_VERSION 1.22.0)
if(WIN32)
  set(_ort_package onnxruntime-win-x64-${VG_ONNXRUNTIME_VERSION}.zip)
  set(_ort_hash 174c616efc0271194488642a72f1a514e01487da4dfe84c49296d66e40ebe0da)
elseif(APPLE)
  set(_ort_package onnxruntime-osx-universal2-${VG_ONNXRUNTIME_VERSION}.tgz)
  set(_ort_hash "")  # Not yet checked on a Mac.
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64|AMD64")
  set(_ort_package onnxruntime-linux-x64-${VG_ONNXRUNTIME_VERSION}.tgz)
  set(_ort_hash 8344d55f93d5bc5021ce342db50f62079daf39aaafb5d311a451846228be49b3)
else()
  message(FATAL_ERROR "VG_CORE_BUILD_ONNX: no prebuilt ONNX Runtime for this platform; "
                      "turn it off (-DVG_CORE_BUILD_ONNX=OFF)")
endif()

set(_ort_url
    https://github.com/microsoft/onnxruntime/releases/download/v${VG_ONNXRUNTIME_VERSION}/${_ort_package})
if(_ort_hash)
  FetchContent_Declare(onnxruntime URL ${_ort_url} URL_HASH SHA256=${_ort_hash}
                       SOURCE_SUBDIR do-not-add DOWNLOAD_EXTRACT_TIMESTAMP TRUE)
else()
  FetchContent_Declare(onnxruntime URL ${_ort_url} SOURCE_SUBDIR do-not-add
                       DOWNLOAD_EXTRACT_TIMESTAMP TRUE)
endif()
FetchContent_MakeAvailable(onnxruntime)

add_library(onnxruntime::onnxruntime SHARED IMPORTED GLOBAL)
set_target_properties(onnxruntime::onnxruntime PROPERTIES
  INTERFACE_INCLUDE_DIRECTORIES ${onnxruntime_SOURCE_DIR}/include)
if(WIN32)
  set_target_properties(onnxruntime::onnxruntime PROPERTIES
    IMPORTED_LOCATION ${onnxruntime_SOURCE_DIR}/lib/onnxruntime.dll
    IMPORTED_IMPLIB ${onnxruntime_SOURCE_DIR}/lib/onnxruntime.lib)
elseif(APPLE)
  set_target_properties(onnxruntime::onnxruntime PROPERTIES
    IMPORTED_LOCATION ${onnxruntime_SOURCE_DIR}/lib/libonnxruntime.dylib)
else()
  set_target_properties(onnxruntime::onnxruntime PROPERTIES
    IMPORTED_LOCATION ${onnxruntime_SOURCE_DIR}/lib/libonnxruntime.so)
endif()

# Windows finds DLLs beside the executable: copy ONNX Runtime's there.
function(vg_core_copy_onnxruntime target)
  if(WIN32)
    add_custom_command(TARGET ${target} POST_BUILD
      COMMAND ${CMAKE_COMMAND} -E copy_if_different
        ${onnxruntime_SOURCE_DIR}/lib/onnxruntime.dll
        ${onnxruntime_SOURCE_DIR}/lib/onnxruntime_providers_shared.dll
        $<TARGET_FILE_DIR:${target}>)
  endif()
endfunction()
