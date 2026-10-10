cmake_minimum_required(VERSION 3.21)
project(ThreeDGAndroid VERSION 0.1 LANGUAGES CXX)

if(NOT ANDROID)
  message(FATAL_ERROR "ThreeDGAndroid must be configured with the Qt Android toolchain")
endif()

find_package(Qt6 6.12 REQUIRED COMPONENTS Widgets Network WebSockets)
qt_policy(SET QTP0002 NEW)
qt_standard_project_setup()

# The generated mission.pb.cc is shared with the desktop client, but Android
# needs its own protobuf runtime. The host protoc must never be built for or
# executed on the Android target.
set(THREEDG_PROTOBUF_SOURCE_DIR
    "${CMAKE_CURRENT_SOURCE_DIR}/../.deps/protobuf-3.21.12"
    CACHE PATH "Source directory of protobuf 3.21.12")
if(NOT EXISTS "${THREEDG_PROTOBUF_SOURCE_DIR}/CMakeLists.txt")
  message(FATAL_ERROR "protobuf 3.21.12 source not found: ${THREEDG_PROTOBUF_SOURCE_DIR}")
endif()
find_program(THREEDG_HOST_PROTOC protoc REQUIRED NO_CMAKE_FIND_ROOT_PATH)
execute_process(COMMAND "${THREEDG_HOST_PROTOC}" --version
                OUTPUT_VARIABLE _protoc_version OUTPUT_STRIP_TRAILING_WHITESPACE)
if(NOT _protoc_version STREQUAL "libprotoc 3.21.12")
  message(FATAL_ERROR "3DG requires host protoc 3.21.12; found ${_protoc_version}")
endif()

set(protobuf_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(protobuf_BUILD_PROTOC_BINARIES OFF CACHE BOOL "" FORCE)
set(protobuf_BUILD_LIBPROTOC OFF CACHE BOOL "" FORCE)
set(protobuf_BUILD_SHARED_LIBS OFF CACHE BOOL "" FORCE)
set(protobuf_WITH_ZLIB OFF CACHE BOOL "" FORCE)
set(protobuf_INSTALL OFF CACHE BOOL "" FORCE)
add_subdirectory("${THREEDG_PROTOBUF_SOURCE_DIR}" "${CMAKE_CURRENT_BINARY_DIR}/protobuf" EXCLUDE_FROM_ALL)
set_target_properties(libprotobuf PROPERTIES AUTOMOC OFF)

set(_mission_dir "${CMAKE_CURRENT_SOURCE_DIR}/../qCC/mission3dg")
set(_mission_proto "${_mission_dir}/protocol/mission.proto")
add_custom_command(
  OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/mission.pb.cc" "${CMAKE_CURRENT_BINARY_DIR}/mission.pb.h"
  COMMAND "${THREEDG_HOST_PROTOC}" "--cpp_out=${CMAKE_CURRENT_BINARY_DIR}"
          -I "${_mission_dir}/protocol" "${_mission_proto}"
  DEPENDS "${_mission_proto}"
  VERBATIM)

qt_add_executable(ThreeDGAndroid MANUAL_FINALIZATION
  main.cpp
  "${_mission_dir}/ProtocolClient.cpp"
  "${_mission_dir}/ProtocolClient.h"
  "${_mission_dir}/CloudIO.cpp"
  "${_mission_dir}/CloudIO.h"
  "${CMAKE_CURRENT_BINARY_DIR}/mission.pb.cc"
  "${CMAKE_CURRENT_BINARY_DIR}/mission.pb.h")
target_include_directories(ThreeDGAndroid PRIVATE
  "${_mission_dir}" "${CMAKE_CURRENT_BINARY_DIR}")
target_link_libraries(ThreeDGAndroid PRIVATE
  Qt6::Widgets Qt6::Network Qt6::WebSockets protobuf::libprotobuf)
set_target_properties(ThreeDGAndroid PROPERTIES
  CXX_STANDARD 17 CXX_STANDARD_REQUIRED ON
  QT_ANDROID_PACKAGE_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/android-package")
qt_finalize_executable(ThreeDGAndroid)
