cmake_minimum_required(VERSION 3.16)

project(plotjuggler_m2
  VERSION 0.1.0
  LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

option(BUILD_PLUGIN "Build the PlotJuggler plugin shared libraries" ON)
option(BUILD_TESTING "Build smoke tests and mock tools" ON)
option(PJ_M2_BUILD_BUNDLE "Assemble runtime bundle directory after build" ON)

set(PJ_M2_BUNDLE_DIRECTORY
  "${CMAKE_CURRENT_SOURCE_DIR}/bundle/plotjuggler_m2"
  CACHE PATH "Directory where the self-contained plugin bundle is generated")

# -----------------------------------------------------------------------------
# CycloneDDS and CycloneDDS-CXX Resolution
# -----------------------------------------------------------------------------
set(M2_SDK_INSTALL_DIR "${CMAKE_CURRENT_SOURCE_DIR}/third_party/m2_sdk/third_party/install")
if(EXISTS "${M2_SDK_INSTALL_DIR}")
  list(PREPEND CMAKE_PREFIX_PATH "${M2_SDK_INSTALL_DIR}")
endif()

find_package(CycloneDDS REQUIRED)
find_package(CycloneDDS-CXX REQUIRED)

# -----------------------------------------------------------------------------
# Svan M2 IDL Code Generation
# -----------------------------------------------------------------------------
file(GLOB IDL_FILES "${CMAKE_CURRENT_SOURCE_DIR}/third_party/m2_sdk/idl/*.idl")

idlcxx_generate(TARGET svan_m2_idl_gen
  FILES ${IDL_FILES}
  INCLUDES "${CMAKE_CURRENT_SOURCE_DIR}/third_party/m2_sdk/idl"
  WARNINGS "no-implicit-extensibility")
get_target_property(_idl_sources svan_m2_idl_gen INTERFACE_SOURCES)

add_library(svan_m2_idl_lib STATIC ${_idl_sources})
add_dependencies(svan_m2_idl_lib svan_m2_idl_gen)
target_include_directories(svan_m2_idl_lib PUBLIC
  $<TARGET_PROPERTY:svan_m2_idl_gen,INTERFACE_INCLUDE_DIRECTORIES>)
target_link_libraries(svan_m2_idl_lib PUBLIC CycloneDDS-CXX::ddscxx)

# -----------------------------------------------------------------------------
# Core Flattening & Telemetry Enhancement Library
# -----------------------------------------------------------------------------
add_library(m2_message_flatten STATIC
  src/m2_canonical_names.cpp
  src/m2_data_enhancement.cpp)

target_include_directories(m2_message_flatten PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<TARGET_PROPERTY:svan_m2_idl_gen,INTERFACE_INCLUDE_DIRECTORIES>)

target_link_libraries(m2_message_flatten PUBLIC
  svan_m2_idl_lib
  CycloneDDS-CXX::ddscxx)

# -----------------------------------------------------------------------------
# Testing & Mock Publisher Executables
# -----------------------------------------------------------------------------
if(BUILD_TESTING)
  enable_testing()

  add_executable(m2_message_flatten_smoke
    tests/m2_message_flatten_smoke.cpp)
  target_link_libraries(m2_message_flatten_smoke PRIVATE
    m2_message_flatten)
  add_test(NAME m2_message_flatten_smoke COMMAND m2_message_flatten_smoke)

  add_executable(m2_mock_publisher
    tests/m2_mock_publisher.cpp)
  target_link_libraries(m2_mock_publisher PRIVATE
    m2_message_flatten)

  add_executable(m2_dds_loopback_test
    tests/m2_dds_loopback_test.cpp)
  target_link_libraries(m2_dds_loopback_test PRIVATE
    m2_message_flatten)
  add_test(NAME m2_dds_loopback_test COMMAND m2_dds_loopback_test)

  # --- Standalone Bundled MCAP Compilation ---
  find_package(zstd QUIET)
  find_library(ZSTD_LIB NAMES zstd libzstd)
  find_library(LZ4_LIB NAMES lz4 liblz4)

  if(ZSTD_LIB AND LZ4_LIB AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/third_party/mcap/reader.hpp")
    file(WRITE "${CMAKE_CURRENT_BINARY_DIR}/mcap_impl.cpp"
      "#define MCAP_IMPLEMENTATION\n#include <mcap/reader.hpp>\n#include <mcap/writer.hpp>\n")
    add_library(bundled_mcap STATIC "${CMAKE_CURRENT_BINARY_DIR}/mcap_impl.cpp")
    target_include_directories(bundled_mcap PUBLIC
      "${CMAKE_CURRENT_SOURCE_DIR}/third_party")
    target_link_libraries(bundled_mcap PUBLIC ${ZSTD_LIB} ${LZ4_LIB})
    set(MCAP_LIB bundled_mcap)
  else()
    find_library(MCAP_LIB mcap PATHS
      "${CMAKE_CURRENT_SOURCE_DIR}/.deps/plotjuggler/lib")
  endif()

  if(MCAP_LIB)
    add_executable(m2_bag_player
      tools/m2_bag_player.cpp)
    target_link_libraries(m2_bag_player PRIVATE
      m2_message_flatten
      ${MCAP_LIB})
    target_include_directories(m2_bag_player PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}
      ${CMAKE_CURRENT_SOURCE_DIR}/third_party)

    add_executable(m2_bag_player_smoke
      tests/m2_bag_player_smoke.cpp)
    target_link_libraries(m2_bag_player_smoke PRIVATE
      m2_message_flatten
      ${MCAP_LIB})
    target_include_directories(m2_bag_player_smoke PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}
      ${CMAKE_CURRENT_SOURCE_DIR}/third_party)
    add_test(NAME m2_bag_player_smoke COMMAND m2_bag_player_smoke)
  endif()
endif()

# -----------------------------------------------------------------------------
# PlotJuggler Plugins (DataStreamer & 3D Robot View Toolbox)
# -----------------------------------------------------------------------------
if(BUILD_PLUGIN)
  set(CMAKE_AUTOMOC ON)

  # Resolve PlotJuggler SDK & Qt5Svg
  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/.deps/plotjuggler")
    list(PREPEND CMAKE_PREFIX_PATH
      "${CMAKE_CURRENT_SOURCE_DIR}/.deps/plotjuggler"
      "${CMAKE_CURRENT_SOURCE_DIR}/.deps/plotjuggler/usr")
    set(Qt5Svg_DIR "${CMAKE_CURRENT_SOURCE_DIR}/.deps/plotjuggler/usr/lib/x86_64-linux-gnu/cmake/Qt5Svg")
  endif()

  find_package(Qt5 REQUIRED COMPONENTS Core Widgets Xml OpenGL Network Concurrent PrintSupport Svg)
  find_package(plotjuggler REQUIRED)

  # 1. DDS DataStreamer Plugin
  add_library(plotjuggler_m2 SHARED
    src/m2_datastreamer.cpp
    src/m2_datastreamer.h)

  target_link_libraries(plotjuggler_m2 PRIVATE
    m2_message_flatten
    Qt5::Core
    Qt5::Widgets
    Qt5::Xml
    Qt5::Network
    plotjuggler::plotjuggler_base)

  target_include_directories(plotjuggler_m2 PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>)

  # 2. 3D Robot View Toolbox Plugin
  set(M2_ROBOT_ASSETS_DIR
    "${CMAKE_CURRENT_SOURCE_DIR}/third_party/xterra_m2_assets/m2_metal_description")

  add_library(plotjuggler_m2_robot_view SHARED
    src/m2_robot_view_toolbox.cpp
    src/m2_robot_view_toolbox.h)

  target_link_libraries(plotjuggler_m2_robot_view PRIVATE
    Qt5::Core
    Qt5::Widgets
    Qt5::Xml
    Qt5::OpenGL
    plotjuggler::plotjuggler_base
    ${CMAKE_DL_LIBS})

  target_include_directories(plotjuggler_m2_robot_view PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>)

  target_compile_definitions(plotjuggler_m2_robot_view PRIVATE
    PJ_M2_ASSETS_DIR="${M2_ROBOT_ASSETS_DIR}")

  # -----------------------------------------------------------------------------
  # Bundle Assembly
  # -----------------------------------------------------------------------------
  if(PJ_M2_BUILD_BUNDLE)
    add_custom_command(TARGET plotjuggler_m2 POST_BUILD
      COMMAND "${CMAKE_COMMAND}" -E make_directory "${PJ_M2_BUNDLE_DIRECTORY}"
      COMMAND "${CMAKE_COMMAND}" -E copy_if_different
              "$<TARGET_FILE:plotjuggler_m2>"
              "${PJ_M2_BUNDLE_DIRECTORY}/$<TARGET_FILE_NAME:plotjuggler_m2>")

    add_custom_command(TARGET plotjuggler_m2_robot_view POST_BUILD
      COMMAND "${CMAKE_COMMAND}" -E make_directory "${PJ_M2_BUNDLE_DIRECTORY}"
      COMMAND "${CMAKE_COMMAND}" -E copy_if_different
              "$<TARGET_FILE:plotjuggler_m2_robot_view>"
              "${PJ_M2_BUNDLE_DIRECTORY}/$<TARGET_FILE_NAME:plotjuggler_m2_robot_view>")

    add_custom_command(TARGET plotjuggler_m2_robot_view POST_BUILD
      COMMAND "${CMAKE_COMMAND}" -E make_directory "${PJ_M2_BUNDLE_DIRECTORY}/assets"
      COMMAND "${CMAKE_COMMAND}" -E copy_directory
              "${M2_ROBOT_ASSETS_DIR}"
              "${PJ_M2_BUNDLE_DIRECTORY}/assets/m2_metal_description")
  endif()
endif()
