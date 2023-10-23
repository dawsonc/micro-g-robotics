cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)

project(micro_g_gazebo_plugins)

find_package(ignition-cmake2 REQUIRED)

find_package(ignition-plugin1 REQUIRED COMPONENTS register)
set(IGN_PLUGIN_VER ${ignition-plugin1_VERSION_MAJOR})

find_package(ignition-gazebo6 REQUIRED)
set(IGN_SIM_VER ${ignition-gazebo6_VERSION_MAJOR})

find_package(ignition-utils1 REQUIRED)
set(IGN_UTILS_VER ${ignition-utils1_VERSION_MAJOR})

find_package(ignition-common4 REQUIRED COMPONENTS profiler)
set(IGN_COMMON_VER ${ignition-common4_VERSION_MAJOR})

add_library(TargetSpawnerPlugin
  SHARED src/TargetSpawnerPlugin.cc
)
target_include_directories(TargetSpawnerPlugin PRIVATE
  include
)

set_property(TARGET TargetSpawnerPlugin PROPERTY CXX_STANDARD 17)

target_link_libraries(TargetSpawnerPlugin
  PRIVATE ignition-plugin${IGN_PLUGIN_VER}::ignition-plugin${IGN_PLUGIN_VER}
  PRIVATE ignition-gazebo${IGN_SIM_VER}::ignition-gazebo${IGN_SIM_VER}
  PRIVATE ignition-utils${IGN_UTILS_VER}::ignition-utils${IGN_UTILS_VER}
  PRIVATE ignition-common${IGN_COMMON_VER}::ignition-common${IGN_COMMON_VER}
)

install(
  TARGETS TargetSpawnerPlugin
  DESTINATION ${INSTALL_LIB}
)
