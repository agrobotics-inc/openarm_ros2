# Copyright 2025 Enactic, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

cmake_minimum_required(VERSION 3.22)
project(openarm_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

# Find openarm_can library
find_package(OpenArmCAN REQUIRED)

# KDL for gravity compensation
find_package(orocos_kdl REQUIRED)
find_package(kdl_parser REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/v10_simple_hardware.cpp
  src/v10_grav_comp_hardware.cpp
)

target_include_directories(
  ${PROJECT_NAME}
  PRIVATE
  include
  ${orocos_kdl_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME}
  OpenArmCAN::openarm_can
  ${orocos_kdl_LIBRARIES}
)

ament_target_dependencies(${PROJECT_NAME}
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  orocos_kdl
  kdl_parser
)

pluginlib_export_plugin_description_file(hardware_interface openarm_hardware.xml)

install(
  TARGETS ${PROJECT_NAME}
  DESTINATION lib
  )

install(DIRECTORY include/
DESTINATION include
)

ament_export_include_directories(
  include
)

ament_export_libraries(
  ${PROJECT_NAME}
)
ament_export_dependencies(
  hardware_interface
  openarm_can
  pluginlib
  rclcpp
  orocos_kdl
  kdl_parser
)

ament_package()
