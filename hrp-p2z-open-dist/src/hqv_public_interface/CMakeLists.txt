cmake_minimum_required(VERSION 3.5)
project(hqv_public_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

#set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE BOTH)
# find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)

# Automatically find all msg/srv/action files in our package
file(GLOB msg_files CONFIGURE_DEPENDS  RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} "${CMAKE_CURRENT_SOURCE_DIR}/msg/*.msg")
file(GLOB srv_files CONFIGURE_DEPENDS  RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} "${CMAKE_CURRENT_SOURCE_DIR}/srv/*.srv")
file(GLOB action_files CONFIGURE_DEPENDS  RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} "${CMAKE_CURRENT_SOURCE_DIR}/action/*.action")

# Generate the specified interfaces
rosidl_generate_interfaces( ${PROJECT_NAME} ${msg_files} ${srv_files} ${action_files} DEPENDENCIES std_msgs geometry_msgs)
ament_export_dependencies(rosidl_default_runtime)

ament_package()
