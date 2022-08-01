cmake_minimum_required(VERSION 3.10.0)
project(applanix_driver)

set(ROS_VERSION $ENV{ROS_VERSION})
set(CMAKE_CXX_STANDARD 17)

include(cmake/macros.cmake)
list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake")
find_package(PCAP REQUIRED)
find_package(GeographicLib REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)
find_package(Threads REQUIRED)

set(COMMON_COMPILE_FLAGS
    -Wall
    -Wextra
    -Wcast-qual
    -Wdouble-promotion
    -Werror
    -Wlogical-op
    -Wno-error=attributes
    -Wold-style-cast
    -Woverlength-strings
    -Wpedantic
    -Wpointer-arith
    -Wshadow
    -Wswitch-enum
    -Wunused-local-typedefs
    -Wunused-result
    -Wuseless-cast
    -Wvarargs
    -Wvla
    -Wwrite-strings)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(applanix_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(autoware_sensing_msgs REQUIRED)

add_subdirectory(src)
add_subdirectory(test)


install(
    DIRECTORY config
    DESTINATION share/${PROJECT_NAME})
install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME})


# These macros only work in the same scope as ament_package() due to a
# variable not being set in the cache
rclcpp_components_register_nodes(
  appl_lvx_client_ros "applanix_driver_ros::LvxClientRos")
ament_package()
