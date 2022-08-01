appl_cc_lib(
    NAME
      ros_conversions
    SRCS
      conversions.cpp
    DEPS
      appl::applanix_driver
    INC_SYS
      ${tf2_geometry_msgs_INCLUDE_DIRS}
    COPTS
      ${COMMON_COMPILE_FLAGS}
)

ament_target_dependencies(appl_ros_conversions
    PUBLIC
      "applanix_msgs"
      "GeographicLib"
      "geometry_msgs"
      "nav_msgs"
      "rclcpp"
      "sensor_msgs"
      "tf2_geometry_msgs"
      "autoware_sensing_msgs"
)

set(CLIENT_LIBRARIES
    "applanix_msgs"
    "GeographicLib"
    "nav_msgs"
    "rclcpp"
    "rclcpp_components"
    "sensor_msgs"
    "std_srvs"
    "tf2"
    "tf2_geometry_msgs"
    "tf2_ros"
    "autoware_sensing_msgs"
)


appl_cc_lib(
    SHARED
    NAME
      lvx_client_ros
    SRCS
      lvx_client_ros.cpp
    DEPS
      appl::applanix_driver
      appl::ros_conversions
    COPTS
      ${COMMON_COMPILE_FLAGS}
)

ament_target_dependencies(appl_lvx_client_ros PUBLIC ${CLIENT_LIBRARIES})

