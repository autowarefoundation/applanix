add_subdirectory(applanix_driver)
add_subdirectory(applanix_driver_ros)
add_subdirectory(util)
add_subdirectory(network)

appl_cc_exec(
  NAME
    lvx_client_node
  SRCS
    lvx_client_node_ros.cpp
  DEPS
    appl::lvx_client_ros
    ${rclcpp_LIBRARIES}
  INC_SYS
    ${rclcpp_INCLUDE_DIRS}
  COPTS
    ${COMMON_COMPILE_FLAGS}
)
