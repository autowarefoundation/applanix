find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

list_transform_prepend(SERVICE_FILES "srv/")
list_transform_prepend(MESSAGE_FILES "msg/")

rosidl_generate_interfaces(${PROJECT_NAME}
  ${MESSAGE_FILES}
  ${SERVICE_FILES}
  DEPENDENCIES std_msgs
)

add_library(${PROJECT_NAME}_conversions INTERFACE)
target_include_directories(${PROJECT_NAME}_conversions INTERFACE 
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# We want the library usage to be similar between ros1 and ros2. The file
# thus gets stripped of its _ros2 suffix and installed in include/${PROJECT_NAME}
# In the ROS2 examples the destination is include but we want to merge our
# header library with the generated message and service headers therefore
# the destination is "include/${PROJECT_NAME}/"
install(
  FILES ${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}/time_conversions_ros2.h
  RENAME time_conversions.h
  DESTINATION include/${PROJECT_NAME}/
)

install(TARGETS ${PROJECT_NAME}_conversions
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_dependencies(rosidl_default_runtime)

ament_export_include_directories(
  include
)

ament_export_interfaces(
  export_${PROJECT_NAME}
)

ament_package()
