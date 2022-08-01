find_package(catkin REQUIRED COMPONENTS
  std_msgs
  message_generation
)

add_service_files(
  FILES ${SERVICE_FILES}
)

add_message_files(FILES ${MESSAGE_FILES})

generate_messages(
  DEPENDENCIES
  std_msgs 
)

catkin_package(
 INCLUDE_DIRS include
# LIBRARIES applanix_msgs # Don't include this line because it is an interface library
 CATKIN_DEPENDS message_generation
#  DEPENDS system_lib
)

add_library(${PROJECT_NAME} INTERFACE)
target_include_directories(${PROJECT_NAME} INTERFACE include)

add_dependencies(${PROJECT_NAME} 
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS})

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)