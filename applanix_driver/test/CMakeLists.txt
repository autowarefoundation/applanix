if (BUILD_TESTING)
  enable_testing()
  find_package(ament_cmake_gtest REQUIRED)
  
  ament_add_gtest(${PROJECT_NAME}_test
      WORKING_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/data
      test_gsof.cpp
      test_byteswap.cpp)

  target_include_directories(${PROJECT_NAME}_test PUBLIC
      test
      ${GMOCK_INCLUDE_DIRS})
  target_compile_options(${PROJECT_NAME}_test PRIVATE
      ${COMMON_COMPILE_FLAGS})
  target_link_libraries(${PROJECT_NAME}_test
      appl::${PROJECT_NAME}
      appl::gsof)
endif (BUILD_TESTING)
