add_subdirectory(gsof)

appl_cc_lib(
    NAME
      ${PROJECT_NAME}
    SRCS
      lvx_client.cpp
    INC
      include
    COPTS
      ${COMMON_COMPILE_FLAGS}
    DEPS
      appl::gsof
      appl::network
      appl::util
      Threads::Threads
)

install(
  DIRECTORY ${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME})
