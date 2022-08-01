include(CMakeParseArguments)

# appl_cc_lib()
# Inspiration from abseil's absl_cc_library which emulates Bazel's cc_library
# NAME: name of target
# SRCS: List of source files
# DEPS: List of libraries to link to
# COPTS: Compile options
# INC: Public non-system includes
# INC_SYS: System includes
# DEFINES: List of public defines
# INTERFACE: Mark this library as an interface library with no .cpp files to compile
# SHARED: Force this library to be a shared library
function(appl_cc_lib)
  cmake_parse_arguments(APPL_CC_LIB
      "INTERFACE;SHARED"
      "NAME"
      "SRCS;DEPS;COPTS;INC;INC_SYS"
      ${ARGN})
  set(_NAME "appl_${APPL_CC_LIB_NAME}")
  if (${APPL_CC_LIB_INTERFACE})
    add_library(${_NAME} INTERFACE)
    target_sources(${_NAME} INTERFACE ${APPL_CC_LIB_SRCS})
    target_include_directories(${_NAME} INTERFACE ${APPL_CC_LIB_INC})
  elseif(${APPL_CC_LIB_SHARED})
    add_library(${_NAME} SHARED "")
    target_sources(${_NAME} PRIVATE ${APPL_CC_LIB_SRCS})
    target_include_directories(${_NAME} PUBLIC ${APPL_CC_LIB_INC})
  else ()
    add_library(${_NAME} "")
    target_sources(${_NAME} PRIVATE ${APPL_CC_LIB_SRCS})
    target_include_directories(${_NAME} PUBLIC ${APPL_CC_LIB_INC})

    # Enable position independent code
    set_property(TARGET ${_NAME} PROPERTY POSITION_INDEPENDENT_CODE ON)
  endif (${APPL_CC_LIB_INTERFACE})
  target_include_directories(${_NAME} SYSTEM PUBLIC ${APPL_CC_LIB_INC_SYS})
  target_compile_options(${_NAME}
      PRIVATE ${APPL_CC_LIB_COPTS})
  target_link_libraries(${_NAME}
      PUBLIC ${APPL_CC_LIB_DEPS})
  target_compile_definitions(${_NAME} PUBLIC ${APPL_CC_LIB_DEFINES})

  install(TARGETS ${_NAME} 
      EXPORT ${PROJECT_NAME}Targets
      ARCHIVE DESTINATION lib
      LIBRARY DESTINATION lib
      RUNTIME DESTINATION bin)
  ament_export_libraries(${_NAME})

  add_library(appl::${APPL_CC_LIB_NAME} ALIAS ${_NAME})
endfunction()

# appl_cc_exec()
# NAME: Name of target
# SRCS: List of source files
# DEPS: List of libraries to link to
# COPTS: List of private compile options
# INC: Public non-system includes
# INC_SYS: System includes
function(appl_cc_exec)
  cmake_parse_arguments(APPL_CC_EXEC
      ""
      "NAME"
      "SRCS;DEPS;COPTS;INC;INC_SYS"
      ${ARGN})
  set(_NAME "${APPL_CC_EXEC_NAME}")
  add_executable(${_NAME} "")
  target_sources(${_NAME} PRIVATE ${APPL_CC_EXEC_SRCS})
  target_include_directories(${_NAME} PUBLIC ${APPL_CC_EXEC_INC})
  target_include_directories(${_NAME} SYSTEM PRIVATE ${APPL_CC_EXEC_INC_SYS})
  target_compile_options(${_NAME} PRIVATE ${APPL_CC_EXEC_COPTS})
  target_link_libraries(${_NAME}
      PUBLIC ${APPL_CC_EXEC_DEPS})
  
  install(TARGETS ${_NAME}
      EXPORT ${PROJECT_NAME}Targets
      DESTINATION lib/${PROJECT_NAME})

endfunction(appl_cc_exec)

