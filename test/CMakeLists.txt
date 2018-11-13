enable_testing()

find_package(Threads REQUIRED)
find_package(Glog REQUIRED)
find_package(Protobuf REQUIRED)

include_directories(${GTEST_INCLUDE_DIRS}
  ${CMAKE_SOURCE_DIR}/src/main ${CMAKE_BINARY_DIR}/src/main)

SET(test_src
  test_network
  test_system
  test_rwPursuer
  test_gridWorldPuruser
  test_rwEvader
  test_mcSimEvaderPost
  test_transMatEvaderPost
  test_utilities)

foreach(test_name ${test_src})
  add_executable(${test_name} ${test_name}.cpp)
  target_link_libraries(${test_name} gtest_main coopPE
    ${CMAKE_THREAD_LIBS_INIT} ${GLOG_LIBRARIES} ${PROTOBUF_LIBRARIES})
  add_test(NAME ${test_name} COMMAND ${test_name})
endforeach(test_name)
