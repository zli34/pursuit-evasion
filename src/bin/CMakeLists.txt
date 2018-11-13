find_package(Protobuf REQUIRED)
find_package(GFlags REQUIRED)

include_directories(${CMAKE_CURRENT_BINARY_DIR} ${CMAKE_CURRENT_SOURCE_DIR}
  "${CMAKE_BINARY_DIR}/src/main" "${CMAKE_SOURCE_DIR}/src/main")

SET(bin_file_names
  simple
  run_experiment
  sandbox
  )

foreach(bin_file ${bin_file_names})
  add_executable(${bin_file} ${bin_file}.cpp)
  target_link_libraries(${bin_file} coopPE ${PROTOBUF_LIBRARIES}
    ${GFLAGS_LIBRARIES})
endforeach(bin_file)
