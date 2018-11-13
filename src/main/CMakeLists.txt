find_package(Protobuf REQUIRED)
find_package(Armadillo REQUIRED)
find_package(Glog REQUIRED)

include_directories(${ARMADILLO_INCLUDE_DIRS} ${Protobuf_INCLUDE_DIRS}
  ${CMAKE_CURRENT_BINARY_DIR} ${CMAKE_CURRENT_SOURCE_DIR} njm_cpp)

protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS coopPE.proto)
protobuf_generate_python(PROTO_PY coopPE.proto)

add_library(coopPE
  ## infrastructure
  network.cpp system.cpp

  ## pursuers
  pursuer.cpp rwPursuer.cpp gridWorldPursuer.cpp postPeakPursuer.cpp
  bruteForcePursuer.cpp futurePeakPursuer.cpp vfnMaxPursuer.cpp
  featMaxPursuer.cpp qfnRolloutPursuer.cpp

  ## evaders
  evader.cpp rwEvader.cpp

  ## posterior distributions for evaders location
  evaderPost.cpp mcSimEvaderPost.cpp transMatEvaderPost.cpp

  ## informants
  informant.cpp quadrantInformant.cpp
  informantTimer.cpp ewtInformantTimer.cpp

  ## features
  features.cpp futurePeakFeatures.cpp postLocFeatures.cpp

  ## extra
  runner.cpp
  utilities.cpp
  historian.cpp
  ${PROTO_SRCS}
  ${PROTO_PY})

target_link_libraries(coopPE njm_cpp ${ARMADILLO_LIBRARIES} ${Glog_LIBRARIES})
