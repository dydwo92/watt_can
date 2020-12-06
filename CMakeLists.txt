cmake_minimum_required(VERSION 3.0.2)
project(watt_can)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
)

find_package(Threads REQUIRED)

add_message_files(
  FILES
  Gyemsreq.msg
  Wattreq.msg
  Gyemsres.msg
  Wattres.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
  src/CANOpen
  src/GYEMS
)

add_executable(talker
  src/talker.cpp
  src/CANOpen/CANOpen_hw_appl.c
  src/CANOpen/CANOpen.c
  src/GYEMS/gyems.c
)

add_dependencies(talker ${PROJECT_NAME}_generate_messages_cpp)

target_link_libraries(talker ${catkin_LIBRARIES})
target_link_libraries(talker ${CMAKE_THREAD_LIBS_INIT})
install(TARGETS talker
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})