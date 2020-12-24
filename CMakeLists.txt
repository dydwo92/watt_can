cmake_minimum_required(VERSION 3.0.2)
project(watt_can)

find_package(catkin REQUIRED COMPONENTS
  controller_manager
  hardware_interface
  roscpp
  std_msgs
  message_generation
)

find_package(Threads REQUIRED)

add_message_files(
  FILES
  Gyemsreq.msg
  Gyemsres.msg
  Zlactestreq.msg
  Zlactestres.msg
  Innfosreq.msg
  Wattreq.msg
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
  include
  src/CANOpen
  src/GYEMS
)

## taker node 
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


## zlac_tester node
add_executable(zlac_tester
  src/zlac_tester.cpp
  src/CANOpen/CANOpen_hw_appl.c
  src/CANOpen/CANOpen.c
)
add_dependencies(zlac_tester ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(zlac_tester ${catkin_LIBRARIES})
target_link_libraries(zlac_tester ${CMAKE_THREAD_LIBS_INIT})
install(TARGETS zlac_tester
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
  
## innfos_tester node
add_executable(innfos_tester
  src/innfos_tester.cpp
)
add_dependencies(innfos_tester ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(innfos_tester ${catkin_LIBRARIES})
target_link_libraries(innfos_tester ${CMAKE_THREAD_LIBS_INIT})
install(TARGETS innfos_tester
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
  
## zlac8015_canopen node
add_executable(zlac8015_driver
  src/k3_mobile_hw_node.cpp
  src/motor_driver_zlac8015_canopen
  src/CANOpen/CANOpen_hw_appl.c
  src/CANOpen/CANOpen.c  
)
target_link_libraries(zlac8015_driver ${catkin_LIBRARIES})
target_link_libraries(zlac8015_driver ${CMAKE_THREAD_LIBS_INIT})