# %Tag(FULLTEXT)%
cmake_minimum_required(VERSION 2.8.3)
project(sunfish_ecu)

find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs genmsg)
add_message_files(FILES
                  Depth.msg
                  INS.msg
                  MagField.msg
                  Power.msg
                  Status.msg )

add_service_files(FILES setPWM.srv)
generate_messages(DEPENDENCIES std_msgs)

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})

catkin_add_gtest(${PROJECT_NAME}_test
    test/utest.cpp
    src/ComsPacket.cpp
    src/VectorMap.cpp
    src/outputChannel.cpp
    src/SensorHandler.cpp
    test/test_VectorMap.cpp
    test/test_outputChannel.cpp
    test/test_SensorHandler.cpp
    )
target_link_libraries(${PROJECT_NAME}_test ${catkin_LIBRARIES})

# Dive Controller.
add_executable(ecu
    src/ComsPacket.cpp
    src/HardwareComs.cpp
    src/VectorMap.cpp
    src/outputChannel.cpp
    src/SensorHandler.cpp
    src/ecu_driver.cpp )
target_link_libraries(ecu ${catkin_LIBRARIES})
add_dependencies(ecu sunfish_ecu_generate_messages_cpp)

# %EndTag(FULLTEXT)%
