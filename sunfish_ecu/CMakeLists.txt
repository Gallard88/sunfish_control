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

## Declare a catkin package
catkin_package()

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})

catkin_add_gtest(${PROJECT_NAME}_test
    test/utest.cpp
    src/VectorMap.cpp
    src/outputChannel.cpp
    test/test_VectorMap.cpp
    test/test_outputChannel.cpp
    )
target_link_libraries(${PROJECT_NAME}_test ${catkin_LIBRARIES})

# Dive Controller.
add_executable(ecu
    src/VectorMap.cpp
    src/outputChannel.cpp
    src/ecu_driver.cpp )
target_link_libraries(ecu ${catkin_LIBRARIES})
add_dependencies(ecu sunfish_ecu_generate_messages_cpp)

#find_package(catkin REQUIRED COMPONENTS roslaunch)
#roslaunch_add_file_check(launch)

# %EndTag(FULLTEXT)%
