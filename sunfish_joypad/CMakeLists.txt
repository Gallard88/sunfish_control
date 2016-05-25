# %Tag(FULLTEXT)%
cmake_minimum_required(VERSION 2.8.3)
project(sunfish_joypad)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs genmsg)
#add_message_files(FILES )
#add_service_files(FILES )
#generate_messages(DEPENDENCIES std_msgs)

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})

catkin_add_gtest(${PROJECT_NAME}_test
    test/utest.cpp
    )
target_link_libraries(${PROJECT_NAME}_test ${catkin_LIBRARIES})

add_executable(joypad src/JoypadMain.cpp)
target_link_libraries(joypad ${catkin_LIBRARIES})
#add_dependencies(joypad sunfish_joypad_generate_service_cpp)

