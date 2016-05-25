# %Tag(FULLTEXT)%
cmake_minimum_required(VERSION 2.8.3)
project(sunfish_camera)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs genmsg)

add_message_files(FILES Status.msg)
add_service_files(FILES Record.srv Stream.srv)
generate_messages(DEPENDENCIES std_msgs)

catkin_package(CATKIN_DEPENDS message_runtime)
