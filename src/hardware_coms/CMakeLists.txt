cmake_minimum_required(VERSION 2.8.3)
project(hardware_coms)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  message_generation
)

catkin_python_setup()



generate_messages(
 DEPENDENCIES
 std_msgs
)


catkin_package(

)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
