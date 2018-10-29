cmake_minimum_required(VERSION 2.8.3)
project(master_controller)

find_package(catkin REQUIRED COMPONENTS
  common_msgs_gl
  rospy
  std_msgs
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
