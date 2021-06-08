cmake_minimum_required(VERSION 2.8.3)
project(custom_msg)

find_package(catkin REQUIRED COMPONENTS
	message_generation
	std_msgs
)

add_message_files(
  FILES
  set_angles.msg
  status_arm.msg
  reset_COT.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime
)