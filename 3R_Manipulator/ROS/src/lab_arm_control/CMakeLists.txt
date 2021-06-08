cmake_minimum_required(VERSION 2.8.3)
project(lab_arm_control)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  custom_msg
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_package(
  CATKIN_DEPENDS
)



add_executable(Publica_Angulos src/Publica_Angulos.cpp)
target_link_libraries(Publica_Angulos ${catkin_LIBRARIES})
add_dependencies(Publica_Angulos ${catkin_EXPORTED_TARGETS})

add_executable(IK src/IK.cpp)
target_link_libraries(IK ${catkin_LIBRARIES})
add_dependencies(IK ${catkin_EXPORTED_TARGETS})
