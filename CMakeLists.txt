cmake_minimum_required(VERSION 3.0.2)
project(dronedays2021)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(uav src/exercises.cpp src/uav.cpp src/robot.cpp)
add_dependencies(uav ${catkin_EXPORTED_TARGETS})
target_link_libraries(uav ${catkin_LIBRARIES})

add_executable(ugv src/exercises.cpp src/ugv.cpp src/robot.cpp)
add_dependencies(ugv ${catkin_EXPORTED_TARGETS})
target_link_libraries(ugv ${catkin_LIBRARIES})

add_executable(human src/human.cpp src/robot.cpp)
add_dependencies(human ${catkin_EXPORTED_TARGETS})
target_link_libraries(human ${catkin_LIBRARIES})
