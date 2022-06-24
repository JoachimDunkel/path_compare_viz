cmake_minimum_required(VERSION 3.0.2)
project(path_compare_viz)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(SINGLE_PATH_VISUALIZER single_path_visualizer)

add_definitions(-Wno-deprecated-declarations -Wno-comment -Wno-deprecated -Wall -Werror )




find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  std_msgs
  tf2
  tf2_ros
)


catkin_package(
  INCLUDE_DIRS 
   include
  LIBRARIES 
  ${PROJECT_NAME}_core
  CATKIN_DEPENDS
   geometry_msgs 
   nav_msgs
   roscpp
   rospy std_msgs
  DEPENDS
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


add_library(${PROJECT_NAME}_core
  src/PathVisualizer.cpp
)

target_compile_features(${PROJECT_NAME}_core INTERFACE cxx_std_14)

target_link_libraries(${PROJECT_NAME}_core
  ${catkin_LIBRARIES}
)

#PATH COMPARE
add_executable(${PROJECT_NAME}
  src/${PROJECT_NAME}_node.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${PROJECT_NAME}_core
  ${catkin_LIBRARIES}
)

add_dependencies(${PROJECT_NAME}
  ${catkin_EXPORTED_TARGETS}
)

#SINGLE PATH VISUALIZATION
add_executable(${SINGLE_PATH_VISUALIZER}
	src/${SINGLE_PATH_VISUALIZER}_node.cpp
)

target_link_libraries(${SINGLE_PATH_VISUALIZER}
  ${PROJECT_NAME}_core
  ${catkin_LIBRARIES}
)

add_dependencies(${SINGLE_PATH_VISUALIZER}
  ${catkin_EXPORTED_TARGETS}
)


## Mark libraries for installation
install(TARGETS ${SINGLE_PATH_VISUALIZER} ${PROJECT_NAME} ${PROJECT_NAME}_core
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hpp"
)

