cmake_minimum_required(VERSION 2.8.3)
project(roboteq_diff_driver)

find_package(catkin REQUIRED COMPONENTS roscpp roboteq_diff_msgs std_msgs nav_msgs sensor_msgs topic_tools tf serial)
find_package(Boost REQUIRED COMPONENTS system thread)

catkin_package(
#  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp roboteq_diff_msgs std_msgs nav_msgs sensor_msgs topic_tools tf serial
)

include_directories(
  include
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_driver src/driver.cpp)
target_link_libraries(${PROJECT_NAME}_driver ${catkin_LIBRARIES})
set_target_properties(${PROJECT_NAME}_driver PROPERTIES OUTPUT_NAME driver PREFIX "")
#add_dependencies(${PROJECT_NAME}_driver roboteq_diff_msgs_gencpp)

install(
  TARGETS ${PROJECT_NAME}_driver
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
