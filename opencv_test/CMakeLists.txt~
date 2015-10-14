cmake_minimum_required(VERSION 2.8.3)
project(opencv_test)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs 
)

find_package(OpenCV REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES milestone_0_follow_object
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIR}
)


# catkin_add_nosetests(test)
add_executable(display_image src/display_image.cpp)
target_link_libraries(display_image ${catkin_LIBRARIES} ${OpenCV_LIBS})
