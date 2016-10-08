cmake_minimum_required(VERSION 2.8.3)
project(driver)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -fPIC -O3")

find_package(catkin REQUIRED COMPONENTS nav_msgs roscpp tf  rospy std_msgs)

catkin_package()

include_directories(
  include
  include/moons
  ${catkin_INCLUDE_DIRS}
)

link_directories(
  ${driver_SOURCE_DIR}/libs/
  /lib/x86_64-linux-gnu/
)

###############################################################
# Build library
add_library(moonsdrv
  src/moons/MoonsCmd.cpp 
  src/moons/cansend.cpp 
)
target_link_libraries(moonsdrv ${catkin_LIBRARIES} controlcan)


add_library(chassisdrv
  src/chassis/WheelCtr.cpp 
)

target_link_libraries(chassisdrv ${catkin_LIBRARIES} controlcan moonsdrv)

###############################################################
# Build the executable
add_executable(${PROJECT_NAME} src/main.cc)

# Add dependency
target_link_libraries(${PROJECT_NAME} chassisdrv)

###############################################################
# Install the executable
install(TARGETS #${PROJECT_NAME} moonsdrv chassisdrv
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
