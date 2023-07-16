cmake_minimum_required(VERSION 2.8.3)
project(imu_gps_localizer)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")

find_package(Eigen3  REQUIRED)

# Add GeographicLib.
add_subdirectory(third_party/GeographicLib)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
  third_party/GeographicLib/include
)

add_library(imu_gps_localizer_lib
	src/imu_gps_localizer.cpp
  src/initializer.cpp
  src/imu_processor.cpp
  src/gps_processor.cpp
)

target_link_libraries(imu_gps_localizer_lib
	${EIGEN3_LIBS}
  libGeographiccc
)
# Node for GPS path
find_package(catkin  REQUIRED COMPONENTS
  roscpp
)
include_directories(include ${catkin_INCLUDE_DIRS} )
add_executable(gps_to_rviz src/gps_to_rviz.cpp)
target_link_libraries(gps_to_rviz  ${catkin_LIBRARIES} )
