cmake_minimum_required(VERSION 2.8.3)
project(img_bucket)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  std_msgs
  cv_bridge
  image_transport
  roslint
)

catkin_package()
include_directories(${catkin_INCLUDE_DIRS})
find_package(OpenCV REQUIRED)

roslint_cpp()

add_executable(img_bucket src/img_bucket.cpp)

target_link_libraries(img_bucket  ${OpenCV_LIBS} ${catkin_LIBRARIES} )


install(PROGRAMS
   nodes/time_sync.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


foreach(dir config launch test)
	install(DIRECTORY ${dir}/
		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)
