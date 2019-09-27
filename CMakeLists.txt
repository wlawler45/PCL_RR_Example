    
cmake_minimum_required(VERSION 3.3)
project(PCLRR)

set (CMAKE_CXX_STANDARD 11)
set( CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}" ${CMAKE_MODULE_PATH} )



find_package(RobotRaconteur REQUIRED)

find_package(PCL REQUIRED)
find_package(Flann)
find_package(VTK)

include_directories(${RobotRaconteur_INCLUDE_DIR} ${CMAKE_CURRENT_BINARY_DIR})
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

ROBOTRACONTEUR_GENERATE_THUNK(RR_THUNK_SRCS RR_THUNK_HDRS 
	kinect2_interface.robdef
	MASTER_HEADER
	IMPORT_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/../robdef
	)

add_executable(PCLRR
	${RR_THUNK_HDRS}
	${RR_THUNK_SRCS}
	Template.cpp
	Template.h
	PCLRR.cpp
	PCLRR.h)



target_link_libraries(PCLRR RobotRaconteurCore ${Boost_LIBRARIES} ${RobotRaconteurCore_Extra_libraries} )

target_link_libraries(PCLRR ${PCL_LIBRARIES})