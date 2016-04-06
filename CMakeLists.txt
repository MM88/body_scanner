cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(body_scanner)


#################################### Locate PCL ############################################

find_package(PCL 1.7 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


set(LIBS ${LIBS} ${PCL_LIBRARIES})


#################################### Locate OpenCV ############################################

find_package(OpenCV REQUIRED)
include_directories( ${OpenCV_INCLUDE_DIRS} )
message(${OpenCV_INCLUDE_DIRS})
set(LIBS ${LIBS} ${OpenCV_LIBS})

#################################### Locate Freenect2 ############################################

#find_package(Freenect2 REQUIRED)
#include_directories(${FREENECT2_INCLUDE_DIRS})
#message(${FREENECT2_INCLUDE_DIRS})
#set(LIBS ${LIBS} ${FREENECT2_LIBS})

add_executable (body_scanner body_scanner.cpp)
target_link_libraries (body_scanner ${LIBS} )