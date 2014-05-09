cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(TempChangeDetect)

find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )
find_package(PCL 1.3 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(TempChangeDetect masterThesis.cpp util/utilIO.cpp util/meshProcess.cpp
 /usr/include/wrap/ply/plylib.cpp util/pbaUtil.cpp chngDet/chngDet.cpp)
 
target_link_libraries(TempChangeDetect ${PCL_LIBRARIES} ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES})

target_link_libraries(TempChangeDetect ${OpenCV_LIBS})