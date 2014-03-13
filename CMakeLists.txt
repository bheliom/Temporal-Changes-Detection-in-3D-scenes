cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(masterThesis)

#find_package(PCL 1.3 REQUIRED COMPONENTS common io)

#include_directories(${PCL_INCLUDE_DIRS})
#link_directories(${PCL_LIBRARY_DIRS})
#add_definitions(${PCL_DEFINITIONS})

add_executable(masterThesis masterThesis.cpp util/utilIO.cpp util/meshProcess.cpp
 /usr/include/wrap/ply/plylib.cpp)
 
#target_link_libraries(masterThesis ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES})

