cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

find_package(PCL 1.3 REQUIRED COMPONENTS common io)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
  
add_executable(util utilIO.cpp)
 
target_link_libraries(util ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES})