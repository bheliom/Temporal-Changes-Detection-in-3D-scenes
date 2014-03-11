cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(utilCode)
find_package(PCL 1.3 REQUIRED COMPONENTS common io)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
  
file(GLOB utilCode_SRC
    "src/*.h"
    "src/*.cpp"
)
add_executable(utilCode ${utilCode_SRC})
 
target_link_libraries(utilCode ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES})