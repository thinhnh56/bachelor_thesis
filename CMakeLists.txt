cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(semi_auto)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

#add_executable (semi_auto_gicp semi_auto_gicp.cpp )
#target_link_libraries (semi_auto_gicp ${PCL_LIBRARIES})

add_executable (error_estimate error_estimate.cpp )
target_link_libraries (error_estimate ${PCL_LIBRARIES})
