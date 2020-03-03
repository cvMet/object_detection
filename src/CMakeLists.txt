cmake_minimum_required(VERSION 2.6 FATAL_ERROR)

project(objectdetection)
#SET(CMAKE_BUILD_TYPE Release)


set(Boost_DEBUG ON)
set(Boost_USE_STATIC_LIBS ON)
set(Boost_USE_STATIC ON)

find_package(PCL 1.7.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
link_directories(${PCL_INCLUDE_LIBS})
add_definitions(${PCL_DEFINITIONS})

file(GLOB SOURCES
    objectdetection.cpp
    ../include/*.h
)

add_executable (objectdetection ${SOURCES})
target_link_libraries (objectdetection ${PCL_LIBRARIES})


