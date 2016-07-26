cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(NiViewer_Simple)

find_package(openni)

include_directories(${OPENNI_INCLUDE_DIRS})
link_directories(${OPENNI_LIBRARY_DIRS})
add_definitions(${OPENNI_DEFINITIONS})

add_executable (ni NiSimpleViewer.cpp)
target_link_libraries (ni ${OPENNI_LIBRARIES})

