cmake_minimum_required(VERSION 2.8)
project( cali )
find_package( OpenCV REQUIRED )
add_executable( cali camera_calibration.cpp )
target_link_libraries( cali ${OpenCV_LIBS} )

