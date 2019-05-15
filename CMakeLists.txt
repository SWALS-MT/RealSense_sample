# The project name
cmake_minimum_required (VERSION 3.6)
project (RealSenseCapture)
add_executable( RealSenseCapture main.cpp )

######################################################################
# OpenCV
find_package(OpenCV REQUIRED PATHS "D:/ThirdParty_MT/opencv/build")

######################################################################
# PCL
find_package(PCL REQUIRED PATHS "D:/ThirdParty_MT/PCL 1.9.1/cmake")

######################################################################
# Add Common Library

SET( PATH_REALSENSE "D:/ThirdParty_MT/librealsense")

SET( PROJ_INCLUDES  
"${PATH_REALSENSE}/include"
"${PATH_REALSENSE}/examples"
"${PATH_REALSENSE}/third-party/glfw/include"
"${PATH_REALSENSE}/third-party/imgui"
"${PATH_REALSENSE}/third-party/glfw/include"
"${PATH_REALSENSE}/third-party/imgui"

"D:/ThirdParty_MT/eigen"
)

SET( PROJ_LIBRARIES
optimized "${PATH_REALSENSE}/build/Release/realsense2.lib"
debug "${PATH_REALSENSE}/build/Debug/realsense2.lib"
)

######################################################################
# Definitions

ADD_DEFINITIONS(-DUNICODE)
ADD_DEFINITIONS(-D_UNICODE)
ADD_DEFINITIONS(-DNOMINMAX)

######################################################################
# set the includes and libs

include_directories(${OpenCV_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} ${PROJ_INCLUDES})
target_link_libraries(RealSenseCapture ${OpenCV_LIBS} ${PCL_LIBRARIES} ${PROJ_LIBRARIES})