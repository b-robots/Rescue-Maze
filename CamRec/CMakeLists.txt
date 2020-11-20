# CMakeList.txt : CMake project for CamRec, include source and define
# project specific logic here.
#
cmake_minimum_required (VERSION 3.8)

find_package(OpenCV REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})
# Add source to this project's executable.
add_executable (CamRec "CamRec.cpp" "CamRec.h")

target_link_libraries(CamRec ${OpenCV_LIBS})