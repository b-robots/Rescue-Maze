# CMakeList.txt : CMake project for CamRec, include source and define
# project specific logic here.
#
cmake_minimum_required (VERSION 3.8)

# Add source to this project's executable.
add_executable (CamRec "CamRec.cpp" "CamRec.h")

# TODO: Add tests and install targets if needed.
add_library(opencv STATIC IMPORTED) # or STATIC instead of SHARED
set_target_properties(opencv PROPERTIES
  INTERFACE_INCLUDE_DIRECTORIES "${PROJECT_SOURCE_DIR}/CamRec/libs/opencv-4.2.0/modules/coe/include"
)

target_link_libraries(CamRec opencv) # also adds the required include path