# CMakeList.txt : Top-level CMake project file, do global configuration
# and include sub-projects here.
#
cmake_minimum_required (VERSION 3.8)

project ("CamRec")

# Include sub-projects.
add_subdirectory ("CamRec")

# Copy dlls into remote Trget
#add_custom_command(TARGET CamRec POST_BUILD
#	COMMAND $(CMAKE_COMMAND) -E copy_if_different
#	"${PROJECT_SOURCE_DIR}/libs/dlls"
#	$<TARGET_FILE_DIR:CamRec>)
