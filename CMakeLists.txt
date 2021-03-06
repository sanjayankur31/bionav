cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
set(ROS_BUILD_TYPE Debug)
#set(ROS_COMPILE_FLAGS "-W -Wall -Wextra -pedantic")
set(ROS_COMPILE_FLAGS "-ansi -g -O2 -m64 -mtune=generic")
rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

find_package(Eigen REQUIRED)
include_directories(${Eigen_INCLUDE_DIRS})

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
rosbuild_add_executable(bionav src/main.cpp src/Bionavigator.cpp src/HDCells.cpp  src/GridCells.cpp src/HD_RotationSynapseSet.cpp  src/HDSynapseSet.cpp  src/VisionCells.cpp src/HD_VisionSynapseSet.cpp src/RotationCellClockwise.cpp src/RotationCellCounterClockwise.cpp src/GridCells_HD_VelocitySynapseSet.cpp src/GridCellsSynapseSet.cpp src/GridCells_VisionSynapseSet.cpp src/VelocityCell.cpp)
#target_link_libraries(example ${PROJECT_NAME})
