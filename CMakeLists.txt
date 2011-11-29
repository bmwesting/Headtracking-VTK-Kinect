project(headtrack)

cmake_minimum_required(VERSION 2.4.0)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-cpp")

# needed for VTK library
if(COMMAND cmake_policy)
    cmake_policy(SET CMP0003 NEW)
endif(COMMAND cmake_policy)

# enable debugging, and show all warnings
set(CMAKE_CXX_FLAGS "-g -Wall")

find_package(VTK REQUIRED)
include(${VTK_USE_FILE})
set(LIBS ${LIBS} QVTK vtkCharts)

# OpenNI
include_directories("/usr/include/ni")
set(LIBS ${LIBS} OpenNI)

set(SRCS
    main.cpp
    SensorDevice.cpp
    SkeletonMath.cpp
    callbacks.cpp
)

add_executable(headtrack ${SRCS})

target_link_libraries(headtrack ${LIBS})
