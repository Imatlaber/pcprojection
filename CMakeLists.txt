cmake_minimum_required(VERSION 3.5)
project(pcprojection)

if(CMAKE_COMPILER_IS_GNUCXX) 
	set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}") 
	message(STATUS "optional:-std=c++11") 
endif(CMAKE_COMPILER_IS_GNUCXX)

SET(CMAKE_BUILD_TYPE Debug)
#SET(CMAKE_BUILD_TYPE Release)

file(GLOB H_FILES *.h)
file(GLOB HPP_FILES *.hpp)
set(ALL_HEADERS ${H_FILES} ${HPP_FILES})

file(GLOB CPP_FILES *.cpp)
set(ALL_CPP ${CPP_FILES})


# lcm
find_package(LCM)
#set(LCM_INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/externals/lcm/include)
#set(LCM_LIBRARIES ${PROJECT_SOURCE_DIR}/externals/lcm/lib)
#link_directories(${LCM_LIBRARIES})
#include_directories(${LCM_INCLUDE_DIRS})
message("${LCM_INCLUDE_DIRS}")
# pod
include(${PROJECT_SOURCE_DIR}/cmake/pods.cmake)

# build lcm types
include(${PROJECT_SOURCE_DIR}/cmake/lcmtypes.cmake)

# OpenCV
find_package(OpenCV 3 REQUIRED PATHS ${PROJECT_SOURCE_DIR}/externals/opencv)
message("opencv headers ${OpenCV_INCLUDE_DIRS}")

## boost
#find_package(Boost REQUIRED COMPONENTS system signals python thread program_options filesystem)
#
## Eigen
#include_directories("${PROJECT_SOURCE_DIR}/externals/Eigen")

#ceres
find_package(Ceres REQUIRED)

#pcl
find_package(PCL 1.8 REQUIRED PATHS ${PROJECT_SOURCE_DIR}/externals/pcl)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# opengl
set(OPENGL_LIBRARIES glut GL GLU)

# gtk
include(${PROJECT_SOURCE_DIR}/cmake/gtk.cmake)

# libbot
set(LIBBOT_INCLUDE_DIRS "${PROJECT_SOURCE_DIR}/externals/libbot/include")
set(LIBBOT_LIBRARIES libbot2-core.so
        libbot2-frames-renderers.so
        libbot2-frames.so
        libbot2-lcmgl-client.so
        libbot2-lcmgl-renderer.so
        libbot2-param-client.so
        libbot2-vis.so)
link_directories("${PROJECT_SOURCE_DIR}/externals/libbot/lib")
include_directories(${LIBBOT_INCLUDE_DIRS})

#liblas
set(libLAS_INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/externals/liblas/include)
set(libLAS_LIBRARIES ${PROJECT_SOURCE_DIR}/externals/liblas/lib/liblas.so)

# omp
FIND_PACKAGE(OpenMP REQUIRED)
if(OPENMP_FOUND)
    message("OPENMP FOUND")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

# sub moudles
add_subdirectory(common)
add_subdirectory(utilities)
add_subdirectory(lidar_features)

set(calibration_include
        ${PCL_INCLUDE_DIRS}
        ${libLAS_INCLUDE_DIRS}
        ${CERES_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
        )

set(calibration_lib
        ${PCL_LIBRARIES}
        ${Boost_LIBRARIES}
        ${libLAS_LIBRARIES}
        ${CERES_LIBRARIES}
        ${OpenCV_LIBRARIES}
        bot2-core
        hadif
        velodyne_drivers
        pthread
        )

include_directories(${calibration_include})


add_executable(pcprojection main.cpp ${ALL_HEADERS})
target_link_libraries(pcprojection  ${calibration_lib})
