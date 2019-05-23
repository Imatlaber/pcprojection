cmake_minimum_required(VERSION 2.8)
project(lidar_features)

find_package(Boost REQUIRED COMPONENTS system signals python thread program_options filesystem)

file(GLOB H_FILES *.h)
file(GLOB HPP_FILES *.hpp)
set(ALL_HEADERS ${H_FILES} ${HPP_FILES})

set(lidar_features_include 
	${PCL_INCLUDE_DIRS}
	)

set(lidar_features_lib
    ${PCL_LIBRARIES}
	${Boost_LIBRARIES}
    bot2-core
	hadif
	velodyne_drivers
	)

include_directories(${lidar_features_include})

add_library(lidar_features lidar_feature_base.cpp ${ALL_HEADERS})
target_link_libraries(lidar_features ${lidar_features_lib})

#add_executable(lidar_feature_plane_test lidar_feature_plane_test.cpp)
#target_link_libraries(lidar_feature_plane_test lidar_features trans_pcl_calib)

