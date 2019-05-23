cmake_minimum_required(VERSION 2.8)
project(velodyne_drivers)
SET(CMAKE_BUILD_TYPE Debug)
find_package(Boost REQUIRED COMPONENTS system signals python thread program_options filesystem)

file(GLOB H_FILES *.h)
file(GLOB HPP_FILES *.hpp)
set(ALL_HEADERS ${H_FILES} ${HPP_FILES})

include_directories(${PCL_INCLUDE_DIRS})

add_library(${PROJECT_NAME}
    velodyne_driver.cpp
	velodyne_udp_receiver.cpp 
	pc_type_translator.cpp)

target_link_libraries(${PROJECT_NAME}
    ${PCL_LIBRARIES}
	${Boost_LIBRARIES}
    pcap 
	snappy
	z
    bot2-core
	hadif
	)

add_executable(velodyne_driver_main velodyne_driver_main.cpp)
target_link_libraries(velodyne_driver_main ${PROJECT_NAME})

add_executable(velodyne_packet_driver velodyne_packet_driver.cpp)
target_link_libraries(velodyne_packet_driver ${PROJECT_NAME})

add_executable(velodyne_packet_pointcloud velodyne_packet_pointcloud.cpp)
target_link_libraries(velodyne_packet_pointcloud ${PROJECT_NAME})

add_executable(velodyne_from_file velodyne_from_file.cpp)
target_link_libraries(velodyne_from_file ${PROJECT_NAME})

set_target_properties(velodyne_driver_main PROPERTIES
	PUBLIC_HEADER "${ALL_HEADERS}"
)

install(TARGETS ${PROJECT_NAME} velodyne_driver_main velodyne_packet_driver velodyne_packet_pointcloud velodyne_from_file
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  PUBLIC_HEADER DESTINATION include/utilities/drivers/velodyne
)
