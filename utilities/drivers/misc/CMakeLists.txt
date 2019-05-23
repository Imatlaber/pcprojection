project(misc)

find_package(Boost REQUIRED COMPONENTS system signals python thread program_options filesystem)

file(GLOB H_FILES *.h)
file(GLOB HPP_FILES *.hpp)
set(ALL_HEADERS ${H_FILES} ${HPP_FILES})

set(${PROJECT_NAME}_INCLUDE
    ${libLAS_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    )

set(${PROJECT_NAME}_LIB
    ${PCL_LIBRARIES}
    ${libLAS_LIBRARIES}
	${LCM_LIBRARIES}
	${LIBBOT_LIBRARIES}
	${Boost_LIBRARIES}
    ${OpenCV_LIBRARIES}
	glib-2.0
	hadif
    velodyne_drivers
    turbojpeg
    )

include_directories(${${PROJECT_NAME}_INCLUDE})

add_executable(relative_odometry_from_file relative_odometry_from_file.cpp)
target_link_libraries(relative_odometry_from_file ${${PROJECT_NAME}_LIB})

add_executable(inspva2odometry_bridge inspva2odometry_bridge.cpp)
target_link_libraries(inspva2odometry_bridge ${${PROJECT_NAME}_LIB})

add_executable(pp2odometry_bridge pp2odometry_bridge.cpp)
target_link_libraries(pp2odometry_bridge ${${PROJECT_NAME}_LIB})

add_executable(velodyne_odometry_playback velodyne_odometry_playback.cpp)
target_link_libraries(velodyne_odometry_playback ${${PROJECT_NAME}_LIB})

add_executable(rt3000_to_cpt_coordinate rt3000_to_cpt.cpp)
target_link_libraries(rt3000_to_cpt_coordinate ${${PROJECT_NAME}_LIB})

set_target_properties(relative_odometry_from_file PROPERTIES
	PUBLIC_HEADER "${ALL_HEADERS}"
)

install(TARGETS relative_odometry_from_file inspva2odometry_bridge pp2odometry_bridge velodyne_odometry_playback rt3000_to_cpt_coordinate
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  PUBLIC_HEADER DESTINATION include/utilities/drivers/misc
)
