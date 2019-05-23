cmake_minimum_required(VERSION 2.8)
project(tools)

find_package(Boost REQUIRED COMPONENTS system signals python thread program_options filesystem)

file(GLOB H_FILES *.h)
file(GLOB HPP_FILES *.hpp)
set(ALL_HEADERS ${H_FILES} ${HPP_FILES})

set(LIBS
	${PCL_LIBRARIES}
	${LIBBOT_LIBRARIES}
	${Boost_LIBRARIES}
	glib-2.0
	hadif
	snappy
    )

include_directories(${PCL_INCLUDE_DIRS})

add_executable(ypr2rpy ypr2rpy.cpp)
target_link_libraries(ypr2rpy ${LIBS})

add_executable(rpy2mat rpy2mat.cpp)
target_link_libraries(rpy2mat ${LIBS})

add_executable(plane_seg plane.cpp)
target_link_libraries(plane_seg ${LIBS})

add_executable(parse_gprmc parse_gprmc.c)

set_target_properties(ypr2rpy PROPERTIES
	PUBLIC_HEADER "${ALL_HEADERS}"
)

install(TARGETS ypr2rpy rpy2mat plane_seg parse_gprmc
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  PUBLIC_HEADER DESTINATION include/utilities/tools
)
