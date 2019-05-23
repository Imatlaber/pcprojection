cmake_minimum_required(VERSION 2.8)
project(trans_pcl_calib)

find_package(libLAS REQUIRED)

set(${PROJECT_NAME}_include_dirs
	${PCL_INCLUDE_DIRS}
    ${libLAS_INCLUDE_DIRS}
	${OpenCV_LIBRARIES}
	${Boost_INCLUDE_DIRS}
	${G2O_INCLUDE_DIRS}
	${CSPARSE_INCLUDE_DIR}
	)

set(${PROJECT_NAME}_lib
	${PCL_LIBRARIES}
    ${libLAS_LIBRARIES}
	${Boost_LIBRARIES}
	${OpenCV_LIBRARIES}
	${G2O_LIBRARIES}
	${CSPARSE_LIBRARY}
    hadif
	)

file(GLOB H_FILES *.h)
file(GLOB HPP_FILES *.hpp)
set(${PROJECT_NAME}_header_files ${H_FILES} ${HPP_FILES})

file(GLOB C_FILES *.c)
file(GLOB CPP_FILES *.cpp)
set(${PROJECT_NAME}_src_files ${C_FILES} ${CPP_FILES})
list(REMOVE_ITEM ${PROJECT_NAME}_src_files
    ${CMAKE_CURRENT_SOURCE_DIR}/trans_pcl_calib_test.cpp 
    ${CMAKE_CURRENT_SOURCE_DIR}/trans_pcl_calib_test_full.cpp)

include_directories(${${PROJECT_NAME}_include_dirs})
add_library(${PROJECT_NAME} ${${PROJECT_NAME}_src_files} ${${PROJECT_NAME}_header_files})
target_link_libraries(${PROJECT_NAME} ${${PROJECT_NAME}_lib})

add_executable(trans_pcl_calib_test trans_pcl_calib_test.cpp)
target_link_libraries(trans_pcl_calib_test ${PROJECT_NAME})

add_executable(trans_pcl_calib_test_full trans_pcl_calib_test_full.cpp)
target_link_libraries(trans_pcl_calib_test_full ${PROJECT_NAME})

set_target_properties(${PROJECT_NAME} PROPERTIES
	PUBLIC_HEADER "${${PROJECT_NAME}_header_files}"
	)

install(TARGETS ${PROJECT_NAME} trans_pcl_calib_test trans_pcl_calib_test_full
	RUNTIME DESTINATION bin
	LIBRARY DESTINATION lib
	ARCHIVE DESTINATION lib
	PUBLIC_HEADER DESTINATION include/common/trans_solver/trans_pcl_calib
	)
