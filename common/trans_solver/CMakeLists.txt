cmake_minimum_required(VERSION 2.8)
project(trans_solver)

file(GLOB H_FILES *.h)
file(GLOB HPP_FILES *.hpp)
set(${PROJECT_NAME}_header_files ${H_FILES} ${HPP_FILES})

install(FILES ${${PROJECT_NAME}_header_files} DESTINATION include/common/trans_solver)

#add_subdirectory(trans_g2o_calib)
#add_subdirectory(trans_g2o_icp)
add_subdirectory(trans_pcl_icp)
#add_subdirectory(trans_pcl_calib)

