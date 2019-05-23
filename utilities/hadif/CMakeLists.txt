cmake_minimum_required(VERSION 2.8)
project(hadif)

file(GLOB H_FILES *.h)
file(GLOB HPP_FILES *.hpp)
set(ALL_HEADERS ${H_FILES} ${HPP_FILES})

file(GLOB_RECURSE C_FILES *.c) 
file(GLOB_RECURSE CC_FILES *.cc) 
file(GLOB_RECURSE CPP_FILES *.cpp)
set(ALL_SOURCES ${C_FILES} ${CC_FILES} ${CPP_FILES})

set(LIBS
	${LCM_LIBRARIES}
	${LIBBOT_LIBRARIES}
	${PCL_LIBRARIES}
	${OpenCV_LIBRARIES}
	glib-2.0
    proj
    )

include_directories(${PCL_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})

add_library(hadif ${ALL_SOURCES} ${ALL_HEADERS})

target_link_libraries(hadif ${LIBS})

set_target_properties(hadif PROPERTIES
	PUBLIC_HEADER "${ALL_HEADERS}"
)

install(TARGETS hadif
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  PUBLIC_HEADER DESTINATION include/utilities/hadif
)
