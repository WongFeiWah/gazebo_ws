cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

find_package(gazebo REQUIRED)
find_package(Boost REQUIRED COMPONENTS system thread)
#find_package(Threads REQUIRED)
find_library(
        NAMES Zmq
        PATHS /usr/local/lib/
)

include_directories(${GAZEBO_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} src/ src/comm/ src/plugin/)
link_directories(${GAZEBO_LIBRARY_DIRS} ${Boost_LIBRARIES})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

file(GLOB COMM_CPP_PATH RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} src/comm/*.cpp)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=c99 -Wall -Wextra -pthread")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Wextra -pthread")

##==========================================================================
##   lib
##==========================================================================
###################################
##   diff driver
###################################
add_library(cleanpack_diff_driver SHARED src/plugin/cleanpack_diff_driver.cc)
target_link_libraries(cleanpack_diff_driver ${GAZEBO_LIBRARIES})

##==========================================================================
##   exec
##==========================================================================
add_executable(cleanpack_ui src/cleanpack_ui/cleanpack_ui.cpp ${COMM_CPP_PATH})
target_link_libraries(cleanpack_ui zmq ${Boost_LIBRARIES})

add_executable(test1 src/test1.cpp ${COMM_CPP_PATH})
target_link_libraries(test1 zmq ${Boost_LIBRARIES})

add_executable(test2 src/test2.cpp ${COMM_CPP_PATH})
target_link_libraries(test2 zmq ${Boost_LIBRARIES})