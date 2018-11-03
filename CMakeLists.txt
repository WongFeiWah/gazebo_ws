cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

find_package(gazebo REQUIRED)
find_package(Boost REQUIRED COMPONENTS system thread)
find_library(
        NAMES Zmq
        PATHS /usr/local/lib
)

set(ZMQ_INC_DIR /usr/local/include/)
set(ZMQ_LINK_DIR /usr/local/lib)

include_directories(
        ${GAZEBO_INCLUDE_DIRS}
        ${Boost_INCLUDE_DIRS}
        ${ZMQ_INC_DIR}
        src
        src/comm
        src/plugin
        src/cleanpack_mcu
)

link_directories(
        ${GAZEBO_LIBRARY_DIRS}
        ${Boost_LIBRARIES}
        ${ZMQ_LINK_DIR}
)
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

file(GLOB COMM_CPP_PATH RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} src/comm/*.cpp)
file(GLOB MCU_CPP_PATH RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} src/cleanpack_mcu/*.cpp)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=c99 -Wall -Wextra -pthread -lzmq -g")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Wextra -pthread -lzmq -g")

##==========================================================================
##   lib
##==========================================================================
add_library(cleanpack_diff_driver SHARED src/plugin/cleanpack_diff_driver.cc ${COMM_CPP_PATH})
target_link_libraries(cleanpack_diff_driver ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES} zmq)

add_library(cleanpack_lidar SHARED src/plugin/cleanpack_lidar.cc ${COMM_CPP_PATH})
target_link_libraries(cleanpack_lidar ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES} zmq)

add_library(cleanpack_collider SHARED src/plugin/cleanpack_collider.cc ${COMM_CPP_PATH})
target_link_libraries(cleanpack_collider ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES} zmq)

add_library(cleanpack_alongwall SHARED src/plugin/cleanpack_alongwall.cc ${COMM_CPP_PATH})
target_link_libraries(cleanpack_alongwall ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES} zmq)

add_library(cleanpack_imu SHARED src/plugin/cleanpack_imu.cc ${COMM_CPP_PATH})
target_link_libraries(cleanpack_imu ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES} zmq)

##==========================================================================
##   exec
##==========================================================================
add_executable(cleanpack_ui src/cleanpack_ui/cleanpack_ui.cpp ${COMM_CPP_PATH})
target_link_libraries(cleanpack_ui ${Boost_LIBRARIES} zmq)

add_executable(cleanpack_mcu ${MCU_CPP_PATH} ${COMM_CPP_PATH})
target_link_libraries(cleanpack_mcu ${Boost_LIBRARIES} zmq)

add_executable(test1 src/test1.cpp ${COMM_CPP_PATH})
target_link_libraries(test1 ${Boost_LIBRARIES} zmq)

add_executable(test2 src/test2.cpp ${COMM_CPP_PATH})
target_link_libraries(test2 ${Boost_LIBRARIES} zmq)