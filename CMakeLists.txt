cmake_minimum_required(VERSION 3.22)

project(Railway_Network_CCBS)


set(CMAKE_CXX_STANDARD 14)

set(BOOST_ROOT "C:/Boost/boost_1_83_0")
set(Boost_USE_STATIC_LIBS OFF)


find_package(Boost REQUIRED)
include_directories( ${Boost_INCLUDE_DIRS} )
include_directories( "." )
add_executable(Railway_Network_CCBS main.cpp map.cpp heuristic.cpp heuristic.h task.cpp task.h CBS.cpp CBS.h sipp.cpp sipp.h const.h)


if(Boost_FOUND)
    target_include_directories(${PROJECT_NAME} PRIVATE ${Boost_INCLUDE_DIRS})
    target_link_libraries(${PROJECT_NAME} PRIVATE ${Boost_LIBRARIES})
endif()


add_library(tinyxml2 tinyxml2.cpp)

target_link_libraries(Railway_Network_CCBS tinyxml2)


set(CMAKE_CXX_FLAGS_DEBUG "-g -O0")

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wuninitialized")

