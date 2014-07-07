cmake_minimum_required(VERSION 2.8)
project(RobocupUlaval)

set(CMAKE_BUILD_TYPE Release)

ADD_CUSTOM_COMMAND(
    OUTPUT BUILD PROTOC
	COMMAND ${CMAKE_ROOT}/compile.sh
	WORKING_DIRECTORY ${CMAKE_ROOT}
)


#Find boost
set(Boost_USE_STATIC_LIBS ON)
set(Boost_USE_MULTITHREADED ON)
set(Boost_USE_STATIC_RUNTIME OFF)
set(BOOST_ALL_DYN_LINK ON)

if (MINGW)
	set(CMAKE_FIND_LIBRARY_PREFIXES ${CMAKE_FIND_LIBRARY_PREFIXES} "")
endif ()

set(Boost_ADDITIONAL_VERSIONS "1.55")
set(Boost_ADDITIONAL_VERSIONS "1.53")


find_package(Boost 1.54 COMPONENTS filesystem system program_options thread REQUIRED)
include_directories(${Boost_INCLUDE_DIR})
link_directories(${Boost_LIBRARY_DIRS})
list(APPEND libs ${Boost_LIBRARIES})


set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")


# Include files
file(GLOB_RECURSE files_SRC
    "./USBDebug/*.cpp"
    "./USBDebug/*.h"
)

set (PROJECT_INCLUDE_DIR include)
include_directories(${PROJECT_INCLUDE_DIR})



add_executable(usbDebugTool ${files_SRC} main.cpp)
target_link_libraries(usbDebugTool ${libs})
