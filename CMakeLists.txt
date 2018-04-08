cmake_minimum_required(VERSION 3.9)
project(CPPSim)

set(CMAKE_CXX_STANDARD 11)
#set(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -std=c++0x")

include_directories(src)
include_directories(lib)

FILE(GLOB SOURCES
        src/*.cpp
        src/Drawing/*.cpp
        src/Math/*.cpp
        src/Simulation/*.cpp
        src/Utility/*.cpp
        src/MavlinkNode/*.cpp
        src/MavlinkNode/*.h)

FILE(GLOB HEADERS
        src/*.h
        src/Drawing/*.h
        src/Math/*.h
        src/Simulation/*.h
        src/Utility/*.h
        lib/matrix/*.hpp
        lib/mavlink/*.h
        lib/mavlink/common/*.h)

find_package(Qt5Core REQUIRED)
find_package(Qt5Network REQUIRED)
find_package(Qt5Widgets REQUIRED)

find_package(GLU REQUIRED)
find_package(OpenGL REQUIRED)
#find_package(GL REQUIRED)
#find_package(pthread REQUIRED)

add_executable(CPPSim
        ${SOURCES}
        ${HEADERS}
        )

target_link_libraries(CPPSim
        Qt5::Core
        Qt5::Network
        Qt5::Widgets
        glut
        GLU
        GL
        pthread
        )

