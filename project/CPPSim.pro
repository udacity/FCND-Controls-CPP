TEMPLATE = app
CONFIG += c++11 console
CONFIG -= qt

TARGET = CPPSim

INCLUDEPATH += ../src
INCLUDEPATH += ../lib

SOURCES += ../src/*.cpp
SOURCES += ../src/Drawing/*.cpp
SOURCES += ../src/Math/*.cpp
SOURCES += ../src/Simulation/*.cpp
SOURCES += ../src/Utility/*.cpp
SOURCES += ../src/MavlinkNode/*.cpp

HEADERS += ../src/*.h
HEADERS += ../src/Drawing/*.h
HEADERS += ../src/Math/*.h
HEADERS += ../src/Simulation/*.h
HEADERS += ../src/Utility/*.h
SOURCES += ../src/MavlinkNode/*.h
HEADERS += ../lib/matrix/*.hpp
HEADERS += ../lib/mavlink/*.h
HEADERS += ../lib/mavlink/common/*.h

LIBS += -lglut -lGLU -lGL -lpthread

QMAKE_CXXFLAGS += -Wno-unused-parameter -Wno-unused-local-typedefs
