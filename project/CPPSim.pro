CONFIG += c++11 console

TARGET = CPPSim

INCLUDEPATH += ../src
INCLUDEPATH += ../lib

SOURCES += ../src/*.cpp
SOURCES += ../src/Drawing/*.cpp
SOURCES += ../src/Math/*.cpp
SOURCES += ../src/Simulation/*.cpp
SOURCES += ../src/Utility/*.cpp

HEADERS += ../src/*.h
HEADERS += ../src/Drawing/*.h
HEADERS += ../src/Math/*.h
HEADERS += ../src/Simulation/*.h
HEADERS += ../src/Utility/*.h
HEADERS += ../lib/matrix/*.hpp

LIBS += -lglut -lGLU

QMAKE_CXXFLAGS += -Wno-unused-parameter -Wno-unused-local-typedefs
