! include(../common.pri){
    error("Could not add common config");
}

CONFIG += dll
CONFIG -= app_bundle
CONFIG -= qt

TARGET = Harris3D
TEMPLATE = lib

INCLUDEPATH += include/
INCLUDEPATH += ../Util/include

HEADERS += include/Distance.h \
           include/Face.h \
           include/HarrisDetector.h \
           include/Mesh.h \
           include/Vertex.h

SOURCES += src/Face.cpp \
           src/HarrisDetector.cpp \
           src/Mesh.cpp \
           src/Vertex.cpp

