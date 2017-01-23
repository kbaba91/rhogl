! include(../common.pri){
    error("Could not add common config");
}

CONFIG += dll
CONFIG -= app_bundle
CONFIG -= qt

TARGET = Util
TEMPLATE = lib

INCLUDEPATH += include/

HEADERS += include/Clock.h \
           include/Properties.h \
           include/util.h

SOURCES += src/Properties.cpp \
           src/util.cpp


