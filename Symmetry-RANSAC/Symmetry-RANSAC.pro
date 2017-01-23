! include(../common.pri){
    error("Could not add common config");
}

QT       += core gui \
    gui \
    widgets

TEMPLATE = app

SOURCES += main.cpp\
        mainwindow.cpp \
    window.cpp \
    glwidget.cpp \
    logo.cpp \
    offwindow.cpp \
    shape.cpp \
    gloffwidget.cpp \
    rotation.cpp \
    ransacwindow.cpp \
    glransacwidget.cpp \
    optdialog.cpp \
    vecmath.cpp \
    core.cpp

HEADERS  += mainwindow.h \
    window.h \
    glwidget.h \
    logo.h \
    offwindow.h \
    shape.h \
    gloffwidget.h \
    rotation.h \
    ransacwindow.h \
    glransacwidget.h \
    optdialog.h \
    obj.h \
    simplify.h \
    vecmath.h \
    core.h \
    vector.h \
    _vector2.h \
    _vector3.h \
    _vector4.h \
    nmath.h \
    ntypes.h

FORMS    += mainwindow.ui

CONFIG += console
CONFIG += no_keywords
CONFIG -= app_bundle

#-----------------CGAL------------------
unix {
    INCLUDEPATH += /usr/include
    LIBS += -L/usr/lib64 -lCGAL -lCGAL_Core
}else{

}

#-----------------GMP------------------
unix {
    INCLUDEPATH += /usr/include
    LIBS += -L/usr/lib64 -lgmp
}else{

}

#-----------------MPFR------------------
unix {
    INCLUDEPATH += /usr/include
    LIBS += -L/usr/lib64 -lmpfr
}else{

}

#-----------------EIGEN------------------
unix {
    INCLUDEPATH += /usr/include/eigen3
}else{

}

#-----------------HALF------------------
unix {
    INCLUDEPATH += /usr/include
    LIBS += -L/usr/lib64 -lHalf
}else{

}

#-----------------TBB------------------
unix {
    INCLUDEPATH += /usr/include/tbb
    LIBS += -L/usr/lib64 -ltbb
}else{

}

#-----------------LOG4CPLUS------------------
unix {
    INCLUDEPATH += /usr/include/log4cplus
    LIBS += -L/usr/lib64 -llog4cplus
}else{

}

#-----------------OPENVDB------------------
unix {
    INCLUDEPATH += /usr/include/openvdb
    LIBS += -L/usr/lib -lopenvdb
}else{
}

#-----------------BOOST------------------
unix {
    INCLUDEPATH += /usr/include
    LIBS += -L/usr/lib/x86_64-linux-gnu -lboost_system -lboost_iostreams -llog4cplus -lboost_thread

}else{

}

#----------------PCL---------------------
unix{
    message("Detected UNIX platform, using pkg-config to determine PCL location")
    INCLUDEPATH += /usr/include/pcl-1.7
    LIBS += -lpcl_common -lpcl_kdtree -lpcl_sample_consensus -lpcl_search
    #PKGCONFIG += pcl_common-1.7 pcl_kdtree-1.7 pcl_sample_consensus-1.7 pcl_search-1.7
}else{

}

#---------------HARRIS_3D---------------
INCLUDEPATH += $${PWD}/../Harris3D/include
INCLUDEPATH += $${PWD}/../Util/include

LIBS += -L$${PWD}/../bin/release -lHarris3D -lUtil

#win32:CONFIG(release, debug|release): LIBS += -L$$PWD/../../../../../opt/openvdb/release/ -lopenvdb
#else:win32:CONFIG(debug, debug|release): LIBS += -L$$PWD/../../../../../opt/openvdb/debug/ -lopenvdb
#else:unix: LIBS += -L$$PWD/../../../../../opt/openvdb/ -lopenvdb

INCLUDEPATH += $$PWD/../../../../../opt
DEPENDPATH += $$PWD/../../../../../opt
