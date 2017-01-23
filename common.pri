CONFIG *= warn_on ordered c++11  silent

unix|mingw|linux {
    CXXFLAGS *= -std=c11 -Wall -Wextra -Weffc++ -pedantic -Wno-parentheses
}

CONFIG (release, release|debug) {
    CXXFLAGS *= -O2
    unix|mingw {
        CXXFLAGS *= -mtune=generic -fomit-framepointer -ffast-math
    }
}

CONFIG (debug, debug|release) {
    DESTDIR = $${PWD}/bin/debug
    unix|mingw {
        CXXFLAGS *= -g -rdynamic -lmcheck
    }
}

CONFIG (release, release|debug) {
    DESTDIR = $${PWD}/bin/release
}

LIBS += -L$${DESTDIR}
