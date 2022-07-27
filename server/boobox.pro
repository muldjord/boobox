TEMPLATE = app
TARGET = BooBox
DEPENDPATH += .
INCLUDEPATH += .
CONFIG += release
QT += core network
QMAKE_CXXFLAGS += -std=c++11

include(./VERSION)
DEFINES+=VERSION=\\\"$$VERSION\\\"

HEADERS += src/boobox.h
SOURCES += src/main.cpp \
           src/boobox.cpp
