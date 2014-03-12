#-------------------------------------------------
#
# Project created by QtCreator 2012-03-23T13:23:34
#
#-------------------------------------------------

QT       -= core gui

TARGET = iso5436_2_demo
TEMPLATE = app

DEFINES += ISO5436_2_LIBRARY \
        UNICODE \
        _UNICODE

INCLUDEPATH +=  . \
                ./.. \
                /usr/include \
                ../../include \
                ../../include/opengps \
                ../../include/opengps/cxx \
                ../zlib \
                ../zlib/contrib/minizip

LIBS += -L../ISO5436_2_XML/ -liso5436_2  \
        -lxerces-c-3.1 \
        -L$$PWD/../zlib/contrib/minizip/.libs/ -lminizip


QMAKE_LFLAGS += -Wl,-R./:../ISO5436_2_XML -municode
QMAKE_CFLAGS += -fopenmp -fexceptions
QMAKE_CXXFLAGS += -fopenmp -fexceptions

HEADERS += ../../include/opengps/data_point.h \
        ../../include/opengps/data_point_type.h \
        ../../include/opengps/info.h \
        ../../include/opengps/iso5436_2.h \
        ../../include/opengps/messages.h \
        ../../include/opengps/opengps.h \
        ../../include/opengps/point_iterator.h \
        ../../include/opengps/point_vector.h \
        ../../include/opengps/cxx/data_point.hxx \
        ../../include/opengps/cxx/exceptions.hxx \
        ../../include/opengps/cxx/info.hxx \
        ../../include/opengps/cxx/iso5436_3.hxx \
        ../../include/opengps/cxx/iso5436_2_handle.hxx \
        ../../include/opengps/cxx/iso5436_2_xsd.hxx \
        ../../include/opengps/cxx/opengps.hxx \
        ../../include/opengps/cxx/point_vector.hxx \
        ../../include/opengps/cxx/point_vector_base.hxx \
        ../../include/opengps/cxx/string.hxx \
        ../../include/opengps/cxx/version.h

SOURCES +=  ./ISO5436_2_XML_Demo.cxx
