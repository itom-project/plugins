#-------------------------------------------------
#
# Project created by QtCreator 2012-03-23T13:23:34
#
#-------------------------------------------------


# load constants

TEMPLATE = subdirs
CONFIG   += ordered

TEMPLATE = subdirs
SUBDIRS +=  ./ISO5436_2_XML/ISO5436_2_XML.pro \
        ./ISO5436_2_XML_Demo/ISO5436_2_XML_Demo.pro

QMAKE_LFLAGS += -Wl,-R./:./lib
