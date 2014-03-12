#-------------------------------------------------
#
# Project created by QtCreator 2012-03-23T13:23:34
#
#-------------------------------------------------


# load constants

TEMPLATE = subdirs
CONFIG   += ordered

TEMPLATE = subdirs
SUBDIRS +=  ./x3plib/src/ISO5436_2_XML/ISO5436_2_XML.pro \
            ./x3pio.pro

#QMAKE_LFLAGS += -Wl,-R./:../lib/,-rpath-link./:../lib/
