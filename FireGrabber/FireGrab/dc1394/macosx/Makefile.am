pkgmacosxincludedir = $(pkgincludedir)/macosx

if HAVE_MACOSX
noinst_LTLIBRARIES = libdc1394-macosx.la

# headers to be installed
pkgmacosxinclude_HEADERS = \
	capture.h
endif

AM_CFLAGS = -I..
libdc1394_macosx_la_LDFLAGS = -framework CoreFoundation -framework Carbon
libdc1394_macosx_la_LIBADD = -lIOKit
libdc1394_macosx_la_SOURCES =  \
	control.c \
	capture.c \
	capture.h \
	macosx.h


