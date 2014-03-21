if HAVE_WINDOWS
noinst_LTLIBRARIES = libdc1394-msw.la
endif

AM_CFLAGS = -DDC1394_DLL_EXPORTS -I$(top_srcdir) -I$(srcdir)/.. -I$(WINDDK)/inc -I$(WINDDK)/src/wdm/1394/inc -I$(WINDDK)/src/wdm/1394
libdc1394_msw_la_SOURCES =  \
	control-msw.c \
	msw1394.c \
	msw1394.h \
	msw.h


