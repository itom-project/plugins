MAINTAINERCLEANFILES = Makefile.in
lib_LTLIBRARIES = libdc1394.la

SUBDIRS = linux juju macosx msw usb vendor
AM_CFLAGS = $(platform_CFLAGS) -I$(top_srcdir)

libdc1394_la_LDFLAGS = $(platform_LDFLAGS) \
	-version-info @lt_current@:@lt_revision@:@lt_age@

libdc1394_la_SOURCES =  \
	control.c 	\
	control.h	\
	enumeration.c   \
	platform.h      \
	capture.c       \
	offsets.h	\
	format7.c       \
	register.c      \
	register.h      \
	utils.c         \
	utils.h         \
	internal.c      \
	internal.h      \
	conversions.c   \
	conversions.h   \
	bayer_avt.c \
	bayer.c         \
	log.c		\
	log.h		\
	iso.c 		\
	iso.h

if HAVE_LINUX
if HAVE_LIBRAW1394
  LINUX_LIBADD = linux/libdc1394-linux.la
endif
  JUJU_LIBADD = juju/libdc1394-juju.la
endif
if HAVE_MACOSX
  MACOSX_LIBADD = macosx/libdc1394-macosx.la
endif
if HAVE_WINDOWS
  MSW_LIBADD = msw/libdc1394-msw.la
endif
if HAVE_LIBUSB
  USB_LIBADD = usb/libdc1394-usb.la
endif

libdc1394_la_LIBADD = \
	$(LINUX_LIBADD) \
	$(JUJU_LIBADD) \
	$(MACOSX_LIBADD) \
	$(MSW_LIBADD) \
	$(USB_LIBADD) \
	vendor/libdc1394-vendor.la

# headers to be installed
pkginclude_HEADERS =  	\
	dc1394.h	\
	types.h		\
	camera.h	\
	control.h     	\
	capture.h	\
	video.h		\
	format7.h	\
	utils.h       	\
	conversions.h 	\
	register.h    	\
	log.h	      	\
	iso.h
