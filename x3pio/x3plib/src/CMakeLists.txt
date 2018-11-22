project(x3plib)

cmake_minimum_required(VERSION 2.8)

OPTION(BUILD_UNICODE "Build with unicode charset if set to ON, else multibyte charset." ON)
OPTION(BUILD_SHARED_LIBS "Build shared library." ON)
OPTION(BUILD_TARGET64 "Build for 64 bit target if set to ON or 32 bit if set to OFF." ON)
SET(ITOM_DIR "" CACHE PATH "base path to itom") 

ADD_SUBDIRECTORY(zlib/contrib/minizip)
ADD_SUBDIRECTORY(ISO5436_2_XML)

IF(BUILD_SHARED_LIBS)
ELSE(BUILD_SHARED_LIBS)
  SET(iso54362LIBSUFFIX S)
ENDIF(BUILD_SHARED_LIBS)

if(CMAKE_CL_64)
	SET(iso54362LIBSUFFIX ${iso54362LIBSUFFIX}64)
endif(CMAKE_CL_64)

ADD_DEPENDENCIES("iso5436-2-xml${iso54362LIBSUFFIX}" zlibwapi)
ADD_DEPENDENCIES("iso5436-2-xml${iso54362LIBSUFFIX}" genxmlxsd)