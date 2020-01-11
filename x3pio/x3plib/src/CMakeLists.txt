project(x3plib)

cmake_minimum_required(VERSION 3.0.2)

option(BUILD_UNICODE "Build with unicode charset if set to ON, else multibyte charset." ON)
option(BUILD_SHARED_LIBS "Build shared library." ON)
option(BUILD_TARGET64 "Build for 64 bit target if set to ON or 32 bit if set to OFF." ON)

add_subdirectory(zlib/contrib/minizip)
add_subdirectory(ISO5436_2_XML)

if(BUILD_SHARED_LIBS)
else(BUILD_SHARED_LIBS)
  set(iso54362LIBSUFFIX S)
endif(BUILD_SHARED_LIBS)

if(CMAKE_CL_64)
    set(iso54362LIBSUFFIX ${iso54362LIBSUFFIX}64)
endif(CMAKE_CL_64)

add_dependencies("iso5436-2-xml${iso54362LIBSUFFIX}" zlibwapi)
add_dependencies("iso5436-2-xml${iso54362LIBSUFFIX}" genxmlxsd)