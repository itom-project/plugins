cmake_minimum_required(VERSION 3.12...3.24)

project(x3plib)

option(BUILD_TARGET64 "Build for 64 bit target if set to ON or 32 bit if set to OFF." ON)

add_subdirectory(zlib/contrib/minizip)
add_subdirectory(ISO5436_2_XML)

set(iso54362LIBSUFFIX "")

if(CMAKE_CL_64)
    set(iso54362LIBSUFFIX ${iso54362LIBSUFFIX}64)
endif(CMAKE_CL_64)

add_dependencies("iso5436-2-xml${iso54362LIBSUFFIX}" zlibwapi)
add_dependencies("iso5436-2-xml${iso54362LIBSUFFIX}" genxmlxsd)
