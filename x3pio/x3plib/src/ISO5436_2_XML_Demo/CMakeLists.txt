project(ISO5436_2-demo)

cmake_minimum_required(VERSION 2.8)

OPTION(BUILD_SHARED_LIBS "Build shared library." ON)
OPTION(BUILD_TARGET64 "Build for 64 bit target if set to ON or 32 bit if set to OFF." OFF)

IF(BUILD_SHARED_LIBS)
  SET(LIBRARY_TYPE SHARED)
ELSE(BUILD_SHARED_LIBS)
  SET(LIBRARY_TYPE STATIC)
ENDIF(BUILD_SHARED_LIBS)

SET (CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR} ${ITOM_SDK_DIR})

find_package(XSD REQUIRED)
find_package(Xerces REQUIRED)

FIND_PATH(ISO5436_2_PATH iso5436_2.xsd
  PATHS $ENV{OPENGPS_DIR}
  ./ISO5436_2_XML
  ./x3p
  ../ISO5436_2_XML
  ../x3p
)

IF(BUILD_SHARED_LIBS)
MESSAGE(STATUS "Build shared")
  FIND_PATH(ISO5436_2_LIB_PATH libiso5436-2-xml.so
    PATHS $ENV{OPENGPS_DIR}
    ./ISO5436_2_XML
    ./x3p
    ../ISO5436_2_XML
    ../x3p
    ../../build
    ../../build/ISO5436_2_XML
    ../../build/x3p
  )
ELSE(BUILD_SHARED_LIBS)
MESSAGE(STATUS "Build static")
  FIND_PATH(ISO5436_2_LIB_PATH libiso5436-2-xml.a
    PATHS $ENV{OPENGPS_DIR}
    ./ISO5436_2_XML
    ./x3p
    ../ISO5436_2_XML
    ../x3p
    ../../build
    ../../build/ISO5436_2_XML
    ../../build/x3p
  )
ENDIF(BUILD_SHARED_LIBS)

INCLUDE_DIRECTORIES( 
	${XSD_INCLUDE_DIR}
	"../../include"
	"../../include/opengps/cxx"
)

add_definitions(-DUNICODE -D_UNICODE -DISO5436_2_LIBRARY)

# default build types are None, Debug, Release, RelWithDebInfo and MinRelSize
IF (DEFINED CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE ${CMAKE_BUILD_TYPE} CACHE STRING "Choose the type of build, options are: None(CMAKE_CXX_FLAGS or CMAKE_C_FLAGS used) Debug Release RelWithDebInfo MinSizeRel.")
ELSE(CMAKE_BUILD_TYPE)
  SET (CMAKE_BUILD_TYPE Debug CACHE STRING "Choose the type of build,
options are: None(CMAKE_CXX_FLAGS or CMAKE_C_FLAGS used) Debug Release
RelWithDebInfo MinSizeRel.")
ENDIF (DEFINED CMAKE_BUILD_TYPE)

add_executable(iso5436-2-xml-demo ISO5436_2_XML_Demo.cxx)

set(CMAKE_SHARED_LINKER_FLAGS "-Wl,-R./:./lib")
set(CMAKE_MODULE_LINKER_FLAGS "-Wl,-R./:./lib")
set(CMAKE_EXE_LINKER_FLAGS "-Wl,-R./:./lib")

IF(BUILD_SHARED_LIBS)
  ADD_CUSTOM_TARGET( copyzlib ALL)
  ADD_CUSTOM_COMMAND( TARGET copyzlib POST_BUILD
			COMMAND /bin/mkdir
			ARGS -p ${CMAKE_CURRENT_BINARY_DIR}/lib/
			COMMAND /bin/cp
			ARGS -u ${ISO5436_2_PATH}/../zlib/contrib/minizip/.libs/*.so* ${CMAKE_CURRENT_BINARY_DIR}/lib/
			COMMAND /bin/cp 
			ARGS -u ${ISO5436_2_LIB_PATH}/*.so* ${CMAKE_CURRENT_BINARY_DIR}/lib/
			DEPENDS ISO5436-2-demo)
ENDIF(BUILD_SHARED_LIBS)

target_link_libraries(iso5436-2-xml-demo 
	    -L${XERCESC_LIBRARY} -lxerces-c-3.1
	    -L${ISO5436_2_PATH}/../zlib/contrib/minizip/.libs/ -lminizip
	    -L${ISO5436_2_LIB_PATH}/ -liso5436-2-xml)