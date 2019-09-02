SET (target_name ptp2)

project(${target_name})

message(STATUS "\n--------------- PLUGIN ${target_name} ---------------")

cmake_minimum_required(VERSION 3.0)

OPTION(BUILD_UNICODE "Build with unicode charset if set to ON, else multibyte charset." ON)
OPTION(BUILD_SHARED_LIBS "Build shared library." ON)
OPTION(BUILD_TARGET64 "Build for 64 bit target if set to ON or 32 bit if set to OFF." ON)
SET (ITOM_SDK_DIR "" CACHE PATH "base path to itom_sdk")
SET (CMAKE_DEBUG_POSTFIX "d" CACHE STRING "Adds a postfix for debug-built libraries.")

SET (CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR} ${ITOM_SDK_DIR})
SET (PACKAGE_VERSION CACHE STRING "2.5.9")
#SET (CAMLIBS CACHE STRING "")

IF(BUILD_SHARED_LIBS)
    SET(LIBRARY_TYPE SHARED)
ELSE(BUILD_SHARED_LIBS)
    SET(LIBRARY_TYPE STATIC)
ENDIF(BUILD_SHARED_LIBS)

include("${ITOM_SDK_DIR}/ItomBuildMacros.cmake")
find_package(VisualLeakDetector QUIET)

IF (BUILD_UNICODE)
    ADD_DEFINITIONS(-DUNICODE -D_UNICODE)
ENDIF (BUILD_UNICODE)
ADD_DEFINITIONS(-DCMAKE)
ADD_DEFINITIONS(-DPACKAGE_VERSION="${PACKAGE_VERSION}")
#ADD_DEFINITIONS(-DCAMLIBS="${CAMLIBS}")
ADD_DEFINITIONS(-DHAVE_LIMITS_H)
ADD_DEFINITIONS(-DDLLEXPORT)

IF(VISUALLEAKDETECTOR_FOUND AND VISUALLEAKDETECTOR_ENABLED)
    ADD_DEFINITIONS(-DVISUAL_LEAK_DETECTOR_CMAKE)
ENDIF(VISUALLEAKDETECTOR_FOUND AND VISUALLEAKDETECTOR_ENABLED)

# default build types are None, Debug, Release, RelWithDebInfo and MinRelSize
IF (DEFINED CMAKE_BUILD_TYPE)
    SET(CMAKE_BUILD_TYPE ${CMAKE_BUILD_TYPE} CACHE STRING "Choose the type of build, options are: None(CMAKE_CXX_FLAGS or CMAKE_C_FLAGS used) Debug Release RelWithDebInfo MinSizeRel.")
ELSE(CMAKE_BUILD_TYPE)
    SET (CMAKE_BUILD_TYPE Debug CACHE STRING "Choose the type of build, options are: None(CMAKE_CXX_FLAGS or CMAKE_C_FLAGS used) Debug Release RelWithDebInfo MinSizeRel.")
ENDIF (DEFINED CMAKE_BUILD_TYPE)

message(STATUS ${CMAKE_CURRENT_BINARY_DIR})

INCLUDE_DIRECTORIES(
    ${CMAKE_CURRENT_BINARY_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}
	${CMAKE_CURRENT_SOURCE_DIR}/../libtool
	${CMAKE_CURRENT_SOURCE_DIR}/../../libgphoto2
	${CMAKE_CURRENT_SOURCE_DIR}/../../libgphoto2_port
	${CMAKE_CURRENT_SOURCE_DIR}/../../ #for gphoto2 directory
    ${CMAKE_CURRENT_BINARY_DIR}/../../ #for gphoto2 directory
    ${VISUALLEAKDETECTOR_INCLUDE_DIR}
)

set(plugin_HEADERS
	${CMAKE_CURRENT_SOURCE_DIR}/chdk_live_view.h
	${CMAKE_CURRENT_SOURCE_DIR}/chdk_ptp.h
	${CMAKE_CURRENT_SOURCE_DIR}/device-flags.h
	${CMAKE_CURRENT_SOURCE_DIR}/music-players.h
	${CMAKE_CURRENT_SOURCE_DIR}/olympus-wrap.h
	${CMAKE_CURRENT_SOURCE_DIR}/ptp.h
	${CMAKE_CURRENT_SOURCE_DIR}/ptp-bugs.h
	${CMAKE_CURRENT_SOURCE_DIR}/ptp-private.h
)

set(plugin_SOURCES 
    ${CMAKE_CURRENT_SOURCE_DIR}/chdk.c
	${CMAKE_CURRENT_SOURCE_DIR}/config.c
	${CMAKE_CURRENT_SOURCE_DIR}/library.c
	${CMAKE_CURRENT_SOURCE_DIR}/olympus-wrap.c
	${CMAKE_CURRENT_SOURCE_DIR}/ptp.c
	${CMAKE_CURRENT_SOURCE_DIR}/ptpip.c
#	${CMAKE_CURRENT_SOURCE_DIR}/ptp-pack.c
	${CMAKE_CURRENT_SOURCE_DIR}/usb.c
	${CMAKE_CURRENT_SOURCE_DIR}/../../libgphoto2/auxfuncs.c
)

#Add version information to the plugIn-dll unter MSVC
if(MSVC)
#    list(APPEND plugin_SOURCES ${ITOM_SDK_INCLUDE_DIR}/../pluginLibraryVersion.rc)
endif(MSVC)

SET(LTDLLIB optimized ../../libtool/$(Configuration)/libltdl debug ../../libtool/$(Configuration)/libltdld)
SET(REGEXLIB optimized ../../regex/$(Configuration)/regex debug ../../regex/$(Configuration)/regexd)
SET(LIBGPHOTO2 optimized ../../libgphoto2/$(Configuration)/libgphoto2 debug ../../libgphoto2/$(Configuration)/libgphoto2d)
SET(LIBGPHOTO2PORT optimized ../../libgphoto2_port/$(Configuration)/libgphoto2_port debug ../../libgphoto2_port/$(Configuration)/libgphoto2_portd)

ADD_LIBRARY(${target_name} ${LIBRARY_TYPE} ${plugin_SOURCES} ${plugin_HEADERS})
TARGET_LINK_LIBRARIES(${target_name} ${VISUALLEAKDETECTOR_LIBRARIES} 
	${LTDLLIB}
	${REGEXLIB}
	${LIBGPHOTO2}
	${LIBGPHOTO2PORT}
	shell32)

if(MSVC)
#    list(APPEND plugin_SOURCES ${ITOM_SDK_INCLUDE_DIR}/../pluginLibraryVersion.rc)
#  set_target_properties(${target_name}  PROPERTIES COMPILE_FLAGS "/Zc:wchar_t")
endif(MSVC)

#documentation
PLUGIN_DOCUMENTATION(${target_name} ptp2)

# COPY SECTION
set(COPY_SOURCES "")
set(COPY_DESTINATIONS "")
LIST(APPEND COPY_SOURCES "$<TARGET_FILE:${target_name}>")
LIST(APPEND COPY_DESTINATIONS "${ITOM_APP_DIR}/plugins/DslrRemote")
#ADD_PLUGINLIBRARY_TO_COPY_LIST(${target_name} COPY_SOURCES COPY_DESTINATIONS)
POST_BUILD_COPY_FILES(${target_name} COPY_SOURCES COPY_DESTINATIONS)
