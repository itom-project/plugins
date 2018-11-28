SET (target_name libgphoto2_port)

project(${target_name})

message(STATUS "\n--------------- PLUGIN ${target_name} ---------------")

cmake_minimum_required(VERSION 3.0)

OPTION(BUILD_UNICODE "Build with unicode charset if set to ON, else multibyte charset." ON)
OPTION(BUILD_SHARED_LIBS "Build shared library." ON)
OPTION(BUILD_TARGET64 "Build for 64 bit target if set to ON or 32 bit if set to OFF." ON)
SET (ITOM_SDK_DIR "" CACHE PATH "base path to itom_sdk")
SET (CMAKE_DEBUG_POSTFIX "d" CACHE STRING "Adds a postfix for debug-built libraries.")
SET (PACKAGE_VERSION CACHE STRING "2.5.9")

SET (CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR} ${ITOM_SDK_DIR})

#IF(BUILD_SHARED_LIBS)
#    SET(LIBRARY_TYPE SHARED)
#ELSE(BUILD_SHARED_LIBS)
#    SET(LIBRARY_TYPE STATIC)
#ENDIF(BUILD_SHARED_LIBS)
SET(LIBRARY_TYPE STATIC)

include("${ITOM_SDK_DIR}/ItomBuildMacros.cmake")
find_package(LibUSB QUIET)
find_package(VisualLeakDetector QUIET)

ADD_SUBDIRECTORY(disk)
ADD_SUBDIRECTORY(libusb1)
ADD_SUBDIRECTORY(ptpip)

IF (BUILD_UNICODE)
    ADD_DEFINITIONS(-DUNICODE -D_UNICODE)
ENDIF (BUILD_UNICODE)
ADD_DEFINITIONS(-DCMAKE)
ADD_DEFINITIONS(-DPACKAGE_VERSION="${PACKAGE_VERSION}")
ADD_DEFINITIONS(-DHAVE_REGEX)

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
    ${VISUALLEAKDETECTOR_INCLUDE_DIR}
	${CMAKE_CURRENT_SOURCE_DIR}/..
	${CMAKE_CURRENT_SOURCE_DIR}/../libtool
	${CMAKE_CURRENT_SOURCE_DIR}/../regex
	${LibUSB_INCLUDE_DIRS}	
)

set(plugin_HEADERS
	${CMAKE_CURRENT_SOURCE_DIR}/gphoto2/gphoto2-port.h
	${CMAKE_CURRENT_SOURCE_DIR}/gphoto2/gphoto2-port-info-list.h
	${CMAKE_CURRENT_SOURCE_DIR}/gphoto2/gphoto2-port-library.h
	${CMAKE_CURRENT_SOURCE_DIR}/gphoto2/gphoto2-port-log.h
	${CMAKE_CURRENT_SOURCE_DIR}/gphoto2/gphoto2-port-portability.h
	${CMAKE_CURRENT_SOURCE_DIR}/gphoto2/gphoto2-port-result.h
	${CMAKE_CURRENT_SOURCE_DIR}/gphoto2/gphoto2-port-version.h
	${CMAKE_CURRENT_SOURCE_DIR}/libgphoto2_port/gphoto2-port-info.h
	${CMAKE_CURRENT_SOURCE_DIR}/config.h
	${CMAKE_CURRENT_SOURCE_DIR}/dirent.h
)

set(plugin_SOURCES 
	${CMAKE_CURRENT_SOURCE_DIR}/libgphoto2_port/gphoto2-port.c
	${CMAKE_CURRENT_SOURCE_DIR}/libgphoto2_port/gphoto2-port-info-list.c
	${CMAKE_CURRENT_SOURCE_DIR}/libgphoto2_port/gphoto2-port-log.c
	${CMAKE_CURRENT_SOURCE_DIR}/libgphoto2_port/gphoto2-port-portability.c
	${CMAKE_CURRENT_SOURCE_DIR}/libgphoto2_port/gphoto2-port-result.c
	${CMAKE_CURRENT_SOURCE_DIR}/libgphoto2_port/gphoto2-port-version.c
#	${CMAKE_CURRENT_SOURCE_DIR}/serial/unix.c
#	${CMAKE_CURRENT_SOURCE_DIR}/test/test-gp-port.c
	${CMAKE_CURRENT_SOURCE_DIR}/test/test-port-list.c
	${CMAKE_CURRENT_SOURCE_DIR}/dirent.c
)

#Add version information to the plugIn-dll unter MSVC
if(MSVC)
#    list(APPEND plugin_SOURCES ${ITOM_SDK_INCLUDE_DIR}/../pluginLibraryVersion.rc)
endif(MSVC)

SET(LTDLLIB optimized ../libtool/$(Configuration)/libltdl debug ../libtool/$(Configuration)/libltdld)
SET(REGEXLIB optimized ../regex/$(Configuration)/regex debug ../regex/$(Configuration)/regexd)
SET(LIBUSB1LIB optimized ./libusb1/$(Configuration)/libgphoto2_port_libusb1 debug ./libusb1/$(Configuration)/libgphoto2_port_libusb1d)
SET(PTPPORTLIB optimized ./ptpip/$(Configuration)/libgphoto2_port_ptpip debug ./ptpip/$(Configuration)/libgphoto2_port_ptpipd)
SET(DISKLIB optimized ./disk/$(Configuration)/libgphoto2_port_disk debug ./disk/$(Configuration)/libgphoto2_port_diskd)

ADD_LIBRARY(${target_name} ${LIBRARY_TYPE} ${plugin_SOURCES} ${plugin_HEADERS})
TARGET_LINK_LIBRARIES(${target_name} ${VISUALLEAKDETECTOR_LIBRARIES} ${LibUSB_LIBRARIES} 
	${LTDLLIB}
	${REGEXLIB}
	${LIBUSB1LIB}
	${PTPPORTLIB}
	${DISKLIB})

#documentation
PLUGIN_DOCUMENTATION(${target_name} dslrRemote)

# COPY SECTION
set(COPY_SOURCES "")
set(COPY_DESTINATIONS "")
#LIST(APPEND COPY_SOURCES "$<TARGET_FILE:libgphoto2_port>")
#LIST(APPEND COPY_DESTINATIONS "${ITOM_APP_DIR}/lib")
#ADD_PLUGINLIBRARY_TO_COPY_LIST(${target_name} COPY_SOURCES COPY_DESTINATIONS)
#POST_BUILD_COPY_FILES(${target_name} COPY_SOURCES COPY_DESTINATIONS)
