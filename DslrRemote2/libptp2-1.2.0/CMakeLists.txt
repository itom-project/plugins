SET (target_name libptp2)

project(${target_name})

message(STATUS "\n--------------- PLUGIN ${target_name} ---------------")

cmake_minimum_required(VERSION 2.8)

OPTION(BUILD_UNICODE "Build with unicode charset if set to ON, else multibyte charset." ON)
OPTION(BUILD_SHARED_LIBS "Build shared library." ON)
OPTION(BUILD_TARGET64 "Build for 64 bit target if set to ON or 32 bit if set to OFF." ON)
SET (CMAKE_DEBUG_POSTFIX "d" CACHE STRING "Adds a postfix for debug-built libraries.")

SET (CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR})

IF(BUILD_SHARED_LIBS)
    SET(LIBRARY_TYPE SHARED)
ELSE(BUILD_SHARED_LIBS)
    SET(LIBRARY_TYPE STATIC)
ENDIF(BUILD_SHARED_LIBS)

find_package(VisualLeakDetector QUIET)

IF(LibUSB_INCLUDE_DIRS)

    IF (BUILD_UNICODE)
        ADD_DEFINITIONS(-DUNICODE -D_UNICODE)
    ENDIF (BUILD_UNICODE)
    ADD_DEFINITIONS(-DCMAKE -DPTP_DLL)

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
		${CMAKE_CURRENT_SOURCE_DIR}/src
        ${VISUALLEAKDETECTOR_INCLUDE_DIR}
    )

    set(plugin_HEADERS
        ${CMAKE_CURRENT_SOURCE_DIR}/src/libptp-endian.h
		${CMAKE_CURRENT_SOURCE_DIR}/src/libptp-stdint.h
		${CMAKE_CURRENT_SOURCE_DIR}/src/ptp.h
		#${CMAKE_CURRENT_SOURCE_DIR}/src/ptpcam.h
    )

    set(plugin_SOURCES 
        ${CMAKE_CURRENT_SOURCE_DIR}/src/properties.c
		${CMAKE_CURRENT_SOURCE_DIR}/src/ptp.c
		#${CMAKE_CURRENT_SOURCE_DIR}/myusb.c
		#${CMAKE_CURRENT_SOURCE_DIR}/ptpcam.c
		#${CMAKE_CURRENT_SOURCE_DIR}/src/ptp-pack.c
    )
	
	SET_SOURCE_FILES_PROPERTIES( ${plugin_SOURCES} PROPERTIES LANGUAGE CXX )
	
    ADD_LIBRARY(${target_name} ${LIBRARY_TYPE} ${plugin_SOURCES} ${plugin_HEADERS})

	IF (WIN32)
		TARGET_LINK_LIBRARIES(${target_name} ${VISUALLEAKDETECTOR_LIBRARIES} wsock32)
	ELSE(WIN32)
		TARGET_LINK_LIBRARIES(${target_name} ${VISUALLEAKDETECTOR_LIBRARIES})
	ENDIF(WIN32)
	
	LIST(APPEND COPY_SOURCES "$<TARGET_FILE:${target_name}>")
    LIST(APPEND COPY_DESTINATIONS "${ITOM_APP_DIR}/lib")	
    POST_BUILD_COPY_FILES(${target_name} COPY_SOURCES COPY_DESTINATIONS)	

ELSE (LibUSB_INCLUDE_DIRS)
    message(WARNING "LibUSB_INCLUDE_DIRS directory could not be found. ${target_name} will not be build.")    
ENDIF (LibUSB_INCLUDE_DIRS)
