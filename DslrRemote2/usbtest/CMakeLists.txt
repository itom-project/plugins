SET (target_name usbtest)

project(${target_name})

message(STATUS "\n--------------- PLUGIN ${target_name} ---------------")

cmake_minimum_required(VERSION 2.8)

OPTION(BUILD_UNICODE "Build with unicode charset if set to ON, else multibyte charset." ON)
OPTION(BUILD_SHARED_LIBS "Build shared library." ON)
OPTION(BUILD_TARGET64 "Build for 64 bit target if set to ON or 32 bit if set to OFF." ON)
OPTION(UPDATE_TRANSLATIONS "Update source translation translation/*.ts files (WARNING: make clean will delete the source .ts files! Danger!)")
SET (CMAKE_DEBUG_POSTFIX "d" CACHE STRING "Adds a postfix for debug-built libraries.")

SET (CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR})

IF(BUILD_SHARED_LIBS)
    SET(LIBRARY_TYPE SHARED)
ELSE(BUILD_SHARED_LIBS)
    SET(LIBRARY_TYPE STATIC)
ENDIF(BUILD_SHARED_LIBS)

find_package(VisualLeakDetector QUIET)
find_package(LibUSB QUIET)

IF(LibUSB_INCLUDE_DIRS)


    IF (WIN32)
        IF (BUILD_UNICODE)
            ADD_DEFINITIONS(-DUNICODE -D_UNICODE)
        ENDIF (BUILD_UNICODE)
        ADD_DEFINITIONS(-DCMAKE)

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

        LINK_DIRECTORIES(
            ${LIBUSB_API_DIR}/lib
        )
        
        INCLUDE_DIRECTORIES(
            ${CMAKE_CURRENT_BINARY_DIR}
            ${CMAKE_CURRENT_SOURCE_DIR}
            ${LibUSB_INCLUDE_DIRS}
            ${VISUALLEAKDETECTOR_INCLUDE_DIR}
        )

        set(plugin_HEADERS
        )

        set(plugin_SOURCES 
            ${CMAKE_CURRENT_SOURCE_DIR}/usbtest.cpp
        )

        ADD_EXECUTABLE(${target_name} ${plugin_SOURCES} ${plugin_HEADERS})

        IF (WIN32)
            TARGET_LINK_LIBRARIES(${target_name} ${VISUALLEAKDETECTOR_LIBRARIES} ${LibUSB_LIBRARY})
        ELSE(WIN32)
            TARGET_LINK_LIBRARIES(${target_name} ${VISUALLEAKDETECTOR_LIBRARIES} ${LibUSB_LIBRARY})
        ENDIF(WIN32)
    ENDIF(WIN32)

ELSE (LibUSB_INCLUDE_DIRS)
    message(WARNING "LibUSB_INCLUDE_DIRS directory could not be found. ${target_name} will not be build.")    
ENDIF (LibUSB_INCLUDE_DIRS)
