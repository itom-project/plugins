set(target_name libptp2)

project(${target_name})

message(STATUS "\n--------------- PLUGIN ${target_name} ---------------")

cmake_minimum_required(VERSION 3.0.2)

option(BUILD_UNICODE "Build with unicode charset if set to ON, else multibyte charset." ON)
option(BUILD_SHARED_LIBS "Build shared library." ON)
option(BUILD_TARGET64 "Build for 64 bit target if set to ON or 32 bit if set to OFF." ON)
set(CMAKE_DEBUG_POSTFIX "d" CACHE STRING "Adds a postfix for debug-built libraries.")

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR})

if(BUILD_SHARED_LIBS)
    set(LIBRARY_TYPE SHARED)
else(BUILD_SHARED_LIBS)
    set(LIBRARY_TYPE STATIC)
endif(BUILD_SHARED_LIBS)

find_package(VisualLeakDetector QUIET)

if(LibUSB_INCLUDE_DIRS)

    if(BUILD_UNICODE)
        add_definitions(-DUNICODE -D_UNICODE)
    endif(BUILD_UNICODE)
    add_definitions(-DCMAKE -DPTP_DLL)

    if(VISUALLEAKDETECTOR_FOUND AND VISUALLEAKDETECTOR_ENABLED)
        add_definitions(-DVISUAL_LEAK_DETECTOR_CMAKE)
    endif(VISUALLEAKDETECTOR_FOUND AND VISUALLEAKDETECTOR_ENABLED)

    # default build types are None, Debug, Release, RelWithDebInfo and MinRelSize
    if(DEFINED CMAKE_BUILD_TYPE)
        set(CMAKE_BUILD_TYPE ${CMAKE_BUILD_TYPE} CACHE STRING "Choose the type of build, options are: None(CMAKE_CXX_FLAGS or CMAKE_C_FLAGS used) Debug Release RelWithDebInfo MinSizeRel.")
    else()
        set(CMAKE_BUILD_TYPE Debug CACHE STRING "Choose the type of build, options are: None(CMAKE_CXX_FLAGS or CMAKE_C_FLAGS used) Debug Release RelWithDebInfo MinSizeRel.")
    endif()

    message(STATUS ${CMAKE_CURRENT_BINARY_DIR})
    
    include_directories(
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
    
    set_source_files_properties( ${plugin_SOURCES} PROPERTIES LANGUAGE CXX )
    
    add_library(${target_name} ${LIBRARY_TYPE} ${plugin_SOURCES} ${plugin_HEADERS})

    if(WIN32)
        target_link_libraries(${target_name} ${VISUALLEAKDETECTOR_LIBRARIES} wsock32)
    else(WIN32)
        target_link_libraries(${target_name} ${VISUALLEAKDETECTOR_LIBRARIES})
    endif(WIN32)
    
    LIST(APPEND COPY_SOURCES "$<TARGET_FILE:${target_name}>")
    LIST(APPEND COPY_DESTINATIONS "${ITOM_APP_DIR}/lib")    
    POST_BUILD_COPY_FILES(${target_name} COPY_SOURCES COPY_DESTINATIONS)    

else(LibUSB_INCLUDE_DIRS)
    message(WARNING "LibUSB_INCLUDE_DIRS directory could not be found. ${target_name} will not be build.")    
endif(LibUSB_INCLUDE_DIRS)
