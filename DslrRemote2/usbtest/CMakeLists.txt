set(target_name usbtest)

project(${target_name})

message(STATUS "\n--------------- PLUGIN ${target_name} ---------------")

cmake_minimum_required(VERSION 3.0.2)

option(BUILD_UNICODE "Build with unicode charset if set to ON, else multibyte charset." ON)
option(BUILD_SHARED_LIBS "Build shared library." ON)
option(BUILD_TARGET64 "Build for 64 bit target if set to ON or 32 bit if set to OFF." ON)
option(UPDATE_TRANSLATIONS "Update source translation translation/*.ts files (WARNING: make clean will delete the source .ts files! Danger!)")
set(CMAKE_DEBUG_POSTFIX "d" CACHE STRING "Adds a postfix for debug-built libraries.")

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR})

if(BUILD_SHARED_LIBS)
    set(LIBRARY_TYPE SHARED)
else(BUILD_SHARED_LIBS)
    set(LIBRARY_TYPE STATIC)
endif(BUILD_SHARED_LIBS)

find_package(VisualLeakDetector QUIET)
find_package(LibUSB QUIET)

if(LibUSB_INCLUDE_DIRS)


    if(WIN32)
        if(BUILD_UNICODE)
            add_definitions(-DUNICODE -D_UNICODE)
        endif(BUILD_UNICODE)
        add_definitions(-DCMAKE)

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

        link_directories(
            ${LIBUSB_API_DIR}/lib
        )
        
        include_directories(
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

        add_executable(${target_name} ${plugin_SOURCES} ${plugin_HEADERS})

        if(WIN32)
            target_link_libraries(${target_name} ${VISUALLEAKDETECTOR_LIBRARIES} ${LibUSB_LIBRARY})
        else(WIN32)
            target_link_libraries(${target_name} ${VISUALLEAKDETECTOR_LIBRARIES} ${LibUSB_LIBRARY})
        endif(WIN32)
    endif(WIN32)

else(LibUSB_INCLUDE_DIRS)
    message(WARNING "LibUSB_INCLUDE_DIRS directory could not be found. ${target_name} will not be build.")    
endif(LibUSB_INCLUDE_DIRS)
