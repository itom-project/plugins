set(target_name libgphoto2_port_ptpip)

project(${target_name})

message(STATUS "\n--------------- PLUGIN ${target_name} ---------------")

cmake_minimum_required(VERSION 3.0.2)

option(BUILD_UNICODE "Build with unicode charset if set to ON, else multibyte charset." ON)
option(BUILD_SHARED_LIBS "Build shared library." ON)
option(BUILD_TARGET64 "Build for 64 bit target if set to ON or 32 bit if set to OFF." ON)
set(ITOM_SDK_DIR "" CACHE PATH "base path to itom_sdk")
set(CMAKE_DEBUG_POSTFIX "d" CACHE STRING "Adds a postfix for debug-built libraries.")
set(PACKAGE_VERSION CACHE STRING "2.5.9")

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR} ${ITOM_SDK_DIR})

if(BUILD_SHARED_LIBS)
    set(LIBRARY_TYPE SHARED)
else(BUILD_SHARED_LIBS)
    set(LIBRARY_TYPE STATIC)
endif(BUILD_SHARED_LIBS)


include("${ITOM_SDK_DIR}/ItomBuildMacros.cmake")
INIT_ITOM_LIBRARY() #apply general project settings, CMake configurations, policies... for itom and its libraries
find_package(VisualLeakDetector QUIET)

if(BUILD_UNICODE)
    add_definitions(-DUNICODE -D_UNICODE)
endif(BUILD_UNICODE)
add_definitions(-DCMAKE)
add_definitions(-DPACKAGE_VERSION="${PACKAGE_VERSION}")
add_definitions(-DHAVE_REGEX)
add_definitions(-DDLLEXPORT)

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
    ${VISUALLEAKDETECTOR_INCLUDE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/..
    ${CMAKE_CURRENT_SOURCE_DIR}/../../libtool
    ${CMAKE_CURRENT_SOURCE_DIR}/../../regex
)

set(plugin_HEADERS
    ${CMAKE_CURRENT_SOURCE_DIR}/../gphoto2/gphoto2-port.h
    ${CMAKE_CURRENT_SOURCE_DIR}/../gphoto2/gphoto2-port-info-list.h
    ${CMAKE_CURRENT_SOURCE_DIR}/../gphoto2/gphoto2-port-library.h
    ${CMAKE_CURRENT_SOURCE_DIR}/../gphoto2/gphoto2-port-log.h
    ${CMAKE_CURRENT_SOURCE_DIR}/../gphoto2/gphoto2-port-portability.h
    ${CMAKE_CURRENT_SOURCE_DIR}/../gphoto2/gphoto2-port-result.h
    ${CMAKE_CURRENT_SOURCE_DIR}/../gphoto2/gphoto2-port-version.h
    ${CMAKE_CURRENT_SOURCE_DIR}/../config.h    
)

set(plugin_SOURCES 
    ${CMAKE_CURRENT_SOURCE_DIR}/../libgphoto2_port/gphoto2-port-info-list.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../libgphoto2_port/gphoto2-port-log.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../libgphoto2_port/gphoto2-port-result.c
    ${CMAKE_CURRENT_SOURCE_DIR}/ptpip.c
)

#Add version information to the plugIn-dll unter MSVC
if(MSVC)
#    list(APPEND plugin_SOURCES ${ITOM_SDK_INCLUDE_DIR}/../pluginLibraryVersion.rc)
endif()

set(LTDLLIB optimized ../../libtool/$(Configuration)/libltdl debug ../../libtool/$(Configuration)/libltdld)
set(REGEXLIB optimized ../../regex/$(Configuration)/regex debug ../../regex/$(Configuration)/regexd)

add_library(${target_name} ${LIBRARY_TYPE} ${plugin_SOURCES} ${plugin_HEADERS})
target_link_libraries(${target_name} ${VISUALLEAKDETECTOR_LIBRARIES} 
    ${LTDLLIB}
    ${REGEXLIB})

#documentation
#PLUGIN_DOCUMENTATION(${target_name} dslrRemote)

# COPY SECTION
set(COPY_SOURCES "")
set(COPY_DESTINATIONS "")
LIST(APPEND COPY_SOURCES "$<TARGET_FILE:${target_name}>")
LIST(APPEND COPY_DESTINATIONS "${ITOM_APP_DIR}/plugins/DslrRemote")
#ADD_PLUGINLIBRARY_TO_COPY_LIST(${target_name} COPY_SOURCES COPY_DESTINATIONS)
POST_BUILD_COPY_FILES(${target_name} COPY_SOURCES COPY_DESTINATIONS)
