cmake_minimum_required(VERSION 3.12...3.29)
set(target_name libgphoto2_port_ptpip)

option(BUILD_UNICODE "Build with unicode charset if set to ON, else multibyte charset." ON)
option(BUILD_SHARED_LIBS "Build shared library." ON)
option(BUILD_TARGET64 "Build for 64 bit target if set to ON or 32 bit if set to OFF." ON)

#this is to automatically detect the SDK subfolder of the itom build directory.
if(NOT EXISTS ${ITOM_SDK_DIR})
    find_path(ITOM_SDK_DIR "cmake/itom_sdk.cmake"
    HINTS "$ENV{ITOM_SDK_ROOT}"
          "${CMAKE_CURRENT_BINARY_DIR}/../itom/SDK"
    DOC "Path of SDK subfolder of itom root (build) directory")
endif(NOT EXISTS ${ITOM_SDK_DIR})

if(NOT EXISTS ${ITOM_SDK_DIR})
    message(FATAL_ERROR "ITOM_SDK_DIR is invalid. Provide itom SDK directory path first")
endif(NOT EXISTS ${ITOM_SDK_DIR})

#this is to automatically detect the SDK subfolder of the itom build directory.
find_path(ITOM_SDK_DIR "cmake/itom_sdk.cmake"
    HINTS "C:/itom/build/itom/SDK"
          "${CMAKE_CURRENT_BINARY_DIR}/../../itom/SDK"
    DOC "path of SDK subfolder of itom root (build) directory")

if(NOT ITOM_SDK_DIR)
    message(FATAL_ERROR "ITOM_SDK_DIR is invalid. Provide itom SDK directory path first")
endif()

set(PACKAGE_VERSION CACHE STRING "2.5.9")

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR} ${ITOM_SDK_DIR}/cmake)

if(BUILD_SHARED_LIBS)
    set(LIBRARY_TYPE SHARED)
else(BUILD_SHARED_LIBS)
    set(LIBRARY_TYPE STATIC)
endif(BUILD_SHARED_LIBS)


include(ItomBuildMacros)
itom_init_cmake_policy(3.12)
itom_init_plugin_library(${target_name}) #Start the project, init compiler settings and set default configurations for plugins

if(BUILD_UNICODE)
    endif(BUILD_UNICODE)
add_definitions(-DCMAKE)
add_definitions(-DPACKAGE_VERSION="${PACKAGE_VERSION}")
add_definitions(-DHAVE_REGEX)
add_definitions(-DDLLEXPORT)


# default build types are None, Debug, Release, RelWithDebInfo and MinRelSize
if(DEFINED CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE ${CMAKE_BUILD_TYPE} CACHE STRING "Choose the type of build, options are: None(CMAKE_CXX_FLAGS or CMAKE_C_FLAGS used) Debug Release RelWithDebInfo MinSizeRel.")
else()
    set(CMAKE_BUILD_TYPE Debug CACHE STRING "Choose the type of build, options are: None(CMAKE_CXX_FLAGS or CMAKE_C_FLAGS used) Debug Release RelWithDebInfo MinSizeRel.")
endif()

include_directories(
    ${CMAKE_CURRENT_BINARY_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/..
    ${CMAKE_CURRENT_SOURCE_DIR}/../../libtool
    ${CMAKE_CURRENT_SOURCE_DIR}/../../regex
)

set(PLUGIN_HEADERS
    ${CMAKE_CURRENT_SOURCE_DIR}/../gphoto2/gphoto2-port.h
    ${CMAKE_CURRENT_SOURCE_DIR}/../gphoto2/gphoto2-port-info-list.h
    ${CMAKE_CURRENT_SOURCE_DIR}/../gphoto2/gphoto2-port-library.h
    ${CMAKE_CURRENT_SOURCE_DIR}/../gphoto2/gphoto2-port-log.h
    ${CMAKE_CURRENT_SOURCE_DIR}/../gphoto2/gphoto2-port-portability.h
    ${CMAKE_CURRENT_SOURCE_DIR}/../gphoto2/gphoto2-port-result.h
    ${CMAKE_CURRENT_SOURCE_DIR}/../gphoto2/gphoto2-port-version.h
    ${CMAKE_CURRENT_SOURCE_DIR}/../config.h
)

set(PLUGIN_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/../libgphoto2_port/gphoto2-port-info-list.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../libgphoto2_port/gphoto2-port-log.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../libgphoto2_port/gphoto2-port-result.c
    ${CMAKE_CURRENT_SOURCE_DIR}/ptpip.c
)

#Add version information to the plugIn-dll unter MSVC
if(MSVC)
#    list(APPEND PLUGIN_SOURCES ${ITOM_SDK_INCLUDE_DIR}/../pluginLibraryVersion.rc)
endif()

set(LTDLLIB optimized ../../libtool/$(Configuration)/libltdl debug ../../libtool/$(Configuration)/libltdld)
set(REGEXLIB optimized ../../regex/$(Configuration)/regex debug ../../regex/$(Configuration)/regexd)

add_library(${target_name} ${LIBRARY_TYPE} ${PLUGIN_SOURCES} ${PLUGIN_HEADERS})

add_dependencies(${target_name} libltdl)

target_link_libraries(${target_name} ${VISUALLEAKDETECTOR_LIBRARIES}
    ${LTDLLIB}
    ${REGEXLIB})

#documentation
#itom_configure_plugin_documentation(${target_name} dslrRemote)

# COPY SECTION
set(COPY_SOURCES "")
set(COPY_DESTINATIONS "")
LIST(APPEND COPY_SOURCES "$<TARGET_FILE:${target_name}>")
LIST(APPEND COPY_DESTINATIONS "${ITOM_APP_DIR}/plugins/DslrRemote")
#itom_add_pluginlibrary_to_copy_list(${target_name} COPY_SOURCES COPY_DESTINATIONS)
itom_post_build_copy_files(${target_name} COPY_SOURCES COPY_DESTINATIONS)
