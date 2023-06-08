cmake_minimum_required(VERSION 3.12...3.24)

set(target_name libgphoto2_port)

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

set(PACKAGE_VERSION CACHE STRING "2.5.9")

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR} ${ITOM_SDK_DIR}/cmake)

#if(BUILD_SHARED_LIBS)
#    set(LIBRARY_TYPE SHARED)
#else(BUILD_SHARED_LIBS)
#    set(LIBRARY_TYPE STATIC)
#endif(BUILD_SHARED_LIBS)
set(LIBRARY_TYPE STATIC)


include(ItomBuildMacros)
itom_init_cmake_policy(3.12)
itom_init_plugin_library(${target_name}) #Start the project, init compiler settings and set default configurations for plugins
find_package(LibUSB QUIET)

add_subdirectory(disk)
add_subdirectory(libusb1)
add_subdirectory(ptpip)

if(BUILD_UNICODE)
    endif(BUILD_UNICODE)
add_definitions(-DCMAKE)
add_definitions(-DPACKAGE_VERSION="${PACKAGE_VERSION}")
add_definitions(-DHAVE_REGEX)


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
    ${CMAKE_CURRENT_SOURCE_DIR}/..
    ${CMAKE_CURRENT_SOURCE_DIR}/../libtool
    ${CMAKE_CURRENT_SOURCE_DIR}/../regex
    ${LibUSB_INCLUDE_DIRS}
)

set(PLUGIN_HEADERS
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

set(PLUGIN_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/libgphoto2_port/gphoto2-port.c
    ${CMAKE_CURRENT_SOURCE_DIR}/libgphoto2_port/gphoto2-port-info-list.c
    ${CMAKE_CURRENT_SOURCE_DIR}/libgphoto2_port/gphoto2-port-log.c
    ${CMAKE_CURRENT_SOURCE_DIR}/libgphoto2_port/gphoto2-port-portability.c
    ${CMAKE_CURRENT_SOURCE_DIR}/libgphoto2_port/gphoto2-port-result.c
    ${CMAKE_CURRENT_SOURCE_DIR}/libgphoto2_port/gphoto2-port-version.c
#    ${CMAKE_CURRENT_SOURCE_DIR}/serial/unix.c
#    ${CMAKE_CURRENT_SOURCE_DIR}/test/test-gp-port.c
    ${CMAKE_CURRENT_SOURCE_DIR}/test/test-port-list.c
    ${CMAKE_CURRENT_SOURCE_DIR}/dirent.c
)

#Add version information to the plugIn-dll unter MSVC
if(MSVC)
#    list(APPEND PLUGIN_SOURCES ${ITOM_SDK_INCLUDE_DIR}/../pluginLibraryVersion.rc)
endif()

set(LTDLLIB optimized ../libtool/$(Configuration)/libltdl debug ../libtool/$(Configuration)/libltdld)
set(REGEXLIB optimized ../regex/$(Configuration)/regex debug ../regex/$(Configuration)/regexd)
set(LIBUSB1LIB optimized ./libusb1/$(Configuration)/libgphoto2_port_libusb1 debug ./libusb1/$(Configuration)/libgphoto2_port_libusb1d)
set(PTPPORTLIB optimized ./ptpip/$(Configuration)/libgphoto2_port_ptpip debug ./ptpip/$(Configuration)/libgphoto2_port_ptpipd)
set(DISKLIB optimized ./disk/$(Configuration)/libgphoto2_port_disk debug ./disk/$(Configuration)/libgphoto2_port_diskd)

add_library(${target_name} ${LIBRARY_TYPE} ${PLUGIN_SOURCES} ${PLUGIN_HEADERS})

target_link_libraries(${target_name} ${VISUALLEAKDETECTOR_LIBRARIES} ${LibUSB_LIBRARIES}
    ${LTDLLIB}
    ${REGEXLIB}
    ${LIBUSB1LIB}
    ${PTPPORTLIB}
    ${DISKLIB})

#documentation
itom_configure_plugin_documentation(${target_name} dslrRemote)

# COPY SECTION
set(COPY_SOURCES "")
set(COPY_DESTINATIONS "")
#LIST(APPEND COPY_SOURCES "$<TARGET_FILE:libgphoto2_port>")
#LIST(APPEND COPY_DESTINATIONS "${ITOM_APP_DIR}/lib")
#itom_add_pluginlibrary_to_copy_list(${target_name} COPY_SOURCES COPY_DESTINATIONS)
#itom_post_build_copy_files(${target_name} COPY_SOURCES COPY_DESTINATIONS)
