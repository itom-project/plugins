cmake_minimum_required(VERSION 3.12...3.29)
set(target_name ptp2)

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

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR} ${ITOM_SDK_DIR}/cmake)
set(PACKAGE_VERSION CACHE STRING "2.5.9")
#set(CAMLIBS CACHE STRING "")

include(ItomBuildMacros)
itom_init_cmake_policy(3.12)
itom_init_plugin_library(${target_name}) #Start the project, init compiler settings and set default configurations for plugins

if(BUILD_SHARED_LIBS)
    set(LIBRARY_TYPE SHARED)
else(BUILD_SHARED_LIBS)
    set(LIBRARY_TYPE STATIC)
endif(BUILD_SHARED_LIBS)


if(BUILD_UNICODE)
    endif(BUILD_UNICODE)
add_definitions(-DCMAKE)
add_definitions(-DPACKAGE_VERSION="${PACKAGE_VERSION}")
#add_definitions(-DCAMLIBS="${CAMLIBS}")
add_definitions(-DHAVE_LIMITS_H)
add_definitions(-DDLLEXPORT)


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
    ${CMAKE_CURRENT_SOURCE_DIR}/../libtool
    ${CMAKE_CURRENT_SOURCE_DIR}/../../libgphoto2
    ${CMAKE_CURRENT_SOURCE_DIR}/../../libgphoto2_port
    ${CMAKE_CURRENT_SOURCE_DIR}/../../ #for gphoto2 directory
    ${CMAKE_CURRENT_BINARY_DIR}/../../ #for gphoto2 directory
)

set(PLUGIN_HEADERS
    ${CMAKE_CURRENT_SOURCE_DIR}/chdk_live_view.h
    ${CMAKE_CURRENT_SOURCE_DIR}/chdk_ptp.h
    ${CMAKE_CURRENT_SOURCE_DIR}/device-flags.h
    ${CMAKE_CURRENT_SOURCE_DIR}/music-players.h
    ${CMAKE_CURRENT_SOURCE_DIR}/olympus-wrap.h
    ${CMAKE_CURRENT_SOURCE_DIR}/ptp.h
    ${CMAKE_CURRENT_SOURCE_DIR}/ptp-bugs.h
    ${CMAKE_CURRENT_SOURCE_DIR}/ptp-private.h
)

set(PLUGIN_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/chdk.c
    ${CMAKE_CURRENT_SOURCE_DIR}/config.c
    ${CMAKE_CURRENT_SOURCE_DIR}/library.c
    ${CMAKE_CURRENT_SOURCE_DIR}/olympus-wrap.c
    ${CMAKE_CURRENT_SOURCE_DIR}/ptp.c
    ${CMAKE_CURRENT_SOURCE_DIR}/ptpip.c
#    ${CMAKE_CURRENT_SOURCE_DIR}/ptp-pack.c
    ${CMAKE_CURRENT_SOURCE_DIR}/usb.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../libgphoto2/auxfuncs.c
)

#Add version information to the plugIn-dll unter MSVC
if(MSVC)
#    list(APPEND PLUGIN_SOURCES ${ITOM_SDK_INCLUDE_DIR}/../pluginLibraryVersion.rc)
endif()

set(LTDLLIB optimized ../../libtool/$(Configuration)/libltdl debug ../../libtool/$(Configuration)/libltdld)
set(REGEXLIB optimized ../../regex/$(Configuration)/regex debug ../../regex/$(Configuration)/regexd)
set(LIBGPHOTO2 optimized ../../libgphoto2/$(Configuration)/libgphoto2 debug ../../libgphoto2/$(Configuration)/libgphoto2d)
set(LIBGPHOTO2PORT optimized ../../libgphoto2_port/$(Configuration)/libgphoto2_port debug ../../libgphoto2_port/$(Configuration)/libgphoto2_portd)

add_library(${target_name} ${LIBRARY_TYPE} ${PLUGIN_SOURCES} ${PLUGIN_HEADERS})

add_dependencies(${target_name} libltdl)
add_dependencies(${target_name} libgphoto2)

target_link_libraries(${target_name} ${VISUALLEAKDETECTOR_LIBRARIES}
    ${LTDLLIB}
    ${REGEXLIB}
    ${LIBGPHOTO2}
    ${LIBGPHOTO2PORT}
    shell32)

if(MSVC)
#    list(APPEND PLUGIN_SOURCES ${ITOM_SDK_INCLUDE_DIR}/../pluginLibraryVersion.rc)
#  set_target_properties(${target_name}  PROPERTIES COMPILE_FLAGS "/Zc:wchar_t")
endif()

#documentation
itom_configure_plugin_documentation(${target_name} ptp2)

# COPY SECTION
set(COPY_SOURCES "")
set(COPY_DESTINATIONS "")
LIST(APPEND COPY_SOURCES "$<TARGET_FILE:${target_name}>")
LIST(APPEND COPY_DESTINATIONS "${ITOM_APP_DIR}/plugins/DslrRemote")
#itom_add_pluginlibrary_to_copy_list(${target_name} COPY_SOURCES COPY_DESTINATIONS)
itom_post_build_copy_files(${target_name} COPY_SOURCES COPY_DESTINATIONS)
