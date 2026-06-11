cmake_minimum_required(VERSION 3.12...3.29)
set(target_name libltdl)

#option(BUILD_UNICODE "Build with unicode charset if set to ON, else multibyte charset." ON)
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
set(LT_OBJDIR CACHE STRING "")

if(BUILD_SHARED_LIBS)
    set(LIBRARY_TYPE SHARED)
else(BUILD_SHARED_LIBS)
    set(LIBRARY_TYPE STATIC)
endif(BUILD_SHARED_LIBS)


include(ItomBuildMacros)
itom_init_cmake_policy(3.12)
itom_init_plugin_library(${target_name}) #Start the project, init compiler settings and set default configurations for plugins

#if(BUILD_UNICODE)
#    #endif(BUILD_UNICODE)
remove_definitions(-DUNICODE -D_UNICODE)
add_definitions(-DCMAKE)
add_definitions(-DLTDL)
add_definitions(-DLT_OBJDIR="${LT_OBJDIR}")
add_definitions(-DDLL_EXPORT)


# default build types are None, Debug, Release, RelWithDebInfo and MinRelSize
if(DEFINED CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE ${CMAKE_BUILD_TYPE} CACHE STRING "Choose the type of build, options are: None(CMAKE_CXX_FLAGS or CMAKE_C_FLAGS used) Debug Release RelWithDebInfo MinSizeRel.")
else()
    set(CMAKE_BUILD_TYPE Debug CACHE STRING "Choose the type of build, options are: None(CMAKE_CXX_FLAGS or CMAKE_C_FLAGS used) Debug Release RelWithDebInfo MinSizeRel.")
endif()

include_directories(
    ${CMAKE_CURRENT_BINARY_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/libltdl
)

set(PLUGIN_HEADERS
    ${CMAKE_CURRENT_SOURCE_DIR}/libltdl/lt__alloc.h
    ${CMAKE_CURRENT_SOURCE_DIR}/libltdl/lt__dirent.h
    ${CMAKE_CURRENT_SOURCE_DIR}/libltdl/lt__glibc.h
    ${CMAKE_CURRENT_SOURCE_DIR}/libltdl/lt__private.h
    ${CMAKE_CURRENT_SOURCE_DIR}/libltdl/lt__strl.h
    ${CMAKE_CURRENT_SOURCE_DIR}/libltdl/lt_dlloader.h
    ${CMAKE_CURRENT_SOURCE_DIR}/libltdl/lt_error.h
    ${CMAKE_CURRENT_SOURCE_DIR}/libltdl/lt_system.h
    ${CMAKE_CURRENT_SOURCE_DIR}/libltdl/slist.h
    ${CMAKE_CURRENT_SOURCE_DIR}/argz.h
    ${CMAKE_CURRENT_SOURCE_DIR}/ltdl.h
)

set(PLUGIN_SOURCES
#    ${CMAKE_CURRENT_SOURCE_DIR}/loaders/dld_link.c #linux?
#    ${CMAKE_CURRENT_SOURCE_DIR}/loaders/dlopen.c #linux? or we can use dlfcn
#    ${CMAKE_CURRENT_SOURCE_DIR}/loaders/dyld.c #OS-X
#    ${CMAKE_CURRENT_SOURCE_DIR}/loaders/load_add_on.c
    ${CMAKE_CURRENT_SOURCE_DIR}/loaders/loadlibrary.c #win
    ${CMAKE_CURRENT_SOURCE_DIR}/loaders/preopen.c
#    ${CMAKE_CURRENT_SOURCE_DIR}/loaders/shl_load.c #HP-UX
    ${CMAKE_CURRENT_SOURCE_DIR}/lt__alloc.c
    ${CMAKE_CURRENT_SOURCE_DIR}/lt__dirent.c
    ${CMAKE_CURRENT_SOURCE_DIR}/lt__strl.c
    ${CMAKE_CURRENT_SOURCE_DIR}/lt_dlloader.c
    ${CMAKE_CURRENT_SOURCE_DIR}/lt_error.c
    ${CMAKE_CURRENT_SOURCE_DIR}/argz.c
    ${CMAKE_CURRENT_SOURCE_DIR}/ltdl.c
    ${CMAKE_CURRENT_SOURCE_DIR}/slist.c
)

#Add version information to the plugIn-dll unter MSVC
if(MSVC)
#    list(APPEND PLUGIN_SOURCES ${ITOM_SDK_INCLUDE_DIR}/../pluginLibraryVersion.rc)
endif()

add_library(${target_name} ${LIBRARY_TYPE} ${PLUGIN_SOURCES} ${PLUGIN_HEADERS})

target_link_libraries(${target_name} ${VISUALLEAKDETECTOR_LIBRARIES}) # ../../libdl/$(Configuration)/dl)

#documentation
itom_configure_plugin_documentation(${target_name} libltdl)

# COPY SECTION
set(COPY_SOURCES "")
set(COPY_DESTINATIONS "")
LIST(APPEND COPY_SOURCES "$<TARGET_FILE:libltdl>")
LIST(APPEND COPY_DESTINATIONS "${ITOM_APP_DIR}/lib")
#itom_add_pluginlibrary_to_copy_list(${target_name} COPY_SOURCES COPY_DESTINATIONS)
itom_post_build_copy_files(${target_name} COPY_SOURCES COPY_DESTINATIONS)
