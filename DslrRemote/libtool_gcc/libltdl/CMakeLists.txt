set(target_name libltdl)

project(${target_name})

message(STATUS "\n--------------- PLUGIN ${target_name} ---------------")

cmake_minimum_required(VERSION 3.0.2)

#option(BUILD_UNICODE "Build with unicode charset if set to ON, else multibyte charset." ON)
option(BUILD_SHARED_LIBS "Build shared library." ON)
option(BUILD_TARGET64 "Build for 64 bit target if set to ON or 32 bit if set to OFF." ON)
set(ITOM_SDK_DIR "" CACHE PATH "base path to itom_sdk")
set(CMAKE_DEBUG_POSTFIX "d" CACHE STRING "Adds a postfix for debug-built libraries.")

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR} ${ITOM_SDK_DIR})
set(LT_OBJDIR CACHE STRING "")

if(BUILD_SHARED_LIBS)
    set(LIBRARY_TYPE SHARED)
else(BUILD_SHARED_LIBS)
    set(LIBRARY_TYPE STATIC)
endif(BUILD_SHARED_LIBS)


include("${ITOM_SDK_DIR}/ItomBuildMacros.cmake")
INIT_ITOM_LIBRARY() #apply general project settings, CMake configurations, policies... for itom and its libraries
find_package(VisualLeakDetector QUIET)

#if(BUILD_UNICODE)
#    add_definitions(-DUNICODE -D_UNICODE)
#endif(BUILD_UNICODE)
remove_definitions(-DUNICODE -D_UNICODE)
add_definitions(-DCMAKE)
add_definitions(-DLTDL)
add_definitions(-DLT_OBJDIR="${LT_OBJDIR}")
add_definitions(-DDLL_EXPORT)

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
    ${CMAKE_CURRENT_SOURCE_DIR}/libltdl
    ${VISUALLEAKDETECTOR_INCLUDE_DIR}
)

set(plugin_HEADERS
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

set(plugin_SOURCES 
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
#    list(APPEND plugin_SOURCES ${ITOM_SDK_INCLUDE_DIR}/../pluginLibraryVersion.rc)
endif()

add_library(${target_name} ${LIBRARY_TYPE} ${plugin_SOURCES} ${plugin_HEADERS})
target_link_libraries(${target_name} ${VISUALLEAKDETECTOR_LIBRARIES}) # ../../libdl/$(Configuration)/dl)

#documentation
PLUGIN_DOCUMENTATION(${target_name} libltdl)

# COPY SECTION
set(COPY_SOURCES "")
set(COPY_DESTINATIONS "")
LIST(APPEND COPY_SOURCES "$<TARGET_FILE:libltdl>")
LIST(APPEND COPY_DESTINATIONS "${ITOM_APP_DIR}/lib")
#ADD_PLUGINLIBRARY_TO_COPY_LIST(${target_name} COPY_SOURCES COPY_DESTINATIONS)
POST_BUILD_COPY_FILES(${target_name} COPY_SOURCES COPY_DESTINATIONS)
