set(target_name libgphoto2)

project(${target_name})

message(STATUS "\n--------------- PLUGIN ${target_name} ---------------")

cmake_minimum_required(VERSION 3.0.2)

option(BUILD_UNICODE "Build with unicode charset if set to ON, else multibyte charset." ON)
option(BUILD_SHARED_LIBS "Build shared library." ON)
option(BUILD_TARGET64 "Build for 64 bit target if set to ON or 32 bit if set to OFF." ON)
set(ITOM_SDK_DIR "" CACHE PATH "base path to itom_sdk")
set(CMAKE_DEBUG_POSTFIX "d" CACHE STRING "Adds a postfix for debug-built libraries.")

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR} ${ITOM_SDK_DIR})
set(PACKAGE_VERSION CACHE STRING "2.5.9")
set(CAMLIBS CACHE PATH "")

#if(BUILD_SHARED_LIBS)
#    set(LIBRARY_TYPE SHARED)
#else(BUILD_SHARED_LIBS)
#    set(LIBRARY_TYPE STATIC)
#endif(BUILD_SHARED_LIBS)
set(LIBRARY_TYPE STATIC)


include("${ITOM_SDK_DIR}/ItomBuildMacros.cmake")
INIT_ITOM_LIBRARY() #apply general project settings, CMake configurations, policies... for itom and its libraries
find_package(VisualLeakDetector QUIET)

if(BUILD_UNICODE)
    add_definitions(-DUNICODE -D_UNICODE)
endif(BUILD_UNICODE)
add_definitions(-DCMAKE)
add_definitions(-DPACKAGE_VERSION="${PACKAGE_VERSION}")
add_definitions(-DCAMLIBS="${CAMLIBS}")

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
    ${CMAKE_CURRENT_SOURCE_DIR}/../libtool
    ${CMAKE_CURRENT_SOURCE_DIR}/..
    ${CMAKE_CURRENT_SOURCE_DIR}/../libgphoto2_port
    ${VISUALLEAKDETECTOR_INCLUDE_DIR}
)

set(plugin_HEADERS
    ${CMAKE_CURRENT_SOURCE_DIR}/bayer.h
    ${CMAKE_CURRENT_SOURCE_DIR}/exif.h
    ${CMAKE_CURRENT_SOURCE_DIR}/gamma.h
    ${CMAKE_CURRENT_SOURCE_DIR}/jpeg.h
)

set(plugin_SOURCES 
    ${CMAKE_CURRENT_SOURCE_DIR}/ahd_bayer.c
    ${CMAKE_CURRENT_SOURCE_DIR}/bayer.c
    ${CMAKE_CURRENT_SOURCE_DIR}/exif.c
    ${CMAKE_CURRENT_SOURCE_DIR}/gamma.c
    ${CMAKE_CURRENT_SOURCE_DIR}/gphoto2-abilities-list.c
    ${CMAKE_CURRENT_SOURCE_DIR}/gphoto2-camera.c
    ${CMAKE_CURRENT_SOURCE_DIR}/gphoto2-context.c
    ${CMAKE_CURRENT_SOURCE_DIR}/gphoto2-file.c
    ${CMAKE_CURRENT_SOURCE_DIR}/gphoto2-filesys.c
    ${CMAKE_CURRENT_SOURCE_DIR}/gphoto2-library.c
    ${CMAKE_CURRENT_SOURCE_DIR}/gphoto2-list.c
    ${CMAKE_CURRENT_SOURCE_DIR}/gphoto2-result.c
    ${CMAKE_CURRENT_SOURCE_DIR}/gphoto2-setting.c
    ${CMAKE_CURRENT_SOURCE_DIR}/gphoto2-version.c
    ${CMAKE_CURRENT_SOURCE_DIR}/gphoto2-widget.c
    ${CMAKE_CURRENT_SOURCE_DIR}/jpeg.c
)

#Add version information to the plugIn-dll unter MSVC
if(MSVC)
#    list(APPEND plugin_SOURCES ${ITOM_SDK_INCLUDE_DIR}/../pluginLibraryVersion.rc)
endif()

add_library(${target_name} ${LIBRARY_TYPE} ${plugin_SOURCES} ${plugin_HEADERS})


target_link_libraries(${target_name} ${VISUALLEAKDETECTOR_LIBRARIES} 
    ../libgphoto2_port/$(Configuration)/libgphoto2_port 
    ../libtool/$(Configuration)/libltdl
    ../regex/$(Configuration)/regex)

if(MSVC)
#    list(APPEND plugin_SOURCES ${ITOM_SDK_INCLUDE_DIR}/../pluginLibraryVersion.rc)
#  set_target_properties(${target_name}  PROPERTIES COMPILE_FLAGS "/Zc:wchar_t")
endif()

#documentation
PLUGIN_DOCUMENTATION(${target_name} libgphoto2)

# COPY SECTION
set(COPY_SOURCES "")
set(COPY_DESTINATIONS "")
#LIST(APPEND COPY_SOURCES "$<TARGET_FILE:libgphoto2>")
#LIST(APPEND COPY_DESTINATIONS "${ITOM_APP_DIR}/lib")
#ADD_PLUGINLIBRARY_TO_COPY_LIST(${target_name} COPY_SOURCES COPY_DESTINATIONS)
#POST_BUILD_COPY_FILES(${target_name} COPY_SOURCES COPY_DESTINATIONS)
