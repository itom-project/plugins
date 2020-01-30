set(target_name dcraw)

project(${target_name})

message(STATUS "\n--------------- PLUGIN ${target_name} ---------------")

cmake_minimum_required(VERSION 3.0.2)

option(BUILD_TARGET64 "Build for 64 bit target if set to ON or 32 bit if set to OFF." ON)
option(UPDATE_TRANSLATIONS "Update source translation translation/*.ts files (WARNING: make clean will delete the source .ts files! Danger!)")

set(ITOM_SDK_DIR "" CACHE PATH "base path to itom_sdk")
set(CMAKE_DEBUG_POSTFIX "d" CACHE STRING "Adds a postfix for debug-built libraries.")
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR} ${ITOM_SDK_DIR})
set(CMAKE_INCLUDE_CURRENT_DIR ON)

find_package(VisualLeakDetector QUIET)

add_definitions(-DUNICODE -D_UNICODE)

if(VISUALLEAKDETECTOR_FOUND AND VISUALLEAKDETECTOR_ENABLED)
    add_definitions(-DVISUAL_LEAK_DETECTOR_CMAKE)
endif(VISUALLEAKDETECTOR_FOUND AND VISUALLEAKDETECTOR_ENABLED)


endif()


include_directories(
    ${CMAKE_CURRENT_BINARY_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${VISUALLEAKDETECTOR_INCLUDE_DIR}
)

set(plugin_HEADERS
    ${CMAKE_CURRENT_SOURCE_DIR}/dcraw.h
)

set(plugin_SOURCES 
    ${CMAKE_CURRENT_SOURCE_DIR}/dcraw.c
)

#Add version information to the plugIn-dll unter MSVC
#if(MSVC)
#    list(APPEND plugin_SOURCES ${ITOM_SDK_INCLUDE_DIR}/../pluginLibraryVersion.rc)
#endif()

add_executable(${target_name} ${plugin_SOURCES} ${plugin_HEADERS})
if(WIN32)
    target_link_libraries(${target_name} ${VISUALLEAKDETECTOR_LIBRARIES})
else(WIN32)
    target_link_libraries(${target_name} m ${VISUALLEAKDETECTOR_LIBRARIES})
endif(WIN32)

# COPY SECTION
set(COPY_SOURCES "")
set(COPY_DESTINATIONS "")
#ADD_PLUGINLIBRARY_TO_COPY_LIST(${target_name} COPY_SOURCES COPY_DESTINATIONS)
#POST_BUILD_COPY_FILES(${target_name} COPY_SOURCES COPY_DESTINATIONS)
