cmake_minimum_required(VERSION 3.12...3.29)
set(target_name DslrRemote2)

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
set(CMAKE_INCLUDE_CURRENT_DIR ON)

find_package(ITOM_SDK COMPONENTS dataobject itomCommonLib itomCommonQtLib REQUIRED)
find_package(LibUSB QUIET)

include(ItomBuildMacros)
itom_init_cmake_policy(3.12)
itom_init_plugin_library(${target_name}) #Start the project, init compiler settings and set default configurations for plugins
itom_find_package_qt(ON Core Widgets LinguistTools)

add_subdirectory(libptp2-1.2.0)

link_directories(
    ${LIBUSB_API_DIR}/lib
)

include_directories(
    ${CMAKE_CURRENT_BINARY_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/libptp2
    ${LibUSB_INCLUDE_DIRS}
    ${ITOM_SDK_INCLUDE_DIRS}
    )

set(PLUGIN_HEADERS
    ${CMAKE_CURRENT_BINARY_DIR}/pluginVersion.h
    ${CMAKE_CURRENT_BINARY_DIR}/gitVersion.h
    ${CMAKE_CURRENT_SOURCE_DIR}/DslrRemote.h
    ${CMAKE_CURRENT_SOURCE_DIR}/ptpCam.h
)

set(PLUGIN_UI
)

set(PLUGIN_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/ptpCam.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/DslrRemote.cpp
)

#Define Version in pluginVersion.h
configure_file( ${CMAKE_CURRENT_SOURCE_DIR}/pluginVersion.h.in ${CMAKE_CURRENT_BINARY_DIR}/pluginVersion.h)

set(PLUGIN_RCC
    #add absolute paths to any *.qrc resource files here
)

#Add version information to the plugIn-dll under MSVC
if(MSVC)
    list(APPEND PLUGIN_SOURCES ${ITOM_SDK_INCLUDE_DIR}/../pluginLibraryVersion.rc)
endif()

if(WIN32)
    set(PTPLIB optimized ./libptp2-1.2.0/$(Configuration)/libptp2 debug ./libptp2-1.2.0/$(Configuration)/libptp2d)
else(WIN32)
    set(PTPLIB libptp2)
endif(WIN32)

add_library(${target_name} SHARED ${PLUGIN_SOURCES} ${PLUGIN_HEADERS})

add_dependencies(${target_name} libptp2)
add_dependencies(${target_name} LibUSB)

# Qt: enable all automoc, autouic and autorcc.
set_target_properties(${target_name} PROPERTIES AUTOMOC ON AUTORCC ON AUTOUIC ON)

target_link_libraries(${target_name} ${ITOM_SDK_LIBRARIES} ${VISUALLEAKDETECTOR_LIBRARIES} ${QT5_LIBRARIES} ${QT_LIBRARIES} ${PTPLIB} ${LibUSB_LIBRARY})

#translation
set(FILES_TO_TRANSLATE ${PLUGIN_SOURCES} ${PLUGIN_HEADERS} ${PLUGIN_UI})
itom_library_translation(QM_FILES TARGET ${target_name} FILES_TO_TRANSLATE ${FILES_TO_TRANSLATE})

#documentation
itom_configure_plugin_documentation(${target_name} dslrRemote)

# COPY SECTION
set(COPY_SOURCES "")
set(COPY_DESTINATIONS "")
#LIST(APPEND COPY_SOURCES "$<TARGET_FILE:dcraw>")
#LIST(APPEND COPY_DESTINATIONS "${ITOM_APP_DIR}/lib")
itom_add_pluginlibrary_to_copy_list(${target_name} COPY_SOURCES COPY_DESTINATIONS)
itom_add_plugin_qm_files_to_copy_list(${target_name} QM_FILES COPY_SOURCES COPY_DESTINATIONS)
itom_post_build_copy_files(${target_name} COPY_SOURCES COPY_DESTINATIONS)
