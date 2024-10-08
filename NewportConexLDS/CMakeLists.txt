cmake_minimum_required(VERSION 3.12...3.29)

set(target_name NewportConexLDS)

if(NOT EXISTS ${ITOM_SDK_DIR})
    find_path(ITOM_SDK_DIR "cmake/itom_sdk.cmake"
    HINTS "$ENV{ITOM_SDK_ROOT}"
          "${CMAKE_CURRENT_BINARY_DIR}/../itom/SDK"
    DOC "Path of SDK subfolder of itom root (build) directory")
endif(NOT EXISTS ${ITOM_SDK_DIR})

if(NOT EXISTS ${ITOM_SDK_DIR})
    message(FATAL_ERROR "ITOM_SDK_DIR is invalid. Provide itom SDK directory path first")
endif(NOT EXISTS ${ITOM_SDK_DIR})

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR} ${ITOM_SDK_DIR}/cmake)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

option(BUILD_TARGET64 "Build for 64 bit target if set to ON or 32 bit if set to OFF." ON)

find_package(ITOM_SDK COMPONENTS dataobject itomCommonLib itomCommonQtLib itomWidgets REQUIRED)
include(ItomBuildMacros)

itom_init_cmake_policy(3.12)
itom_find_package_qt(ON Core Widgets LinguistTools)
itom_init_plugin_library(${target_name})

include_directories(
    ${CMAKE_CURRENT_BINARY_DIR} #build directory of this plugin (recommended)
    ${CMAKE_CURRENT_SOURCE_DIR} #source directory of this plugin (recommended)
    ${ITOM_SDK_INCLUDE_DIRS}    #include directory of the itom SDK (recommended) as well as necessary 3rd party
)

link_directories(
)

set(PLUGIN_HEADERS
    ${CMAKE_CURRENT_SOURCE_DIR}/NewportConexLDS.h
    ${CMAKE_CURRENT_SOURCE_DIR}/dialogNewportConexLDS.h
    ${CMAKE_CURRENT_SOURCE_DIR}/dockWidgetNewportConexLDS.h
    ${CMAKE_CURRENT_BINARY_DIR}/pluginVersion.h
    ${CMAKE_CURRENT_BINARY_DIR}/gitVersion.h
)

set(PLUGIN_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/dialogNewportConexLDS.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/dockWidgetNewportConexLDS.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/NewportConexLDS.cpp
)

#Define Version in pluginVersion.h
configure_file( ${CMAKE_CURRENT_SOURCE_DIR}/pluginVersion.h.in ${CMAKE_CURRENT_BINARY_DIR}/pluginVersion.h)

if(MSVC)
    list(APPEND PLUGIN_SOURCES ${ITOM_SDK_INCLUDE_DIR}/../pluginLibraryVersion.rc)
endif()

set(PLUGIN_UI
    ${CMAKE_CURRENT_SOURCE_DIR}/dialogNewportConexLDS.ui
    ${CMAKE_CURRENT_SOURCE_DIR}/dockWidgetNewportConexLDS.ui
)

set(PLUGIN_RCC
)

add_library(${target_name} SHARED
    ${PLUGIN_SOURCES}
    ${PLUGIN_HEADERS}
    ${PLUGIN_UI}
    ${PLUGIN_RCC}
    )

set_target_properties(${target_name} PROPERTIES AUTOMOC ON AUTORCC ON AUTOUIC ON)

target_link_libraries(${target_name} ${ITOM_SDK_LIBRARIES} ${QT5_LIBRARIES} ${VISUALLEAKDETECTOR_LIBRARIES})

set(FILES_TO_TRANSLATE ${PLUGIN_SOURCES} ${PLUGIN_HEADERS} ${PLUGIN_UI} ${PLUGIN_RCC})
itom_library_translation(
    QM_FILES
    TARGET ${target_name}
    FILES_TO_TRANSLATE ${FILES_TO_TRANSLATE}
    )

itom_configure_plugin_documentation(${target_name} NewportConexLDS)

set(COPY_SOURCES "")
set(COPY_DESTINATIONS "")

itom_add_pluginlibrary_to_copy_list(${target_name} COPY_SOURCES COPY_DESTINATIONS)
itom_add_plugin_qm_files_to_copy_list(${target_name} QM_FILES COPY_SOURCES COPY_DESTINATIONS)

itom_post_build_copy_files(${target_name} COPY_SOURCES COPY_DESTINATIONS)
