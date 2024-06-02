cmake_minimum_required(VERSION 3.12...3.29)
set(target_name Ximea)

option(BUILD_TARGET64 "Build for 64 bit target if set to ON or 32 bit if set to OFF." ON)
set(XIMEA_BUILD_API_VERSION "4.25 or newer" CACHE STRING "Expected API version")
set_property(CACHE XIMEA_BUILD_API_VERSION PROPERTY STRINGS "4.25 or newer" 4.25)
option(XIMEA_BUILD_INCLUDE_SHADING "Build with shading support from API-Version" OFF)

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

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR} ${ITOM_SDK_DIR}/cmake)

find_package(ITOM_SDK COMPONENTS dataobject itomCommonLib itomCommonQtLib itomWidgets REQUIRED)
find_package(OpenCV COMPONENTS core REQUIRED)
find_package(Ximea)

include(ItomBuildMacros)
itom_init_cmake_policy(3.12)
itom_init_plugin_library(${target_name}) #Start the project, init compiler settings and set default configurations for plugins
itom_find_package_qt(ON Core Widgets Xml LinguistTools)

if(XIMEA_SDK_FOUND)

    #if(XIMEA_BUILD_API_VERSION VERSION_EQUAL "4.25")
    #    message(STATUS "USE_API_4_10 defined")
    #    add_definitions(-DUSE_API_4_10)
    #endif(XIMEA_BUILD_API_VERSION VERSION_EQUAL "4.25")

    message(STATUS "Ximea used API VERSION: ${XIMEA_SDK_VERSION}" )

    if(XIMEA_BUILD_INCLUDE_SHADING)
        add_definitions(-DUSE_SHADING)
    endif(XIMEA_BUILD_INCLUDE_SHADING)

    include_directories(
        ${CMAKE_CURRENT_BINARY_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${ITOM_SDK_INCLUDE_DIRS}
        ${ITOM_SDK_INCLUDE_DIR}/itomWidgets
        ${XIMEA_SDK_INCLUDE_DIR}
    )

    link_directories( ${XIMEA_SDK_BINARY_DIR} )

    set(PLUGIN_HEADERS
        ${CMAKE_CURRENT_SOURCE_DIR}/dialogXimea.h
        ${CMAKE_CURRENT_SOURCE_DIR}/dockWidgetXimea.h
        ${CMAKE_CURRENT_SOURCE_DIR}/Ximea.h
        ${CMAKE_CURRENT_BINARY_DIR}/pluginVersion.h
        ${CMAKE_CURRENT_BINARY_DIR}/gitVersion.h
    )

    set(PLUGIN_UI
        ${CMAKE_CURRENT_SOURCE_DIR}/dialogXimea.ui
        ${CMAKE_CURRENT_SOURCE_DIR}/dockWidgetXimea.ui
    )

    set(PLUGIN_SOURCES
        ${CMAKE_CURRENT_SOURCE_DIR}/dialogXimea.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/dockWidgetXimea.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/Ximea.cpp
    )

    set(PLUGIN_RCC    )

    if(MSVC)
        list(APPEND PLUGIN_SOURCES ${ITOM_SDK_INCLUDE_DIR}/../pluginLibraryVersion.rc)
    endif()

    #handle translations BEGIN STEP 1
    set(FILES_TO_TRANSLATE ${PLUGIN_SOURCES} ${PLUGIN_HEADERS} ${PLUGIN_UI})
    set(TRANSLATION_OUTPUT_FILES)
    set(TRANSLATIONS_FILES)

    #Define Version in pluginVersion.h
    configure_file( ${CMAKE_CURRENT_SOURCE_DIR}/pluginVersion.h.in ${CMAKE_CURRENT_BINARY_DIR}/pluginVersion.h)

    add_library(${target_name} SHARED ${PLUGIN_SOURCES} ${PLUGIN_HEADERS} ${PLUGIN_UI} ${PLUGIN_RCC})

    # Qt: enable all automoc, autouic and autorcc.
    set_target_properties(${target_name} PROPERTIES AUTOMOC ON AUTORCC ON AUTOUIC ON)

    target_link_libraries(${target_name} ${QT5_LIBRARIES} ${OpenCV_LIBS} ${ITOM_SDK_LIBRARIES} ${VISUALLEAKDETECTOR_LIBRARIES})

    #translation
    set(FILES_TO_TRANSLATE ${PLUGIN_SOURCES} ${PLUGIN_HEADERS} ${PLUGIN_UI})
    itom_library_translation(QM_FILES TARGET ${target_name} FILES_TO_TRANSLATE ${FILES_TO_TRANSLATE})

    itom_configure_plugin_documentation(${target_name} ximea)

    # COPY SECTION
    set(COPY_SOURCES "")
    set(COPY_DESTINATIONS "")
    itom_add_pluginlibrary_to_copy_list(${target_name} COPY_SOURCES COPY_DESTINATIONS)
    itom_add_plugin_qm_files_to_copy_list(${target_name} QM_FILES COPY_SOURCES COPY_DESTINATIONS)
    itom_post_build_copy_files(${target_name} COPY_SOURCES COPY_DESTINATIONS)

if(WIN32)
    file(GLOB_RECURSE dlls "${XIMEA_SDK_BINARY_DIR}/*.dll")
    itom_post_build_copy_files_to_lib_folder(${target_name} dlls)
endif(WIN32)

else(XIMEA_SDK_FOUND)
    message(FATAL_ERROR "${target_name}-API directory could not be found. ${target_name} will not be build")
endif(XIMEA_SDK_FOUND)
