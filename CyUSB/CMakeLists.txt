cmake_minimum_required(VERSION 3.12...3.29)
set(target_name CyUSB)

project(${target_name})

message(STATUS "----")

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

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR} ${ITOM_SDK_DIR}/cmake)

find_package(ITOM_SDK COMPONENTS dataobject itomCommonLib itomCommonQtLib itomWidgets REQUIRED)
find_package(OpenCV COMPONENTS core REQUIRED)
find_package(CyAPI QUIET)

include(ItomBuildMacros)
itom_init_cmake_policy(3.12)
itom_init_plugin_library(${target_name}) #Start the project, init compiler settings and set default configurations for plugins
itom_find_package_qt(ON Widgets Xml LinguistTools)

if(CYAPI_INCLUDE_DIRS)

    include_directories(
        ${CMAKE_CURRENT_BINARY_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${ITOM_SDK_INCLUDE_DIRS}
        ${CYAPI_INCLUDE_DIRS}
    )

    set(PLUGIN_HEADERS
        ${CMAKE_CURRENT_SOURCE_DIR}/ItomCyUSB.h
        ${CMAKE_CURRENT_SOURCE_DIR}/dockWidgetItomCyUSB.h
        ${CMAKE_CURRENT_BINARY_DIR}/pluginVersion.h
        ${CMAKE_CURRENT_BINARY_DIR}/gitVersion.h
    )

    set(PLUGIN_UI
        ${CMAKE_CURRENT_SOURCE_DIR}/dockWidgetItomCyUSB.ui
    )

    set(PLUGIN_SOURCES
        ${CMAKE_CURRENT_SOURCE_DIR}/ItomCyUSB.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/dockWidgetItomCyUSB.cpp
    )

	#Define Version in pluginVersion.h
	configure_file( ${CMAKE_CURRENT_SOURCE_DIR}/pluginVersion.h.in ${CMAKE_CURRENT_BINARY_DIR}/pluginVersion.h)

    set(PLUGIN_RCC
    )

    link_directories(${CYAPI_LIB_DIRS})

    #Add version information to the plugIn-dll under MSVC
    if(MSVC)
        list(APPEND PLUGIN_SOURCES ${ITOM_SDK_INCLUDE_DIR}/../pluginLibraryVersion.rc)
    endif()

    add_library(${target_name} SHARED ${PLUGIN_SOURCES} ${PLUGIN_HEADERS} ${PLUGIN_UI} ${PLUGIN_RCC})

    add_dependencies(${target_name} LibUSB)

    # Qt: enable all automoc, autouic and autorcc.
    set_target_properties(${target_name} PROPERTIES AUTOMOC ON AUTORCC ON AUTOUIC ON)

    if(MSVC)
        if(MSVC_VERSION GREATER 1800)
            target_link_libraries(${target_name} ${ITOM_SDK_LIBRARIES} ${CYAPI_LIB_DIRS} ${QT5_LIBRARIES} ${VISUALLEAKDETECTOR_LIBRARIES} setupapi legacy_stdio_definitions)
        else(MSVC_VERSION GREATER 1800)
            target_link_libraries(${target_name} ${ITOM_SDK_LIBRARIES} ${CYAPI_LIB_DIRS} ${QT5_LIBRARIES} ${VISUALLEAKDETECTOR_LIBRARIES} setupapi)
        endif(MSVC_VERSION GREATER 1800)
    else()
        target_link_libraries(${target_name} ${ITOM_SDK_LIBRARIES} ${CYAPI_LIB_DIRS} ${QT5_LIBRARIES} ${VISUALLEAKDETECTOR_LIBRARIES} setupapi)
    endif()

    #translation
    set(FILES_TO_TRANSLATE ${PLUGIN_SOURCES} ${PLUGIN_HEADERS} ${PLUGIN_UI})
    itom_library_translation(QM_FILES TARGET ${target_name} FILES_TO_TRANSLATE ${FILES_TO_TRANSLATE})

    itom_configure_plugin_documentation(${target_name} CyUSB)

    # COPY SECTION
    set(COPY_SOURCES "")
    set(COPY_DESTINATIONS "")
    itom_add_pluginlibrary_to_copy_list(${target_name} COPY_SOURCES COPY_DESTINATIONS)
    itom_add_plugin_qm_files_to_copy_list(${target_name} QM_FILES COPY_SOURCES COPY_DESTINATIONS)
    itom_post_build_copy_files(${target_name} COPY_SOURCES COPY_DESTINATIONS)
    #itom_post_build_copy_files_to_lib_folder(${target_name} CyAPI_BIN_DIR)

else(CYAPI_INCLUDE_DIRS)
    message(SEND_ERROR "${target_name}-API directory could not be found. ${target_name} will not be build")
endif(CYAPI_INCLUDE_DIRS)
