cmake_minimum_required(VERSION 3.12...3.29)
set(target_name Newport2936) #->replace MyGrabber by the name of your plugin (one word)

message(STATUS "Project ${target_name} (${CMAKE_CURRENT_BINARY_DIR})")

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

include(ItomBuildMacros)
itom_init_cmake_policy(3.12)
itom_init_plugin_library(${target_name}) #Start the project, init compiler settings and set default configurations for plugins

set(NEWPORT_POWER_METER_DIR "" CACHE PATH "path to the install directory of Newport Power Meter Application (e.g. C:/Program Files/Newport/Newport Power Meter Application")
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR} ${ITOM_SDK_DIR}/cmake)

if(WIN32)
    find_path(NEWPORT_NEWPDLL_INCLUDE_DIR "NewpDll.h" PATHS ${NEWPORT_POWER_METER_DIR} $ENV{NEWPORT_2936_ROOT} PATH_SUFFIXES /Samples/C++/UseDeviceKey/UseDeviceKey)
endif(WIN32)

find_library(NEWPORT_USBDLL_LIBRARY "usbdll" HINTS ${NEWPORT_POWER_METER_DIR} $ENV{NEWPORT_2936_ROOT} PATH_SUFFIXES "Bin")

if(NEWPORT_NEWPDLL_INCLUDE_DIR)
    itom_find_package_qt(ON Core Widgets LinguistTools)

    include_directories(
        ${CMAKE_CURRENT_BINARY_DIR} #build directory of this plugin (recommended)
        ${CMAKE_CURRENT_SOURCE_DIR} #source directory of this plugin (recommended)
        ${ITOM_SDK_INCLUDE_DIRS}    #include directory of the itom SDK (recommended)
        ${NEWPORT_NEWPDLL_INCLUDE_DIR}
    )

    link_directories(
        #add all linker directories
    )

    set(PLUGIN_HEADERS
        ${CMAKE_CURRENT_SOURCE_DIR}/newport2936.h
        ${CMAKE_CURRENT_SOURCE_DIR}/dialogNewport2936.h
        ${CMAKE_CURRENT_SOURCE_DIR}/dockWidgetNewport2936.h
        ${CMAKE_CURRENT_BINARY_DIR}/pluginVersion.h
        ${CMAKE_CURRENT_BINARY_DIR}/gitVersion.h
        #add further header files (absolute paths e.g. using CMAKE_CURRENT_SOURCE_DIR)
    )

    set(PLUGIN_SOURCES
        ${CMAKE_CURRENT_SOURCE_DIR}/dialogNewport2936.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/dockWidgetNewport2936.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/newport2936.cpp
        #add further source files here
    )

    #Define Version in pluginVersion.h
    configure_file( ${CMAKE_CURRENT_SOURCE_DIR}/pluginVersion.h.in ${CMAKE_CURRENT_BINARY_DIR}/pluginVersion.h)

    if(MSVC)
        list(APPEND PLUGIN_SOURCES ${ITOM_SDK_INCLUDE_DIR}/../pluginLibraryVersion.rc)
    endif()

    set(PLUGIN_UI
        ${CMAKE_CURRENT_SOURCE_DIR}/dialogNewport2936.ui
        ${CMAKE_CURRENT_SOURCE_DIR}/dockWidgetNewport2936.ui
    )

    set(PLUGIN_RCC
        #add absolute paths to any *.qrc resource files here
    )

    add_library(${target_name} SHARED ${PLUGIN_SOURCES} ${PLUGIN_HEADERS} ${PLUGIN_UI} ${PLUGIN_RCC})

    set_target_properties(${target_name} PROPERTIES AUTOMOC ON AUTORCC ON AUTOUIC ON)

    target_link_libraries(${target_name} ${ITOM_SDK_LIBRARIES} ${QT5_LIBRARIES} ${VISUALLEAKDETECTOR_LIBRARIES} ${NEWPORT_USBDLL_LIBRARY})

    set(FILES_TO_TRANSLATE ${PLUGIN_SOURCES} ${PLUGIN_HEADERS} ${PLUGIN_UI})
    itom_library_translation(QM_FILES TARGET ${target_name} FILES_TO_TRANSLATE ${FILES_TO_TRANSLATE})

        itom_configure_plugin_documentation(${target_name} newport2936)

    set(COPY_SOURCES "")
    set(COPY_DESTINATIONS "")

    itom_add_pluginlibrary_to_copy_list(${target_name} COPY_SOURCES COPY_DESTINATIONS)
    itom_add_plugin_qm_files_to_copy_list(${target_name} QM_FILES COPY_SOURCES COPY_DESTINATIONS)

    itom_post_build_copy_files(${target_name} COPY_SOURCES COPY_DESTINATIONS)

else(NEWPORT_NEWPDLL_INCLUDE_DIR)
    message(SEND_ERROR "NewpDll.h for plugin ${target_name} could not be found. ${target_name} will not be build. Please properly indicate Newport_NEWPDLL_INCLUDE_DIR  or set Environment Variable NEWPORT2936_ROOT.")
endif(NEWPORT_NEWPDLL_INCLUDE_DIR)
