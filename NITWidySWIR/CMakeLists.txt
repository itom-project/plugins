cmake_minimum_required(VERSION 3.12...3.24)

set(target_name NITWidySWIR) #->replace NITWidySWIR by the name of your plugin (one word)

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

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR} ${ITOM_SDK_DIR}/cmake)

find_package(ITOM_SDK COMPONENTS dataobject itomCommonLib itomCommonQtLib itomWidgets REQUIRED)
find_package(OpenCV COMPONENTS core imgproc REQUIRED)

include(ItomBuildMacros)
itom_init_cmake_policy(3.12)
itom_init_plugin_library(${target_name}) #Start the project, init compiler settings and set default configurations for plugins
itom_find_package_qt(ON Core LinguistTools Widgets)

set(NITLIBRARY_SDK_DIR "" CACHE PATH "path of the NITLIBRARY SDK. Must contain subfolders like bin, bin64, include, lib, lib64")

if(NOT NITLIBRARY_SDK_DIR)
	set(NITLIBRARY_SDK_DIR $ENV{NIT_SDK_ROOT})
endif(NOT NITLIBRARY_SDK_DIR)

find_path(NITLIBRARY_INCLUDE_DIR NITManager.h PATHS ${NITLIBRARY_SDK_DIR} PATH_SUFFIXES /include /inc DOC "NITLibrary include directory")

if(NITLIBRARY_INCLUDE_DIR)

    include_directories(
        ${CMAKE_CURRENT_BINARY_DIR} #build directory of this plugin (recommended)
        ${CMAKE_CURRENT_SOURCE_DIR} #source directory of this plugin (recommended)
        ${ITOM_SDK_INCLUDE_DIRS}    #include directory of the itom SDK (recommended)
        ${ITOM_SDK_INCLUDE_DIR}/itomWidgets
        ${NITLIBRARY_INCLUDE_DIR}
        }
        #add further include directories here
    )

    if(BUILD_TARGET64)
		find_library(NITLIBRARY_LIB NITLibrary NITLabView HINTS ${NITLIBRARY_SDK_DIR} PATH_SUFFIXES /lib/x64)
    else(BUILD_TARGET64)
        find_library(NITLIBRARY_LIB NITLibrary NITLabView HINTS ${NITLIBRARY_SDK_DIR} PATH_SUFFIXES /lib/x86)
    endif(BUILD_TARGET64)

    set(PLUGIN_HEADERS
        ${CMAKE_CURRENT_SOURCE_DIR}/NITWidySWIR.h
        ${CMAKE_CURRENT_SOURCE_DIR}/dialogNITWidySWIR.h
        ${CMAKE_CURRENT_SOURCE_DIR}/dockWidgetNITWidySWIR.h
        ${CMAKE_CURRENT_SOURCE_DIR}/pluginVersion.h
        ${CMAKE_CURRENT_BINARY_DIR}/gitVersion.h
        #add further header files (absolute pathes e.g. using CMAKE_CURRENT_SOURCE_DIR)
    )

    set(PLUGIN_SOURCES
        ${CMAKE_CURRENT_SOURCE_DIR}/dialogNITWidySWIR.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/dockWidgetNITWidySWIR.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/NITWidySWIR.cpp
        #add further source files here
    )

    if(MSVC)
        list(APPEND PLUGIN_SOURCES ${ITOM_SDK_INCLUDE_DIR}/../pluginLibraryVersion.rc)
    endif()

    set(PLUGIN_UI
        ${CMAKE_CURRENT_SOURCE_DIR}/dialogNITWidySWIR.ui
        ${CMAKE_CURRENT_SOURCE_DIR}/dockWidgetNITWidySWIR.ui
    )

    set(PLUGIN_RCC
        #add absolute pathes to any *.qrc resource files here
    )

    add_library(${target_name} SHARED ${PLUGIN_SOURCES} ${PLUGIN_HEADERS} ${PLUGIN_UI} ${PLUGIN_RCC})

    set_target_properties(${target_name} PROPERTIES AUTOMOC ON AUTORCC ON AUTOUIC ON)

    target_link_libraries(${target_name} ${ITOM_SDK_LIBRARIES} ${QT5_LIBRARIES} ${VISUALLEAKDETECTOR_LIBRARIES} ${NITLIBRARY_LIB} ${OPENCV240_LIB_DIR})

    set_target_properties(${target_name} PROPERTIES LINK_FLAGS "/DELAYLOAD:NITLibrary.dll")

    set(FILES_TO_TRANSLATE ${PLUGIN_SOURCES} ${PLUGIN_HEADERS} ${PLUGIN_UI})
    itom_library_translation(QM_FILES TARGET ${target_name} FILES_TO_TRANSLATE ${FILES_TO_TRANSLATE})

    itom_configure_plugin_documentation(${target_name} NITWidySWIR)

    set(COPY_SOURCES "")
    set(COPY_DESTINATIONS "")

    itom_add_pluginlibrary_to_copy_list(${target_name} COPY_SOURCES COPY_DESTINATIONS)
    itom_add_plugin_qm_files_to_copy_list(${target_name} QM_FILES COPY_SOURCES COPY_DESTINATIONS)

    itom_post_build_copy_files(${target_name} COPY_SOURCES COPY_DESTINATIONS)

    if(BUILD_TARGET64)
        file(GLOB NITLIBRARY_DLL "${NITLIBRARY_SDK_DIR}/bin/x64/*.dll")
    else(BUILD_TARGET64)
        file(GLOB NITLIBRARY_DLL "${NITLIBRARY_SDK_DIR}/bin/x86/*.dll")
    endif(BUILD_TARGET64)

    itom_post_build_copy_files_to_lib_folder(${target_name} NITLIBRARY_DLL)

else(NITLIBRARY_INCLUDE_DIR)
    message(SEND_ERROR "${target_name}-API directory could not be found. ${target_name} will not be build.")
endif(NITLIBRARY_INCLUDE_DIR)
