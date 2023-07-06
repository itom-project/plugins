cmake_minimum_required(VERSION 3.12...3.24)

set(target_name Xeneth)

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

set(XENETH_SDK_DIR "" CACHE PATH "base path to the Xeneth SDK (that contains the include folder)")

if(NOT XENETH_SDK_DIR)
	set(XENETH_SDK_DIR $ENV{XENETH_ROOT})
endif(NOT XENETH_SDK_DIR)

find_path(XENETH_INCLUDE_DIR XCamera.h PATHS ${XENETH_SDK_DIR} PATH_SUFFIXES include)

if(XENETH_INCLUDE_DIR)

    set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR} ${ITOM_SDK_DIR}/cmake)

    find_package(ITOM_SDK COMPONENTS dataobject itomCommonLib itomCommonQtLib itomWidgets REQUIRED)
    find_package(OpenCV COMPONENTS core REQUIRED) #if you require openCV indicate all components that are required (e.g. core, imgproc...)

    include(ItomBuildMacros)
    itom_init_cmake_policy(3.12)
    itom_init_plugin_library(${target_name}) #Start the project, init compiler settings and set default configurations for plugins
    itom_find_package_qt(ON Core LinguistTools Widgets)

    if(BUILD_TARGET64)
        set( XENETH_LIBRARY ${XENETH_SDK_DIR}/Lib/xeneth64.lib )
    else(BUILD_TARGET64)
        set( XENETH_LIBRARY ${XENETH_SDK_DIR}/Lib/xeneth.lib )
    endif(BUILD_TARGET64)

    include_directories(
        ${CMAKE_CURRENT_BINARY_DIR} #build directory of this plugin (recommended)
        ${CMAKE_CURRENT_SOURCE_DIR} #source directory of this plugin (recommended)
        ${ITOM_SDK_INCLUDE_DIRS}     #include directory of the itom SDK (recommended)
        ${XENETH_INCLUDE_DIR}
    )

    link_directories(
        #add all linker directories
    )

    set(PLUGIN_HEADERS
        ${CMAKE_CURRENT_SOURCE_DIR}/DialogXeneth.h
        ${CMAKE_CURRENT_SOURCE_DIR}/DockWidgetXeneth.h
        ${CMAKE_CURRENT_SOURCE_DIR}/Xeneth.h
        ${CMAKE_CURRENT_SOURCE_DIR}/XenethInterface.h
        ${CMAKE_CURRENT_SOURCE_DIR}/pluginVersion.h
        ${CMAKE_CURRENT_BINARY_DIR}/gitVersion.h
        #add further header files (absolute pathes e.g. using CMAKE_CURRENT_SOURCE_DIR)
    )

    set(PLUGIN_SOURCES
        ${CMAKE_CURRENT_SOURCE_DIR}/dialogXeneth.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/dockWidgetXeneth.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/Xeneth.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/XenethInterface.cpp
        #add further source files here
    )

    #Append rc file to the source files for adding information about the plugin
    # to the properties of the DLL under Visual Studio.
    if(MSVC)
        list(APPEND PLUGIN_SOURCES ${ITOM_SDK_INCLUDE_DIR}/../pluginLibraryVersion.rc)
    endif()

    set(PLUGIN_UI
        ${CMAKE_CURRENT_SOURCE_DIR}/dialogXeneth.ui
        ${CMAKE_CURRENT_SOURCE_DIR}/dockWidgetXeneth.ui
    )

    set(PLUGIN_RCC
        #add absolute pathes to any *.qrc resource files here
    )

    add_library(${target_name} SHARED ${PLUGIN_SOURCES} ${PLUGIN_HEADERS} ${PLUGIN_UI} ${PLUGIN_RCC})

    set_target_properties(${target_name} PROPERTIES AUTOMOC ON AUTORCC ON AUTOUIC ON)

    target_link_libraries(${target_name} ${ITOM_SDK_LIBRARIES} ${QT5_LIBRARIES} ${VISUALLEAKDETECTOR_LIBRARIES} ${XENETH_LIBRARY})

    set(FILES_TO_TRANSLATE ${PLUGIN_SOURCES} ${PLUGIN_HEADERS} ${PLUGIN_UI})
    itom_library_translation(QM_FILES TARGET ${target_name} FILES_TO_TRANSLATE ${FILES_TO_TRANSLATE})

    itom_configure_plugin_documentation(${target_name} xeneth)

    set(COPY_SOURCES "")
    set(COPY_DESTINATIONS "")

    itom_add_pluginlibrary_to_copy_list(${target_name} COPY_SOURCES COPY_DESTINATIONS)
    itom_add_plugin_qm_files_to_copy_list(${target_name} QM_FILES COPY_SOURCES COPY_DESTINATIONS)

    itom_post_build_copy_files(${target_name} COPY_SOURCES COPY_DESTINATIONS)

else(XENETH_INCLUDE_DIR)
    message(SEND_ERROR "XCamera.h for plugin ${target_name} could not be found. ${target_name} will not be build.")
endif(XENETH_INCLUDE_DIR)
