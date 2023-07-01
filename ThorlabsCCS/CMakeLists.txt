cmake_minimum_required(VERSION 3.12...3.24)

set(target_name ThorlabsCCS)

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

set(THORLABS_IVI_VISA_SDK "" CACHE PATH "base path to the sdk of IVI Foundation, VISA (e.g. folder C:/Program Files/IVI Foundation/VISA/Win64 or C:/Program Files (x86)/IVI Foundation/VISA/WinNT, having subfolders include, lib, lib_x64...)")

if(NOT THORLABS_IVI_VISA_SDK)
	set(THORLABS_IVI_VISA_SDK $ENV{NI_VISA_ROOT})
endif(NOT THORLABS_IVI_VISA_SDK)

find_path(THORLABS_IVI_VISA_INCLUDE_DIR "TLCCS.h" PATHS "${THORLABS_IVI_VISA_SDK}" PATH_SUFFIXES "include")

if(THORLABS_IVI_VISA_INCLUDE_DIR)

    set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR} ${ITOM_SDK_DIR}/cmake)

    find_package(ITOM_SDK COMPONENTS dataobject itomCommonLib itomCommonQtLib itomWidgets REQUIRED)

    include(ItomBuildMacros)
    itom_init_cmake_policy(3.12)
    itom_init_plugin_library(${target_name}) #Start the project, init compiler settings and set default configurations for plugins
    itom_find_package_qt(ON Core LinguistTools Widgets)

    set(THORLABS_IVI_VISA_LIBRARIES "")

    if(MSVC)
        if(BUILD_TARGET64)
            FIND_LIBRARY(THORLABS_IVI_VISA_LIB1 visa64 PATHS ${THORLABS_IVI_VISA_SDK} PATH_SUFFIXES Lib_x64/msc)
            FIND_LIBRARY(THORLABS_IVI_VISA_LIB2 TLCCS_64 PATHS ${THORLABS_IVI_VISA_SDK} PATH_SUFFIXES Lib_x64/msc)
        else(BUILD_TARGET64)
            FIND_LIBRARY(THORLABS_IVI_VISA_LIB1 visa32 PATHS ${THORLABS_IVI_VISA_SDK} PATH_SUFFIXES lib/msc)
            FIND_LIBRARY(THORLABS_IVI_VISA_LIB2 TLCCS_32 PATHS ${THORLABS_IVI_VISA_SDK} PATH_SUFFIXES lib/msc)
        endif(BUILD_TARGET64)

        set(THORLABS_IVI_VISA_LIBRARIES
            ${THORLABS_IVI_VISA_LIB1}
            ${THORLABS_IVI_VISA_LIB2}
        )
    endif()

endif(THORLABS_IVI_VISA_INCLUDE_DIR)

if(THORLABS_IVI_VISA_LIBRARIES)

    include_directories(
        ${CMAKE_CURRENT_BINARY_DIR} #build directory of this plugin (recommended)
        ${CMAKE_CURRENT_SOURCE_DIR} #source directory of this plugin (recommended)
        ${ITOM_SDK_INCLUDE_DIRS}    #include directory of the itom SDK (recommended)
        ${THORLABS_IVI_VISA_INCLUDE_DIR}
        #add further include directories here
    )

    link_directories(
        #add all linker directories
    )

    set(PLUGIN_HEADERS
        ${CMAKE_CURRENT_SOURCE_DIR}/thorlabsCCS.h
        ${CMAKE_CURRENT_SOURCE_DIR}/dialogThorlabsCCS.h
        ${CMAKE_CURRENT_SOURCE_DIR}/dockWidgetThorlabsCCS.h
        ${CMAKE_CURRENT_SOURCE_DIR}/pluginVersion.h
        ${CMAKE_CURRENT_BINARY_DIR}/gitVersion.h
        #add further header files (absolute pathes e.g. using CMAKE_CURRENT_SOURCE_DIR)
    )

    set(PLUGIN_SOURCES
        ${CMAKE_CURRENT_SOURCE_DIR}/thorlabsCCS.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/dialogThorlabsCCS.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/dockWidgetThorlabsCCS.cpp
        #add further source files here
    )

    #Append rc file to the source files for adding information about the plugin
    # to the properties of the DLL under Visual Studio.
    if(MSVC)
        list(APPEND PLUGIN_SOURCES ${ITOM_SDK_INCLUDE_DIR}/../pluginLibraryVersion.rc)
    endif()

    set(PLUGIN_UI
        ${CMAKE_CURRENT_SOURCE_DIR}/dialogThorlabsCCS.ui
        ${CMAKE_CURRENT_SOURCE_DIR}/dockWidgetThorlabsCCS.ui
    )

    set(PLUGIN_RCC
        #add absolute pathes to any *.qrc resource files here
    )

    add_library(${target_name} SHARED ${PLUGIN_SOURCES} ${PLUGIN_HEADERS} ${PLUGIN_UI} ${PLUGIN_RCC})

    set_target_properties(${target_name} PROPERTIES AUTOMOC ON AUTORCC ON AUTOUIC ON)

    target_link_libraries(${target_name} ${ITOM_SDK_LIBRARIES} ${QT5_LIBRARIES} ${VISUALLEAKDETECTOR_LIBRARIES} ${THORLABS_IVI_VISA_LIBRARIES})

    set(FILES_TO_TRANSLATE ${PLUGIN_SOURCES} ${PLUGIN_HEADERS} ${PLUGIN_UI})
    itom_library_translation(QM_FILES TARGET ${target_name} FILES_TO_TRANSLATE ${FILES_TO_TRANSLATE})

    itom_configure_plugin_documentation(${target_name} thorlabsCCS)

    set(COPY_SOURCES "")
    set(COPY_DESTINATIONS "")

    itom_add_pluginlibrary_to_copy_list(${target_name} COPY_SOURCES COPY_DESTINATIONS)
    itom_add_plugin_qm_files_to_copy_list(${target_name} QM_FILES COPY_SOURCES COPY_DESTINATIONS)

    itom_post_build_copy_files(${target_name} COPY_SOURCES COPY_DESTINATIONS)

else(THORLABS_IVI_VISA_LIBRARIES)
    message(SEND_ERROR "THORLABS_IVI_VISA_SDK directory could not be found. ${target_name} will not be build")
endif(THORLABS_IVI_VISA_LIBRARIES)
