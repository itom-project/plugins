cmake_minimum_required(VERSION 3.12...3.29)

set(target_name ThorlabsDMH)

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

set(THORLABS_DFM_VISA_DIRECTORY "" CACHE PATH "Path to the install directory of Thorlabs Deformable Mirror VISA (e.g. C:/Program Files/IVI Foundation/VISA")
set(THORLABS_DFM_VISA_x86_DIRECTORY "" CACHE PATH "Path to the install directory of Thorlabs Deformable Mirror VISA x86 (e.g. C:/Program Files (x86)/IVI Foundation/VISA")

if(NOT THORLABS_DFM_VISA_DIRECTORY AND THORLABS_DFM_VISA_x86_DIRECTORY)
    set(THORLABS_DFM_VISA_DIRECTORY $ENV{NI_VISA_ROOT})
endif(NOT THORLABS_DFM_VISA_DIRECTORY AND THORLABS_DFM_VISA_x86_DIRECTORY)

if(THORLABS_DFM_VISA_DIRECTORY AND THORLABS_DFM_VISA_x86_DIRECTORY)
    if(WIN32)
        find_path(THORLABS_DFMX_INCLUDE_DIR "TLDFMX_def.h" PATHS ${THORLABS_DFM_VISA_DIRECTORY} PATH_SUFFIXES "Win64/Include")
        find_path(THORLABS_DFM_x86_INCLUDE_DIR "vpptype.h" PATHS ${THORLABS_DFM_VISA_x86_DIRECTORY} PATH_SUFFIXES "WinNT/Include")
        find_library(THORLABS_DFM_LIBRARY "TLDFM_64" PATHS ${THORLABS_DFM_VISA_DIRECTORY} PATH_SUFFIXES "Win64/Lib_x64/msc")
        find_library(THORLABS_DFMX_LIBRARY "TLDFMX_64" PATHS ${THORLABS_DFM_VISA_DIRECTORY} PATH_SUFFIXES "Win64/Lib_x64/msc")
    endif(WIN32)
endif()

option(BUILD_TARGET64 "Build for 64 bit target if set to ON or 32 bit if set to OFF." ON)

if(THORLABS_DFMX_INCLUDE_DIR AND THORLABS_DFM_x86_INCLUDE_DIR)
    find_package(ITOM_SDK COMPONENTS dataobject itomCommonLib itomCommonQtLib itomWidgets REQUIRED)

    include(ItomBuildMacros)

    itom_init_cmake_policy(3.12)
    itom_find_package_qt(ON Core Widgets LinguistTools)
    itom_init_plugin_library(${target_name})

    include_directories(
        ${CMAKE_CURRENT_BINARY_DIR} #build directory of this plugin (recommended)
        ${CMAKE_CURRENT_SOURCE_DIR} #source directory of this plugin (recommended)
        ${ITOM_SDK_INCLUDE_DIRS}    #include directory of the itom SDK (recommended) as well as necessary 3rd party directories (e.g. from OpenCV)
        #add further include directories here
        ${THORLABS_DFMX_INCLUDE_DIR}
        ${THORLABS_DFM_INCLUDE_DIR}
        ${THORLABS_DFM_x86_INCLUDE_DIR}
    )

    link_directories(
        #add all linker directories
    )

    set(PLUGIN_HEADERS
        ${CMAKE_CURRENT_SOURCE_DIR}/ThorlabsDMH.h
        ${CMAKE_CURRENT_SOURCE_DIR}/dialogThorlabsDMH.h
        ${CMAKE_CURRENT_SOURCE_DIR}/dockWidgetThorlabsDMH.h
        ${CMAKE_CURRENT_BINARY_DIR}/pluginVersion.h
        ${CMAKE_CURRENT_BINARY_DIR}/gitVersion.h #provided by the init script, contains currently checked out git tag
        #add further header files (absolute paths e.g. using CMAKE_CURRENT_SOURCE_DIR)
    )

    set(PLUGIN_SOURCES
        ${CMAKE_CURRENT_SOURCE_DIR}/dialogThorlabsDMH.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/dockWidgetThorlabsDMH.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/ThorlabsDMH.cpp
        #add further source files here
    )

    if(MSVC)
        list(APPEND PLUGIN_SOURCES ${ITOM_SDK_INCLUDE_DIR}/../pluginLibraryVersion.rc)
    endif()

    set(PLUGIN_UI
        ${CMAKE_CURRENT_SOURCE_DIR}/dialogThorlabsDMH.ui
        ${CMAKE_CURRENT_SOURCE_DIR}/dockWidgetThorlabsDMH.ui
    )

    set(PLUGIN_RCC
        #add absolute paths to any *.qrc resource files here
    )

    #Define Version in pluginVersion.h
    configure_file( ${CMAKE_CURRENT_SOURCE_DIR}/pluginVersion.h.in ${CMAKE_CURRENT_BINARY_DIR}/pluginVersion.h)

    add_library(${target_name} SHARED
        ${PLUGIN_SOURCES}
        ${PLUGIN_HEADERS}
        ${PLUGIN_UI}
        ${PLUGIN_RCC}
        )

    set_target_properties(${target_name} PROPERTIES AUTOMOC ON AUTORCC ON AUTOUIC ON)

    target_link_libraries(${target_name}
        ${ITOM_SDK_LIBRARIES}
        ${QT5_LIBRARIES}
        ${VISUALLEAKDETECTOR_LIBRARIES}
        ${THORLABS_DFM_LIBRARY}
        ${THORLABS_DFMX_LIBRARY}
        )

    set(FILES_TO_TRANSLATE ${PLUGIN_SOURCES} ${PLUGIN_HEADERS} ${PLUGIN_UI} ${PLUGIN_RCC})
    itom_library_translation(
        QM_FILES
        TARGET ${target_name}
        FILES_TO_TRANSLATE ${FILES_TO_TRANSLATE}
        )

    itom_configure_plugin_documentation(${target_name} ThorlabsDMH)

    set(COPY_SOURCES "")
    set(COPY_DESTINATIONS "")

    itom_add_pluginlibrary_to_copy_list(${target_name} COPY_SOURCES COPY_DESTINATIONS)
    itom_add_plugin_qm_files_to_copy_list(${target_name} QM_FILES COPY_SOURCES COPY_DESTINATIONS)

    itom_post_build_copy_files(${target_name} COPY_SOURCES COPY_DESTINATIONS)

    itom_post_build_copy_files_to_lib_folder(${target_name} THORLABS_DFMX_BINARY)

    set(BINARY_FILES
        "${THORLABS_DFM_VISA_DIRECTORY}/Win64/Bin/TLDFM_64.dll"
        "${THORLABS_DFM_VISA_DIRECTORY}/Win64/Bin/TLDFMX_64.dll"
        )

    itom_post_build_copy_files_to_lib_folder(${target_name} BINARY_FILES)

else(THORLABS_DFMX_INCLUDE_DIR AND THORLABS_DFM_x86_INCLUDE_DIR)
    message(SEND_ERROR "${THORLABS_DFMX_INCLUDE_DIR} or ${THORLABS_DFM_x86_INCLUDE_DIR} for plugin ${target_name} could not be found. ${target_name} will not be build. Please properly indicate THORLABS_DFM_VISA_DIRECTORY and THORLABS_DFM_VISA_x86_DIRECTORY.")
ENDIF(THORLABS_DFMX_INCLUDE_DIR AND THORLABS_DFM_x86_INCLUDE_DIR)
