cmake_minimum_required(VERSION 3.12...3.29)
set(target_name Tucsen)

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

include(ItomBuildMacros)
itom_init_cmake_policy(3.12)
itom_init_plugin_library(${target_name}) #Start the project, init compiler settings and set default configurations for plugins
itom_find_package_qt(ON Core LinguistTools Widgets)

if(NOT EXISTS ${TUCSEN_INCLUDE_DIR})
    if(EXISTS $ENV{TUCSEN_ROOT})
        set(TUCSEN_INCLUDE_DIR "$ENV{TUCSEN_ROOT}/sdk/inc" CACHE PATH "path to Tucsen SDK directory")
    else(EXISTS $ENV{TUCSEN_ROOT})
        set(TUCSEN_INCLUDE_DIR "TUCSEN_INCLUDE_DIR_NOTFOUND" CACHE PATH "path to Tucsen SDK directory")
    endif(EXISTS $ENV{TUCSEN_ROOT})
endif(NOT EXISTS ${TUCSEN_INCLUDE_DIR})

if(EXISTS ${TUCSEN_INCLUDE_DIR})
    find_file( TUCSEN_HEADER_FILE TUDefine.h PATHS ${TUCSEN_INCLUDE_DIR})
endif(EXISTS ${TUCSEN_INCLUDE_DIR})

if(TUCSEN_HEADER_FILE)

    set(TUCSEN_API_DIR ${TUCSEN_INCLUDE_DIR}/../ CACHE PATH "Base directory of Tucsen SDK")

    include_directories(
        ${CMAKE_CURRENT_BINARY_DIR} #build directory of this plugin (recommended)
        ${CMAKE_CURRENT_SOURCE_DIR} #source directory of this plugin (recommended)
        ${ITOM_SDK_INCLUDE_DIRS}    #include directory of the itom SDK (recommended)
        ${TUCSEN_INCLUDE_DIR}
        #add further include directories here
    )

    link_directories(
      ${TUCSEN_API_DIR}/sdk/lib/x64
        #add all linker directories
    )

    set(PLUGIN_HEADERS
        ${CMAKE_CURRENT_SOURCE_DIR}/tucsen.h
        ${CMAKE_CURRENT_SOURCE_DIR}/dialogTucsen.h
        ${CMAKE_CURRENT_SOURCE_DIR}/dockWidgetTucsen.h
        ${CMAKE_CURRENT_BINARY_DIR}/pluginVersion.h
        ${CMAKE_CURRENT_BINARY_DIR}/gitVersion.h
        #add further header files (absolute paths e.g. using CMAKE_CURRENT_SOURCE_DIR)
    )

    set(PLUGIN_SOURCES
        ${CMAKE_CURRENT_SOURCE_DIR}/dialogTucsen.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/dockWidgetTucsen.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/tucsen.cpp
        #add further source files here
    )

    #Define Version in pluginVersion.h
	configure_file( ${CMAKE_CURRENT_SOURCE_DIR}/pluginVersion.h.in ${CMAKE_CURRENT_BINARY_DIR}/pluginVersion.h)

    if(MSVC)
        list(APPEND PLUGIN_SOURCES ${ITOM_SDK_INCLUDE_DIR}/../pluginLibraryVersion.rc)
    endif()

    set(PLUGIN_UI
        ${CMAKE_CURRENT_SOURCE_DIR}/dialogTucsen.ui
        ${CMAKE_CURRENT_SOURCE_DIR}/dockWidgetTucsen.ui
    )

    set(PLUGIN_RCC
        #add absolute paths to any *.qrc resource files here
    )

    set(TUCSEN_LIBDIR_HINT "${TUCSEN_API_DIR}/lib/x64")
    set(TUCSEN_BINDIR_HINT "${TUCSEN_API_DIR}/lib/x64")

    find_library(TUCSEN_LIBRARY NAMES "TUCam"  PATHS ${TUCSEN_LIBDIR_HINT} NO_DEFAULT_PATH)

    if(WIN32)
        find_file(TUCSEN_BINARY "TUCam.dll"  PATHS ${TUCSEN_BINDIR_HINT} NO_DEFAULT_PATH)
    endif(WIN32)

    add_library(${target_name} SHARED ${PLUGIN_SOURCES} ${PLUGIN_HEADERS} ${PLUGIN_UI} ${PLUGIN_RCC})

    set_target_properties(${target_name} PROPERTIES AUTOMOC ON AUTORCC ON AUTOUIC ON)

    target_link_libraries(${target_name} ${ITOM_SDK_LIBRARIES} ${QT5_LIBRARIES} ${VISUALLEAKDETECTOR_LIBRARIES} ${TUCSEN_LIBRARY})

    set(FILES_TO_TRANSLATE ${PLUGIN_SOURCES} ${PLUGIN_HEADERS} ${PLUGIN_UI})
    itom_library_translation(QM_FILES TARGET ${target_name} FILES_TO_TRANSLATE ${FILES_TO_TRANSLATE})

    itom_configure_plugin_documentation(${target_name} tucsen)

    set(COPY_SOURCES "")
    set(COPY_DESTINATIONS "")

    itom_add_pluginlibrary_to_copy_list(${target_name} COPY_SOURCES COPY_DESTINATIONS)
    itom_add_plugin_qm_files_to_copy_list(${target_name} QM_FILES COPY_SOURCES COPY_DESTINATIONS)

    itom_post_build_copy_files(${target_name} COPY_SOURCES COPY_DESTINATIONS)

    itom_post_build_copy_files_to_lib_folder(${target_name} TUCSEN_BINARY)

else(TUCSEN_HEADER_FILE)
    message(SEND_ERROR "${target_name}-API directory could not be found. ${target_name} will not be build")
endif(TUCSEN_HEADER_FILE)
