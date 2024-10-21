cmake_minimum_required(VERSION 3.12...3.29)
set(target_name AVTVimbaX)

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

if(NOT EXISTS ${AVTVIMBAX_INCLUDE_DIR})
    if(EXISTS $ENV{AVTVIMBAX_ROOT})
        set(AVTVIMBAX_INCLUDE_DIR "$ENV{AVTVIMBAX_ROOT}" CACHE PATH "path to AVT Vimba SDK directory")
    else(EXISTS $ENV{AVTVIMBAX_ROOT})
        set(AVTVIMBAX_INCLUDE_DIR "AVTVIMBAX_INCLUDE_DIR_NOTFOUND" CACHE PATH "path to AVT Vimba SDK directory")
    endif(EXISTS $ENV{AVTVIMBAX_ROOT})
endif(NOT EXISTS ${AVTVIMBAX_INCLUDE_DIR})

if(EXISTS ${AVTVIMBAX_INCLUDE_DIR})
    find_file( AVT_VIMBAX_HEADER_FILE VmbCPP.h PATHS ${AVTVIMBAX_INCLUDE_DIR}/VmbCPP)
endif(EXISTS ${AVTVIMBAX_INCLUDE_DIR})

if(AVT_VIMBAX_HEADER_FILE)

    set(AVTVIMBAX_API_DIR ${AVTVIMBAX_INCLUDE_DIR}/../ CACHE PATH "Base directory of VimbaCPP SDK")

    include_directories(
        ${CMAKE_CURRENT_BINARY_DIR} #build directory of this plugin (recommended)
        ${CMAKE_CURRENT_SOURCE_DIR} #source directory of this plugin (recommended)
        ${ITOM_SDK_INCLUDE_DIRS}    #include directory of the itom SDK (recommended)
        ${AVTVIMBAX_INCLUDE_DIR}
        #add further include directories here
    )

    link_directories(
        #add all linker directories
    )

    set(PLUGIN_HEADERS
        ${CMAKE_CURRENT_SOURCE_DIR}/avtVimbaX.h
        ${CMAKE_CURRENT_SOURCE_DIR}/dialogAvtVimbaX.h
        ${CMAKE_CURRENT_SOURCE_DIR}/dockWidgetAvtVimbaX.h
        ${CMAKE_CURRENT_SOURCE_DIR}/avtVimbaXEnums.h
        ${CMAKE_CURRENT_BINARY_DIR}/pluginVersion.h
        ${CMAKE_CURRENT_BINARY_DIR}/gitVersion.h
        #add further header files (absolute paths e.g. using CMAKE_CURRENT_SOURCE_DIR)
    )

    set(PLUGIN_SOURCES
        ${CMAKE_CURRENT_SOURCE_DIR}/dialogAvtVimbaX.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/dockWidgetAvtVimbaX.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/avtVimbaX.cpp
        #add further source files here
    )

    #Define Version in pluginVersion.h
	configure_file( ${CMAKE_CURRENT_SOURCE_DIR}/pluginVersion.h.in ${CMAKE_CURRENT_BINARY_DIR}/pluginVersion.h)

    if(MSVC)
        list(APPEND PLUGIN_SOURCES ${ITOM_SDK_INCLUDE_DIR}/../pluginLibraryVersion.rc)
    endif()

    set(PLUGIN_UI
        ${CMAKE_CURRENT_SOURCE_DIR}/dialogAvtVimbaX.ui
        ${CMAKE_CURRENT_SOURCE_DIR}/dockWidgetAvtVimbaX.ui
    )

    set(PLUGIN_RCC
        #add absolute paths to any *.qrc resource files here
    )

    set(AVT_LIBDIR_HINT "${AVTVIMBAX_API_DIR}/lib")
    set(AVT_BINDIR_HINT "${AVTVIMBAX_API_DIR}/bin")

    find_library(AVTVIMBAX_LIBRARY NAMES "VmbCPP"  PATHS ${AVT_LIBDIR_HINT} NO_DEFAULT_PATH)

    if(WIN32)
        find_file(AVTVIMBAX_BINARY_C "VmbC.dll"  PATHS ${AVT_BINDIR_HINT} NO_DEFAULT_PATH)
        find_file(AVTVIMBAX_BINARY_CPP "VmbCPP.dll"  PATHS ${AVT_BINDIR_HINT} NO_DEFAULT_PATH)
    endif(WIN32)

    add_library(${target_name} SHARED ${PLUGIN_SOURCES} ${PLUGIN_HEADERS} ${PLUGIN_UI} ${PLUGIN_RCC})

    set_target_properties(${target_name} PROPERTIES AUTOMOC ON AUTORCC ON AUTOUIC ON)

    target_link_libraries(${target_name} ${ITOM_SDK_LIBRARIES} ${QT5_LIBRARIES} ${VISUALLEAKDETECTOR_LIBRARIES} ${AVTVIMBAX_LIBRARY})

    set(FILES_TO_TRANSLATE ${PLUGIN_SOURCES} ${PLUGIN_HEADERS} ${PLUGIN_UI})
    itom_library_translation(QM_FILES TARGET ${target_name} FILES_TO_TRANSLATE ${FILES_TO_TRANSLATE})

    itom_configure_plugin_documentation(${target_name} avtVimbaX)

    set(COPY_SOURCES "")
    set(COPY_DESTINATIONS "")

    itom_add_pluginlibrary_to_copy_list(${target_name} COPY_SOURCES COPY_DESTINATIONS)
    itom_add_plugin_qm_files_to_copy_list(${target_name} QM_FILES COPY_SOURCES COPY_DESTINATIONS)

    itom_post_build_copy_files(${target_name} COPY_SOURCES COPY_DESTINATIONS)

    itom_post_build_copy_files_to_lib_folder(${target_name} AVTVIMBAX_BINARY_C)
    itom_post_build_copy_files_to_lib_folder(${target_name} AVTVIMBAX_BINARY_CPP)

else(AVT_VIMBAX_HEADER_FILE)
    message(SEND_ERROR "${target_name}-API directory could not be found. ${target_name} will not be build")
endif(AVT_VIMBAX_HEADER_FILE)
