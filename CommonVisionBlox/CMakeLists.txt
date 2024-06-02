cmake_minimum_required(VERSION 3.12...3.29)
set(target_name CommonVisionBlox)

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

if(NOT EXISTS ${CVB_DIR})
    if(EXISTS $ENV{CVB_ROOT})
        set(CVB_DIR "$ENV{CVB_ROOT}" CACHE PATH "path to CommonVisionBlox SDK directory")
    else(EXISTS $ENV{CVB_ROOT})
        set(CVB_DIR "CVB_DIR-NOTFOUND" CACHE PATH "path to CommonVisionBlox SDK directory")
    endif(EXISTS $ENV{CVB_ROOT})
endif(NOT EXISTS ${CVB_DIR})

if(EXISTS ${CVB_DIR})
    set(CVB_INCLUDE_DIRECTORY "${CVB_DIR}/Lib/C" )
    find_file( CVB_HEADER_FILE iCVCore.h PATHS ${CVB_INCLUDE_DIRECTORY})
endif(EXISTS ${CVB_DIR})

if(CVB_HEADER_FILE)

    set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR} ${ITOM_SDK_DIR}/cmake)

    find_package(ITOM_SDK COMPONENTS dataobject itomCommonLib itomCommonQtLib itomWidgets REQUIRED)
    find_package(OpenCV COMPONENTS core REQUIRED) #if you require openCV indicate all components that are required (e.g. core, imgproc...)

    include(ItomBuildMacros)
    itom_init_cmake_policy(3.12)
    itom_init_plugin_library(${target_name}) #Start the project, init compiler settings and set default configurations for plugins
    itom_find_package_qt(ON Core LinguistTools Widgets)

    include_directories(
        ${CMAKE_CURRENT_BINARY_DIR} #build directory of this plugin (recommended)
        ${CMAKE_CURRENT_SOURCE_DIR} #source directory of this plugin (recommended)
        ${ITOM_SDK_INCLUDE_DIRS}    #include directory of the itom SDK (recommended)
        "${CVB_INCLUDE_DIRECTORY}"
    )

    link_directories(
        #add all linker directories
        "${CVB_INCLUDE_DIRECTORY}"
    )

    set(PLUGIN_HEADERS
        ${CMAKE_CURRENT_SOURCE_DIR}/commonVisionBlox.h
        ${CMAKE_CURRENT_SOURCE_DIR}/commonVisionBloxInterface.h
        ${CMAKE_CURRENT_BINARY_DIR}/pluginVersion.h
        ${CMAKE_CURRENT_BINARY_DIR}/gitVersion.h
        #add further header files (absolute paths e.g. using CMAKE_CURRENT_SOURCE_DIR)
    )

    set(PLUGIN_SOURCES
        ${CMAKE_CURRENT_SOURCE_DIR}/commonVisionBlox.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/commonVisionBloxInterface.cpp
        #add further source files here
    )

	#Define Version in pluginVersion.h
	configure_file( ${CMAKE_CURRENT_SOURCE_DIR}/pluginVersion.h.in ${CMAKE_CURRENT_BINARY_DIR}/pluginVersion.h)

    if(MSVC)
        list(APPEND PLUGIN_SOURCES ${ITOM_SDK_INCLUDE_DIR}/../pluginLibraryVersion.rc)
    endif()

    set(PLUGIN_UI
    )

    set(PLUGIN_RCC
        #add absolute paths to any *.qrc resource files here
    )

    add_library(${target_name} SHARED ${PLUGIN_SOURCES} ${PLUGIN_HEADERS} ${PLUGIN_UI} ${PLUGIN_RCC})

    set_target_properties(${target_name} PROPERTIES AUTOMOC ON AUTORCC ON AUTOUIC ON)

    target_link_libraries(${target_name} ${ITOM_SDK_LIBRARIES} ${QT5_LIBRARIES} ${VISUALLEAKDETECTOR_LIBRARIES})

    set(FILES_TO_TRANSLATE ${PLUGIN_SOURCES} ${PLUGIN_HEADERS} ${PLUGIN_UI})
    itom_library_translation(QM_FILES TARGET ${target_name} FILES_TO_TRANSLATE ${FILES_TO_TRANSLATE})

    itom_configure_plugin_documentation(${target_name} commonVisionBlox)

    set(COPY_SOURCES "")
    set(COPY_DESTINATIONS "")

    itom_add_pluginlibrary_to_copy_list(${target_name} COPY_SOURCES COPY_DESTINATIONS)
    itom_add_plugin_qm_files_to_copy_list(${target_name} QM_FILES COPY_SOURCES COPY_DESTINATIONS)

    itom_post_build_copy_files(${target_name} COPY_SOURCES COPY_DESTINATIONS)

else(CVB_HEADER_FILE)
    message(SEND_ERROR "Cmake Variabke CVB_DIR or Environment variable CVB_ROOT for common vision blox not found. ${target_name} will not be build.")
endif(CVB_HEADER_FILE)
