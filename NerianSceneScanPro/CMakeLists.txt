cmake_minimum_required(VERSION 3.12...3.24)

set(target_name NerianSceneScanPro)

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

set(NERIAN_VISION_SOFTWARE_DIR "" CACHE PATH "path to the install directory of Nerian Vision Software (e.g. C:/Program Files (x86)/Nerian Vision Software 7.0.0)" )

if(NOT NERIAN_VISION_SOFTWARE_DIR)
	set(NERIAN_VISION_SOFTWARE_DIR $ENV{NERIAN_ROOT})
endif(NOT NERIAN_VISION_SOFTWARE_DIR)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR} ${ITOM_SDK_DIR}/cmake)

find_package(ITOM_SDK COMPONENTS dataobject itomCommonLib itomCommonQtLib itomWidgets REQUIRED)
find_package(OpenCV COMPONENTS core imgproc REQUIRED)

include(ItomBuildMacros)
itom_init_cmake_policy(3.12)
itom_init_plugin_library(${target_name}) #Start the project, init compiler settings and set default configurations for plugins
itom_find_package_qt(ON Core Widgets LinguistTools)

if(NERIAN_VISION_SOFTWARE_DIR)

    find_path(NERIAN_VISION_INCLUDE_DIR "imagetransfer.h" PATHS ${NERIAN_VISION_SOFTWARE_DIR}/visiontransfer/include ${NERIAN_VISION_SOFTWARE_DIR}/cpp/include PATH_SUFFIXES "visiontransfer" DOC "path to the folder containing all headers")

    if(BUILD_TARGET64)
		find_library(NERIAN_VISION_LIB_DEBUG visiontransfer visiontransfer-debug HINTS ${NERIAN_VISION_SOFTWARE_DIR} PATH_SUFFIXES /visiontransfer/lib64/Debug /cpp/lib64/Debug)
		find_library(NERIAN_VISION_LIB_RELEASE visiontransfer HINTS ${NERIAN_VISION_SOFTWARE_DIR} PATH_SUFFIXES visiontransfer/lib64/Release cpp/lib64/Release)
    else(BUILD_TARGET64)
        find_library(NERIAN_VISION_LIB_DEBUG visiontransfer visiontransfer-debug HINTS ${NERIAN_VISION_SOFTWARE_DIR} PATH_SUFFIXES /visiontransfer/lib64/Debug /cpp/lib32/Debug)
		find_library(NERIAN_VISION_LIB_RELEASE visiontransfer HINTS ${NERIAN_VISION_SOFTWARE_DIR} PATH_SUFFIXES visiontransfer/lib64/Release cpp/lib32/Release)
    endif(BUILD_TARGET64)

    include_directories(
        ${CMAKE_CURRENT_BINARY_DIR} #build directory of this plugin (recommended)
        ${CMAKE_CURRENT_SOURCE_DIR} #source directory of this plugin (recommended)
        ${ITOM_SDK_INCLUDE_DIRS}    #include directory of the itom SDK (recommended)
        ${NERIAN_VISION_INCLUDE_DIR}/..
    )

    link_directories(
        #add all linker directories
    )

    set(PLUGIN_HEADERS
        ${CMAKE_CURRENT_SOURCE_DIR}/nerianSceneScanPro.h
        ${CMAKE_CURRENT_SOURCE_DIR}/dialogNerianSceneScanPro.h
        ${CMAKE_CURRENT_SOURCE_DIR}/dockWidgetNerianSceneScanPro.h
        ${CMAKE_CURRENT_SOURCE_DIR}/pluginVersion.h
        ${CMAKE_CURRENT_BINARY_DIR}/gitVersion.h
        #add further header files (absolute pathes e.g. using CMAKE_CURRENT_SOURCE_DIR)
    )

    set(PLUGIN_SOURCES
        ${CMAKE_CURRENT_SOURCE_DIR}/dialogNerianSceneScanPro.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/dockWidgetNerianSceneScanPro.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/nerianSceneScanPro.cpp
        #add further source files here
    )

    if(MSVC)
        list(APPEND PLUGIN_SOURCES ${ITOM_SDK_INCLUDE_DIR}/../pluginLibraryVersion.rc)
    endif()

    set(PLUGIN_UI
        ${CMAKE_CURRENT_SOURCE_DIR}/dialogNerianSceneScanPro.ui
        ${CMAKE_CURRENT_SOURCE_DIR}/dockWidgetNerianSceneScanPro.ui
    )

    set(PLUGIN_RCC
        #add absolute pathes to any *.qrc resource files here
    )

    add_library(${target_name} SHARED ${PLUGIN_SOURCES} ${PLUGIN_HEADERS} ${PLUGIN_UI} ${PLUGIN_RCC})

    set_target_properties(${target_name} PROPERTIES AUTOMOC ON AUTORCC ON AUTOUIC ON)

    target_link_libraries(${target_name} ${ITOM_SDK_LIBRARIES} ${OpenCV_LIBS} ${QT5_LIBRARIES} ${VISUALLEAKDETECTOR_LIBRARIES} debug ${NERIAN_VISION_LIB_DEBUG} optimized ${NERIAN_VISION_LIB_RELEASE})

    set(FILES_TO_TRANSLATE ${PLUGIN_SOURCES} ${PLUGIN_HEADERS} ${PLUGIN_UI})
    itom_library_translation(QM_FILES TARGET ${target_name} FILES_TO_TRANSLATE ${FILES_TO_TRANSLATE})

    itom_configure_plugin_documentation(${target_name} nerianSceneScanPro)

    set(COPY_SOURCES "")
    set(COPY_DESTINATIONS "")

    itom_add_pluginlibrary_to_copy_list(${target_name} COPY_SOURCES COPY_DESTINATIONS)
    itom_add_plugin_qm_files_to_copy_list(${target_name} QM_FILES COPY_SOURCES COPY_DESTINATIONS)

    itom_post_build_copy_files(${target_name} COPY_SOURCES COPY_DESTINATIONS)

else()
    message(SEND_ERROR "NERIAN_VISION_SOFTWARE_DIR is not given. Plugin NerianSceneScanPro cannot be build.")
endif()
