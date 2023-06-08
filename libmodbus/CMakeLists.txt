cmake_minimum_required(VERSION 3.12...3.24)

set(target_name LibModBus)

message(STATUS "Project ${target_name} (${CMAKE_CURRENT_BINARY_DIR})")

option(BUILD_TARGET64 "Build for 64 bit target if set to ON or 32 bit if set to OFF." ON)
option(BUILD_LIBMODBUS_SHARED "Build libmodbus as shared library (DLL) or static library (must be OFF)" OFF)

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
set(CMAKE_INCLUDE_CURRENT_DIR ON)

add_subdirectory(libmodbus-3.1.2)

find_package(ITOM_SDK COMPONENTS dataobject itomCommonLib itomCommonQtLib itomWidgets REQUIRED)

include(ItomBuildMacros)
itom_init_cmake_policy(3.12)
itom_init_plugin_library(${target_name}) #Start the project, init compiler settings and set default configurations for plugins
itom_find_package_qt(ON Core LinguistTools Widgets)

if(BUILD_LIBMODBUS_SHARED)
else(BUILD_LIBMODBUS_SHARED)
    add_definitions(-DLIBMODBUSSTATIC)
endif(BUILD_LIBMODBUS_SHARED)

include_directories(
    ${CMAKE_CURRENT_BINARY_DIR} #build directory of this plugin (recommended)
    ${CMAKE_CURRENT_SOURCE_DIR} #source directory of this plugin (recommended)
    ${OpenCV_DIR}/include       #include directory of OpenCV
    ${ITOM_SDK_INCLUDE_DIRS}    #include directory of the itom SDK (recommended)
    ${CMAKE_CURRENT_SOURCE_DIR}/libmodbus-3.1.2/src
    #add further include directories here
)

link_directories(
    #add all linker directories
)

set(PLUGIN_HEADERS
    ${CMAKE_CURRENT_SOURCE_DIR}/LibModBus.h
    #${CMAKE_CURRENT_SOURCE_DIR}/dialogLibModBus.h
    #${CMAKE_CURRENT_SOURCE_DIR}/dockWidgetLibModBus.h
    ${CMAKE_CURRENT_SOURCE_DIR}/pluginVersion.h
    ${CMAKE_CURRENT_BINARY_DIR}/gitVersion.h
    #add further header files (absolute pathes e.g. using CMAKE_CURRENT_SOURCE_DIR)
)

set(PLUGIN_SOURCES
    #${CMAKE_CURRENT_SOURCE_DIR}/dialogLibModBus.cpp
    #${CMAKE_CURRENT_SOURCE_DIR}/dockWidgetLibModBus.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/LibModBus.cpp
    #add further source files here
)

if(MSVC)
    list(APPEND PLUGIN_SOURCES ${ITOM_SDK_INCLUDE_DIR}/../pluginLibraryVersion.rc)
endif()

set(PLUGIN_UI
    #${CMAKE_CURRENT_SOURCE_DIR}/dialogLibModBus.ui
    #${CMAKE_CURRENT_SOURCE_DIR}/dockWidgetLibModBus.ui
)

set(PLUGIN_RCC
    #add absolute pathes to any *.qrc resource files here
)

add_library(${target_name} SHARED ${PLUGIN_SOURCES} ${PLUGIN_HEADERS})

set_target_properties(${target_name} PROPERTIES AUTOMOC ON AUTORCC ON AUTOUIC ON)

target_link_libraries(${target_name} ${QT_LIBRARIES} ${ITOM_SDK_LIBRARIES} ${QT5_LIBRARIES} ${VISUALLEAKDETECTOR_LIBRARIES} libmodbus_library) #${LibModBus_LIBRARY})

set(FILES_TO_TRANSLATE ${PLUGIN_SOURCES} ${PLUGIN_HEADERS} ${PLUGIN_UI})
itom_library_translation(QM_FILES TARGET ${target_name} FILES_TO_TRANSLATE ${FILES_TO_TRANSLATE})

itom_configure_plugin_documentation(${target_name} LibModBus)

set(COPY_SOURCES "")
set(COPY_DESTINATIONS "")

itom_add_pluginlibrary_to_copy_list(${target_name} COPY_SOURCES COPY_DESTINATIONS)
itom_add_plugin_qm_files_to_copy_list(${target_name} QM_FILES COPY_SOURCES COPY_DESTINATIONS)

itom_post_build_copy_files(${target_name} COPY_SOURCES COPY_DESTINATIONS)
