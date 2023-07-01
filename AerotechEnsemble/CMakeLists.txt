cmake_minimum_required(VERSION 3.12...3.24)

set(target_name AerotechEnsemble)

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

find_package(ITOM_SDK COMPONENTS dataobject itomCommonLib itomCommonQtLib REQUIRED)

include(ItomBuildMacros)
itom_init_cmake_policy(3.12)
itom_init_plugin_library(${target_name}) #Start the project, init compiler settings and set default configurations for plugins
itom_find_package_qt(ON Widgets UiTools Xml LinguistTools)

include_directories(
    ${CMAKE_CURRENT_BINARY_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/CLibrary/Include
    ${ITOM_SDK_INCLUDE_DIRS}
)

if(BUILD_TARGET64)
    set(LIB_PATH ${CMAKE_CURRENT_SOURCE_DIR}/CLibrary/Lib64)
    set(BINARY_PATH ${CMAKE_CURRENT_SOURCE_DIR}/CLibrary/Bin64)
    set(LIB_NAME EnsembleC64)
else(BUILD_TARGET64)
    set(LIB_PATH ${CMAKE_CURRENT_SOURCE_DIR}/CLibrary/Lib)
    set(BINARY_PATH ${CMAKE_CURRENT_SOURCE_DIR}/CLibrary/Bin)
    set(LIB_NAME EnsembleC)
endif(BUILD_TARGET64)

link_directories(
    ${LIB_PATH}
)

set(PLUGIN_HEADERS
    ${CMAKE_CURRENT_SOURCE_DIR}/aerotechEnsemble.h
    ${CMAKE_CURRENT_SOURCE_DIR}/dockWidgetAerotechEnsemble.h
    ${CMAKE_CURRENT_SOURCE_DIR}/dialogAerotechEnsemble.h
    ${CMAKE_CURRENT_SOURCE_DIR}/pluginVersion.h
    ${CMAKE_CURRENT_BINARY_DIR}/gitVersion.h
)

set(PLUGIN_UI
    ${CMAKE_CURRENT_SOURCE_DIR}/dialogAerotechEnsemble.ui
    ${CMAKE_CURRENT_SOURCE_DIR}/dockWidgetAerotechEnsemble.ui
)

set(PLUGIN_RCC
)

set(PLUGIN_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/aerotechEnsemble.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/dockWidgetAerotechEnsemble.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/dialogAerotechEnsemble.cpp
)

#Add version information to the plugIn-dll unter MSVC
if(MSVC)
    list(APPEND PLUGIN_SOURCES ${ITOM_SDK_INCLUDE_DIR}/../pluginLibraryVersion.rc)
endif()

add_library(${target_name} SHARED ${PLUGIN_SOURCES} ${PLUGIN_HEADERS} ${PLUGIN_UI} ${PLUGIN_RCC})

# Qt: enable all automoc, autouic and autorcc.
set_target_properties(${target_name} PROPERTIES AUTOMOC ON AUTORCC ON AUTOUIC ON)

target_link_libraries(${target_name} ${ITOM_SDK_LIBRARIES} ${LIB_NAME} ${QT5_LIBRARIES} ${VISUALLEAKDETECTOR_LIBRARIES})

if(CMAKE_SIZEOF_VOID_P EQUAL 8)
    set_target_properties(${target_name} PROPERTIES COMPILE_FLAGS "/Zc:wchar_t-")
endif()

#translation
set(FILES_TO_TRANSLATE ${PLUGIN_SOURCES} ${PLUGIN_HEADERS} ${PLUGIN_UI})
itom_library_translation(QM_FILES TARGET ${target_name} FILES_TO_TRANSLATE ${FILES_TO_TRANSLATE})

itom_configure_plugin_documentation(${target_name} aerotechEnsemble)

# COPY SECTION
set(COPY_SOURCES "")
set(COPY_DESTINATIONS "")
itom_add_pluginlibrary_to_copy_list(${target_name} COPY_SOURCES COPY_DESTINATIONS)
itom_add_plugin_qm_files_to_copy_list(${target_name} QM_FILES COPY_SOURCES COPY_DESTINATIONS)
itom_post_build_copy_files(${target_name} COPY_SOURCES COPY_DESTINATIONS)

# copy dlls in itoms lib folder
file(GLOB dlls "${BINARY_PATH}/*.dll")
itom_post_build_copy_files_to_lib_folder(${target_name} dlls)
