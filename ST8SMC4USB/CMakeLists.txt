cmake_minimum_required(VERSION 3.12...3.29)
set(target_name Standa8SMC4USB)

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
itom_find_package_qt(ON Widgets Xml LinguistTools)

add_definitions(-DXIMC_NO_STDINT)

include_directories(
    ${CMAKE_CURRENT_BINARY_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/Lib
    ${ITOM_SDK_INCLUDE_DIRS}
)

if(BUILD_TARGET64)
    set(LIBDIR "${CMAKE_CURRENT_SOURCE_DIR}/Lib/win64")
else(BUILD_TARGET64)
    set(LIBDIR "${CMAKE_CURRENT_SOURCE_DIR}/Lib/win32")
endif(BUILD_TARGET64)

if(WIN32)
    LIST(APPEND dlls "")
    LIST(APPEND dlls ${LIBDIR}/libximc.dll)
    LIST(APPEND dlls ${LIBDIR}/libjximc.dll)
    LIST(APPEND dlls ${LIBDIR}/xiwrapper.dll)
    LIST(APPEND dlls ${LIBDIR}/bindy.dll)
endif(WIN32)

set(PLUGIN_HEADERS
    ${CMAKE_CURRENT_SOURCE_DIR}/dialogST8SMC4USB.h
    ${CMAKE_CURRENT_SOURCE_DIR}/dockWidgetST8SMC4USB.h
    ${CMAKE_CURRENT_SOURCE_DIR}/ST8SMC4USB.h
    ${CMAKE_CURRENT_SOURCE_DIR}/ST8SMC4USBInterface.h
    ${CMAKE_CURRENT_BINARY_DIR}/pluginVersion.h
    ${CMAKE_CURRENT_SOURCE_DIR}/Lib/ximc.h
    ${CMAKE_CURRENT_BINARY_DIR}/gitVersion.h
)

set(PLUGIN_UI
    ${CMAKE_CURRENT_SOURCE_DIR}/dialogST8SMC4USB.ui
    ${CMAKE_CURRENT_SOURCE_DIR}/dockWidgetST8SMC4USB.ui
)

set(PLUGIN_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/dialogST8SMC4USB.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/dockWidgetST8SMC4USB.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/ST8SMC4USB.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/ST8SMC4USBInterface.cpp
)

set(PLUGIN_RCC
    #add absolute paths to any *.qrc resource files here
)

#Add version information to the plugIn-dll under MSVC
if(MSVC)
    list(APPEND PLUGIN_SOURCES ${ITOM_SDK_INCLUDE_DIR}/../pluginLibraryVersion.rc)
endif()

#Define Version in pluginVersion.h
configure_file( ${CMAKE_CURRENT_SOURCE_DIR}/pluginVersion.h.in ${CMAKE_CURRENT_BINARY_DIR}/pluginVersion.h)

add_library(${target_name} SHARED ${PLUGIN_SOURCES} ${PLUGIN_HEADERS} ${PLUGIN_UI} ${PLUGIN_RCC})

# Qt: enable all automoc, autouic and autorcc.
set_target_properties(${target_name} PROPERTIES AUTOMOC ON AUTORCC ON AUTOUIC ON)

target_link_libraries(${target_name} ${OpenCV_LIBS} ${ITOM_SDK_LIBRARIES} ${LIBDIR}/libximc.lib ${QT5_LIBRARIES} ${VISUALLEAKDETECTOR_LIBRARIES})

#translation
set(FILES_TO_TRANSLATE ${PLUGIN_SOURCES} ${PLUGIN_HEADERS} ${PLUGIN_UI})
itom_library_translation(QM_FILES TARGET ${target_name} FILES_TO_TRANSLATE ${FILES_TO_TRANSLATE})

itom_configure_plugin_documentation(${target_name} ST8SMC4USB)

# COPY SECTION
set(COPY_SOURCES "")
set(COPY_DESTINATIONS "")
itom_add_pluginlibrary_to_copy_list(${target_name} COPY_SOURCES COPY_DESTINATIONS)
itom_add_plugin_qm_files_to_copy_list(${target_name} QM_FILES COPY_SOURCES COPY_DESTINATIONS)
itom_post_build_copy_files(${target_name} COPY_SOURCES COPY_DESTINATIONS)

itom_post_build_copy_files_to_lib_folder(${target_name} dlls)
