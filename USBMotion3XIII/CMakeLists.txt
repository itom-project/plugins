cmake_minimum_required(VERSION 3.12...3.29)
set(target_name USBMotion3XIII)

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

find_package(ITOM_SDK REQUIRED)
find_package(OpenCV COMPONENTS core REQUIRED)

include(ItomBuildMacros)
itom_init_cmake_policy(3.12)
itom_init_plugin_library(${target_name}) #Start the project, init compiler settings and set default configurations for plugins
itom_find_package_qt(ON Widgets Xml LinguistTools)


include_directories(
    ${CMAKE_CURRENT_BINARY_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${ITOM_SDK_INCLUDE_DIRS}
)

set(PLUGIN_HEADERS
    ${CMAKE_CURRENT_SOURCE_DIR}/dialogUSBMotion3XIII.h
    ${CMAKE_CURRENT_SOURCE_DIR}/dockWidgetUSBMotion3XIII.h
    ${CMAKE_CURRENT_SOURCE_DIR}/USB3xIII_dll.h
    ${CMAKE_CURRENT_SOURCE_DIR}/USBMotion3XIII.h
)

set(PLUGIN_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/dialogUSBMotion3XIII.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/dockWidgetUSBMotion3XIII.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/USBMotion3XIII.cpp
    ${CMAKE_CURRENT_BINARY_DIR}/pluginVersion.h
    ${CMAKE_CURRENT_BINARY_DIR}/gitVersion.h
)

set(PLUGIN_UI
    ${CMAKE_CURRENT_SOURCE_DIR}/dialogUSBMotion3XIII.ui
    ${CMAKE_CURRENT_SOURCE_DIR}/dockWidgetUSBMotion3XIII.ui
)

set(PLUGIN_RCC
    ${CMAKE_CURRENT_SOURCE_DIR}/resource.qrc
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

target_link_libraries(${target_name} ${OpenCV_LIBS} ${ITOM_SDK_LIBRARIES} ${QT5_LIBRARIES} ${VISUALLEAKDETECTOR_LIBRARIES})



if(NOT (QT5_FOUND OR QT6_FOUND))
    if(CMAKE_SIZEOF_VOID_P EQUAL 8)
        set_target_properties(${target_name} PROPERTIES COMPILE_FLAGS "/Zc:wchar_t-")
    endif()
endif()

#translation
set(FILES_TO_TRANSLATE ${PLUGIN_SOURCES} ${PLUGIN_HEADERS} ${PLUGIN_UI})
itom_library_translation(QM_FILES TARGET ${target_name} FILES_TO_TRANSLATE ${FILES_TO_TRANSLATE})

itom_configure_plugin_documentation(${target_name} usbMotion3XIII)

# COPY SECTION
set(COPY_SOURCES "")
set(COPY_DESTINATIONS "")
itom_add_pluginlibrary_to_copy_list(${target_name} COPY_SOURCES COPY_DESTINATIONS)

if(CMAKE_HOST_WIN32)
    if(CMAKE_SIZEOF_VOID_P EQUAL 8)
        set(COPY_SOURCES ${COPY_SOURCES} ${CMAKE_CURRENT_SOURCE_DIR}/USB3xIII64.dll)
        set(COPY_DESTINATIONS ${COPY_DESTINATIONS} ${ITOM_APP_DIR}/plugins/${target_name})
    else()
        set(COPY_SOURCES ${COPY_SOURCES} ${CMAKE_CURRENT_SOURCE_DIR}/USB3xIII.dll)
        set(COPY_DESTINATIONS ${COPY_DESTINATIONS} ${ITOM_APP_DIR}/plugins/${target_name})
    endif()
endif()

itom_add_plugin_qm_files_to_copy_list(${target_name} QM_FILES COPY_SOURCES COPY_DESTINATIONS)
itom_post_build_copy_files(${target_name} COPY_SOURCES COPY_DESTINATIONS)
