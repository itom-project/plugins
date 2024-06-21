cmake_minimum_required(VERSION 3.12...3.29)
set(target_name hidApi)

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
find_package(OpenCV COMPONENTS core REQUIRED)

include(ItomBuildMacros)
itom_init_cmake_policy(3.12)
itom_init_plugin_library(${target_name}) #Start the project, init compiler settings and set default configurations for plugins
itom_find_package_qt(ON Widgets Xml LinguistTools)

if(UNIX)
    #find_package(LibUSB QUIET)
    find_package(Libudev QUIET)
    #find_file(LIBUDEV libudev.h)
    if(LIBUDEV_FOUND)
        set(HIDAPI_REQUIREMENTS_FOUND TRUE)
    else()
    set(HIDAPI_REQUIREMENTS_FOUND FALSE)
    endif()
else()
    set(HIDAPI_REQUIREMENTS_FOUND TRUE) #no requirements
endif(UNIX)

if(HIDAPI_REQUIREMENTS_FOUND)

    include_directories(
        ${CMAKE_CURRENT_BINARY_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${ITOM_SDK_INCLUDE_DIRS}
            ${CMAKE_CURRENT_SOURCE_DIR}/hidapi-0.7.0/hidapi
        #${LibUSB_INCLUDE_DIRS}
        ${LIBUDEV_INCLUDE_DIR}
    )

    set(PLUGIN_HEADERS
        ${CMAKE_CURRENT_SOURCE_DIR}/itomHidApi.h
        ${CMAKE_CURRENT_SOURCE_DIR}/dockWidgetHidApi.h
        ${CMAKE_CURRENT_BINARY_DIR}/pluginVersion.h
        ${CMAKE_CURRENT_BINARY_DIR}/gitVersion.h
    )

    set(PLUGIN_UI
        ${CMAKE_CURRENT_SOURCE_DIR}/dockWidgetHidApi.ui
    )

    if(WIN32)
        set(PLUGIN_SOURCES
            ${CMAKE_CURRENT_SOURCE_DIR}/itomHidApi.cpp
            ${CMAKE_CURRENT_SOURCE_DIR}/dockWidgetHidApi.cpp
            ${CMAKE_CURRENT_SOURCE_DIR}/hidapi-0.7.0/windows/hid.c
        )
        set(HIDAPI_LIBRARIES Setupapi)
    elseif(UNIX)
        set(PLUGIN_SOURCES
            ${CMAKE_CURRENT_SOURCE_DIR}/itomHidApi.cpp
            ${CMAKE_CURRENT_SOURCE_DIR}/dockWidgetHidApi.cpp
            ${CMAKE_CURRENT_SOURCE_DIR}/hidapi-0.7.0/linux/hid.c
            #${CMAKE_CURRENT_SOURCE_DIR}/hidapi-0.7.0/linux/hid-libusb.c
        )
        set(HIDAPI_LIBRARIES ${LIBUDEV_LIBRARY})
    elseif(APPLE)
        set(PLUGIN_SOURCES
            ${CMAKE_CURRENT_SOURCE_DIR}/itomHidApi.cpp
            ${CMAKE_CURRENT_SOURCE_DIR}/dockWidgetHidApi.cpp
            ${CMAKE_CURRENT_SOURCE_DIR}/hidapi-0.7.0/mac/hid.c
        )
        set(HIDAPI_LIBRARIES )
    endif(WIN32)

    #Define Version in pluginVersion.h
    configure_file( ${CMAKE_CURRENT_SOURCE_DIR}/pluginVersion.h.in ${CMAKE_CURRENT_BINARY_DIR}/pluginVersion.h)

    #Add version information to the plugIn-dll under MSVC
    if(MSVC)
        list(APPEND PLUGIN_SOURCES ${ITOM_SDK_INCLUDE_DIR}/../pluginLibraryVersion.rc)
    endif()

    add_library(${target_name} SHARED ${PLUGIN_SOURCES} ${PLUGIN_HEADERS} ${PLUGIN_UI} ${PLUGIN_RCC})

    add_dependencies(${target_name} LibUSB)

    # Qt: enable all automoc, autouic and autorcc.
    set_target_properties(${target_name} PROPERTIES AUTOMOC ON AUTORCC ON AUTOUIC ON)

    target_link_libraries(${target_name} ${ITOM_SDK_LIBRARIES} ${HIDAPI_LIBRARIES} ${QT5_LIBRARIES} ${VISUALLEAKDETECTOR_LIBRARIES}) #${LibUSB_LIBRARIES}

    #translation
    set(FILES_TO_TRANSLATE ${PLUGIN_SOURCES} ${PLUGIN_HEADERS} ${PLUGIN_UI})
    itom_library_translation(QM_FILES TARGET ${target_name} FILES_TO_TRANSLATE ${FILES_TO_TRANSLATE})

    itom_configure_plugin_documentation(${target_name} hidApi)

    # COPY SECTION
    set(COPY_SOURCES "")
    set(COPY_DESTINATIONS "")
    itom_add_pluginlibrary_to_copy_list(${target_name} COPY_SOURCES COPY_DESTINATIONS)
    itom_add_plugin_qm_files_to_copy_list(${target_name} QM_FILES COPY_SOURCES COPY_DESTINATIONS)
    itom_post_build_copy_files(${target_name} COPY_SOURCES COPY_DESTINATIONS)

else(HIDAPI_REQUIREMENTS_FOUND)
    message(SEND_ERROR "Linux package libudev, libudev-dev missing. ${target_name} will not be build")
endif(HIDAPI_REQUIREMENTS_FOUND)
