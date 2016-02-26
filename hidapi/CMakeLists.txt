SET (target_name hidApi)

project(${target_name})

cmake_minimum_required(VERSION 2.8)

OPTION(BUILD_UNICODE "Build with unicode charset if set to ON, else multibyte charset." ON)
OPTION(BUILD_SHARED_LIBS "Build shared library." ON)
OPTION(BUILD_TARGET64 "Build for 64 bit target if set to ON or 32 bit if set to OFF." OFF)
OPTION(UPDATE_TRANSLATIONS "Update source translation translation/*.ts files (WARNING: make clean will delete the source .ts files! Danger!)")
SET (ITOM_SDK_DIR "" CACHE PATH "base path to itom_sdk")
SET (CMAKE_DEBUG_POSTFIX "d" CACHE STRING "Adds a postfix for debug-built libraries.")
SET (ITOM_LANGUAGES "de" CACHE STRING "semicolon separated list of languages that should be created (en must not be given since it is the default)")

set(CMAKE_INCLUDE_CURRENT_DIR ON)
SET (CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR} ${ITOM_SDK_DIR})

IF(BUILD_SHARED_LIBS)
    SET(LIBRARY_TYPE SHARED)
ELSE(BUILD_SHARED_LIBS)
    SET(LIBRARY_TYPE STATIC)
ENDIF(BUILD_SHARED_LIBS)

find_package(ITOM_SDK COMPONENTS dataobject itomCommonLib itomCommonQtLib REQUIRED)
include("${ITOM_SDK_DIR}/ItomBuildMacros.cmake")
FIND_PACKAGE_QT(ON Widgets Xml LinguistTools)
find_package(OpenCV COMPONENTS core REQUIRED)
find_package(VisualLeakDetector QUIET)


IF(UNIX)
    #find_package(LibUSB QUIET)
    find_package(Libudev QUIET)
    #find_file(LIBUDEV libudev.h)
    IF(LIBUDEV_FOUND)
        SET(HIDAPI_REQUIREMENTS_FOUND TRUE)
    ELSE()
    SET(HIDAPI_REQUIREMENTS_FOUND FALSE)
    ENDIF()
ELSE()
    SET(HIDAPI_REQUIREMENTS_FOUND TRUE) #no requirements
ENDIF(UNIX)

ADD_DEFINITIONS(${QT_DEFINITIONS})
IF (BUILD_UNICODE)
    ADD_DEFINITIONS(-DUNICODE -D_UNICODE)
ENDIF (BUILD_UNICODE)
ADD_DEFINITIONS(-DCMAKE)


IF(VISUALLEAKDETECTOR_FOUND AND VISUALLEAKDETECTOR_ENABLED)
    ADD_DEFINITIONS(-DVISUAL_LEAK_DETECTOR_CMAKE)
ENDIF(VISUALLEAKDETECTOR_FOUND AND VISUALLEAKDETECTOR_ENABLED)

# default build types are None, Debug, Release, RelWithDebInfo and MinRelSize
IF (DEFINED CMAKE_BUILD_TYPE)
    SET(CMAKE_BUILD_TYPE ${CMAKE_BUILD_TYPE} CACHE STRING "Choose the type of build, options are: None(CMAKE_CXX_FLAGS or CMAKE_C_FLAGS used) Debug Release RelWithDebInfo MinSizeRel.")
ELSE(CMAKE_BUILD_TYPE)
    SET (CMAKE_BUILD_TYPE Debug CACHE STRING "Choose the type of build, options are: None(CMAKE_CXX_FLAGS or CMAKE_C_FLAGS used) Debug Release RelWithDebInfo MinSizeRel.")
ENDIF (DEFINED CMAKE_BUILD_TYPE)

message(STATUS ${CMAKE_CURRENT_BINARY_DIR})

IF(HIDAPI_REQUIREMENTS_FOUND)

    INCLUDE_DIRECTORIES(
        ${CMAKE_CURRENT_BINARY_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${QT_QTCORE_INCLUDE_DIR}
        ${OpenCV_DIR}/include
        ${ITOM_SDK_INCLUDE_DIR}
        ${VISUALLEAKDETECTOR_INCLUDE_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}/hidapi-0.7.0/hidapi
        #${LibUSB_INCLUDE_DIRS}
        ${LIBUDEV_INCLUDE_DIR}
    )

    set(plugin_HEADERS
        ${CMAKE_CURRENT_SOURCE_DIR}/itomHidApi.h
        ${CMAKE_CURRENT_SOURCE_DIR}/dockWidgetHidApi.h
        ${CMAKE_CURRENT_SOURCE_DIR}/pluginVersion.h
    )

    set(plugin_UI
        ${CMAKE_CURRENT_SOURCE_DIR}/dockWidgetHidApi.ui
    )

    IF(WIN32)
        set(plugin_SOURCES 
            ${CMAKE_CURRENT_SOURCE_DIR}/itomHidApi.cpp
            ${CMAKE_CURRENT_SOURCE_DIR}/dockWidgetHidApi.cpp
            ${CMAKE_CURRENT_SOURCE_DIR}/hidapi-0.7.0/windows/hid.c
        )
        set(HIDAPI_LIBRARIES Setupapi)
    ELSEIF(UNIX)
        set(plugin_SOURCES 
            ${CMAKE_CURRENT_SOURCE_DIR}/itomHidApi.cpp
            ${CMAKE_CURRENT_SOURCE_DIR}/dockWidgetHidApi.cpp
            ${CMAKE_CURRENT_SOURCE_DIR}/hidapi-0.7.0/linux/hid.c
            #${CMAKE_CURRENT_SOURCE_DIR}/hidapi-0.7.0/linux/hid-libusb.c
        )
        set(HIDAPI_LIBRARIES ${LIBUDEV_LIBRARY})
    ELSEIF(APPLE)
        set(plugin_SOURCES 
            ${CMAKE_CURRENT_SOURCE_DIR}/itomHidApi.cpp
            ${CMAKE_CURRENT_SOURCE_DIR}/dockWidgetHidApi.cpp
            ${CMAKE_CURRENT_SOURCE_DIR}/hidapi-0.7.0/mac/hid.c
        )
        set(HIDAPI_LIBRARIES )
    ENDIF(WIN32)

    #Add version information to the plugIn-dll unter MSVC
    if(MSVC)
        list(APPEND plugin_SOURCES ${ITOM_SDK_INCLUDE_DIR}/../pluginLibraryVersion.rc)
    endif(MSVC)

    if (QT5_FOUND)
        #if automoc if OFF, you also need to call QT5_WRAP_CPP here
        QT5_WRAP_UI(plugin_UI_MOC ${plugin_UI})
        QT5_ADD_RESOURCES(plugin_RCC_MOC ${plugin_RCC})
    else (QT5_FOUND)
        QT4_WRAP_CPP_ITOM(plugin_HEADERS_MOC ${plugin_HEADERS})
        QT4_WRAP_UI_ITOM(plugin_UI_MOC ${plugin_UI})
        QT4_ADD_RESOURCES(plugin_RCC_MOC ${plugin_RCC})
    endif (QT5_FOUND)

    file (GLOB EXISTING_TRANSLATION_FILES "translation/*.ts")

    ADD_LIBRARY(${target_name} ${LIBRARY_TYPE} ${plugin_SOURCES} ${plugin_HEADERS} ${plugin_HEADERS_MOC} ${plugin_UI_MOC} ${plugin_RCC_MOC} ${EXISTING_TRANSLATION_FILES})
    TARGET_LINK_LIBRARIES(${target_name} ${QT_LIBRARIES} ${ITOM_SDK_LIBRARIES} ${HIDAPI_LIBRARIES} ${VISUALLEAKDETECTOR_LIBRARIES}) #${LibUSB_LIBRARIES} 
    if (QT5_FOUND)
        qt5_use_modules(${target_name} ${QT_COMPONENTS})
    endif (QT5_FOUND)
        
    #translation
    set (FILES_TO_TRANSLATE ${plugin_SOURCES} ${plugin_HEADERS} ${plugin_UI})
    PLUGIN_TRANSLATION(QM_FILES ${target_name} ${UPDATE_TRANSLATIONS} "${EXISTING_TRANSLATION_FILES}" ITOM_LANGUAGES "${FILES_TO_TRANSLATE}")

    PLUGIN_DOCUMENTATION(${target_name} hidApi)

    # COPY SECTION
    set(COPY_SOURCES "")
    set(COPY_DESTINATIONS "")
    ADD_PLUGINLIBRARY_TO_COPY_LIST(${target_name} COPY_SOURCES COPY_DESTINATIONS)
    ADD_QM_FILES_TO_COPY_LIST(${target_name} QM_FILES COPY_SOURCES COPY_DESTINATIONS)
    POST_BUILD_COPY_FILES(${target_name} COPY_SOURCES COPY_DESTINATIONS)

ELSE(HIDAPI_REQUIREMENTS_FOUND)
    message(WARNING "Linux package libudev, libudev-dev missing. ${target_name} will not be build")    
ENDIF(HIDAPI_REQUIREMENTS_FOUND)