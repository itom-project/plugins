SET (target_name MSMediaFoundation)

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

find_package(ITOM_SDK REQUIRED)
include("${ITOM_SDK_DIR}/ItomBuildMacros.cmake")
FIND_PACKAGE_QT(ON Widgets Xml LinguistTools)
find_package(OpenCV COMPONENTS core imgproc REQUIRED)
find_package(VisualLeakDetector QUIET)
find_package(WindowsSDK QUIET)

if(WINDOWSSDK_FOUND)
    ADD_DEFINITIONS(${QT_DEFINITIONS})
    IF (BUILD_UNICODE)
        ADD_DEFINITIONS(-DUNICODE -D_UNICODE)
    ENDIF (BUILD_UNICODE)
    ADD_DEFINITIONS(-DCMAKE)
	ADD_DEFINITIONS(-DITOMWIDGETS_SHARED)

    IF(VISUALLEAKDETECTOR_FOUND AND VISUALLEAKDETECTOR_ENABLED)
        ADD_DEFINITIONS(-DVISUAL_LEAK_DETECTOR_CMAKE)
    ENDIF(VISUALLEAKDETECTOR_FOUND AND VISUALLEAKDETECTOR_ENABLED)

    # enable some qt stuff
    #SET (QT_USE_QTXML TRUE)

    # default build types are None, Debug, Release, RelWithDebInfo and MinRelSize
    IF (DEFINED CMAKE_BUILD_TYPE)
        SET(CMAKE_BUILD_TYPE ${CMAKE_BUILD_TYPE} CACHE STRING "Choose the type of build, options are: None(CMAKE_CXX_FLAGS or CMAKE_C_FLAGS used) Debug Release RelWithDebInfo MinSizeRel.")
    ELSE(CMAKE_BUILD_TYPE)
        SET (CMAKE_BUILD_TYPE Debug CACHE STRING "Choose the type of build, options are: None(CMAKE_CXX_FLAGS or CMAKE_C_FLAGS used) Debug Release RelWithDebInfo MinSizeRel.")
    ENDIF (DEFINED CMAKE_BUILD_TYPE)

    message(STATUS ${CMAKE_CURRENT_BINARY_DIR})

    INCLUDE_DIRECTORIES(
        ${CMAKE_CURRENT_BINARY_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${QT_QTCORE_INCLUDE_DIR}
        ${OpenCV_DIR}/include
        ${ITOM_SDK_INCLUDE_DIR}
		${ITOM_SDK_INCLUDE_DIR}/itomWidgets
        ${VISUALLEAKDETECTOR_INCLUDE_DIR}
        videoInput
    )

    LINK_DIRECTORIES(
        ${OpenCV_DIR}/lib
    )

    if (NOT QT5_FOUND)
        INCLUDE(${QT_USE_FILE})
    endif (NOT QT5_FOUND)

    set(plugin_HEADERS
        ${ITOM_SDK_INCLUDE_DIR}/common/apiFunctionsInc.h
        ${ITOM_SDK_INCLUDE_DIR}/common/addInGrabber.h
        ${ITOM_SDK_INCLUDE_DIR}/common/addInInterface.h
        ${ITOM_SDK_INCLUDE_DIR}/common/sharedStructures.h
        ${ITOM_SDK_INCLUDE_DIR}/common/sharedStructuresQt.h
        ${CMAKE_CURRENT_SOURCE_DIR}/dialogMSMediaFoundation.h
        ${CMAKE_CURRENT_SOURCE_DIR}/dockWidgetMSMediaFoundation.h
        ${CMAKE_CURRENT_SOURCE_DIR}/MSMediaFoundation.h
        ${CMAKE_CURRENT_SOURCE_DIR}/pluginVersion.h
        ${CMAKE_CURRENT_SOURCE_DIR}/videoInput/Common.h
        ${CMAKE_CURRENT_SOURCE_DIR}/videoInput/DebugPrintOut.h
        ${CMAKE_CURRENT_SOURCE_DIR}/videoInput/FormatReader.h
        ${CMAKE_CURRENT_SOURCE_DIR}/videoInput/ImageGrabber.h
        ${CMAKE_CURRENT_SOURCE_DIR}/videoInput/ImageGrabberThread.h
        ${CMAKE_CURRENT_SOURCE_DIR}/videoInput/Media_Foundation.h
        ${CMAKE_CURRENT_SOURCE_DIR}/videoInput/RawImage.h
        ${CMAKE_CURRENT_SOURCE_DIR}/videoInput/videoDevice.h
        ${CMAKE_CURRENT_SOURCE_DIR}/videoInput/videoDevices.h
        ${CMAKE_CURRENT_SOURCE_DIR}/videoInput/videoInput.h
    )

    if (NOT QT5_FOUND)
        QT4_WRAP_CPP_ITOM(plugin_HEADERS_MOC ${plugin_HEADERS})
    endif (NOT QT5_FOUND)

    set(plugin_ui
        ${CMAKE_CURRENT_SOURCE_DIR}/dialogMSMediaFoundation.ui
        ${CMAKE_CURRENT_SOURCE_DIR}/dockWidgetMSMediaFoundation.ui
    )

    if (QT5_FOUND)
        QT5_WRAP_UI(plugin_ui_MOC ${plugin_ui})
    else (QT5_FOUND)
        QT4_WRAP_UI_ITOM(plugin_ui_MOC ${plugin_ui})
    endif (QT5_FOUND)

    set(plugin_SOURCES
        ${CMAKE_CURRENT_SOURCE_DIR}/dialogMSMediaFoundation.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/dockWidgetMSMediaFoundation.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/MSMediaFoundation.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/videoInput/DebugPrintOut.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/videoInput/FormatReader.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/videoInput/ImageGrabber.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/videoInput/ImageGrabberThread.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/videoInput/Media_Foundation.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/videoInput/RawImage.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/videoInput/videoDevice.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/videoInput/videoDevices.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/videoInput/videoInput.cpp
    )

    ADD_SOURCE_GROUP(videoInput)

    #Add version information to the plugIn-dll unter MSVC
    if(MSVC)
        list(APPEND plugin_SOURCES ${ITOM_SDK_INCLUDE_DIR}/../pluginLibraryVersion.rc)
    endif(MSVC)

    #handle translations BEGIN STEP 1
    set (FILES_TO_TRANSLATE ${plugin_SOURCES} ${plugin_HEADERS} ${plugin_ui})
    set (TRANSLATION_OUTPUT_FILES)
    set (TRANSLATIONS_FILES)

    file (GLOB EXISTING_TRANSLATION_FILES "translation/*.ts")
    #handle translations END STEP 1

    ADD_LIBRARY(${target_name} ${LIBRARY_TYPE} ${plugin_SOURCES} ${plugin_HEADERS} ${plugin_HEADERS_MOC} ${plugin_ui_MOC} ${plugin_rcc_MOC} ${EXISTING_TRANSLATION_FILES})
    if (QT5_FOUND)
        TARGET_LINK_LIBRARIES(${target_name} ${OpenCV_LIBS} ${ITOM_SDK_dataobject_LIBRARY} ${ITOM_SDK_itomCommonLib_LIBRARY} ${ITOM_SDK_itomCommonQtLib_LIBRARY} ${ITOM_SDK_itomWidgets_LIBRARY} ${VISUALLEAKDETECTOR_LIBRARIES})
        
        qt5_use_modules(${target_name} Widgets UiTools Xml)
    else (QT5_FOUND)
        TARGET_LINK_LIBRARIES(${target_name} ${QT_LIBRARIES} ${QT_QTUITOOLS_LIBRARY} ${QT_QTXML_LIBRARY} ${OpenCV_LIBS} ${ITOM_SDK_dataobject_LIBRARY} ${ITOM_SDK_itomCommonLib_LIBRARY} ${ITOM_SDK_itomCommonQtLib_LIBRARY} ${ITOM_SDK_itomWidgets_LIBRARY} ${VISUALLEAKDETECTOR_LIBRARIES})
    endif (QT5_FOUND)

    #handle translations BEGIN STEP 2
    if (UPDATE_TRANSLATIONS)
        if (QT5_FOUND)
            QT5_CREATE_TRANSLATION(TRANSLATION_OUTPUT_FILES TRANSLATIONS_FILES ${target_name} ITOM_LANGUAGES ${FILES_TO_TRANSLATE} ${TRANSLATIONS_FILES})
        else (QT5_FOUND)
            QT4_CREATE_TRANSLATION_ITOM(TRANSLATION_OUTPUT_FILES TRANSLATIONS_FILES ${target_name} ITOM_LANGUAGES ${FILES_TO_TRANSLATE} ${TRANSLATIONS_FILES})
        endif (QT5_FOUND)
        
        add_custom_target (_${target_name}_translation DEPENDS ${TRANSLATION_OUTPUT_FILES})
        add_dependencies(${target_name} _${target_name}_translation)
        
        if (QT5_FOUND)
            QT5_ADD_TRANSLATION(QM_FILES "${CMAKE_CURRENT_BINARY_DIR}/translation" ${target_name} ${TRANSLATIONS_FILES})
        else (QT5_FOUND)
            QT4_ADD_TRANSLATION_ITOM(QM_FILES "${CMAKE_CURRENT_BINARY_DIR}/translation" ${target_name} ${TRANSLATIONS_FILES})
        endif (QT5_FOUND)
    else (UPDATE_TRANSLATIONS)
        if (QT5_FOUND)
            QT5_ADD_TRANSLATION(QM_FILES "${CMAKE_CURRENT_BINARY_DIR}/translation" ${target_name} ${EXISTING_TRANSLATION_FILES})
        else (QT5_FOUND)
            QT4_ADD_TRANSLATION_ITOM(QM_FILES "${CMAKE_CURRENT_BINARY_DIR}/translation" ${target_name} ${EXISTING_TRANSLATION_FILES})
        endif (QT5_FOUND)
    endif (UPDATE_TRANSLATIONS)
    #handle translations END STEP 2
    
    PLUGIN_DOCUMENTATION(${target_name} msMediaFoundation)

    # COPY SECTION
    set(COPY_SOURCES "")
    set(COPY_DESTINATIONS "")
    ADD_PLUGINLIBRARY_TO_COPY_LIST(${target_name} COPY_SOURCES COPY_DESTINATIONS)
    #ADD_QM_FILES_TO_COPY_LIST(${target_name} QM_FILES COPY_SOURCES COPY_DESTINATIONS)
    POST_BUILD_COPY_FILES(${target_name} COPY_SOURCES COPY_DESTINATIONS)
ELSE(WINDOWSSDK_FOUND)
    message(WARNING "WindowsSDK could not be found. ${target_name} will not be build")  
ENDIF(WINDOWSSDK_FOUND)
