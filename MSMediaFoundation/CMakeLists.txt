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

find_package(OpenCV COMPONENTS core imgproc REQUIRED)

find_package(ITOM_SDK COMPONENTS dataobject itomCommonLib itomCommonQtLib itomWidgets REQUIRED)
find_package(VisualLeakDetector QUIET)
find_package(WindowsSDK QUIET)

if(WINDOWSSDK_FOUND)

    include("${ITOM_SDK_DIR}/ItomBuildMacros.cmake")
    FIND_PACKAGE_QT(ON Core Widgets LinguistTools)


    IF (BUILD_UNICODE)
        ADD_DEFINITIONS(-DUNICODE -D_UNICODE)
    ENDIF (BUILD_UNICODE)
    ADD_DEFINITIONS(-DCMAKE)
    ADD_DEFINITIONS(-DITOMWIDGETS_SHARED)

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

    INCLUDE_DIRECTORIES(
        ${CMAKE_CURRENT_BINARY_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${OpenCV_DIR}/include
        ${ITOM_SDK_INCLUDE_DIR}
        ${ITOM_SDK_INCLUDE_DIR}/itomWidgets
        ${VISUALLEAKDETECTOR_INCLUDE_DIR}
        videoInput
    )

    LINK_DIRECTORIES(
        #${OpenCV_DIR}/lib
    )

    set(plugin_HEADERS
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

    set(plugin_UI
        ${CMAKE_CURRENT_SOURCE_DIR}/dialogMSMediaFoundation.ui
        ${CMAKE_CURRENT_SOURCE_DIR}/dockWidgetMSMediaFoundation.ui
    )

	set(plugin_RCC
		#add absolute pathes to any *.qrc resource files here
	)

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

    ADD_SOURCE_GROUP(videoInput)

    file (GLOB EXISTING_TRANSLATION_FILES "translation/*.ts")

    ADD_LIBRARY(${target_name} ${LIBRARY_TYPE} ${plugin_SOURCES} ${plugin_HEADERS} ${plugin_HEADERS_MOC} ${plugin_UI_MOC} ${plugin_RCC_MOC} ${EXISTING_TRANSLATION_FILES})
    if (QT5_FOUND)
        TARGET_LINK_LIBRARIES(${target_name} ${OpenCV_LIBS} ${ITOM_SDK_LIBRARIES} ${VISUALLEAKDETECTOR_LIBRARIES})
        
        qt5_use_modules(${target_name} ${QT_COMPONENTS})
    else (QT5_FOUND)
        TARGET_LINK_LIBRARIES(${target_name} ${QT_LIBRARIES} ${OpenCV_LIBS} ${ITOM_SDK_LIBRARIES} ${VISUALLEAKDETECTOR_LIBRARIES})
    endif (QT5_FOUND)
    
    #translation
    set (FILES_TO_TRANSLATE ${plugin_SOURCES} ${plugin_HEADERS} ${plugin_UI})
    PLUGIN_TRANSLATION(QM_FILES ${target_name} ${UPDATE_TRANSLATIONS} "${EXISTING_TRANSLATION_FILES}" ITOM_LANGUAGES "${FILES_TO_TRANSLATE}")
    
    #documentation
    PLUGIN_DOCUMENTATION(${target_name} msMediaFoundation)

    # COPY SECTION
    set(COPY_SOURCES "")
    set(COPY_DESTINATIONS "")
    ADD_PLUGINLIBRARY_TO_COPY_LIST(${target_name} COPY_SOURCES COPY_DESTINATIONS)
    ADD_QM_FILES_TO_COPY_LIST(${target_name} QM_FILES COPY_SOURCES COPY_DESTINATIONS)
    POST_BUILD_COPY_FILES(${target_name} COPY_SOURCES COPY_DESTINATIONS)
ELSE(WINDOWSSDK_FOUND)
    message(WARNING "WindowsSDK could not be found. ${target_name} will not be build")  
ENDIF(WINDOWSSDK_FOUND)
