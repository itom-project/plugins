SET (target_name QCam)

project(${target_name})

message(STATUS "\n--------------- PLUGIN ${target_name} ---------------")

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
find_package(OpenCV COMPONENTS core REQUIRED)
find_package(VisualLeakDetector QUIET)

ADD_DEFINITIONS(${QT_DEFINITIONS})
IF (BUILD_UNICODE)
	ADD_DEFINITIONS(-DUNICODE -D_UNICODE)
ENDIF (BUILD_UNICODE)
ADD_DEFINITIONS(-DCMAKE)

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
    ${ITOM_SDK_INCLUDE_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}/QImaging_SDK/2.0.13.1/Headers
    ${VISUALLEAKDETECTOR_INCLUDE_DIR}
)

if(BUILD_TARGET64)
	SET( QImaging_Libraries QCaptureCore_x64.lib QCamDriverx64.lib )
    
    LINK_DIRECTORIES( 
	    ${CMAKE_CURRENT_SOURCE_DIR}/QImaging_SDK/2.0.13.1/libs/AMD64
	)
else(BUILD_TARGET64)	
	SET( QImaging_Libraries QCaptureCore.lib QCamDriver.lib )
    
    LINK_DIRECTORIES( 
	    ${CMAKE_CURRENT_SOURCE_DIR}/QImaging_SDK/2.0.13.1/libs/i386
	)
endif(BUILD_TARGET64)

set(plugin_HEADERS
    ${ITOM_SDK_INCLUDE_DIR}/common/addInGrabber.h
    ${ITOM_SDK_INCLUDE_DIR}/common/addInInterface.h
    ${ITOM_SDK_INCLUDE_DIR}/common/sharedFunctionsQt.h
    ${ITOM_SDK_INCLUDE_DIR}/common/sharedStructures.h
    ${ITOM_SDK_INCLUDE_DIR}/common/sharedStructuresQt.h
    ${ITOM_SDK_INCLUDE_DIR}/common/typeDefs.h
    ${CMAKE_CURRENT_SOURCE_DIR}/QCam.h
    ${CMAKE_CURRENT_SOURCE_DIR}/dialogQCam.h
    ${CMAKE_CURRENT_SOURCE_DIR}/dockWidgetQCam.h
    ${CMAKE_CURRENT_SOURCE_DIR}/pluginVersion.h
)

set(plugin_UI
    ${CMAKE_CURRENT_SOURCE_DIR}/dialogQCam.ui
    ${CMAKE_CURRENT_SOURCE_DIR}/dockWidgetQCam.ui
)

set(plugin_SOURCES 
    ${CMAKE_CURRENT_SOURCE_DIR}/QCam.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/dialogQCam.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/dockWidgetQCam.cpp
)

if (QT5_FOUND)
	#if automoc if OFF, you also need to call QT5_WRAP_CPP here
	QT5_WRAP_UI(plugin_UI_MOC ${plugin_UI})
	QT5_ADD_RESOURCES(plugin_RCC_MOC ${plugin_RCC})
else (QT5_FOUND)
	QT4_WRAP_CPP_ITOM(plugin_HEADERS_MOC ${plugin_HEADERS})
	QT4_WRAP_UI_ITOM(plugin_UI_MOC ${plugin_UI})
	QT4_ADD_RESOURCES(plugin_RCC_MOC ${plugin_RCC})
endif (QT5_FOUND)

#Add version information to the plugIn-dll unter MSVC
if(MSVC)
    list(APPEND plugin_SOURCES ${ITOM_SDK_INCLUDE_DIR}/../pluginLibraryVersion.rc)
endif(MSVC)

file (GLOB EXISTING_TRANSLATION_FILES "translation/*.ts")

ADD_LIBRARY(${target_name} ${LIBRARY_TYPE} ${plugin_SOURCES} ${plugin_HEADERS} ${plugin_HEADERS_MOC} ${plugin_UI_MOC} ${plugin_RCC_MOC} ${EXISTING_TRANSLATION_FILES})

TARGET_LINK_LIBRARIES(${target_name} ${QT_LIBRARIES} ${ITOM_SDK_LIBRARIES} ${QImaging_Libraries} ${VISUALLEAKDETECTOR_LIBRARIES})

if (QT5_FOUND)
    qt5_use_modules(${target_name} ${QT_COMPONENTS})
endif (QT5_FOUND)

#translation
set (FILES_TO_TRANSLATE ${plugin_SOURCES} ${plugin_HEADERS} ${plugin_UI})
PLUGIN_TRANSLATION(QM_FILES ${target_name} ${UPDATE_TRANSLATIONS} "${EXISTING_TRANSLATION_FILES}" ITOM_LANGUAGES "${FILES_TO_TRANSLATE}")
PLUGIN_DOCUMENTATION(${target_name} qCam)

# COPY SECTION
set(COPY_SOURCES "")
set(COPY_DESTINATIONS "")
ADD_PLUGINLIBRARY_TO_COPY_LIST(${target_name} COPY_SOURCES COPY_DESTINATIONS)
ADD_QM_FILES_TO_COPY_LIST(${target_name} QM_FILES COPY_SOURCES COPY_DESTINATIONS)
POST_BUILD_COPY_FILES(${target_name} COPY_SOURCES COPY_DESTINATIONS)