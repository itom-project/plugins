SET (target_name glDisplay)

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

SET (CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR} ${ITOM_FIND_PACKAGE_PATH} ${ITOM_FIND_PACKAGE_PATH} ${ITOM_SDK_DIR})

IF(BUILD_SHARED_LIBS)
    SET(LIBRARY_TYPE SHARED)
ELSE(BUILD_SHARED_LIBS)
    SET(LIBRARY_TYPE STATIC)
ENDIF(BUILD_SHARED_LIBS)

find_package(ITOM_SDK COMPONENTS dataobject itomCommonLib itomCommonQtLib itomWidgets REQUIRED)
include("${ITOM_SDK_DIR}/ItomBuildMacros.cmake")
FIND_PACKAGE_QT(ON Widgets Xml OpenGL LinguistTools)

find_package(OpenCV COMPONENTS core REQUIRED)
find_package(OpenGL REQUIRED)

find_package(VisualLeakDetector QUIET)

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

INCLUDE_DIRECTORIES(
    ${CMAKE_CURRENT_BINARY_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${ITOM_SDK_INCLUDE_DIRS}
    ${VISUALLEAKDETECTOR_INCLUDE_DIR}
)

set(plugin_HEADERS
    ${CMAKE_CURRENT_SOURCE_DIR}/dialogGLDisplay.h
    ${CMAKE_CURRENT_SOURCE_DIR}/dockWidgetGLDisplay.h
    ${CMAKE_CURRENT_SOURCE_DIR}/glDisplay.h
    ${CMAKE_CURRENT_SOURCE_DIR}/glWindow.h
    ${CMAKE_CURRENT_SOURCE_DIR}/pluginVersion.h
)

set(plugin_SOURCES 
    ${CMAKE_CURRENT_SOURCE_DIR}/dialogGLDisplay.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/dockWidgetGLDisplay.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/glDisplay.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/glWindow.cpp
)

set(plugin_UI
    ${CMAKE_CURRENT_SOURCE_DIR}/dialogGLDisplay.ui
    ${CMAKE_CURRENT_SOURCE_DIR}/dockWidgetGLDisplay.ui
)

set(plugin_RCC
)

if (QT5_FOUND)
    #if automoc if OFF, you also need to call QT5_WRAP_CPP here
    QT5_WRAP_UI(plugin_UI_MOC ${plugin_UI})
    QT5_ADD_RESOURCES(designer_rcc_MOC ${plugin_RCC})
else (QT5_FOUND)
    QT4_WRAP_CPP_ITOM(plugin_HEADERS_MOC ${plugin_HEADERS})
    QT4_WRAP_UI_ITOM(plugin_UI_MOC ${plugin_UI})
    QT4_ADD_RESOURCES(designer_rcc_MOC ${plugin_RCC})
endif (QT5_FOUND)

#Add version information to the plugIn-dll unter MSVC
if(MSVC)
    list(APPEND plugin_SOURCES ${ITOM_SDK_INCLUDE_DIR}/../pluginLibraryVersion.rc)
endif(MSVC)    

file (GLOB EXISTING_TRANSLATION_FILES "translation/*.ts")

ADD_LIBRARY(${target_name} ${LIBRARY_TYPE} ${plugin_SOURCES} ${plugin_HEADERS} ${plugin_HEADERS_MOC} ${plugin_UI_MOC} ${plugin_RCC_MOC} ${EXISTING_TRANSLATION_FILES})
TARGET_LINK_LIBRARIES(${target_name} ${QT_LIBRARIES} ${OPENGL_LIBRARIES} ${OpenCV_LIBS} ${ITOM_SDK_LIBRARIES} ${VISUALLEAKDETECTOR_LIBRARIES})

if (QT5_FOUND)
        qt5_use_modules(${target_name} ${QT_COMPONENTS})
endif (QT5_FOUND)

#translation
set (FILES_TO_TRANSLATE ${plugin_SOURCES} ${plugin_HEADERS} ${plugin_UI})
PLUGIN_TRANSLATION(QM_FILES ${target_name} ${UPDATE_TRANSLATIONS} "${EXISTING_TRANSLATION_FILES}" ITOM_LANGUAGES "${FILES_TO_TRANSLATE}")

# COPY SECTION
set(COPY_SOURCES "")
set(COPY_DESTINATIONS "")
ADD_PLUGINLIBRARY_TO_COPY_LIST(${target_name} COPY_SOURCES COPY_DESTINATIONS)

IF(WIN32 AND NOT QT5_FOUND)
    POST_BUILD_COPY_FILE_TO_LIB_FOLDER(${target_name} GLEW_RUNTIME_LIBRARIES)
ENDIF(WIN32 AND NOT QT5_FOUND)

ADD_QM_FILES_TO_COPY_LIST(${target_name} QM_FILES COPY_SOURCES COPY_DESTINATIONS)
POST_BUILD_COPY_FILES(${target_name} COPY_SOURCES COPY_DESTINATIONS)