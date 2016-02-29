SET (target_name UhlText)

project(${target_name})

cmake_minimum_required(VERSION 2.8)

OPTION(BUILD_UNICODE "Build with unicode charset if set to ON, else multibyte charset." ON)
OPTION(BUILD_SHARED_LIBS "Build shared library." ON)
OPTION(BUILD_TARGET64 "Build for 64 bit target if set to ON or 32 bit if set to OFF." OFF)
OPTION(UPDATE_TRANSLATIONS "Update source translation translation/*.ts files (WARNING: make clean will delete the source .ts files! Danger!)")
SET (ITOM_SDK_DIR "" CACHE PATH "base path to itom_sdk")
SET (CMAKE_DEBUG_POSTFIX "d" CACHE STRING "Adds a postfix for debug-built libraries.")
set(CMAKE_INCLUDE_CURRENT_DIR ON)

SET (CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR} ${ITOM_SDK_DIR})

IF(BUILD_SHARED_LIBS)
    SET(LIBRARY_TYPE SHARED)
ELSE(BUILD_SHARED_LIBS)
    SET(LIBRARY_TYPE STATIC)
ENDIF(BUILD_SHARED_LIBS)


find_package(ITOM_SDK COMPONENTS dataobject itomCommonLib itomCommonQtLib REQUIRED)
include("${ITOM_SDK_DIR}/ItomBuildMacros.cmake") #include this mandatory macro file (important)
FIND_PACKAGE_QT(ON Core LinguistTools Widgets)
find_package(VisualLeakDetector QUIET) #silently detects the VisualLeakDetector for Windows (memory leak detector, optional)

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
    ${CMAKE_CURRENT_SOURCE_DIR}/../UhlRegister
    ${ITOM_SDK_INCLUDE_DIRS}
    ${VISUALLEAKDETECTOR_INCLUDE_DIR}
)

LINK_DIRECTORIES(
)

set(plugin_HEADERS
    ${CMAKE_CURRENT_SOURCE_DIR}/../UhlRegister/dialogUhl.h
    ${CMAKE_CURRENT_SOURCE_DIR}/../UhlRegister/dockWidgetUhl.h
    ${CMAKE_CURRENT_SOURCE_DIR}/pluginVersion.h
    ${CMAKE_CURRENT_SOURCE_DIR}/UhlText.h
)

set(plugin_UI
    ${CMAKE_CURRENT_SOURCE_DIR}/../UhlRegister/dialogUhl.ui
    ${CMAKE_CURRENT_SOURCE_DIR}/../UhlRegister/dockWidgetUhl.ui
)

set(plugin_SOURCES 
    ${CMAKE_CURRENT_SOURCE_DIR}/../UhlRegister/dialogUhl.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../UhlRegister/dockWidgetUhl.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/UhlText.cpp
)

set(plugin_RCC
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

# UhlText
# -------
file (GLOB EXISTING_TRANSLATION_FILES_TEXT "translation/UhlText.ts")

ADD_LIBRARY(${target_name} ${LIBRARY_TYPE} ${plugin_SOURCES} ${plugin_HEADERS} ${plugin_HEADERS_MOC} ${plugin_UI_MOC} ${plugin_RCC_MOC} ${EXISTING_TRANSLATION_FILES_TEXT})
TARGET_LINK_LIBRARIES(${target_name} ${QT_LIBRARIES} ${VISUALLEAKDETECTOR_LIBRARIES} ${ITOM_SDK_LIBRARIES})
if (QT5_FOUND)
    qt5_use_modules(${target_name} ${QT_COMPONENTS}) #special command for Qt5
endif (QT5_FOUND)

#translation
set (QM_FILES_TEXT)
set (FILES_TO_TRANSLATE_TEXT  ${plugin_SOURCES} ${plugin_HEADERS} ${plugin_ui})
PLUGIN_TRANSLATION(QM_FILES_TEXT ${target_name} ${UPDATE_TRANSLATIONS} "${EXISTING_TRANSLATION_FILES_TEXT}" ITOM_LANGUAGES "${FILES_TO_TRANSLATE_TEXT}")

PLUGIN_DOCUMENTATION(${target_name} uhlText)

# COPY SECTION
set(COPY_SOURCES_TEXT "")
set(COPY_DESTINATIONS_TEXT "")
ADD_PLUGINLIBRARY_TO_COPY_LIST(${target_name} COPY_SOURCES_TEXT COPY_DESTINATIONS_TEXT)
ADD_QM_FILES_TO_COPY_LIST(${target_name} QM_FILES_TEXT COPY_SOURCES_TEXT COPY_DESTINATIONS_TEXT)
POST_BUILD_COPY_FILES(${target_name} COPY_SOURCES_TEXT COPY_DESTINATIONS_TEXT)