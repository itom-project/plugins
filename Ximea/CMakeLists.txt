SET (target_name Ximea)

project(${target_name})

cmake_minimum_required(VERSION 2.8)

OPTION(BUILD_UNICODE "Build with unicode charset if set to ON, else multibyte charset." ON)
OPTION(BUILD_SHARED_LIBS "Build shared library." ON)
OPTION(BUILD_TARGET64 "Build for 64 bit target if set to ON or 32 bit if set to OFF." OFF)
OPTION(UPDATE_TRANSLATIONS "Update source translation translation/*.ts files (WARNING: make clean will delete the source .ts files! Danger!)")
OPTION(XIMEA_BUILD_OLD_API "Build old API-Version from 2012" OFF)
OPTION(XIMEA_BUILD_INCLUDE_SHADING "Build with shading support from API-Version" OFF)
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

find_package(ITOM_SDK COMPONENTS dataobject itomCommonLib itomCommonQtLib itomWidgets REQUIRED)
include("${ITOM_SDK_DIR}/ItomBuildMacros.cmake")
FIND_PACKAGE_QT(ON Core Widgets Xml LinguistTools)

find_package(OpenCV COMPONENTS core REQUIRED)
find_package(VisualLeakDetector QUIET)

FIND_PATH( XIMEA_APIDIR m3Api.h CACHE PATH "XIMEA Api directory (must contain the file m3Api.h)")

IF(XIMEA_APIDIR)

    IF(XIMEA_BUILD_OLD_API)
        ADD_DEFINITIONS(-DUSE_OLD_API)
    ENDIF(XIMEA_BUILD_OLD_API)

    IF(XIMEA_BUILD_INCLUDE_SHADING)
        ADD_DEFINITIONS(-DUSE_SHADING)
    ENDIF(XIMEA_BUILD_INCLUDE_SHADING)    
    
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

    INCLUDE_DIRECTORIES(
        ${CMAKE_CURRENT_BINARY_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${OpenCV_DIR}/include
        ${ITOM_SDK_INCLUDE_DIRS}
        ${VISUALLEAKDETECTOR_INCLUDE_DIR}
        ${XIMEA_APIDIR}
    )

    if(BUILD_TARGET64)
        LINK_DIRECTORIES(
            ${OpenCV_DIR}/lib
            ${XIMEA_APIDIR}/x64
        )
    else(BUILD_TARGET64)
        LINK_DIRECTORIES(
            ${OpenCV_DIR}/lib
            ${XIMEA_APIDIR}/x86
        )
    endif(BUILD_TARGET64)

    set(plugin_HEADERS
        ${CMAKE_CURRENT_SOURCE_DIR}/dialogXimea.h
        ${CMAKE_CURRENT_SOURCE_DIR}/dockWidgetXimea.h
        ${CMAKE_CURRENT_SOURCE_DIR}/Ximea.h
        ${CMAKE_CURRENT_SOURCE_DIR}/pluginVersion.h
    )

    set(plugin_UI
        ${CMAKE_CURRENT_SOURCE_DIR}/dialogXimea.ui
        ${CMAKE_CURRENT_SOURCE_DIR}/dockWidgetXimea.ui
    )

    set(plugin_SOURCES 
        ${CMAKE_CURRENT_SOURCE_DIR}/dialogXimea.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/dockWidgetXimea.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/Ximea.cpp
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

    file (GLOB EXISTING_TRANSLATION_FILES "translation/*.ts")

    ADD_LIBRARY(${target_name} ${LIBRARY_TYPE} ${plugin_SOURCES} ${plugin_HEADERS} ${plugin_HEADERS_MOC} ${plugin_UI_MOC} ${plugin_RCC_MOC} ${EXISTING_TRANSLATION_FILES})
    TARGET_LINK_LIBRARIES(${target_name} ${OpenCV_LIBS} ${ITOM_SDK_LIBRARIES} ${VISUALLEAKDETECTOR_LIBRARIES} ${QT_LIBRARIES})
    if (QT5_FOUND)
        qt5_use_modules(${target_name} ${QT_COMPONENTS})
    endif (QT5_FOUND)

    #translation
    set (FILES_TO_TRANSLATE ${plugin_SOURCES} ${plugin_HEADERS} ${plugin_UI})
    PLUGIN_TRANSLATION(QM_FILES ${target_name} ${UPDATE_TRANSLATIONS} "${EXISTING_TRANSLATION_FILES}" ITOM_LANGUAGES "${FILES_TO_TRANSLATE}")
    
    PLUGIN_DOCUMENTATION(${target_name} ximea)

    # COPY SECTION
    set(COPY_SOURCES "")
    set(COPY_DESTINATIONS "")
    ADD_PLUGINLIBRARY_TO_COPY_LIST(${target_name} COPY_SOURCES COPY_DESTINATIONS)
    ADD_QM_FILES_TO_COPY_LIST(${target_name} QM_FILES COPY_SOURCES COPY_DESTINATIONS)
    POST_BUILD_COPY_FILES(${target_name} COPY_SOURCES COPY_DESTINATIONS)

    if(BUILD_TARGET64)
        file(GLOB dlls "${XIMEA_APIDIR}/x64/*.dll")
    else(BUILD_TARGET64)    
        file(GLOB dlls "${XIMEA_APIDIR}/x86/*.dll")
    endif(BUILD_TARGET64)
    POST_BUILD_COPY_FILE_TO_LIB_FOLDER(${target_name} dlls)
ELSE (XIMEA_APIDIR)
    message(WARNING "${target_name}-API directory could not be found. ${target_name} will not be build")    
ENDIF (XIMEA_APIDIR)