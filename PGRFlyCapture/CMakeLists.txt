SET (target_name PGRFlyCapture)

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

include("${ITOM_SDK_DIR}/ItomBuildMacros.cmake")

IF(BUILD_SHARED_LIBS)
    SET(LIBRARY_TYPE SHARED)
ELSE(BUILD_SHARED_LIBS)
    SET(LIBRARY_TYPE STATIC)
ENDIF(BUILD_SHARED_LIBS)

find_package(ITOM_SDK COMPONENTS dataobject itomCommonLib itomCommonQtLib itomWidgets REQUIRED)
include("${ITOM_SDK_DIR}/ItomBuildMacros.cmake")
FIND_PACKAGE_QT(ON Core Widgets Xml LinguistTools)
find_package(VisualLeakDetector QUIET)

IF(WIN32)
    FIND_PATH( PGRFLYCAP_INCLUDE_DIR FlyCapture2.h DOC "Include directory of Point Gray Research FlyCapture API")
ELSE(WIN32)
    FIND_PATH( PGRFLYCAP_INCLUDE_DIR FlyCapture2.h PATHS /usr/include/flycapture DOC "Include directory of Point Gray Research FlyCapture API")
ENDIF(WIN32)

IF(PGRFLYCAP_INCLUDE_DIR)
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
        ${ITOM_SDK_INCLUDE_DIRS}
        ${ITOM_SDK_INCLUDE_DIR}/itomWidgets
        ${VISUALLEAKDETECTOR_INCLUDE_DIR}
        ${PGRFLYCAP_INCLUDE_DIR}
    )

    if(BUILD_TARGET64)
        SET(PGR_LIBDIR_HINT "${PGRFLYCAP_INCLUDE_DIR}/../lib64")
        SET(PGR_BINDIR_HINT "${PGRFLYCAP_INCLUDE_DIR}/../bin64")
    else(BUILD_TARGET64)
        SET(PGR_LIBDIR_HINT "${PGRFLYCAP_INCLUDE_DIR}/../lib")
        SET(PGR_BINDIR_HINT "${PGRFLYCAP_INCLUDE_DIR}/../bin")
    endif(BUILD_TARGET64)

    set(plugin_HEADERS
        ${CMAKE_CURRENT_SOURCE_DIR}/dialogPGRFlyCapture.h
        ${CMAKE_CURRENT_SOURCE_DIR}/dockWidgetPGRFlyCapture.h
        ${CMAKE_CURRENT_SOURCE_DIR}/PGRFlyCapture.h
        ${CMAKE_CURRENT_SOURCE_DIR}/pluginVersion.h
    )
    
    set(plugin_SOURCES 
        ${CMAKE_CURRENT_SOURCE_DIR}/dialogPGRFlyCapture.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/dockWidgetPGRFlyCapture.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/PGRFlyCapture.cpp
    )

    set(plugin_UI
        ${CMAKE_CURRENT_SOURCE_DIR}/dialogPGRFlyCapture.ui
        ${CMAKE_CURRENT_SOURCE_DIR}/dockWidgetPGRFlyCapture.ui
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
        QT5_ADD_RESOURCES(designer_rcc_MOC ${plugin_RCC})
    else (QT5_FOUND)
        QT4_WRAP_CPP_ITOM(plugin_HEADERS_MOC ${plugin_HEADERS})
        QT4_WRAP_UI_ITOM(plugin_UI_MOC ${plugin_UI})
        QT4_ADD_RESOURCES(designer_rcc_MOC ${plugin_RCC})
    endif (QT5_FOUND)

    file (GLOB EXISTING_TRANSLATION_FILES "translation/*.ts")
    
    IF(UNIX)
        FIND_LIBRARY(FLYCAPTURE_LIBRARY NAMES libflycapture flycapture PATHS /usr/lib NO_DEFAULT_PATH)
    ELSE(UNIX)
        IF(MSVC10)
            find_library(FLYCAPTURE_LIBRARY NAMES "FlyCapture2_v100"  PATHS ${PGR_LIBDIR_HINT} NO_DEFAULT_PATH)
        ELSEIF(MSVC9)
            find_library(FLYCAPTURE_LIBRARY NAMES "FlyCapture2_v90"  PATHS ${PGR_LIBDIR_HINT} NO_DEFAULT_PATH)
        ELSE(MSVC10)
            find_library(FLYCAPTURE_LIBRARY NAMES "FlyCapture2"  PATHS ${PGR_LIBDIR_HINT} NO_DEFAULT_PATH)
        ENDIF(MSVC10)
    ENDIF(UNIX)
    
    IF(FLYCAPTURE_LIBRARY)
        MESSAGE(STATUS "FlyCapture library found at ${FLYCAPTURE_LIBRARY}")
    ELSE(FLYCAPTURE_LIBRARY)
        MESSAGE(FATAL "FlyCapture library not found")
    ENDIF(FLYCAPTURE_LIBRARY)
    
    IF(WIN32)
        IF(MSVC10)
            find_file(FLYCAPTURE_BINARY "FlyCapture2_v100.dll"  PATHS ${PGR_BINDIR_HINT} NO_DEFAULT_PATH)
        ELSEIF(MSVC9)
            find_file(FLYCAPTURE_BINARY "FlyCapture2_v90.dll"  PATHS ${PGR_BINDIR_HINT} NO_DEFAULT_PATH)
        ELSE(MSVC10)
            find_file(FLYCAPTURE_BINARY "FlyCapture2.dll"  PATHS ${PGR_BINDIR_HINT} NO_DEFAULT_PATH)
        ENDIF(MSVC10)
    
        find_file(FLYCAPTURE_LIBIOMP5MD "libiomp5md.dll"  PATHS ${PGR_BINDIR_HINT} NO_DEFAULT_PATH)
    
        IF(FLYCAPTURE_BINARY)
            MESSAGE(STATUS "FlyCapture binary found at ${FLYCAPTURE_BINARY}")
        ELSE(FLYCAPTURE_BINARY)
            MESSAGE(FATAL "FlyCapture binary not found")
        ENDIF(FLYCAPTURE_BINARY)

        IF(FLYCAPTURE_LIBIOMP5MD)
            MESSAGE(STATUS "libiomp5md.dll found at ${FLYCAPTURE_LIBIOMP5MD}")
        ELSE(FLYCAPTURE_LIBIOMP5MD)
            MESSAGE(FATAL "libiomp5md.dll not found")
        ENDIF(FLYCAPTURE_LIBIOMP5MD)
    ENDIF(WIN32)
    
    ADD_LIBRARY(${target_name} ${LIBRARY_TYPE} ${plugin_SOURCES} ${plugin_HEADERS} ${plugin_HEADERS_MOC} ${plugin_UI_MOC} ${plugin_RCC_MOC} ${EXISTING_TRANSLATION_FILES})
    TARGET_LINK_LIBRARIES(${target_name} ${QT_LIBRARIES} ${ITOM_SDK_LIBRARIES} ${FLYCAPTURE_LIBRARY} ${VISUALLEAKDETECTOR_LIBRARIES}) 

    if (QT5_FOUND)
        qt5_use_modules(${target_name} ${QT_COMPONENTS})
    endif (QT5_FOUND)
    
    PLUGIN_DOCUMENTATION(${target_name} pgrFlyCapture)
    
    #translation
    set (FILES_TO_TRANSLATE ${plugin_SOURCES} ${plugin_HEADERS} ${plugin_UI})
    PLUGIN_TRANSLATION(QM_FILES ${target_name} ${UPDATE_TRANSLATIONS} "${EXISTING_TRANSLATION_FILES}" ITOM_LANGUAGES "${FILES_TO_TRANSLATE}")

    # COPY SECTION
    set(COPY_SOURCES "")
    set(COPY_DESTINATIONS "")
    ADD_PLUGINLIBRARY_TO_COPY_LIST(${target_name} COPY_SOURCES COPY_DESTINATIONS)
    ADD_QM_FILES_TO_COPY_LIST(${target_name} QM_FILES COPY_SOURCES COPY_DESTINATIONS)
    POST_BUILD_COPY_FILES(${target_name} COPY_SOURCES COPY_DESTINATIONS)
    
    IF(WIN32)
        POST_BUILD_COPY_FILE_TO_LIB_FOLDER(${target_name} FLYCAPTURE_BINARY)
        POST_BUILD_COPY_FILE_TO_LIB_FOLDER(${target_name} FLYCAPTURE_LIBIOMP5MD)
    ENDIF(WIN32)
    
ELSE (PGRFLYCAP_INCLUDE_DIR)
    message(WARNING "${target_name}-API directory could not be found. ${target_name} will not be build")    
ENDIF (PGRFLYCAP_INCLUDE_DIR)
