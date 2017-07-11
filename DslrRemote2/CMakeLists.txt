SET (target_name DslrRemote2)

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

SET (CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR} ${ITOM_SDK_DIR})

IF(BUILD_SHARED_LIBS)
    SET(LIBRARY_TYPE SHARED)
ELSE(BUILD_SHARED_LIBS)
    SET(LIBRARY_TYPE STATIC)
ENDIF(BUILD_SHARED_LIBS)

find_package(ITOM_SDK COMPONENTS dataobject itomCommonLib itomCommonQtLib REQUIRED)
include("${ITOM_SDK_DIR}/ItomBuildMacros.cmake")
FIND_PACKAGE_QT(ON Core Widgets LinguistTools)
find_package(VisualLeakDetector QUIET)
find_package(LibUSB QUIET)

IF(LibUSB_INCLUDE_DIRS)
    ADD_SUBDIRECTORY(libptp2-1.2.0)

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

        LINK_DIRECTORIES(
                ${LIBUSB_API_DIR}/lib
        )

    INCLUDE_DIRECTORIES(
        ${CMAKE_CURRENT_BINARY_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}
                ${CMAKE_CURRENT_SOURCE_DIR}/libptp2
        ${LibUSB_INCLUDE_DIRS}
        ${ITOM_SDK_INCLUDE_DIRS}
        ${VISUALLEAKDETECTOR_INCLUDE_DIR}
    )

    set(plugin_HEADERS
        ${CMAKE_CURRENT_SOURCE_DIR}/pluginVersion.h
        ${CMAKE_CURRENT_SOURCE_DIR}/DslrRemote.h
        ${CMAKE_CURRENT_SOURCE_DIR}/ptpCam.h
    )

    set(plugin_UI
    )

    set(plugin_SOURCES
        ${CMAKE_CURRENT_SOURCE_DIR}/ptpCam.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/DslrRemote.cpp
    )

    set(plugin_RCC
        #add absolute pathes to any *.qrc resource files here
    )

    #Add version information to the plugIn-dll unter MSVC
    if(MSVC)
        list(APPEND plugin_SOURCES ${ITOM_SDK_INCLUDE_DIR}/../pluginLibraryVersion.rc)
    endif(MSVC)

    #################################################################
    # Qt related pre-processing of the files above
    # (These methods create the moc, rcc and uic process.)
    #################################################################
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
    #handle translations END STEP 1

    IF (WIN32)
        SET(PTPLIB optimized ./libptp2-1.2.0/$(Configuration)/libptp2 debug ./libptp2-1.2.0/$(Configuration)/libptp2d)
    ELSE (WIN32)
        SET(PTPLIB libptp2)
    ENDIF (WIN32)

    ADD_LIBRARY(${target_name} ${LIBRARY_TYPE} ${plugin_SOURCES} ${plugin_HEADERS} ${plugin_HEADERS_MOC} ${plugin_UI_MOC} ${plugin_RCC_MOC} ${EXISTING_TRANSLATION_FILES})

    IF (WIN32)
        TARGET_LINK_LIBRARIES(${target_name} ${ITOM_SDK_LIBRARIES} ${VISUALLEAKDETECTOR_LIBRARIES} ${QT5_LIBRARIES} ${QT_LIBRARIES} ${PTPLIB} ${LibUSB_LIBRARY})
    ELSE(WIN32)
        TARGET_LINK_LIBRARIES(${target_name} ${ITOM_SDK_LIBRARIES} ${VISUALLEAKDETECTOR_LIBRARIES} ${QT5_LIBRARIES} ${QT_LIBRARIES} ${PTPLIB} ${LibUSB_LIBRARY})
    ENDIF(WIN32)

    IF (QT5_FOUND AND CMAKE_VERSION VERSION_LESS 3.0.2)
        qt5_use_modules(${target_name} ${QT_COMPONENTS})
    ENDIF (QT5_FOUND AND CMAKE_VERSION VERSION_LESS 3.0.2)

    #translation
    set (FILES_TO_TRANSLATE ${plugin_SOURCES} ${plugin_HEADERS} ${plugin_UI})
    PLUGIN_TRANSLATION(QM_FILES ${target_name} ${UPDATE_TRANSLATIONS} "${EXISTING_TRANSLATION_FILES}" ITOM_LANGUAGES "${FILES_TO_TRANSLATE}")

    #documentation
    PLUGIN_DOCUMENTATION(${target_name} dslrRemote)

    # COPY SECTION
    set(COPY_SOURCES "")
    set(COPY_DESTINATIONS "")
    #LIST(APPEND COPY_SOURCES "$<TARGET_FILE:dcraw>")
    #LIST(APPEND COPY_DESTINATIONS "${ITOM_APP_DIR}/lib")
    ADD_PLUGINLIBRARY_TO_COPY_LIST(${target_name} COPY_SOURCES COPY_DESTINATIONS)
    ADD_QM_FILES_TO_COPY_LIST(${target_name} QM_FILES COPY_SOURCES COPY_DESTINATIONS)
    POST_BUILD_COPY_FILES(${target_name} COPY_SOURCES COPY_DESTINATIONS)

ELSE (LibUSB_INCLUDE_DIRS)
    message(WARNING "LibUSB_INCLUDE_DIRS directory could not be found. ${target_name} will not be build.")    
ENDIF (LibUSB_INCLUDE_DIRS)