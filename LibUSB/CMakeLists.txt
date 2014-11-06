SET (target_name LibUSB)

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
find_package(OpenCV COMPONENTS core REQUIRED)
find_package(VisualLeakDetector QUIET)
find_package(LibUSB QUIET)

#FIND_PATH( LIBUSB_INC_DIR libusb.h CACHE PATH "Include directory of libUSB static include directory")

IF(LibUSB_FOUND)
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
        ${QT_QTCORE_INCLUDE_DIR}
        ${OpenCV_DIR}/include
        ${ITOM_SDK_INCLUDE_DIR}
        ${VISUALLEAKDETECTOR_INCLUDE_DIR}
	    ${LibUSB_INCLUDE_DIRS}
    )

    LINK_DIRECTORIES(
        ${OpenCV_DIR}/lib
    )

    if (NOT QT5_FOUND)
        INCLUDE(${QT_USE_FILE})
    endif (NOT QT5_FOUND)

    set(plugin_HEADERS
        ${ITOM_SDK_INCLUDE_DIR}/common/addInInterface.h
        ${ITOM_SDK_INCLUDE_DIR}/common/sharedStructures.h
        ${ITOM_SDK_INCLUDE_DIR}/common/sharedStructuresQt.h
        ${CMAKE_CURRENT_SOURCE_DIR}/ItomLibUSB.h
	    ${CMAKE_CURRENT_SOURCE_DIR}/dockWidgetLibUSB.h
        ${CMAKE_CURRENT_SOURCE_DIR}/pluginVersion.h
    )

    if (NOT QT5_FOUND)
        QT4_WRAP_CPP_ITOM(plugin_HEADERS_MOC ${plugin_HEADERS})
    endif (NOT QT5_FOUND)

    set(plugin_UI
	    ${CMAKE_CURRENT_SOURCE_DIR}/dockWidgetLibUSB.ui
    )

    if (QT5_FOUND)
        QT5_WRAP_UI(plugin_UI_MOC ${plugin_UI})
    else (QT5_FOUND)
        QT4_WRAP_UI_ITOM(plugin_UI_MOC ${plugin_UI})
    endif (QT5_FOUND)

    set(plugin_SOURCES 
        ${CMAKE_CURRENT_SOURCE_DIR}/ItomLibUSB.cpp
	    ${CMAKE_CURRENT_SOURCE_DIR}/dockWidgetLibUSB.cpp
    )

    #Add version information to the plugIn-dll unter MSVC
    if(MSVC)
        list(APPEND plugin_SOURCES ${ITOM_SDK_INCLUDE_DIR}/../pluginLibraryVersion.rc)
    endif(MSVC)

    file (GLOB EXISTING_TRANSLATION_FILES "translation/*.ts")

    ADD_LIBRARY(${target_name} ${LIBRARY_TYPE} ${plugin_SOURCES} ${plugin_HEADERS} ${plugin_HEADERS_MOC} ${plugin_UI_MOC} ${plugin_RCC_MOC} ${EXISTING_TRANSLATION_FILES})
    if (QT5_FOUND)
        TARGET_LINK_LIBRARIES(${target_name} ${OpenCV_LIBS} ${ITOM_SDK_dataobject_LIBRARY} ${LibUSB_LIBRARIES} ${ITOM_SDK_itomCommonLib_LIBRARY} ${ITOM_SDK_itomCommonQtLib_LIBRARY} ${VISUALLEAKDETECTOR_LIBRARIES})
    else (QT5_FOUND)
        TARGET_LINK_LIBRARIES(${target_name} ${QT_LIBRARIES} ${QT_QTUITOOLS_LIBRARY} ${QT_QTXML_LIBRARY} ${OpenCV_LIBS} ${ITOM_SDK_dataobject_LIBRARY} ${LibUSB_LIBRARIES} ${ITOM_SDK_itomCommonLib_LIBRARY} ${ITOM_SDK_itomCommonQtLib_LIBRARY} ${VISUALLEAKDETECTOR_LIBRARIES})
    endif (QT5_FOUND)
        
    #translation
    set (FILES_TO_TRANSLATE ${plugin_SOURCES} ${plugin_HEADERS} ${plugin_UI})
    PLUGIN_TRANSLATION(QM_FILES ${target_name} ${UPDATE_TRANSLATIONS} "${EXISTING_TRANSLATION_FILES}" ITOM_LANGUAGES "${FILES_TO_TRANSLATE}")

    PLUGIN_DOCUMENTATION(${target_name} libUSB)

    # COPY SECTION
    set(COPY_SOURCES "")
    set(COPY_DESTINATIONS "")
    ADD_PLUGINLIBRARY_TO_COPY_LIST(${target_name} COPY_SOURCES COPY_DESTINATIONS)
    if (QT5_FOUND)
    else (QT5_FOUND)
        ADD_QM_FILES_TO_COPY_LIST(${target_name} QM_FILES COPY_SOURCES COPY_DESTINATIONS)
    endif (QT5_FOUND)
    POST_BUILD_COPY_FILES(${target_name} COPY_SOURCES COPY_DESTINATIONS)
    
ELSE (LibUSB_FOUND)
    message(WARNING "${target_name}-API directory could not be found. ${target_name} will not be build")	
ENDIF (LibUSB_FOUND)
