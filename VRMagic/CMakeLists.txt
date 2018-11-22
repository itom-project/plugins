SET (target_name VRMagic)

project(${target_name})

message(STATUS "\n--------------- PLUGIN ${target_name} ---------------")

cmake_minimum_required(VERSION 2.8)

OPTION(BUILD_UNICODE "Build with unicode charset if set to ON, else multibyte charset." ON)
OPTION(BUILD_SHARED_LIBS "Build shared library." ON)
OPTION(BUILD_TARGET64 "Build for 64 bit target if set to ON or 32 bit if set to OFF." ON)
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

find_package(ITOM_SDK COMPONENTS dataobject itomCommonLib itomCommonQtLib itomWidgets REQUIRED)
include("${ITOM_SDK_DIR}/ItomBuildMacros.cmake")
FIND_PACKAGE_QT(ON Core Widgets Xml LinguistTools)

find_package(VisualLeakDetector QUIET)

IF(WIN32)
    FIND_PATH( VRMAGIC_INCLUDE_DIR vrmusbcam2.h PATHS "$ENV{CommonProgramFiles}\\VRmagic\\VRmUsbCamSDK" PATH_SUFFIXES include DOC "VRmagic include directory (must contain the file vrmusbcam2.h). Usually in %CommonProgramFiles%/VRmagic/VRmUsbCamSDK/include.")
ELSE()
    FIND_PATH( VRMAGIC_INCLUDE_DIR vrmusbcam2.h PATH_SUFFIXES include DOC "VRmagic include directory (must contain the file vrmusbcam2.h).")
ENDIF()

IF(VRMAGIC_INCLUDE_DIR)
    get_filename_component(VRMAGIC_SDK_DIR ${VRMAGIC_INCLUDE_DIR} DIRECTORY)
    
    ADD_DEFINITIONS(${QT_DEFINITIONS})
    IF (BUILD_UNICODE)
        ADD_DEFINITIONS(-DUNICODE -D_UNICODE)
    ENDIF (BUILD_UNICODE)
    ADD_DEFINITIONS(-DCMAKE)
    #ADD_DEFINITIONS(-DITOMWIDGETS_SHARED)

    IF(VISUALLEAKDETECTOR_FOUND AND VISUALLEAKDETECTOR_ENABLED)
        ADD_DEFINITIONS(-DVISUAL_LEAK_DETECTOR_CMAKE)
    ENDIF(VISUALLEAKDETECTOR_FOUND AND VISUALLEAKDETECTOR_ENABLED)

    # HANDLE VERSION (FROM GIT)
    UNSET(GIT_FOUND CACHE)
    find_package(Git)
    IF(BUILD_GIT_TAG AND GIT_FOUND)
    execute_process(COMMAND ${GIT_EXECUTABLE} log -1 --format=%h/%cD
        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
        OUTPUT_VARIABLE GITVERSION
        RESULT_VARIABLE GITRESULT
        ERROR_VARIABLE GITERROR
        OUTPUT_STRIP_TRAILING_WHITESPACE)
    #uncomment to enable output to cmake console
    #message(STATUS "Git-Version: " ${GITVERSION} " Err: " ${GITRESULT} " RES: " ${GITERROR})
    CONFIGURE_FILE(${CMAKE_CURRENT_SOURCE_DIR}/gitVersion.h.in ${CMAKE_CURRENT_BINARY_DIR}/gitVersion.h @ONLY)
    ENDIF ()
    IF(NOT EXISTS ${CMAKE_CURRENT_BINARY_DIR}/gitVersion.h)
    
    configure_file(${CMAKE_CURRENT_SOURCE_DIR}/gitVersion.h.in ${CMAKE_CURRENT_BINARY_DIR}/gitVersion.h @ONLY)
    ENDIF ()
    
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
        ${VRMAGIC_INCLUDE_DIR}
    )

    if(BUILD_TARGET64)
        LINK_DIRECTORIES(
            ${VRMAGIC_SDK_DIR}/lib/x64
        )
    else(BUILD_TARGET64)
        LINK_DIRECTORIES(
            ${VRMAGIC_SDK_DIR}/lib/x86
        )
    endif(BUILD_TARGET64)

    set(plugin_HEADERS
        ${CMAKE_CURRENT_SOURCE_DIR}/dialogVRMagic.h
        ${CMAKE_CURRENT_SOURCE_DIR}/dockWidgetVRMagic.h
        ${CMAKE_CURRENT_SOURCE_DIR}/VRMagic.h
        ${CMAKE_CURRENT_SOURCE_DIR}/pluginVersion.h
        ${CMAKE_CURRENT_BINARY_DIR}/gitVersion.h
    )

    set(plugin_UI
        ${CMAKE_CURRENT_SOURCE_DIR}/dialogVRMagic.ui
        ${CMAKE_CURRENT_SOURCE_DIR}/dockWidgetVRMagic.ui
    )

    set(plugin_SOURCES 

        ${CMAKE_CURRENT_SOURCE_DIR}/dialogVRMagic.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/dockWidgetVRMagic.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/VRMagic.cpp
    )

    set(plugin_RCC
    )

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
    
    TARGET_LINK_LIBRARIES(${target_name} ${QT_LIBRARIES} ${ITOM_SDK_LIBRARIES} vrmusbcam2 ${QT5_LIBRARIES} ${VISUALLEAKDETECTOR_LIBRARIES}) 

    IF (QT5_FOUND AND CMAKE_VERSION VERSION_LESS 3.0.2)
        qt5_use_modules(${target_name} ${QT_COMPONENTS})
    ENDIF (QT5_FOUND AND CMAKE_VERSION VERSION_LESS 3.0.2)
    
    #translation
    set (FILES_TO_TRANSLATE ${plugin_SOURCES} ${plugin_HEADERS} ${plugin_UI})
    PLUGIN_TRANSLATION(QM_FILES ${target_name} ${UPDATE_TRANSLATIONS} "${EXISTING_TRANSLATION_FILES}" ITOM_LANGUAGES "${FILES_TO_TRANSLATE}")
    
    #documentation
    PLUGIN_DOCUMENTATION(${target_name} VRMagic)

    # COPY SECTION
    set(COPY_SOURCES "")
    set(COPY_DESTINATIONS "")
    ADD_PLUGINLIBRARY_TO_COPY_LIST(${target_name} COPY_SOURCES COPY_DESTINATIONS)
    ADD_QM_FILES_TO_COPY_LIST(${target_name} QM_FILES COPY_SOURCES COPY_DESTINATIONS)
    POST_BUILD_COPY_FILES(${target_name} COPY_SOURCES COPY_DESTINATIONS)

    if(BUILD_TARGET64)
        file(GLOB dlls "${VRMAGIC_SDK_DIR}/lib/x64/*.dll")
    else(BUILD_TARGET64)    
        file(GLOB dlls "${VRMAGIC_SDK_DIR}/lib/x86/*.dll")
    endif(BUILD_TARGET64)
    POST_BUILD_COPY_FILE_TO_LIB_FOLDER(${target_name} dlls)
        
ELSE (VRMAGIC_INCLUDE_DIR)
    message(WARNING "${target_name}-SDK directory could not be found. ${target_name} will not be build")    
ENDIF (VRMAGIC_INCLUDE_DIR)