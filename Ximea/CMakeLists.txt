set(target_name Ximea) 

project(${target_name})

message(STATUS "\n--------------- PLUGIN ${target_name} ---------------")

cmake_minimum_required(VERSION 3.0.2)

option(BUILD_UNICODE "Build with unicode charset if set to ON, else multibyte charset." ON)
option(BUILD_SHARED_LIBS "Build shared library." ON)
option(BUILD_TARGET64 "Build for 64 bit target if set to ON or 32 bit if set to OFF." ON)
option(UPDATE_TRANSLATIONS "Update source translation translation/*.ts files (WARNING: make clean will delete the source .ts files! Danger!)")
set(XIMEA_BUILD_API_VERSION "4.10" CACHE STRING "Expected API version")
set_property(CACHE XIMEA_BUILD_API_VERSION PROPERTY STRINGS 3.16 4.10 "4.15 or newer")
option(XIMEA_BUILD_INCLUDE_SHADING "Build with shading support from API-Version" OFF)
set(ITOM_SDK_DIR "" CACHE PATH "base path to itom_sdk")
set(CMAKE_DEBUG_POSTFIX "d" CACHE STRING "Adds a postfix for debug-built libraries.")
set(ITOM_LANGUAGES "de" CACHE STRING "semicolon separated list of languages that should be created (en must not be given since it is the default)")

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR} ${ITOM_SDK_DIR})

include("${ITOM_SDK_DIR}/ItomBuildMacros.cmake")
INIT_ITOM_LIBRARY() #apply general project settings, CMake configurations, policies... for itom and its libraries

if(BUILD_SHARED_LIBS)
    set(LIBRARY_TYPE SHARED)
else(BUILD_SHARED_LIBS)
    set(LIBRARY_TYPE STATIC)
endif(BUILD_SHARED_LIBS)

find_package(ITOM_SDK COMPONENTS dataobject itomCommonLib itomCommonQtLib itomWidgets REQUIRED)
FIND_PACKAGE_QT(ON Core Widgets Xml LinguistTools)

find_package(OpenCV COMPONENTS core REQUIRED)
find_package(VisualLeakDetector QUIET)

find_path( XIMEA_APIDIR m3Api.h DOC "XIMEA Api directory (must contain the file m3Api.h)")

if(XIMEA_APIDIR)

    if(XIMEA_BUILD_API_VERSION VERSION_EQUAL "3.16")
        add_definitions(-DUSE_API_3_16)
    else(XIMEA_BUILD_API_VERSION VERSION_EQUAL "4.10")
        add_definitions(-DUSE_API_4_10)
    endif(XIMEA_BUILD_API_VERSION VERSION_EQUAL "3.16")
    
    message(STATUS "Ximea used API VERSION: ${XIMEA_BUILD_API_VERSION}" )

    if(XIMEA_BUILD_INCLUDE_SHADING)
        add_definitions(-DUSE_SHADING)
    endif(XIMEA_BUILD_INCLUDE_SHADING)    
    
    add_definitions(${QT_DEFINITIONS})
    if(BUILD_UNICODE)
        add_definitions(-DUNICODE -D_UNICODE)
    endif(BUILD_UNICODE)
    add_definitions(-DCMAKE)
    #add_definitions(-DITOMWIDGETS_SHARED)

    if(VISUALLEAKDETECTOR_FOUND AND VISUALLEAKDETECTOR_ENABLED)
        add_definitions(-DVISUAL_LEAK_DETECTOR_CMAKE)
    endif(VISUALLEAKDETECTOR_FOUND AND VISUALLEAKDETECTOR_ENABLED)

    # HANDLE VERSION (FROM GIT)
    unset(GIT_FOUND CACHE)
    find_package(Git)
    if(BUILD_GIT_TAG AND GIT_FOUND)
    execute_process(COMMAND ${GIT_EXECUTABLE} log -1 --format=%h/%cD
        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
        OUTPUT_VARIABLE GITVERSION
        RESULT_VARIABLE GITRESULT
        ERROR_VARIABLE GITERROR
        OUTPUT_STRIP_TRAILING_WHITESPACE)
    #uncomment to enable output to cmake console
    #message(STATUS "Git-Version: " ${GITVERSION} " Err: " ${GITRESULT} " RES: " ${GITERROR})
    configure_file(${CMAKE_CURRENT_SOURCE_DIR}/gitVersion.h.in ${CMAKE_CURRENT_BINARY_DIR}/gitVersion.h @ONLY)
    endif(BUILD_GIT_TAG AND GIT_FOUND)
    if(NOT EXISTS ${CMAKE_CURRENT_BINARY_DIR}/gitVersion.h)
    
    configure_file(${CMAKE_CURRENT_SOURCE_DIR}/gitVersion.h.in ${CMAKE_CURRENT_BINARY_DIR}/gitVersion.h @ONLY)
    endif(NOT EXISTS ${CMAKE_CURRENT_BINARY_DIR}/gitVersion.h)
    
    # default build types are None, Debug, Release, RelWithDebInfo and MinRelSize
    if(DEFINED CMAKE_BUILD_TYPE)
        set(CMAKE_BUILD_TYPE ${CMAKE_BUILD_TYPE} CACHE STRING "Choose the type of build, options are: None(CMAKE_CXX_FLAGS or CMAKE_C_FLAGS used) Debug Release RelWithDebInfo MinSizeRel.")
    else()
        set(CMAKE_BUILD_TYPE Debug CACHE STRING "Choose the type of build, options are: None(CMAKE_CXX_FLAGS or CMAKE_C_FLAGS used) Debug Release RelWithDebInfo MinSizeRel.")
    endif()

    message(STATUS ${CMAKE_CURRENT_BINARY_DIR})

    include_directories(
        ${CMAKE_CURRENT_BINARY_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${QT_QTCORE_INCLUDE_DIR}
        ${OpenCV_DIR}/include
        ${ITOM_SDK_INCLUDE_DIR}
        ${ITOM_SDK_INCLUDE_DIR}/itomWidgets
        ${VISUALLEAKDETECTOR_INCLUDE_DIR}
        ${XIMEA_APIDIR}
    )

    if(BUILD_TARGET64)
        link_directories(
            ${OpenCV_DIR}/lib
            ${XIMEA_APIDIR}/x64
        )
    else(BUILD_TARGET64)
        link_directories(
            ${OpenCV_DIR}/lib
            ${XIMEA_APIDIR}/x86
        )
    endif(BUILD_TARGET64)

    set(plugin_HEADERS
        ${CMAKE_CURRENT_SOURCE_DIR}/dialogXimea.h
        ${CMAKE_CURRENT_SOURCE_DIR}/dockWidgetXimea.h
        ${CMAKE_CURRENT_SOURCE_DIR}/Ximea.h
        ${CMAKE_CURRENT_SOURCE_DIR}/pluginVersion.h
        ${CMAKE_CURRENT_BINARY_DIR}/gitVersion.h
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

    if(MSVC)
        list(APPEND plugin_SOURCES ${ITOM_SDK_INCLUDE_DIR}/../pluginLibraryVersion.rc)
    endif()

    if(QT5_FOUND)
        #if automoc if OFF, you also need to call QT5_WRAP_CPP here
        QT5_WRAP_UI(plugin_UI_MOC ${plugin_UI})
        QT5_ADD_RESOURCES(plugin_RCC_MOC ${plugin_RCC})
    else()
        message(SEND_ERROR "Currently only Qt5 is supported")
    endif()

    #handle translations BEGIN STEP 1
    set(FILES_TO_TRANSLATE ${plugin_SOURCES} ${plugin_HEADERS} ${plugin_UI})
    set(TRANSLATION_OUTPUT_FILES)
    set(TRANSLATIONS_FILES)


    file(GLOB EXISTING_TRANSLATION_FILES "translation/*.ts")

    add_library(${target_name} ${LIBRARY_TYPE} ${plugin_SOURCES} ${plugin_HEADERS} ${plugin_HEADERS_MOC} ${plugin_UI_MOC} ${plugin_RCC_MOC} ${EXISTING_TRANSLATION_FILES})
    target_link_libraries(${target_name} ${QT_LIBRARIES} ${QT_QTUITOOLS_LIBRARY} ${QT_QTXML_LIBRARY} ${OpenCV_LIBS} ${ITOM_SDK_LIBRARIES} ${ITOM_SDK_dataobject_LIBRARY} ${QT5_LIBRARIES} ${VISUALLEAKDETECTOR_LIBRARIES} ${ITOM_SDK_itomCommonLib_LIBRARY} ${ITOM_SDK_itomCommonQtLib_LIBRARY})    

    #translation
    set(FILES_TO_TRANSLATE ${plugin_SOURCES} ${plugin_HEADERS} ${plugin_UI})
    PLUGIN_TRANSLATION(QM_FILES ${target_name} ${UPDATE_TRANSLATIONS} "${EXISTING_TRANSLATION_FILES}" ITOM_LANGUAGES "${FILES_TO_TRANSLATE}")
    
    PLUGIN_DOCUMENTATION(${target_name} ximea)

    # COPY SECTION
    set(COPY_SOURCES "")
    set(COPY_DESTINATIONS "")
    ADD_PLUGINLIBRARY_TO_COPY_LIST(${target_name} COPY_SOURCES COPY_DESTINATIONS)
    ADD_QM_FILES_TO_COPY_LIST(${target_name} QM_FILES COPY_SOURCES COPY_DESTINATIONS)
    POST_BUILD_COPY_FILES(${target_name} COPY_SOURCES COPY_DESTINATIONS)
    
if(WIN32)
    if(BUILD_TARGET64)
        file(GLOB dlls "${XIMEA_APIDIR}/x64/*.dll")
    else(BUILD_TARGET64)    
        file(GLOB dlls "${XIMEA_APIDIR}/x86/*.dll")
    endif(BUILD_TARGET64)
    POST_BUILD_COPY_FILE_TO_LIB_FOLDER(${target_name} dlls)
endif(WIN32)
        
else(XIMEA_APIDIR)
    message(WARNING "${target_name}-API directory could not be found. ${target_name} will not be build")    
endif(XIMEA_APIDIR)
