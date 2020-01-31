set(target_name MSMediaFoundation)

project(${target_name})

message(STATUS "\n--------------- PLUGIN ${target_name} ---------------")

cmake_minimum_required(VERSION 3.0.2)

option(BUILD_TARGET64 "Build for 64 bit target if set to ON or 32 bit if set to OFF." ON)
option(UPDATE_TRANSLATIONS "Update source translation translation/*.ts files (WARNING: make clean will delete the source .ts files! Danger!)")
set(ITOM_SDK_DIR "" CACHE PATH "base path to itom_sdk")
set(CMAKE_DEBUG_POSTFIX "d" CACHE STRING "Adds a postfix for debug-built libraries.")
set(ITOM_LANGUAGES "de" CACHE STRING "semicolon separated list of languages that should be created (en must not be given since it is the default)")

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR} ${ITOM_SDK_DIR})

include("${ITOM_SDK_DIR}/ItomBuildMacros.cmake")
INIT_ITOM_LIBRARY() #apply general project settings, CMake configurations, policies... for itom and its libraries


find_package(OpenCV COMPONENTS core imgproc REQUIRED)

find_package(ITOM_SDK COMPONENTS dataobject itomCommonLib itomCommonQtLib itomWidgets REQUIRED)
find_package(VisualLeakDetector QUIET)
find_package(WindowsSDK QUIET)

if(WINDOWSSDK_FOUND)
    FIND_PACKAGE_QT(ON Core Widgets LinguistTools)




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


    include_directories(
        ${CMAKE_CURRENT_BINARY_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${ITOM_SDK_INCLUDE_DIRS}
        ${ITOM_SDK_INCLUDE_DIR}/itomWidgets
        ${VISUALLEAKDETECTOR_INCLUDE_DIR}
        videoInput
    )

    link_directories(
        #
    )

    set(plugin_HEADERS
        ${CMAKE_CURRENT_SOURCE_DIR}/dialogMSMediaFoundation.h
        ${CMAKE_CURRENT_SOURCE_DIR}/dockWidgetMSMediaFoundation.h
        ${CMAKE_CURRENT_SOURCE_DIR}/MSMediaFoundation.h
        ${CMAKE_CURRENT_SOURCE_DIR}/pluginVersion.h
        ${CMAKE_CURRENT_BINARY_DIR}/gitVersion.h
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
    endif()

    if(QT5_FOUND)
        #if automoc if OFF, you also need to call QT5_WRAP_CPP here
        QT5_WRAP_UI(plugin_UI_MOC ${plugin_UI})
        QT5_ADD_RESOURCES(plugin_RCC_MOC ${plugin_RCC})
    else()
        message(SEND_ERROR "Currently only Qt5 is supported")
    endif()

    ADD_SOURCE_GROUP(videoInput)

    file(GLOB EXISTING_TRANSLATION_FILES "translation/*.ts")

    add_library(${target_name} ${plugin_SOURCES} ${plugin_HEADERS} ${plugin_HEADERS_MOC} ${plugin_UI_MOC} ${plugin_RCC_MOC} ${EXISTING_TRANSLATION_FILES})
    target_link_libraries(${target_name} ${OpenCV_LIBS} ${ITOM_SDK_LIBRARIES} ${QT5_LIBRARIES} ${VISUALLEAKDETECTOR_LIBRARIES})
    
    #translation
    set(FILES_TO_TRANSLATE ${plugin_SOURCES} ${plugin_HEADERS} ${plugin_UI})
    PLUGIN_TRANSLATION(QM_FILES ${target_name} ${UPDATE_TRANSLATIONS} "${EXISTING_TRANSLATION_FILES}" ITOM_LANGUAGES "${FILES_TO_TRANSLATE}")
    
    #documentation
    PLUGIN_DOCUMENTATION(${target_name} msMediaFoundation)

    # COPY SECTION
    set(COPY_SOURCES "")
    set(COPY_DESTINATIONS "")
    ADD_PLUGINLIBRARY_TO_COPY_LIST(${target_name} COPY_SOURCES COPY_DESTINATIONS)
    ADD_QM_FILES_TO_COPY_LIST(${target_name} QM_FILES COPY_SOURCES COPY_DESTINATIONS)
    POST_BUILD_COPY_FILES(${target_name} COPY_SOURCES COPY_DESTINATIONS)
else(WINDOWSSDK_FOUND)
    message(WARNING "WindowsSDK could not be found. ${target_name} will not be build")  
endif(WINDOWSSDK_FOUND)
