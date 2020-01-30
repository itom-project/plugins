set(target_name QCam)

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


find_package(ITOM_SDK REQUIRED)
FIND_PACKAGE_QT(ON Widgets Xml LinguistTools)
find_package(OpenCV COMPONENTS core REQUIRED)
find_package(VisualLeakDetector QUIET)

add_definitions(-DUNICODE -D_UNICODE)

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


endif()


include_directories(
    ${CMAKE_CURRENT_BINARY_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${ITOM_SDK_INCLUDE_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}/QImaging_SDK/2.0.13.1/Headers
    ${VISUALLEAKDETECTOR_INCLUDE_DIR}
)

if(BUILD_TARGET64)
    set( QImaging_Libraries QCaptureCore_x64.lib QCamDriverx64.lib )
    
    link_directories( 
        ${CMAKE_CURRENT_SOURCE_DIR}/QImaging_SDK/2.0.13.1/libs/AMD64
    )
else(BUILD_TARGET64)    
    set( QImaging_Libraries QCaptureCore.lib QCamDriver.lib )
    
    link_directories( 
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
    ${CMAKE_CURRENT_BINARY_DIR}/gitVersion.h
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

if(QT5_FOUND)
    #if automoc if OFF, you also need to call QT5_WRAP_CPP here
    QT5_WRAP_UI(plugin_UI_MOC ${plugin_UI})
    QT5_ADD_RESOURCES(plugin_RCC_MOC ${plugin_RCC})
else()
    message(SEND_ERROR "Currently only Qt5 is supported")
endif()

#Add version information to the plugIn-dll unter MSVC
if(MSVC)
    list(APPEND plugin_SOURCES ${ITOM_SDK_INCLUDE_DIR}/../pluginLibraryVersion.rc)
endif()

file(GLOB EXISTING_TRANSLATION_FILES "translation/*.ts")

add_library(${target_name} ${plugin_SOURCES} ${plugin_HEADERS} ${plugin_HEADERS_MOC} ${plugin_UI_MOC} ${plugin_RCC_MOC} ${EXISTING_TRANSLATION_FILES})

target_link_libraries(${target_name} ${ITOM_SDK_LIBRARIES} ${QImaging_Libraries} ${QT5_LIBRARIES} ${VISUALLEAKDETECTOR_LIBRARIES})



#translation
set(FILES_TO_TRANSLATE ${plugin_SOURCES} ${plugin_HEADERS} ${plugin_UI})
PLUGIN_TRANSLATION(QM_FILES ${target_name} ${UPDATE_TRANSLATIONS} "${EXISTING_TRANSLATION_FILES}" ITOM_LANGUAGES "${FILES_TO_TRANSLATE}")
PLUGIN_DOCUMENTATION(${target_name} qCam)

# COPY SECTION
set(COPY_SOURCES "")
set(COPY_DESTINATIONS "")
ADD_PLUGINLIBRARY_TO_COPY_LIST(${target_name} COPY_SOURCES COPY_DESTINATIONS)
ADD_QM_FILES_TO_COPY_LIST(${target_name} QM_FILES COPY_SOURCES COPY_DESTINATIONS)
POST_BUILD_COPY_FILES(${target_name} COPY_SOURCES COPY_DESTINATIONS)