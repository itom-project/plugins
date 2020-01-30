set(target_name USBMotion3XIII)

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
endif()
if(NOT EXISTS ${CMAKE_CURRENT_BINARY_DIR}/gitVersion.h)

    configure_file(${CMAKE_CURRENT_SOURCE_DIR}/gitVersion.h.in ${CMAKE_CURRENT_BINARY_DIR}/gitVersion.h @ONLY)
endif()


endif()


include_directories(
    ${CMAKE_CURRENT_BINARY_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${ITOM_SDK_INCLUDE_DIRS}
    ${VISUALLEAKDETECTOR_INCLUDE_DIR}
)

set(plugin_HEADERS
    ${CMAKE_CURRENT_SOURCE_DIR}/dialogUSBMotion3XIII.h
    ${CMAKE_CURRENT_SOURCE_DIR}/dockWidgetUSBMotion3XIII.h
    ${CMAKE_CURRENT_SOURCE_DIR}/USB3xIII_dll.h
    ${CMAKE_CURRENT_SOURCE_DIR}/USBMotion3XIII.h
)

set(plugin_SOURCES 
    ${CMAKE_CURRENT_SOURCE_DIR}/dialogUSBMotion3XIII.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/dockWidgetUSBMotion3XIII.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/USBMotion3XIII.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/pluginVersion.h
    ${CMAKE_CURRENT_BINARY_DIR}/gitVersion.h
)

set(plugin_UI
    ${CMAKE_CURRENT_SOURCE_DIR}/dialogUSBMotion3XIII.ui
    ${CMAKE_CURRENT_SOURCE_DIR}/dockWidgetUSBMotion3XIII.ui
)

set(plugin_RCC
    ${CMAKE_CURRENT_SOURCE_DIR}/resource.qrc
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
#handle translations END STEP 1

add_library(${target_name} ${plugin_SOURCES} ${plugin_HEADERS} ${plugin_HEADERS_MOC} ${plugin_UI_MOC} ${plugin_RCC_MOC} ${EXISTING_TRANSLATION_FILES})

target_link_libraries(${target_name} ${OpenCV_LIBS} ${ITOM_SDK_LIBRARIES} ${QT5_LIBRARIES} ${VISUALLEAKDETECTOR_LIBRARIES})



if(NOT QT5_FOUND)
    if(CMAKE_SIZEOF_VOID_P EQUAL 8)
        set_target_properties(${target_name} PROPERTIES COMPILE_FLAGS "/Zc:wchar_t-")
    endif()
endif(NOT QT5_FOUND)

#translation
set(FILES_TO_TRANSLATE ${plugin_SOURCES} ${plugin_HEADERS} ${plugin_UI})
PLUGIN_TRANSLATION(QM_FILES ${target_name} ${UPDATE_TRANSLATIONS} "${EXISTING_TRANSLATION_FILES}" ITOM_LANGUAGES "${FILES_TO_TRANSLATE}")

PLUGIN_DOCUMENTATION(${target_name} usbMotion3XIII)

# COPY SECTION
set(COPY_SOURCES "")
set(COPY_DESTINATIONS "")
ADD_PLUGINLIBRARY_TO_COPY_LIST(${target_name} COPY_SOURCES COPY_DESTINATIONS)

if(CMAKE_HOST_WIN32)
    if(CMAKE_SIZEOF_VOID_P EQUAL 8)
        set(COPY_SOURCES ${COPY_SOURCES} ${CMAKE_CURRENT_SOURCE_DIR}/USB3xIII64.dll)
        set(COPY_DESTINATIONS ${COPY_DESTINATIONS} ${ITOM_APP_DIR}/plugins/${target_name})
    else()
        set(COPY_SOURCES ${COPY_SOURCES} ${CMAKE_CURRENT_SOURCE_DIR}/USB3xIII.dll)
        set(COPY_DESTINATIONS ${COPY_DESTINATIONS} ${ITOM_APP_DIR}/plugins/${target_name})
    endif()
endif()

ADD_QM_FILES_TO_COPY_LIST(${target_name} QM_FILES COPY_SOURCES COPY_DESTINATIONS)
POST_BUILD_COPY_FILES(${target_name} COPY_SOURCES COPY_DESTINATIONS)
