SET (target_name AerotechEnsemble)

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
set(CMAKE_INCLUDE_CURRENT_DIR ON)
SET (CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR} ${ITOM_SDK_DIR})

IF(BUILD_SHARED_LIBS)
    SET(LIBRARY_TYPE SHARED)
ELSE(BUILD_SHARED_LIBS)
    SET(LIBRARY_TYPE STATIC)
ENDIF(BUILD_SHARED_LIBS)

find_package(ITOM_SDK COMPONENTS dataobject itomCommonLib itomCommonQtLib REQUIRED)
include("${ITOM_SDK_DIR}/ItomBuildMacros.cmake")
FIND_PACKAGE_QT(ON Widgets UiTools Xml LinguistTools)
find_package(VisualLeakDetector QUIET)

IF (BUILD_UNICODE)
    ADD_DEFINITIONS(-DUNICODE -D_UNICODE)
ENDIF (BUILD_UNICODE)

#ADD_DEFINITIONS(${QT_DEFINITIONS})
#ADD_DEFINITIONS(-DCMAKE)

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
ENDIF (BUILD_GIT_TAG AND GIT_FOUND)
IF(NOT EXISTS ${CMAKE_CURRENT_BINARY_DIR}/gitVersion.h)
    configure_file(${CMAKE_CURRENT_SOURCE_DIR}/gitVersion.h.in ${CMAKE_CURRENT_BINARY_DIR}/gitVersion.h @ONLY)
ENDIF (NOT EXISTS ${CMAKE_CURRENT_BINARY_DIR}/gitVersion.h)

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
    ${CMAKE_CURRENT_SOURCE_DIR}/CLibrary/Include
    ${ITOM_SDK_INCLUDE_DIRS}
    ${VISUALLEAKDETECTOR_INCLUDE_DIR}
)

IF (BUILD_TARGET64)
    SET(LIB_PATH ${CMAKE_CURRENT_SOURCE_DIR}/CLibrary/Lib64)
    SET(BINARY_PATH ${CMAKE_CURRENT_SOURCE_DIR}/CLibrary/Bin64)
    SET(LIB_NAME EnsembleC64)
ELSE (BUILD_TARGET64)
    SET(LIB_PATH ${CMAKE_CURRENT_SOURCE_DIR}/CLibrary/Lib)
    SET(BINARY_PATH ${CMAKE_CURRENT_SOURCE_DIR}/CLibrary/Bin)
    SET(LIB_NAME EnsembleC)
ENDIF (BUILD_TARGET64)

LINK_DIRECTORIES(
    ${LIB_PATH}
)

set(plugin_HEADERS
    ${CMAKE_CURRENT_SOURCE_DIR}/aerotechEnsemble.h
    ${CMAKE_CURRENT_SOURCE_DIR}/dockWidgetAerotechEnsemble.h
    ${CMAKE_CURRENT_SOURCE_DIR}/dialogAerotechEnsemble.h
    ${CMAKE_CURRENT_SOURCE_DIR}/pluginVersion.h
    ${CMAKE_CURRENT_BINARY_DIR}/gitVersion.h
)

set(plugin_UI
    ${CMAKE_CURRENT_SOURCE_DIR}/dialogAerotechEnsemble.ui
    ${CMAKE_CURRENT_SOURCE_DIR}/dockWidgetAerotechEnsemble.ui
)

set(plugin_RCC
)

set(plugin_SOURCES 
    ${CMAKE_CURRENT_SOURCE_DIR}/aerotechEnsemble.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/dockWidgetAerotechEnsemble.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/dialogAerotechEnsemble.cpp    
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
TARGET_LINK_LIBRARIES(${target_name} ${QT_LIBRARIES} ${ITOM_SDK_LIBRARIES} ${LIB_NAME} ${QT5_LIBRARIES} ${VISUALLEAKDETECTOR_LIBRARIES})

IF (QT5_FOUND AND CMAKE_VERSION VERSION_LESS 3.0.2)
    qt5_use_modules(${target_name} ${QT_COMPONENTS})
ENDIF (QT5_FOUND AND CMAKE_VERSION VERSION_LESS 3.0.2)

IF(CMAKE_SIZEOF_VOID_P EQUAL 8)
    set_target_properties(${target_name} PROPERTIES COMPILE_FLAGS "/Zc:wchar_t-")
ENDIF()

#translation
set (FILES_TO_TRANSLATE ${plugin_SOURCES} ${plugin_HEADERS} ${plugin_UI})
PLUGIN_TRANSLATION(QM_FILES ${target_name} ${UPDATE_TRANSLATIONS} "${EXISTING_TRANSLATION_FILES}" ITOM_LANGUAGES "${FILES_TO_TRANSLATE}")

PLUGIN_DOCUMENTATION(${target_name} aerotechEnsemble)

# COPY SECTION
set(COPY_SOURCES "")
set(COPY_DESTINATIONS "")
ADD_PLUGINLIBRARY_TO_COPY_LIST(${target_name} COPY_SOURCES COPY_DESTINATIONS)
ADD_QM_FILES_TO_COPY_LIST(${target_name} QM_FILES COPY_SOURCES COPY_DESTINATIONS)
POST_BUILD_COPY_FILES(${target_name} COPY_SOURCES COPY_DESTINATIONS)

# copy dlls in itoms lib folder
file(GLOB dlls "${BINARY_PATH}/*.dll")
POST_BUILD_COPY_FILE_TO_LIB_FOLDER(${target_name} dlls)
