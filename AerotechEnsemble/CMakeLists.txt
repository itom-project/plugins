SET (target_name AerotechEnsemble)

project(${target_name})

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

find_package(ITOM_SDK REQUIRED)
find_package(Qt4 REQUIRED)
#find_package(OpenCV REQUIRED)
find_package(VisualLeakDetector QUIET)

include("${ITOM_SDK_DIR}/ItomBuildMacros.cmake")

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
	${CMAKE_CURRENT_SOURCE_DIR}/CLibrary/Include
    ${QT_QTCORE_INCLUDE_DIR}
    #${OpenCV_DIR}/include
    ${ITOM_SDK_INCLUDE_DIR}
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
    #${OpenCV_DIR}/lib
	${LIB_PATH}
)


INCLUDE(${QT_USE_FILE})

set(plugin_HEADERS
    ${ITOM_SDK_INCLUDE_DIR}/common/addInInterface.h
    ${ITOM_SDK_INCLUDE_DIR}/common/sharedStructures.h
    ${ITOM_SDK_INCLUDE_DIR}/common/sharedStructuresQt.h
    ${CMAKE_CURRENT_SOURCE_DIR}/aerotechEnsemble.h
    ${CMAKE_CURRENT_SOURCE_DIR}/dockWidgetAerotechEnsemble.h
	${CMAKE_CURRENT_SOURCE_DIR}/dialogAerotechEnsemble.h
	${CMAKE_CURRENT_SOURCE_DIR}/pluginVersion.h
)

QT4_WRAP_CPP_ITOM(plugin_HEADERS_MOC ${plugin_HEADERS})

set(plugin_ui
    ${CMAKE_CURRENT_SOURCE_DIR}/dialogAerotechEnsemble.ui
    ${CMAKE_CURRENT_SOURCE_DIR}/dockWidgetAerotechEnsemble.ui
)

QT4_WRAP_UI_ITOM(plugin_ui_MOC ${plugin_ui})

set(plugin_rcc
    #${CMAKE_CURRENT_SOURCE_DIR}/resource.qrc
)

set(plugin_SOURCES 
    ${ITOM_SDK_INCLUDE_DIR}/common/addInInterface.cpp
    ${ITOM_SDK_INCLUDE_DIR}/common/sharedStructures.cpp
    ${ITOM_SDK_INCLUDE_DIR}/common/sharedStructuresQt.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/aerotechEnsemble.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/dockWidgetAerotechEnsemble.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/dialogAerotechEnsemble.cpp    
)

#Add version information to the plugIn-dll unter MSVC
if(MSVC)
    list(APPEND plugin_SOURCES ${ITOM_SDK_INCLUDE_DIR}/../pluginLibraryVersion.rc)
endif(MSVC)

#handle translations BEGIN STEP 1
set (FILES_TO_TRANSLATE ${plugin_SOURCES} ${plugin_HEADERS} ${plugin_ui})
set (TRANSLATION_OUTPUT_FILES)
set (TRANSLATIONS_FILES)

file (GLOB EXISTING_TRANSLATION_FILES "translation/*.ts")
#handle translations END STEP 1

ADD_LIBRARY(${target_name} ${LIBRARY_TYPE} ${plugin_SOURCES} ${plugin_HEADERS} ${plugin_HEADERS_MOC} ${plugin_ui_MOC} ${plugin_rcc_MOC} ${EXISTING_TRANSLATION_FILES})
TARGET_LINK_LIBRARIES(${target_name} ${QT_LIBRARIES} ${QT_QTUITOOLS_LIBRARY} ${QT_QTXML_LIBRARY} ${OpenCV_LIBS} ${ITOM_SDK_dataobject_LIBRARY} ${VISUALLEAKDETECTOR_LIBRARIES} ${LIB_NAME})

IF(CMAKE_SIZEOF_VOID_P EQUAL 8)
	set_target_properties(${target_name} PROPERTIES COMPILE_FLAGS "/Zc:wchar_t-")
ENDIF()

#handle translations BEGIN STEP 2
if (UPDATE_TRANSLATIONS)
	QT4_CREATE_TRANSLATION_ITOM(TRANSLATION_OUTPUT_FILES TRANSLATIONS_FILES ${target_name} ITOM_LANGUAGES ${FILES_TO_TRANSLATE} ${TRANSLATIONS_FILES})
	
	add_custom_target (_${target_name}_translation DEPENDS ${TRANSLATION_OUTPUT_FILES})
	add_dependencies(${target_name} _${target_name}_translation)
	
	QT4_ADD_TRANSLATION_ITOM(QM_FILES "${CMAKE_CURRENT_BINARY_DIR}/translation" ${target_name} ${TRANSLATIONS_FILES})
else (UPDATE_TRANSLATIONS)
	QT4_ADD_TRANSLATION_ITOM(QM_FILES "${CMAKE_CURRENT_BINARY_DIR}/translation" ${target_name} ${EXISTING_TRANSLATION_FILES})
endif (UPDATE_TRANSLATIONS)
#handle translations END STEP 2

# COPY SECTION
set(COPY_SOURCES "")
set(COPY_DESTINATIONS "")
ADD_PLUGINLIBRARY_TO_COPY_LIST(${target_name} COPY_SOURCES COPY_DESTINATIONS)

if(CMAKE_HOST_WIN32)
    if(CMAKE_SIZEOF_VOID_P EQUAL 8)
        SET(COPY_SOURCES ${COPY_SOURCES} ${BINARY_PATH}/EnsembleC64.dll)
        #SET(COPY_DESTINATIONS ${COPY_DESTINATIONS} ${ITOM_APP_DIR}/plugins/${target_name})
    else()
        #SET(COPY_SOURCES ${COPY_SOURCES} ${CMAKE_CURRENT_SOURCE_DIR}/USB3xIII.dll)
        #SET(COPY_DESTINATIONS ${COPY_DESTINATIONS} ${ITOM_APP_DIR}/plugins/${target_name})
    endif()
endif()

ADD_QM_FILES_TO_COPY_LIST(${target_name} QM_FILES COPY_SOURCES COPY_DESTINATIONS)
POST_BUILD_COPY_FILES(${target_name} COPY_SOURCES COPY_DESTINATIONS)

# copy dlls in itoms lib folder
file(GLOB dlls "${BINARY_PATH}/*.dll")
POST_BUILD_COPY_FILE_TO_LIB_FOLDER(${target_name} dlls)
