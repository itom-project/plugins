SET (target_name V4L2)

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

find_package(OpenCV COMPONENTS core imgproc REQUIRED)
find_package(ITOM_SDK COMPONENTS dataobject itomCommonLib itomCommonQtLib itomWidgets REQUIRED)
find_package(VisualLeakDetector QUIET)

include("${ITOM_SDK_DIR}/ItomBuildMacros.cmake")
FIND_PACKAGE_QT(ON LinguistTools Designer Widgets)


ADD_DEFINITIONS(${QT_DEFINITIONS})
IF (BUILD_UNICODE)
ADD_DEFINITIONS(-DUNICODE -D_UNICODE)
ENDIF (BUILD_UNICODE)
ADD_DEFINITIONS(-DCMAKE)
ADD_DEFINITIONS(-DITOMWIDGETS_SHARED)

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
	${QT_QTCORE_INCLUDE_DIR}
	${OpenCV_DIR}/include
	${ITOM_SDK_INCLUDE_DIR}
	${ITOM_SDK_INCLUDE_DIR}/itomWidgets
	${VISUALLEAKDETECTOR_INCLUDE_DIR}
)

LINK_DIRECTORIES(

)

set(plugin_HEADERS
	${ITOM_SDK_INCLUDE_DIR}/common/apiFunctionsInc.h
	${ITOM_SDK_INCLUDE_DIR}/common/addInGrabber.h
	${ITOM_SDK_INCLUDE_DIR}/common/addInInterface.h
	${ITOM_SDK_INCLUDE_DIR}/common/sharedStructures.h
	${ITOM_SDK_INCLUDE_DIR}/common/sharedStructuresQt.h
	${CMAKE_CURRENT_SOURCE_DIR}/dialogV4L2.h
	${CMAKE_CURRENT_SOURCE_DIR}/dockWidgetV4L2.h
	${CMAKE_CURRENT_SOURCE_DIR}/ui_dockWidgetV4L2.h
	${CMAKE_CURRENT_SOURCE_DIR}/V4L2.h
	${CMAKE_CURRENT_SOURCE_DIR}/v4l2_itom_api.h
	${CMAKE_CURRENT_SOURCE_DIR}/pluginVersion.h
)

set(plugin_ui
	${CMAKE_CURRENT_SOURCE_DIR}/dialogV4L2.ui
	#${CMAKE_CURRENT_SOURCE_DIR}/dockWidgetV4L2_base.ui
)


set(plugin_SOURCES
	${CMAKE_CURRENT_SOURCE_DIR}/V4L2.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/v4l2_itom_api.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/dialogV4L2.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/dockWidgetV4L2.cpp
)

#Add version information to the plugIn-dll unter MSVC
if(MSVC)
list(APPEND plugin_SOURCES ${ITOM_SDK_INCLUDE_DIR}/../pluginLibraryVersion.rc)
endif(MSVC)

if (QT5_FOUND)
	QT5_WRAP_UI(plugin_ui_MOC ${plugin_ui})
else (QT5_FOUND)
	QT4_WRAP_UI_ITOM(plugin_ui_MOC ${plugin_ui})
	QT4_WRAP_CPP_ITOM(plugin_HEADERS_MOC ${plugin_HEADERS})
endif (QT5_FOUND)



find_library(V4LCONVERT_LIBRARY libv4lconvert v4lconvert PATHS "/usr/lib" "/usr/lib/x86-64-linux-gnu")
IF(NOT V4LCONVERT_LIBRARY)
    message(STATUS "Library libv4lconvert could not be found.")
ENDIF()

file (GLOB EXISTING_TRANSLATION_FILES "translation/*.ts")
ADD_LIBRARY(${target_name} ${LIBRARY_TYPE} ${plugin_SOURCES} ${plugin_HEADERS} ${plugin_HEADERS_MOC} ${plugin_ui_MOC} ${plugin_rcc_MOC} ${EXISTING_TRANSLATION_FILES})

TARGET_LINK_LIBRARIES(${target_name} ${QT_LIBRARIES} ${OpenCV_LIBS} ${ITOM_SDK_LIBRARIES} ${VISUALLEAKDETECTOR_LIBRARIES} ${V4LCONVERT_LIBRARY})
if (QT5_FOUND)
	qt5_use_modules(${target_name} ${QT_COMPONENTS})
endif (QT5_FOUND)

#translation
set (FILES_TO_TRANSLATE ${plugin_SOURCES} ${plugin_HEADERS} ${plugin_ui})
PLUGIN_TRANSLATION(QM_FILES ${target_name} ${UPDATE_TRANSLATIONS} ${EXISTING_TRANSLATION_FILES} ITOM_LANGUAGES FILES_TO_TRANSLATE)

#documentation
PLUGIN_DOCUMENTATION(${target_name} v4l2)

# COPY SECTION
set(COPY_SOURCES "")
set(COPY_DESTINATIONS "")
ADD_PLUGINLIBRARY_TO_COPY_LIST(${target_name} COPY_SOURCES COPY_DESTINATIONS)
#ADD_QM_FILES_TO_COPY_LIST(${target_name} QM_FILES COPY_SOURCES COPY_DESTINATIONS)
POST_BUILD_COPY_FILES(${target_name} COPY_SOURCES COPY_DESTINATIONS)
