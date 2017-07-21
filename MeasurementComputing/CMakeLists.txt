SET (target_name MeasurementComputing)

project(${target_name})

message(STATUS "\n--------------- PLUGIN ${target_name} ---------------")

cmake_minimum_required(VERSION 2.8)

OPTION(BUILD_UNICODE "Build with unicode charset if set to ON, else multibyte charset." ON)
OPTION(BUILD_SHARED "Build shared library." ON)
OPTION(BUILD_TARGET64 "Build for 64 bit target if set to ON or 32 bit if set to OFF." OFF)
OPTION(UPDATE_TRANSLATIONS "Update source translation translation/*.ts files (WARNING: make clean will delete the source .ts files! Danger!)")
SET (ITOM_SDK_DIR "" CACHE PATH "base path to itom_sdk")
SET (CMAKE_DEBUG_POSTFIX "d" CACHE STRING "Adds a postfix for debug-built libraries.")
SET (ITOM_LANGUAGES "de" CACHE STRING "semicolon separated list of languages that should be created (en must not be given since it is the default)")

SET (CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR} ${ITOM_SDK_DIR})

if (BUILD_TARGET64)
   set(CMAKE_SIZEOF_VOID_P 8)
   SET(dll_name "cbw64.dll")
   SET(bit_dest "64")
else (BUILD_TARGET64)
   set(CMAKE_SIZEOF_VOID_P 4)
   SET(dll_name "cbw32.dll")
   SET(bit_dest "32")
endif (BUILD_TARGET64)

IF(BUILD_SHARED)
    SET(LIBRARY_TYPE SHARED)
ELSE(BUILD_SHARED)
    SET(LIBRARY_TYPE STATIC)
ENDIF(BUILD_SHARED)

find_package(ITOM_SDK COMPONENTS dataobject itomCommonLib itomCommonQtLib itomWidgets REQUIRED)
include("${ITOM_SDK_DIR}/ItomBuildMacros.cmake") #include this mandatory macro file (important)
FIND_PACKAGE_QT(ON Core LinguistTools Widgets)
find_package(VisualLeakDetector QUIET) #silently detects the VisualLeakDetector for Windows (memory leak detector, optional)

    
IF (BUILD_UNICODE)
    ADD_DEFINITIONS(-DUNICODE -D_UNICODE)
ENDIF (BUILD_UNICODE)
ADD_DEFINITIONS(-DCMAKE)

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

FIND_PATH(MeasurementComputing_DAQ_C_SDK_DIR cbw.h PATHS "C:/Users/Public/Documents/Measurement Computing/DAQ/C" DOC "Directory of the DAQ C library, must contain the file cbw.h (e.g. C:/Users/Public/Documents/Measurement Computing/DAQ/C)")
FIND_FILE(MeasurementComputing_DAQ_BINARY ${dll_name} PATHS "C:/Program Files/InstaCal" "D:/Programme/InstaCal" "C:/Program Files (x86)/Measurement Computing/DAQ" DOC "Library cbw32.dll or cbw64.dll that is found in the directory of the InstaCal application.")

IF(MeasurementComputing_DAQ_C_SDK_DIR AND MeasurementComputing_DAQ_BINARY)

	# default build types are None, Debug, Release, RelWithDebInfo and MinRelSize
	IF (DEFINED CMAKE_BUILD_TYPE)
		SET(CMAKE_BUILD_TYPE ${CMAKE_BUILD_TYPE} CACHE STRING "Choose the type of build, options are: None(CMAKE_CXX_FLAGS or CMAKE_C_FLAGS used) Debug Release RelWithDebInfo MinSizeRel.")
	ELSE(CMAKE_BUILD_TYPE)
		SET (CMAKE_BUILD_TYPE Debug CACHE STRING "Choose the type of build, options are: None(CMAKE_CXX_FLAGS or CMAKE_C_FLAGS used) Debug Release RelWithDebInfo MinSizeRel.")
	ENDIF (DEFINED CMAKE_BUILD_TYPE)

	message(STATUS ${CMAKE_CURRENT_BINARY_DIR})

	set(MeasurementComputing_LIB debug cbw${bit_dest} optimized cbw${bit_dest}  CACHE STRING "" FORCE)

	INCLUDE_DIRECTORIES(
		${CMAKE_CURRENT_BINARY_DIR}
		${CMAKE_CURRENT_SOURCE_DIR}
		${ITOM_SDK_INCLUDE_DIRS}
		${VISUALLEAKDETECTOR_INCLUDE_DIR}
		${MeasurementComputing_DAQ_C_SDK_DIR}
	)

	LINK_DIRECTORIES(
		${USBADDA_LIB_DIR}
		${MeasurementComputing_DAQ_C_SDK_DIR}
	)

	set(plugin_HEADERS
		${CMAKE_CURRENT_SOURCE_DIR}/MeasurementComputing.h
		${CMAKE_CURRENT_SOURCE_DIR}/pluginVersion.h
        ${CMAKE_CURRENT_BINARY_DIR}/gitVersion.h
		${CMAKE_CURRENT_SOURCE_DIR}/dialogMeasurementComputing.h
	)

	set(plugin_UI
		${CMAKE_CURRENT_SOURCE_DIR}/dialogMeasurementComputing.ui
	)

	set(plugin_SOURCES 
		${CMAKE_CURRENT_SOURCE_DIR}/MeasurementComputing.cpp
		${CMAKE_CURRENT_SOURCE_DIR}/dialogMeasurementComputing.cpp
	)

	set(plugin_RCC
		#add absolute pathes to any *.qrc resource files here
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
	if (MSVC)
		#delayload cbw32.dll or cbw64.dll
		SET_TARGET_PROPERTIES (${target_name} PROPERTIES LINK_FLAGS "/DELAYLOAD:cbw${bit_dest}.dll")
		TARGET_LINK_LIBRARIES(${target_name} ${QT_LIBRARIES} ${ITOM_SDK_LIBRARIES} ${MeasurementComputing_LIB} ${QT5_LIBRARIES} ${VISUALLEAKDETECTOR_LIBRARIES} Delayimp.lib)
	else (MSVC)
		TARGET_LINK_LIBRARIES(${target_name} ${QT_LIBRARIES} ${ITOM_SDK_LIBRARIES} ${MeasurementComputing_LIB} ${QT5_LIBRARIES} ${VISUALLEAKDETECTOR_LIBRARIES})
	endif (MSVC)

    IF (QT5_FOUND AND CMAKE_VERSION VERSION_LESS 3.0.2)
        qt5_use_modules(${target_name} ${QT_COMPONENTS})
    ENDIF (QT5_FOUND AND CMAKE_VERSION VERSION_LESS 3.0.2)

	SET(FILES_TO_TRANSLATE ${plugin_SOURCES} ${plugin_HEADERS} ${plugin_UI})
	PLUGIN_TRANSLATION(QM_FILES ${target_name} ${UPDATE_TRANSLATIONS} "${EXISTING_TRANSLATION_FILES}" ITOM_LANGUAGES "${FILES_TO_TRANSLATE}")

	#documentation
	PLUGIN_DOCUMENTATION(${target_name} MeasurementComputing)

	# COPY SECTION
	set(COPY_SOURCES "")
	set(COPY_DESTINATIONS "")
	ADD_PLUGINLIBRARY_TO_COPY_LIST(${target_name} COPY_SOURCES COPY_DESTINATIONS)
	ADD_QM_FILES_TO_COPY_LIST(${target_name} QM_FILES COPY_SOURCES COPY_DESTINATIONS)
	POST_BUILD_COPY_FILES(${target_name} COPY_SOURCES COPY_DESTINATIONS)

	# copy libs to itoms lib folder
	POST_BUILD_COPY_FILE_TO_LIB_FOLDER(${target_name} MeasurementComputing_DAQ_BINARY)

ELSE()
	message(WARNING "C-SDK directory of Measurement Computing DAQ or install directory of InstaCal application could not be found. ${target_name} will not be build")    
ENDIF()
