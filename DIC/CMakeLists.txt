###################################################################
###################################################################
#    CMake Template for a plugin of itom
#    
#    You can use this template, use it in your plugins, modify it,
#    copy it and distribute it without any license restrictions.
###################################################################
###################################################################

SET (target_name DIC) #->replace PCI6115 by the name of your plugin (one word)

project(${target_name})

cmake_minimum_required(VERSION 3.8...3.15)

option(BUILD_TARGET64 "Build for 64 bit target if set to ON or 32 bit if set to OFF." ON)
set(ITOM_SDK_DIR NOTFOUND CACHE PATH "path of SDK subfolder of itom root (build) directory")

#this is to automatically detect the SDK subfolder of the itom build directory.
find_path(ITOM_SDK_DIR "cmake/itom_sdk.cmake"
    HINTS "C:/itom/build/itom/SDK"
          "${CMAKE_CURRENT_BINARY_DIR}/../../itom/SDK"
    DOC "path of SDK subfolder of itom root (build) directory")

if(NOT ITOM_SDK_DIR)
    message(SEND_ERROR "ITOM_SDK_DIR is invalid. Provide itom SDK directory path first")
endif()


OPTION(USE_CUDA "Use CUDA computation capabilites if CUDA toolkit / drivers are found.")
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR} ${ITOM_SDK_DIR}/cmake)

#################################################################
# Automatic package detection
#   add here find_package commands for searching for 3rd party
#   libraries
#
#   for detecting Qt, use FIND_PACKAGE_QT instead of the
#   native command, since FIND_PACKAGE_QT detects the Qt4 or 5
#   version.
#################################################################

# the itom SDK needs to be detected, use the COMPONENTS keyword
# to define which library components are needed. Possible values
# are:
# - dataobject for the ito::DataObject (usually required)
# - itomCommonLib (RetVal, Param,...) (required)
# - itomCommonQtLib (AddInInterface,...) (required)
# - itomWidgets (further widgets) (may be used in dock widget...)
# - pointcloud (pointCloud, point, polygonMesh...) (optional)
# if no components are indicated, all components above are used
include(ItomBuildMacros)
itom_init_cmake_policy(3.12)
itom_init_plugin_library(${target_name}) #Start the project, init compiler settings and set default configurations for plugins
find_package(ITOM_SDK COMPONENTS dataobject itomCommonLib itomCommonQtLib itomWidgets REQUIRED)

find_package(OpenCV COMPONENTS core imgproc REQUIRED) #if you require openCV indicate all components that are required (e.g. core, imgproc...), 
#       if the dataobject is included in the ITOM_SDK components, the OpenCV core component is detected there and the necessary include
#       directories and libraries to link agains are contained in ITOM_SDK_LIBRARIES and ITOM_SDK_INCLUDE_DIRS
find_package(VisualLeakDetector QUIET) #silently detects the VisualLeakDetector for Windows (memory leak detector, optional)

#usage of itom_find_package_qt(automoc component1, component2, ...)
# automoc is ON or OFF and only relevant for Qt5, usually set it to ON
# possible components are: OpenGL,Core,Designer,Xml,Svg,Sql,Network,UiTools,Widgets,PrintSupport,LinguistTools...
itom_find_package_qt(ON Core LinguistTools Widgets)
#find_package(Eigen3 REQUIRED)
set(EIGEN_INCLUDE_DIR "" CACHE PATH "base directory of header-only library Eigen")

IF (OPENMP_FOUND)
    IF (BUILD_OPENMP_ENABLE)
        message(STATUS "OpenMP found and enabled for release compilation")
        SET ( CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} ${OpenMP_CXX_FLAGS} -DUSEOPENMP" )
        SET ( CMAKE_C_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} ${OpenMP_C_FLAGS} -DUSEOPENMP" )
    ELSE (BUILD_OPENMP_ENABLE)
        message(STATUS "OpenMP found but not enabled for release compilation")
    ENDIF (BUILD_OPENMP_ENABLE)
ELSE(OPENMP_FOUND)
    message(STATUS "OpenMP not found.")
ENDIF(OPENMP_FOUND)

if (USE_CUDA)
    find_package(CUDA)
    if (CUDA_FOUND)
        enable_language(CUDA)
        ADD_DEFINITIONS(-DUSECUDA)
    else (CUDA_FOUND)
	    SET(CUDA_FOUND FALSE)
        MESSAGE(STATUS "Have not found Cuda Toolkit, CUDA disabled")
    endif (CUDA_FOUND)
else (USE_CUDA)
	SET(CUDA_FOUND FALSE)
endif (USE_CUDA)

#################################################################
# General settings and preprocessor settings
#################################################################
IF(BUILD_SHARED_LIBS)
    SET(LIBRARY_TYPE SHARED)
ELSE(BUILD_SHARED_LIBS)
    SET(LIBRARY_TYPE STATIC)
ENDIF(BUILD_SHARED_LIBS)

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


IF(VISUALLEAKDETECTOR_FOUND AND VISUALLEAKDETECTOR_ENABLED)
	ADD_DEFINITIONS(-DVISUAL_LEAK_DETECTOR_CMAKE)
ENDIF(VISUALLEAKDETECTOR_FOUND AND VISUALLEAKDETECTOR_ENABLED)

#################################################################
# List of include directories
#
# Hint: necessary Qt include directories are automatically added
#  via the find_package macro above
#################################################################
INCLUDE_DIRECTORIES(
    ${QT_QTCORE_INCLUDE_DIR}
    ${EIGEN_INCLUDE_DIR}  
	${CMAKE_CURRENT_BINARY_DIR} #build directory of this plugin (recommended)
	${CMAKE_CURRENT_SOURCE_DIR} #source directory of this plugin (recommended)
	${OpenCV_DIR}/include       #include directory of OpenCV
	${ITOM_SDK_INCLUDE_DIRS}    #include directory of the itom SDK (recommended) 
	${VISUALLEAKDETECTOR_INCLUDE_DIR} #include directory to the visual leak detector (recommended, does nothing if not available)
	#add further include directories here
)

#################################################################
# List of linker directories
#
# Hint: libraries detected using find_package usually provide
#  all necessary libraries in a specific variable (e.g.
#  ${OpenCV_LIBS} or ${ITOM_SDK_LIBRARIES}). These variables
#  already contain absolute pathes, therefore no link directory
#  needs to be set for them. Simply add these variables to
#  the link target command below.
#################################################################
LINK_DIRECTORIES(
	#add all linker directories
)

#################################################################
# List of header files, source files, ui files and rcc files
#
# Add all header files to the plugin_HEADERS list.
# Add all source (cpp,...) files to the plugin_SOURCES list.
# Add all ui-files (Qt-Designer layouts) to the plugin_UI list.
#
# Use absolute pathes, e.g. using one of the following variables:
#
# ${ITOM_SDK_INCLUDE_DIR} is the include directory of itom SDK
# ${CMAKE_CURRENT_SOURCE_DIR} is the source directory of this plugin
# ${CMAKE_CURRENT_BINARY_DIR} is the build directory of this plugin
#
#################################################################
set(PLUGIN_HEADERS
	${CMAKE_CURRENT_SOURCE_DIR}/dic.h
	${CMAKE_CURRENT_SOURCE_DIR}/dicPrivate.h
	${CMAKE_CURRENT_SOURCE_DIR}/dicInterpolationMats.h
    ${CMAKE_CURRENT_SOURCE_DIR}/dicInterpolation.h
	${CMAKE_CURRENT_SOURCE_DIR}/pluginVersion.h
    ${CMAKE_CURRENT_BINARY_DIR}/gitVersion.h
	#add further header files (absolute pathes e.g. using CMAKE_CURRENT_SOURCE_DIR)
)

set(PLUGIN_SOURCES
#	${CMAKE_CURRENT_SOURCE_DIR}/dic.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/dicFilterDef.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/dicInterpolation.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/dicInterpolationItom.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/dicInitialGuess.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/dicDisplacementSS.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/dicDisplacementFF.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/dicImageGeneration.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/dicDeformation.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/dicSpline.cpp
	#add further source files here
)

if (CUDA_FOUND)
    list(APPEND PLUGIN_HEADERS ${CMAKE_CURRENT_SOURCE_DIR}/dicInterpolation.hu)
    list(APPEND PLUGIN_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/dicInterpolation.cu)
endif (CUDA_FOUND)

#Append rc file to the source files for adding information about the plugin
# to the properties of the DLL under Visual Studio.
if(MSVC)
	list(APPEND PLUGIN_SOURCES ${ITOM_SDK_INCLUDE_DIR}/../pluginLibraryVersion.rc)
endif(MSVC)

set(PLUGIN_UI
#	${CMAKE_CURRENT_SOURCE_DIR}/dialogSpider8.ui
#	${CMAKE_CURRENT_SOURCE_DIR}/dockSpider8.ui
)

set(PLUGIN_RCC
	#add absolute pathes to any *.qrc resource files here
#	${CMAKE_CURRENT_SOURCE_DIR}/resource.qrc
)


#################################################################
# Group files in their original folder structure (MSVC only)
# If you have some header and source files in a specific
# subfolder, you can even have this subfolder in your
# IDE (mainly Visual Studio supports this). Then call 
# ADD_SOURCE_GROUP(directoryName) for each subdirectory.
#
# HINT: This command does nothing for IDE different than MSVC.
#################################################################
#ADD_SOURCE_GROUP(subdirectory)


#################################################################
# Compile and link the plugin library
# 
#################################################################

#search for all existing translation files in the translation subfolder
file (GLOB EXISTING_TRANSLATION_FILES "translation/*.ts") 

#add all (generated) header and source files to the library (these files are compiled then)
ADD_LIBRARY(${target_name} ${LIBRARY_TYPE} ${PLUGIN_SOURCES} ${PLUGIN_HEADERS} ${PLUGIN_UI} ${PLUGIN_RCC})

#link the compiled library
#append all libraries this plugin should be linked to at the end of the TARGET_LINK_LIBRARIES command
# Important variables are:
# ${ITOM_SDK_LIBRARIES} -> all necessary libraries from find_package(ITOM_SDK)
# ${QT_LIBRARIES} -> all necessary libraries from FIND_PACKAGE_QT (Qt4 or Qt5)
# ${OpenCV_LIBS} -> all necessary libraries opencv libraries from find_package(OpenCV)
#
# if you want to link against one library whose directory is already added to LINK_DIRECTORIES above
# simply add its filename without suffix (*.lib, *.so...). This is automatically done by CMake

set_target_properties(${target_name} PROPERTIES AUTOMOC ON AUTORCC ON AUTOUIC ON)

target_link_libraries(${target_name} ${ITOM_SDK_LIBRARIES} ${QT5_LIBRARIES} ${VISUALLEAKDETECTOR_LIBRARIES} ${QT_LIBRARIES})

#################################################################
# Plugin Translation
# 
# In the source directory of the plugin can be a subfolder 'docs'.
# This folder can contain one or more *.rst files with the docu-
# mentation of the plugin. CMake organizes the rest if you 
# indicate the name of the main documentation file (without
# suffix rst) in the following command:
# 
# PLUGIN_DOCUMENTATION(${target_name} nameOfTheFile)
#################################################################
set (FILES_TO_TRANSLATE ${PLUGIN_SOURCES} ${PLUGIN_HEADERS} ${PLUGIN_UI})
itom_library_translation(QM_FILES TARGET ${target_name} FILES_TO_TRANSLATE ${FILES_TO_TRANSLATE})

#################################################################
# Plugin Documentation
# 
# In the source directory of the plugin can be a subfolder 'docs'.
# This folder can contain one or more *.rst files with the docu-
# mentation of the plugin. CMake organizes the rest if you 
# indicate the name of the main documentation file (without
# suffix rst) in the following command:
# 
# PLUGIN_DOCUMENTATION(${target_name} nameOfTheFile)
#################################################################
itom_configure_plugin_documentation(${target_name} DIC)

#################################################################
# Post-Build Copy Operations
# 
# itom is able to force a post-build process that copies
# different files, like the currently created library, to
# other destination pathes. This is done in this section.
# At first pairs of sources and destinations are added
# to the lists COPY_SOURCES and COPY_DESTINATIONS.
# Afterwards, the post-build process is generated using
# POST_BUILD_COPY_FILES.
#
# The following macros can be used to fill up the source
# and destination list:
#
# ADD_PLUGINLIBRARY_TO_COPY_LIST 
# - this is necessary for each plugin such that the library
#   is automatically copied to the plugins folder of
#   the itom build directory.
#
# ADD_QM_FILES_TO_COPY_LIST
# - installs the generated translation files (qm) at the
#   right place in the itom build directory as well.
#
#################################################################
set(COPY_SOURCES "")
set(COPY_DESTINATIONS "")

itom_add_pluginlibrary_to_copy_list(${target_name} COPY_SOURCES COPY_DESTINATIONS)
itom_add_plugin_qm_files_to_copy_list(${target_name} QM_FILES COPY_SOURCES COPY_DESTINATIONS)

itom_post_build_copy_files(${target_name} COPY_SOURCES COPY_DESTINATIONS)

#if you want to copy one or more files to the lib-folder of
# the itom build directory, use the following macro:
#
# POST_BUILD_COPY_FILE_TO_LIB_FOLDER(${target}, ${listOfFiles})