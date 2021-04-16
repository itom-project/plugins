SET (target_name MatlabInterpolation)

project(${target_name})

cmake_minimum_required(VERSION 3.8)

if (POLICY CMP0053)
    cmake_policy(SET CMP0053 NEW)
ENDIF (POLICY CMP0053)
if (POLICY CMP0043)
    cmake_policy(SET CMP0043 OLD) # use COMPILE_DEFINITIONS_<CONFIG> configuration style
ENDIF (POLICY CMP0043)

#################################################################
# Input elements for CMake GUI (Checkboxes, Pathes, Strings...)
#################################################################
OPTION(BUILD_UNICODE "Build with unicode charset if set to ON, else multibyte charset." ON)
OPTION(BUILD_SHARED_LIBS "Build shared library." ON)
OPTION(BUILD_TARGET64 "Build for 64 bit target if set to ON or 32 bit if set to OFF." OFF)
OPTION(BUILD_OPENMP_ENABLE "Use OpenMP parallelization if available. If TRUE, the definition USEOPENMP is set. This is only the case if OpenMP is generally available and if the build is release." ON)
OPTION(USE_CUDA "Use CUDA computation capabilites if CUDA toolkit / drivers are found.")
SET (CMAKE_DEBUG_POSTFIX "d" CACHE STRING "Adds a postfix for debug-built libraries.")
set(CMAKE_INCLUDE_CURRENT_DIR ON)
SET (CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR} ${ITOM_SDK_DIR})

find_package(OpenCV COMPONENTS core imgproc REQUIRED) #if you require openCV indicate all components that are required (e.g. core, imgproc...), 

#try to enable OpenMP (e.g. not available with VS Express)
find_package(OpenMP QUIET)

# pass some flag to adapt code for building / use of header file in matlab
ADD_DEFINITIONS(-DBUILDLIBRARY)

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

#################################################################
# Automatic package detection
#   add here find_package commands for searching for 3rd party
#   libraries
#
#   for detecting Qt, use FIND_PACKAGE_QT instead of the
#   native command, since FIND_PACKAGE_QT detects the Qt4 or 5
#   version.
#################################################################

if (USE_CUDA)
    find_package(CUDA)
    if (CUDA_FOUND)
        #enable_language(CUDA)
        ADD_DEFINITIONS(-DUSECUDA)
    else (CUDA_FOUND)
        SET(CUDA_FOUND FALSE)
        MESSAGE(STATUS "Have not found Cuda Toolkit, CUDA disabled")
    endif (CUDA_FOUND)
else (USE_CUDA)
	SET(CUDA_FOUND FALSE)
endif (USE_CUDA)
ADD_DEFINITIONS(-DNO_ITOM_BUILD)

#################################################################
# General settings and preprocessor settings
#################################################################
IF(BUILD_SHARED_LIBS)
    SET(LIBRARY_TYPE SHARED)
ELSE(BUILD_SHARED_LIBS)
    SET(LIBRARY_TYPE STATIC)
ENDIF(BUILD_SHARED_LIBS)

#IF (BUILD_UNICODE)
#    ADD_DEFINITIONS(-DUNICODE -D_UNICODE)
#ENDIF (BUILD_UNICODE)
#ADD_DEFINITIONS(-DCMAKE)

# default build types are None, Debug, Release, RelWithDebInfo and MinRelSize
IF (DEFINED CMAKE_BUILD_TYPE)
    SET(CMAKE_BUILD_TYPE ${CMAKE_BUILD_TYPE} CACHE STRING "Choose the type of build, options are: None(CMAKE_CXX_FLAGS or CMAKE_C_FLAGS used) Debug Release RelWithDebInfo MinSizeRel.")
ELSE(CMAKE_BUILD_TYPE)
    SET (CMAKE_BUILD_TYPE Debug CACHE STRING "Choose the type of build, options are: None(CMAKE_CXX_FLAGS or CMAKE_C_FLAGS used) Debug Release RelWithDebInfo MinSizeRel.")
ENDIF (DEFINED CMAKE_BUILD_TYPE)

#################################################################
# List of include directories
#
# Hint: necessary Qt include directories are automatically added
#  via the find_package macro above
#################################################################
INCLUDE_DIRECTORIES(
	${CMAKE_CURRENT_BINARY_DIR} #build directory of this plugin (recommended)
	${CMAKE_CURRENT_SOURCE_DIR} #source directory of this plugin (recommended)
    ${CMAKE_CURRENT_SOURCE_DIR}/../ #source directory of this plugin (recommended)
	${OpenCV_DIR}/include       #include directory of OpenCV
	#add further include directories here
)

#if (CUDA_FOUND)
#    list(APPEND INCLUDE_DIRECTORIES
#        ${CUDA_INCLUDE_DIRECTORIES})
#    set(CUDA_ATTACH_VS_BUILD_RULE_TO_CUDA_FILE ON)
#    set(CUDA_SEPARABLE_COMPILATION ON)
#    set(CUDA_PROPAGATE_HOST_FLAGS OFF)
#    set(CUDA_VERBOSE_BUILD ON)
#    #set(CUDA_HOST_COMPILER clang++) 
#endif (CUDA_FOUND)

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
set(plugin_HEADERS
	${CMAKE_CURRENT_SOURCE_DIR}/../dicInterpolationMats.h
    ${CMAKE_CURRENT_SOURCE_DIR}/../dicInterpolation.h
    ${CMAKE_CURRENT_SOURCE_DIR}/retVal.h
    ${CMAKE_CURRENT_SOURCE_DIR}/byteArray.h
    ${CMAKE_CURRENT_SOURCE_DIR}/dicMatlab.h
	#add further header files (absolute pathes e.g. using CMAKE_CURRENT_SOURCE_DIR)
)

set(plugin_SOURCES
	${CMAKE_CURRENT_SOURCE_DIR}/../dicInterpolation.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/retVal.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/byteArray.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/dicMatlab.cpp
	#add further source files here
)

if (CUDA_FOUND)
    list(APPEND plugin_HEADERS ${CMAKE_CURRENT_SOURCE_DIR}/../dicInterpolation.hu)
    list(APPEND plugin_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/../dicInterpolation.cu)
endif (CUDA_FOUND)

#Append rc file to the source files for adding information about the plugin
# to the properties of the DLL under Visual Studio.
if(MSVC)
#	list(APPEND plugin_SOURCES ${ITOM_SDK_INCLUDE_DIR}/../pluginLibraryVersion.rc)
endif(MSVC)

set(plugin_RCC
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

#add all (generated) header and source files to the library (these files are compiled then)
if (CUDA_FOUND)
    CUDA_ADD_LIBRARY(${target_name} ${LIBRARY_TYPE} ${plugin_SOURCES} ${plugin_HEADERS} ${cuda_generated_files})
else (CUDA_FOUND)
    ADD_LIBRARY(${target_name} ${LIBRARY_TYPE} ${plugin_SOURCES} ${plugin_HEADERS} ${cuda_generated_files})
endif (CUDA_FOUND)

#link the compiled library
#append all libraries this plugin should be linked to at the end of the TARGET_LINK_LIBRARIES command
# Important variables are:
#
# if you want to link against one library whose directory is already added to LINK_DIRECTORIES above
# simply add its filename without suffix (*.lib, *.so...). This is automatically done by CMake

TARGET_LINK_LIBRARIES(${target_name} ${OpenCV_LIBS})