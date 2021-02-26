###################################################################
###################################################################
#    LIVOX plugin for itom
###################################################################
###################################################################

set(target_name Livox)
cmake_minimum_required(VERSION 3.1...3.15)
message(STATUS "Project ${target_name} (${CMAKE_CURRENT_BINARY_DIR})")

set(ITOM_SDK_DIR NOTFOUND CACHE PATH "path of SDK subfolder of itom root (build) directory")

#this is to automatically detect the SDK subfolder of the itom build directory.
find_path(ITOM_SDK_DIR "cmake/itom_sdk.cmake"
    HINTS "C:/itom/build/itom/SDK"
          "${CMAKE_CURRENT_BINARY_DIR}/../../itom/SDK"
    DOC "path of SDK subfolder of itom root (build) directory")

if(NOT ITOM_SDK_DIR)
    message(SEND_ERROR "ITOM_SDK_DIR is invalid. Provide itom SDK directory path first")
endif()

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR} ${ITOM_SDK_DIR}/cmake)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

include(ItomBuildMacros)
itom_init_cmake_policy(3.12)
itom_init_plugin_library(${target_name})

find_path( LIVOX_SDK_DIR livox_sdk.h DOC "LIVOX SDK directory (may contain livox_sdk.h)")

#if(LIVOX_SDK_DIR)

option(BUILD_TARGET64 "Build for 64 bit target if set to ON or 32 bit if set to OFF." ON)

find_package(ITOM_SDK COMPONENTS dataobject itomCommonLib itomCommonQtLib itomWidgets pointcloud REQUIRED)
itom_find_package_qt(ON Core Widgets LinguistTools)


#add here some preprocessors, if necessary, e.g.
#add_definitions(-DMYOPTION)


include_directories(
    ${CMAKE_CURRENT_BINARY_DIR} #build directory of this plugin (recommended)
    ${CMAKE_CURRENT_SOURCE_DIR} #source directory of this plugin (recommended)
    ${ITOM_SDK_INCLUDE_DIRS}    #include directory of the itom SDK (recommended) as well as necessary 3rd party directories (e.g. from OpenCV)
    ${CMAKE_CURRENT_SOURCE_DIR}/include
	${LIVOX_SDK_DIR}/sdk_core/include
)

#################################################################
# List of linker directories
#
# Hint: libraries detected using FIND_PACKAGE usually provide
#  all necessary libraries in a specific variable (e.g.
#  ${OpenCV_LIBS} or ${ITOM_SDK_LIBRARIES}). These variables
#  already contain absolute pathes, therefore no link directory
#  needs to be set for them. Simply add these variables to
#  the link target command below.
#################################################################


###????????????????????????????set (Livox_Libs ${CMAKE_CURRENT_SOURCE_DIR}/lib/livox_sdk_static.lib)
link_directories(
	${CMAKE_CURRENT_SOURCE_DIR}/lib
 )

#################################################################
# List of header files, source files, ui files and rcc files
#
# Add all header files to the PLUGIN_HEADERS list.
# Add all source (cpp,...) files to the PLUGIN_SOURCES list.
# Add all ui-files (Qt-Designer layouts) to the PLUGIN_UI list.
#
# Use absolute pathes, e.g. using one of the following variables:
#
# ${ITOM_SDK_INCLUDE_DIR} is the include directory of itom SDK
# ${CMAKE_CURRENT_SOURCE_DIR} is the source directory of this plugin
# ${CMAKE_CURRENT_BINARY_DIR} is the build directory of this plugin
#
#################################################################
set(PLUGIN_HEADERS
    ${CMAKE_CURRENT_SOURCE_DIR}/Livox.h
    ${CMAKE_CURRENT_SOURCE_DIR}/dialogLivox.h
    ${CMAKE_CURRENT_SOURCE_DIR}/dockWidgetLivox.h
    ${CMAKE_CURRENT_SOURCE_DIR}/pluginVersion.h
    ${CMAKE_CURRENT_BINARY_DIR}/gitVersion.h
	#${LIVOX_SDK_DIR}/sdk_core/include/livox_def.h
	#${LIVOX_SDK_DIR}/sdk_core/include/livox_sdk.h
)

set(PLUGIN_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/dialogLivox.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/dockWidgetLivox.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/Livox.cpp
	#${LIVOX_SDK_DIR}/sdk_core/source/livox_sdk.cpp
)

#Append rc file to the source files for adding information about the plugin
# to the properties of the DLL under Visual Studio.
if(MSVC)
    list(APPEND PLUGIN_SOURCES ${ITOM_SDK_INCLUDE_DIR}/../pluginLibraryVersion.rc)
endif()

set(PLUGIN_UI
    ${CMAKE_CURRENT_SOURCE_DIR}/dialogLivox.ui
    ${CMAKE_CURRENT_SOURCE_DIR}/dockWidgetLivox.ui
)

set(PLUGIN_RCC
    #add absolute pathes to any *.qrc resource files here
)


#################################################################
# Group files in their original folder structure (MSVC only)
# If you have some header and source files in a specific
# subfolder, you can even have this subfolder in your
# IDE (mainly Visual Studio supports this). Then call 
# itom_add_source_group(directoryName) for each subdirectory.
#
# HINT: This command does nothing for IDE different than MSVC.
#################################################################
#itom_add_source_group(subdirectory)


#################################################################
# Compile and link the plugin library
# 
#################################################################

#add all (generated) header and source files to the library (these files are compiled then)
add_library(${target_name} SHARED 
    ${PLUGIN_SOURCES} 
    ${PLUGIN_HEADERS} 
    ${PLUGIN_UI} 
    ${PLUGIN_RCC}
)

# Qt: enable all automoc, autouic and autorcc. Autouic file will be disabled for all files that are processed manually by itom_qt_wrap_ui.
set_target_properties(${target_name} PROPERTIES AUTOMOC ON AUTORCC ON AUTOUIC ON)

#link the compiled library
#append all libraries this plugin should be linked to at the end of the target_link_libraries command
# Important variables are:
# ${ITOM_SDK_LIBRARIES} -> all necessary libraries from find_package(ITOM_SDK)
# -> all necessary libraries from FIND_PACKAGE_QT (Qt4 or Qt5)
# ${OpenCV_LIBS} -> all necessary libraries opencv libraries from find_package(OpenCV)
#
# if you want to link against one library whose directory is already added to link_directories above
# simply add its filename without suffix (*.lib, *.so...). This is automatically done by CMake

find_library(LIVOX_LIB NAMES "livox_sdk_static" PATHS ${LIVOX_SDK_DIR})
###${LIVOX_SDK_DIR}/build/sdk_core/Release/livox_sdk_static.lib
if(LIVOX_LIB)
        message(STATUS "Livox library found at ${LIVOX_LIB}")
    else()
        message(SEND_FATAL "Livox library not found")
    endif()

target_link_libraries(${target_name} ${LIVOX_LIB} ${ITOM_SDK_LIBRARIES} ${QT5_LIBRARIES} ${VISUALLEAKDETECTOR_LIBRARIES})

#################################################################
# Plugin Translation
#
# In the plugin source directory can be a 'translation' subfolder
# with one or multiple *.ts files (name pattern {target_name}_{language}.ts).
# The itom_library_translation macro adds an ITOM_LANGUAGES string
# to the CMake GUI, where all desired language IDs (besides 'en'
# for english (always the default language) should be given as
# semicolon separated string. To parse all translatable strings
# in the source code (for the first time or to update the ts files),
# set the CMake Option ITOM_UPDATE_TRANSLATIONS and generate
# the project and build the solution (there are now ${target_name}_translation
# projects. Then ts files will be generated in the translation source
# directory for each language (check it into the Git repository)
# or existing ts files will be updated. Then use the QtLinguist
# tool to translate these ts files. After re-building your 
# solution, the ts-files are compiled into qm-files, which are then
# put into itom's plugin subdirectory together with the plugin
# library itself.
#################################################################
set(FILES_TO_TRANSLATE ${PLUGIN_SOURCES} ${PLUGIN_HEADERS} ${PLUGIN_UI} ${PLUGIN_RCC})
itom_library_translation(
    QM_FILES 
    TARGET ${target_name} 
    FILES_TO_TRANSLATE ${FILES_TO_TRANSLATE}
    )

#################################################################
# Plugin Documentation
# 
# In the source directory of the plugin can be a subfolder 'docs'.
# This folder can contain one or more *.rst files with the docu-
# mentation of the plugin. CMake organizes the rest if you 
# indicate the name of the main documentation file(without
# suffix rst) in the following command:
# 
# itom_configure_plugin_documentation(${target_name} nameOfTheFile)
#################################################################
itom_configure_plugin_documentation(${target_name} Livox)

#################################################################
# Post-Build Copy Operations
# 
# itom is able to force a post-build process that copies
# different files, like the currently created library, to
# other destination pathes. This is done in this section.
# At first pairs of sources and destinations are added
# to the lists COPY_SOURCES and COPY_DESTINATIONS.
# Afterwards, the post-build process is generated using
# itom_post_build_copy_files.
#
# The following macros can be used to fill up the source
# and destination list:
#
# itom_add_pluginlibrary_to_copy_list 
# - this is necessary for each plugin such that the library
#   is automatically copied to the plugins folder of
#   the itom build directory.
#
# itom_add_plugin_qm_files_to_copy_list
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
file(GLOB dlls "${CMAKE_CURRENT_SOURCE_DIR}/lib/*.dll")
itom_post_build_copy_files_to_lib_folder(${target_name} dlls)



#???????????????????????????????????
###COPY the livox_sdk.h and livox_def.h files to where? and how?
#???????????????????????????????????
