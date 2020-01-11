set(target_name NITWidySWIR) #->replace NITWidySWIR by the name of your plugin (one word)

project(${target_name})

message(STATUS "\n--------------- PLUGIN ${target_name} ---------------")

message(STATUS "Project ${target_name} (${CMAKE_CURRENT_BINARY_DIR})")

cmake_minimum_required(VERSION 3.0.2)

option(BUILD_UNICODE "Build with unicode charset if set to ON, else multibyte charset." ON)
option(BUILD_SHARED_LIBS "Build shared library." ON)
option(BUILD_TARGET64 "Build for 64 bit target if set to ON or 32 bit if set to OFF." ON)
option(UPDATE_TRANSLATIONS "Update source translation translation/*.ts files (WARNING: make clean will delete the source .ts files! Danger!)")
set(ITOM_SDK_DIR "" CACHE PATH "base path to itom_sdk")
set(CMAKE_DEBUG_POSTFIX "d" CACHE STRING "Adds a postfix for debug-built libraries.")
set(ITOM_LANGUAGES "de" CACHE STRING "semicolon separated list of languages that should be created (en must not be given since it is the default)")
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR} ${ITOM_SDK_DIR})

include("${ITOM_SDK_DIR}/ItomBuildMacros.cmake")
INIT_ITOM_LIBRARY() #apply general project settings, CMake configurations, policies... for itom and its libraries

set(NITLIBRARY_SDK_DIR "" CACHE PATH "path of the NITLIBRARY SDK. Must contain subfolders like bin, bin64, include, lib, lib64")

# the itom SDK needs to be detected, use the COMPONENTS keyword
# to define which library components are needed. Possible values
# are:
# - dataobject for the ito::DataObject (usually required)
# - itomCommonLib (RetVal, Param,...) (required)
# - itomCommonQtLib (AddInInterface,...) (required)
# - itomWidgets (further widgets) (may be used in dock widget...)
# - pointcloud (pointCloud, point, polygonMesh...) (optional)
# if no components are indicated, all components above are used
find_package(ITOM_SDK COMPONENTS dataobject itomCommonLib itomCommonQtLib itomWidgets REQUIRED)
#find_package(OpenCV COMPONENTS core REQUIRED) #if you require openCV indicate all components that are required (e.g. core, imgproc...), 
#       if the dataobject is included in the ITOM_SDK components, the OpenCV core component is detected there and the necessary include
#       directories and libraries to link agains are contained in ITOM_SDK_LIBRARIES and ITOM_SDK_INCLUDE_DIRS
find_package(VisualLeakDetector QUIET) #silently detects the VisualLeakDetector for Windows (memory leak detector, optional)
FIND_PACKAGE_QT(ON Core LinguistTools Widgets)

find_path(NITLIBRARY_INCLUDE_DIR NITManager.h PATHS ${NITLIBRARY_SDK_DIR} PATH_SUFFIXES /include DOC "NITLibrary include directory")
find_package(OpenCV COMPONENTS core imgproc REQUIRED)

if(NITLIBRARY_INCLUDE_DIR)
    message(STATUS "NITLibrary found")

    if(BUILD_SHARED_LIBS)
        set(LIBRARY_TYPE SHARED)
    else(BUILD_SHARED_LIBS)
        set(LIBRARY_TYPE STATIC)
    endif(BUILD_SHARED_LIBS)

    if(BUILD_UNICODE)
        add_definitions(-DUNICODE -D_UNICODE)
    endif(BUILD_UNICODE)
    add_definitions(-DCMAKE)
    add_definitions(-DITOMWIDGETS_SHARED)

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
        ${CMAKE_CURRENT_BINARY_DIR} #build directory of this plugin (recommended)
        ${CMAKE_CURRENT_SOURCE_DIR} #source directory of this plugin (recommended)
        ${ITOM_SDK_INCLUDE_DIRS}    #include directory of the itom SDK (recommended) 
        ${VISUALLEAKDETECTOR_INCLUDE_DIR} #include directory to the visual leak detector (recommended, does nothing if not available)
        ${ITOM_SDK_INCLUDE_DIR}/itomWidgets
        ${NITLIBRARY_INCLUDE_DIR}
        }
        #add further include directories here
    )
    
    
    if(BUILD_TARGET64)
        file(GLOB NITLIBRARY_LIB_DIR "${NITLIBRARY_SDK_DIR}/lib/x64/*.lib")
    else(BUILD_TARGET64)
        file(GLOB NITLIBRARY_LIB_DIR "${NITLIBRARY_SDK_DIR}/lib/x86/*.lib")
    endif(BUILD_TARGET64)
    
    link_directories(
        ${NITLIBRARY_LIB_DIR}
        ${OpenCV_DIR}/lib
    )

    set(plugin_HEADERS
        ${CMAKE_CURRENT_SOURCE_DIR}/NITWidySWIR.h
        ${CMAKE_CURRENT_SOURCE_DIR}/dialogNITWidySWIR.h
        ${CMAKE_CURRENT_SOURCE_DIR}/dockWidgetNITWidySWIR.h
        ${CMAKE_CURRENT_SOURCE_DIR}/pluginVersion.h
        ${CMAKE_CURRENT_BINARY_DIR}/gitVersion.h
        #add further header files (absolute pathes e.g. using CMAKE_CURRENT_SOURCE_DIR)
    )

    set(plugin_SOURCES
        ${CMAKE_CURRENT_SOURCE_DIR}/dialogNITWidySWIR.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/dockWidgetNITWidySWIR.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/NITWidySWIR.cpp
        #add further source files here
    )

    #Append rc file to the source files for adding information about the plugin
    # to the properties of the DLL under Visual Studio.
    if(MSVC)
        list(APPEND plugin_SOURCES ${ITOM_SDK_INCLUDE_DIR}/../pluginLibraryVersion.rc)
    endif()

    set(plugin_UI
        ${CMAKE_CURRENT_SOURCE_DIR}/dialogNITWidySWIR.ui
        ${CMAKE_CURRENT_SOURCE_DIR}/dockWidgetNITWidySWIR.ui
    )

    set(plugin_RCC
        #add absolute pathes to any *.qrc resource files here
    )

    if(QT5_FOUND)
        #if automoc if OFF, you also need to call QT5_WRAP_CPP here
        QT5_WRAP_UI(plugin_UI_MOC ${plugin_UI})
        QT5_ADD_RESOURCES(plugin_RCC_MOC ${plugin_RCC})
    else()
        message(SEND_ERROR "Currently only Qt5 is supported")
    endif()

    #search for all existing translation files in the translation subfolder
    file(GLOB EXISTING_TRANSLATION_FILES "translation/*.ts") 

    #add all (generated) header and source files to the library (these files are compiled then)
    add_library(${target_name} ${LIBRARY_TYPE} ${plugin_SOURCES} ${plugin_HEADERS} ${plugin_HEADERS_MOC} ${plugin_UI_MOC} ${plugin_RCC_MOC} ${EXISTING_TRANSLATION_FILES})

    target_link_libraries(${target_name} ${QT_LIBRARIES} ${ITOM_SDK_LIBRARIES} ${QT5_LIBRARIES} ${VISUALLEAKDETECTOR_LIBRARIES} ${NITLIBRARY_LIB_DIR} ${OPENCV240_LIB_DIR})


    
    SET_TARGET_PROPERTIES(${target_name} PROPERTIES LINK_FLAGS "/DELAYLOAD:NITLibrary.dll")

    set(FILES_TO_TRANSLATE ${plugin_SOURCES} ${plugin_HEADERS} ${plugin_UI})
    PLUGIN_TRANSLATION(QM_FILES ${target_name} ${UPDATE_TRANSLATIONS} "${EXISTING_TRANSLATION_FILES}" ITOM_LANGUAGES "${FILES_TO_TRANSLATE}")

    PLUGIN_DOCUMENTATION(${target_name} NITWidySWIR)

    set(COPY_SOURCES "")
    set(COPY_DESTINATIONS "")

    ADD_PLUGINLIBRARY_TO_COPY_LIST(${target_name} COPY_SOURCES COPY_DESTINATIONS)
    ADD_QM_FILES_TO_COPY_LIST(${target_name} QM_FILES COPY_SOURCES COPY_DESTINATIONS)

    POST_BUILD_COPY_FILES(${target_name} COPY_SOURCES COPY_DESTINATIONS)
    
    if(BUILD_TARGET64)
        file(GLOB NITLIBRARY_DLL "${NITLIBRARY_SDK_DIR}/bin/x64/*.dll")
    else(BUILD_TARGET64)    
        file(GLOB NITLIBRARY_DLL "${NITLIBRARY_SDK_DIR}/bin/x86/*.dll")
    endif(BUILD_TARGET64)
        
    POST_BUILD_COPY_FILE_TO_LIB_FOLDER(${target_name} NITLIBRARY_DLL)
 
else(NITLIBRARY_INCLUDE_DIR)
    message(WARNING "${target_name}-API directory could not be found. ${target_name} will not be build.")
endif(NITLIBRARY_INCLUDE_DIR)