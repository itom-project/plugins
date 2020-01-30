set(target_name PGRFlyCapture)

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

if(LINUX)
add_definitions( "--std=c++0x --std=c++11" )
endif(LINUX)


include("${ITOM_SDK_DIR}/ItomBuildMacros.cmake")


find_package(ITOM_SDK COMPONENTS dataobject itomCommonLib itomCommonQtLib itomWidgets REQUIRED)
FIND_PACKAGE_QT(ON Core Widgets Xml LinguistTools)
find_package(VisualLeakDetector QUIET)

if(WIN32)
    find_path( FLYCAPTURE_PGRFLYCAP_INCLUDE_DIR FlyCapture2.h DOC "Include directory of Point Gray Research FlyCapture API")
else(WIN32)
    find_path( FLYCAPTURE_PGRFLYCAP_INCLUDE_DIR FlyCapture2.h PATHS /usr/include/flycapture DOC "Include directory of Point Gray Research FlyCapture API")
endif(WIN32)

if(FLYCAPTURE_PGRFLYCAP_INCLUDE_DIR)
    

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
        ${CMAKE_CURRENT_BINARY_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${ITOM_SDK_INCLUDE_DIRS}
        ${ITOM_SDK_INCLUDE_DIR}/itomWidgets
        ${VISUALLEAKDETECTOR_INCLUDE_DIR}
        ${FLYCAPTURE_PGRFLYCAP_INCLUDE_DIR}
    )

    if(BUILD_TARGET64)
        if(MSVC_VERSION EQUAL 1900)
            set(PGR_LIBDIR_HINT "${FLYCAPTURE_PGRFLYCAP_INCLUDE_DIR}/../lib64/vs2015")
            set(PGR_BINDIR_HINT "${FLYCAPTURE_PGRFLYCAP_INCLUDE_DIR}/../bin64/vs2015")
        elseif(MSVC_VERSION EQUAL 1800)
            set(PGR_LIBDIR_HINT "${FLYCAPTURE_PGRFLYCAP_INCLUDE_DIR}/../lib64/vs2013")
            set(PGR_BINDIR_HINT "${FLYCAPTURE_PGRFLYCAP_INCLUDE_DIR}/../bin64/vs2013")
        else(MSVC_VERSION)
            set(PGR_LIBDIR_HINT "${FLYCAPTURE_PGRFLYCAP_INCLUDE_DIR}/../lib64")
            set(PGR_BINDIR_HINT "${FLYCAPTURE_PGRFLYCAP_INCLUDE_DIR}/../bin64")
        endif(MSVC_VERSION)
    else(BUILD_TARGET64)
        if(MSVC_VERSION EQUAL 1900)
            set(PGR_LIBDIR_HINT "${FLYCAPTURE_PGRFLYCAP_INCLUDE_DIR}/../lib/vs2015")
            set(PGR_BINDIR_HINT "${FLYCAPTURE_PGRFLYCAP_INCLUDE_DIR}/../bin/vs2015")
        elseif(MSVC_VERSION EQUAL 1800)
            set(PGR_LIBDIR_HINT "${FLYCAPTURE_PGRFLYCAP_INCLUDE_DIR}/../lib/vs2013")
            set(PGR_BINDIR_HINT "${FLYCAPTURE_PGRFLYCAP_INCLUDE_DIR}/../bin/vs2013")
        else(MSVC_VERSION)    
            set(PGR_LIBDIR_HINT "${FLYCAPTURE_PGRFLYCAP_INCLUDE_DIR}/../lib")
            set(PGR_BINDIR_HINT "${FLYCAPTURE_PGRFLYCAP_INCLUDE_DIR}/../bin")
        endif(MSVC_VERSION)
    endif(BUILD_TARGET64)

    set(plugin_HEADERS
        ${CMAKE_CURRENT_SOURCE_DIR}/dialogPGRFlyCapture.h
        ${CMAKE_CURRENT_SOURCE_DIR}/dockWidgetPGRFlyCapture.h
        ${CMAKE_CURRENT_SOURCE_DIR}/PGRFlyCapture.h
        ${CMAKE_CURRENT_SOURCE_DIR}/pluginVersion.h
        ${CMAKE_CURRENT_BINARY_DIR}/gitVersion.h
    )
    
    set(plugin_SOURCES 
        ${CMAKE_CURRENT_SOURCE_DIR}/dialogPGRFlyCapture.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/dockWidgetPGRFlyCapture.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/PGRFlyCapture.cpp
    )

    set(plugin_UI
        ${CMAKE_CURRENT_SOURCE_DIR}/dialogPGRFlyCapture.ui
        ${CMAKE_CURRENT_SOURCE_DIR}/dockWidgetPGRFlyCapture.ui
    )
    
    set(plugin_RCC
    )

    #Add version information to the plugIn-dll unter MSVC
    if(MSVC)
        list(APPEND plugin_SOURCES ${ITOM_SDK_INCLUDE_DIR}/../pluginLibraryVersion.rc)
    endif()

    if(QT5_FOUND)
        #if automoc if OFF, you also need to call QT5_WRAP_CPP here
        QT5_WRAP_UI(plugin_UI_MOC ${plugin_UI})
        QT5_ADD_RESOURCES(designer_rcc_MOC ${plugin_RCC})
    else()
        message(SEND_ERROR "Currently only Qt5 is supported")
    endif()

    file(GLOB EXISTING_TRANSLATION_FILES "translation/*.ts")
    
    if(UNIX)
        FIND_LIBRARY(FLYCAPTURE_LIBRARY NAMES libflycapture flycapture PATHS /usr/lib NO_DEFAULT_PATH)
    else(UNIX)
        if(MSVC_VERSION EQUAL 1900)
            find_library(FLYCAPTURE_LIBRARY NAMES "FlyCapture2_v140"  PATHS ${PGR_LIBDIR_HINT} NO_DEFAULT_PATH) #requires Microsoft C++ Redistributible 2010
        elseif(MSVC_VERSION EQUAL 1800)
            find_library(FLYCAPTURE_LIBRARY NAMES "FlyCapture2_v120"  PATHS ${PGR_LIBDIR_HINT} NO_DEFAULT_PATH) #requires Microsoft C++ Redistributible 2010
        elseif(MSVC_VERSION EQUAL 1700)
            find_library(FLYCAPTURE_LIBRARY NAMES "FlyCapture2_v110"  PATHS ${PGR_LIBDIR_HINT} NO_DEFAULT_PATH) #requires Microsoft C++ Redistributible 2012            
        elseif(MSVC_VERSION EQUAL 1600)
            find_library(FLYCAPTURE_LIBRARY NAMES "FlyCapture2_v100"  PATHS ${PGR_LIBDIR_HINT} NO_DEFAULT_PATH) #requires Microsoft C++ Redistributible 2010
        elseif(MSVC_VERSION EQUAL 1500)
            find_library(FLYCAPTURE_LIBRARY NAMES "FlyCapture2_v90"  PATHS ${PGR_LIBDIR_HINT} NO_DEFAULT_PATH) #requires Microsoft C++ Redistributible 2009
        elseif(MSVC_VERSION EQUAL 1400)
            find_library(FLYCAPTURE_LIBRARY NAMES "FlyCapture2"  PATHS ${PGR_LIBDIR_HINT} NO_DEFAULT_PATH) #requires Microsoft C++ Redistributible 2008
        else(MSVC_VERSION)
            find_library(FLYCAPTURE_LIBRARY NAMES "FlyCapture2_v110"  PATHS ${PGR_LIBDIR_HINT} NO_DEFAULT_PATH) #requires Microsoft C++ Redistributible 2012
        endif(MSVC_VERSION)
    endif(UNIX)
    
    if(FLYCAPTURE_LIBRARY)
        message(STATUS "FlyCapture library found at ${FLYCAPTURE_LIBRARY}")
    else(FLYCAPTURE_LIBRARY)
        message(FATAL "FlyCapture library not found")
    endif(FLYCAPTURE_LIBRARY)
    
    if(WIN32)
        if(MSVC_VERSION EQUAL 1900)
            find_file(FLYCAPTURE_BINARY "FlyCapture2_v140.dll"  PATHS ${PGR_BINDIR_HINT} NO_DEFAULT_PATH) #requires Microsoft C++ Redistributible 2010    
        elseif(MSVC_VERSION EQUAL 1800)
            find_file(FLYCAPTURE_BINARY "FlyCapture2_v120.dll"  PATHS ${PGR_BINDIR_HINT} NO_DEFAULT_PATH) #requires Microsoft C++ Redistributible 2010    
        elseif(MSVC_VERSION EQUAL 1700)
            find_file(FLYCAPTURE_BINARY "FlyCapture2_v110.dll"  PATHS ${PGR_BINDIR_HINT} NO_DEFAULT_PATH) #requires Microsoft C++ Redistributible 2010
        elseif(MSVC_VERSION EQUAL 1600)
            find_file(FLYCAPTURE_BINARY "FlyCapture2_v100.dll"  PATHS ${PGR_BINDIR_HINT} NO_DEFAULT_PATH) #requires Microsoft C++ Redistributible 2012            
        elseif(MSVC_VERSION EQUAL 1500)
            find_file(FLYCAPTURE_BINARY "FlyCapture2_v90.dll"  PATHS ${PGR_BINDIR_HINT} NO_DEFAULT_PATH) #requires Microsoft C++ Redistributible 2009
        elseif(MSVC_VERSION EQUAL 1400)
            find_file(FLYCAPTURE_BINARY "FlyCapture2.dll"  PATHS ${PGR_BINDIR_HINT} NO_DEFAULT_PATH) #requires Microsoft C++ Redistributible 2008
        else(MSVC_VERSION)
            find_file(FLYCAPTURE_BINARY "FlyCapture2_v110.dll"  PATHS ${PGR_BINDIR_HINT} NO_DEFAULT_PATH) #requires Microsoft C++ Redistributible 2012
        endif(MSVC_VERSION)
    
        find_file(FLYCAPTURE_LIBIOMP5MD "libiomp5md.dll"  PATHS ${PGR_BINDIR_HINT} NO_DEFAULT_PATH)
    
        if(FLYCAPTURE_BINARY)
            message(STATUS "FlyCapture binary found at ${FLYCAPTURE_BINARY}")
        else(FLYCAPTURE_BINARY)
            message(FATAL "FlyCapture binary not found")
        endif(FLYCAPTURE_BINARY)

        if(FLYCAPTURE_LIBIOMP5MD)
            message(STATUS "libiomp5md.dll found at ${FLYCAPTURE_LIBIOMP5MD}")
        else(FLYCAPTURE_LIBIOMP5MD)
            message(FATAL "libiomp5md.dll not found")
        endif(FLYCAPTURE_LIBIOMP5MD)
    endif(WIN32)
    
    add_library(${target_name} ${plugin_SOURCES} ${plugin_HEADERS} ${plugin_HEADERS_MOC} ${plugin_UI_MOC} ${plugin_RCC_MOC} ${EXISTING_TRANSLATION_FILES})
    
    target_link_libraries(${target_name} ${ITOM_SDK_LIBRARIES} ${FLYCAPTURE_LIBRARY} ${QT5_LIBRARIES} ${VISUALLEAKDETECTOR_LIBRARIES}) 


    
    PLUGIN_DOCUMENTATION(${target_name} pgrFlyCapture)
    
    #translation
    set(FILES_TO_TRANSLATE ${plugin_SOURCES} ${plugin_HEADERS} ${plugin_UI})
    PLUGIN_TRANSLATION(QM_FILES ${target_name} ${UPDATE_TRANSLATIONS} "${EXISTING_TRANSLATION_FILES}" ITOM_LANGUAGES "${FILES_TO_TRANSLATE}")

    # COPY SECTION
    set(COPY_SOURCES "")
    set(COPY_DESTINATIONS "")
    ADD_PLUGINLIBRARY_TO_COPY_LIST(${target_name} COPY_SOURCES COPY_DESTINATIONS)
    ADD_QM_FILES_TO_COPY_LIST(${target_name} QM_FILES COPY_SOURCES COPY_DESTINATIONS)
    POST_BUILD_COPY_FILES(${target_name} COPY_SOURCES COPY_DESTINATIONS)
    
    if(WIN32)
        POST_BUILD_COPY_FILE_TO_LIB_FOLDER(${target_name} FLYCAPTURE_BINARY)
        POST_BUILD_COPY_FILE_TO_LIB_FOLDER(${target_name} FLYCAPTURE_LIBIOMP5MD)
    endif(WIN32)
    
else(FLYCAPTURE_PGRFLYCAP_INCLUDE_DIR)
    message(WARNING "${target_name}-API directory could not be found. ${target_name} will not be build")    
endif(FLYCAPTURE_PGRFLYCAP_INCLUDE_DIR)