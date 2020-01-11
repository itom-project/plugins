set(target_name x3pio)

project(${target_name})

message(STATUS "\n--------------- PLUGIN ${target_name} ---------------")

cmake_minimum_required(VERSION 3.0.2)

option(BUILD_UNICODE "Build with unicode charset if set to ON, else multibyte charset." ON)
option(BUILD_SHARED_LIBS "Build shared library." ON)
option(BUILD_TARGET64 "Build for 64 bit target if set to ON or 32 bit if set to OFF." ON)
option(UPDATE_TRANSLATIONS "Update source translation translation/*.ts files (WARNING: make clean will delete the source .ts files! Danger!)")
set(ITOM_SDK_DIR "" CACHE PATH "base path to itom_sdk")
set(CMAKE_DEBUG_POSTFIX "d" CACHE STRING "Adds a postfix for debug-built libraries.")
set(ITOM_LANGUAGES "de" CACHE STRING "semicolon separated list of languages that should be created (en must not be given since it is the default)")
set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR} ${ITOM_SDK_DIR})

include("${ITOM_SDK_DIR}/ItomBuildMacros.cmake")
INIT_ITOM_LIBRARY() #apply general project settings, CMake configurations, policies... for itom and its libraries

if(BUILD_SHARED_LIBS)
    set(LIBRARY_TYPE SHARED)
else(BUILD_SHARED_LIBS)
    set(LIBRARY_TYPE STATIC)
endif(BUILD_SHARED_LIBS)

find_package(ITOM_SDK COMPONENTS dataobject itomCommonLib itomCommonQtLib REQUIRED)
FIND_PACKAGE_QT(ON Core Widgets Xml LinguistTools)

find_package(Xerces QUIET)
find_package(Xsd QUIET)
find_package(VisualLeakDetector QUIET)

if(XERCESC_FOUND AND XSD_FOUND)
    
    add_subdirectory(x3plib/src)
    
    find_path(ISO5436_2_PATH iso5436_2.xsd
      PATHS $ENV{OPENGPS_DIR}
      ./x3plib/src/ISO5436_2_XML
      ./x3p
      ../ISO5436_2_XML
      ../x3p
    )

    if(BUILD_SHARED_LIBS)
       if(LINUX)
          message(STATUS "Build shared")
            find_path(ISO5436_2_LIB_PATH libiso5436-2-xml.so
              PATHS $ENV{OPENGPS_DIR}
              ./x3plib/src/ISO5436_2_XML
              ./x3p
              ../ISO5436_2_XML
              ../x3p
              ../../build
              ../../build/ISO5436_2_XML
              ../../build/x3p
            )
       endif(LINUX)
    else(BUILD_SHARED_LIBS)
    message(STATUS "Build static")
      if(LINUX)
         find_path(ISO5436_2_LIB_PATH libiso5436-2-xml.a
           PATHS $ENV{OPENGPS_DIR}
           ./x3plib/src/ISO5436_2_XML
           ./x3p
           ../ISO5436_2_XML
           ../x3p
           ../../build
           ../../build/ISO5436_2_XML
           ../../build/x3p
         )
       endif(LINUX)
    endif(BUILD_SHARED_LIBS)
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
    include_directories(
        "x3plib/include"
        #"x3plib/include/opengps/cxx"
        ${CMAKE_CURRENT_BINARY_DIR}
        ${CMAKE_CURRENT_BINARY_DIR}/x3plib/src/ISO5436_2_XML/
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${ITOM_SDK_INCLUDE_DIRS}
        ${XSD_INCLUDE_DIR}
        ${XERCESC_INCLUDE}
        ${CMAKE_CURRENT_SOURCE_DIR}/x3plib/include
        ${VISUALLEAKDETECTOR_INCLUDE_DIR}
    )

    add_definitions(-DUNICODE -D_UNICODE -DISO5436_2_LIBRARY)

    include("${ITOM_SDK_DIR}/ItomBuildMacros.cmake")


    add_definitions(-DCMAKE)

    if(VISUALLEAKDETECTOR_FOUND AND VISUALLEAKDETECTOR_ENABLED)
        add_definitions(-DVISUAL_LEAK_DETECTOR_CMAKE)
    endif(VISUALLEAKDETECTOR_FOUND AND VISUALLEAKDETECTOR_ENABLED)

    # default build types are None, Debug, Release, RelWithDebInfo and MinRelSize
    if(DEFINED CMAKE_BUILD_TYPE)
        set(CMAKE_BUILD_TYPE ${CMAKE_BUILD_TYPE} CACHE STRING "Choose the type of build, options are: None(CMAKE_CXX_FLAGS or CMAKE_C_FLAGS used) Debug Release RelWithDebInfo MinSizeRel.")
    else()
        set(CMAKE_BUILD_TYPE Debug CACHE STRING "Choose the type of build, options are: None(CMAKE_CXX_FLAGS or CMAKE_C_FLAGS used) Debug Release RelWithDebInfo MinSizeRel.")
    endif()


    if(DEFINED MSVC_VERSION)
      # Library postfix/ prefix for different vs version
      #   1300 = VS  7.0
      #   1400 = VS  8.0
      #   1500 = VS  9.0
      #   1600 = VS 10.0
      if(MSVC_VERSION EQUAL 1300)
        set(XSD_LIB_POSTFIX "_vc70")
        set(XSD_LIBPATH_VERS_POSTFIX "vc-7.1/")
      elseif(MSVC_VERSION EQUAL 1400)
        set(XSD_LIB_POSTFIX "_vc80")
        set(XSD_LIBPATH_VERS_POSTFIX "vc-8.0/")  
      elseif(MSVC_VERSION EQUAL 1500)
        set(XSD_LIB_POSTFIX "_vc90")
        set(XSD_LIBPATH_VERS_POSTFIX "vc-9.0/")  
      elseif(MSVC_VERSION EQUAL 1600)
        set(XSD_LIB_POSTFIX "_vc100")
        set(XSD_LIBPATH_VERS_POSTFIX "vc-10.0/")  
      else(MSVC_VERSION EQUAL 1300)
        # since we don't knwo wether we are on windows or not, we just undefined and see what happens
        unset(XSDLIB_PATH_POSTFIX)
      endif(MSVC_VERSION EQUAL 1300)

      # Wiora: Set 64 bit target dir (currently this is windows only. How does this work on linux/mac?)
      if(BUILD_SHARED_LIBS)  
         if(CMAKE_CL_64)
            set(XSD_LIBPATH_POSTFIX lib64/)
          else(CMAKE_CL_64)
            set(XSD_LIBPATH_POSTFIX lib/)
          endif(CMAKE_CL_64)
          set(XSD_LIBPATH_POSTFIX ${XSD_LIBPATH_POSTFIX}${XSD_LIBPATH_VERS_POSTFIX})
      else(BUILD_SHARED_LIBS)
          if(CMAKE_CL_64)
            set(XSD_LIBPATH_POSTFIX lib64/)
          else(CMAKE_CL_64)
            set(XSD_LIBPATH_POSTFIX lib/)
          endif(CMAKE_CL_64)
          set(XSD_LIBPATH_POSTFIX ${XSD_LIBPATH_POSTFIX}${XSD_LIBPATH_VERS_POSTFIX})
      endif(BUILD_SHARED_LIBS)

    else(DEFINED MSVC_VERSION)
      set(XSD_LIB_PATH_POSTFIX "")
      set(XSD_LIB_POSTFIX "")
    endif(DEFINED MSVC_VERSION)

    link_directories(
         ${OpenCV_DIR}/lib
         ${XSD_INCLUDE_DIR}/${XSD_LIBPATH_POSTFIX}
         ${CMAKE_CURRENT_BINARY_DIR}/x3plib/src/zlib/contrib/minizip
    )


    set(plugin_HEADERS
        ${CMAKE_CURRENT_SOURCE_DIR}/x3pio.h
        ${CMAKE_CURRENT_SOURCE_DIR}/pluginVersion.h
        ${CMAKE_CURRENT_BINARY_DIR}/gitVersion.h
    )

    set(plugin_UI)

    set(plugin_SOURCES 
        ${CMAKE_CURRENT_SOURCE_DIR}/x3pio.cpp
    )

    #Add version information to the plugIn-dll unter MSVC
    if(MSVC)
        list(APPEND plugin_SOURCES ${ITOM_SDK_INCLUDE_DIR}/../pluginLibraryVersion.rc)
    endif()    
    
    if(QT5_FOUND)
        #if automoc if OFF, you also need to call QT5_WRAP_CPP here
        QT5_WRAP_UI(plugin_UI_MOC ${plugin_UI})
        QT5_ADD_RESOURCES(plugin_RCC_MOC ${plugin_RCC})
    else()
        message(SEND_ERROR "Currently only Qt5 is supported")
    endif()
    
    #search for all existing translation files in the translation subfolder
    file(GLOB EXISTING_TRANSLATION_FILES "translation/*.ts")

    add_library(${target_name} ${LIBRARY_TYPE} ${plugin_SOURCES} ${plugin_HEADERS} ${plugin_HEADERS_MOC} ${plugin_UI_MOC} ${plugin_RCC_MOC} ${EXISTING_TRANSLATION_FILES})

    if(LINUX)
    set(ZLIBNAME "minizip")
    else(LINUX)
    set(ZLIBNAME "zlibwapi")
    endif(LINUX)

    if(BUILD_SHARED_LIBS)
    else(BUILD_SHARED_LIBS)
    set(iso54362LIBSUFFIX S)
    endif(BUILD_SHARED_LIBS)

    if(CMAKE_CL_64)
        set(iso54362LIBSUFFIX ${iso54362LIBSUFFIX}64)
    endif(CMAKE_CL_64)

    #handle translations BEGIN STEP 1
    set(FILES_TO_TRANSLATE ${plugin_SOURCES} ${plugin_HEADERS} ${plugin_UI})
    set(TRANSLATION_OUTPUT_FILES)
    set(TRANSLATIONS_FILES)

    target_link_libraries(${target_name} ${QT_LIBRARIES} ${ITOM_SDK_LIBRARIES} ${XERCESC_LIBRARY} ${zlibname} iso5436-2-xml${iso54362LIBSUFFIX} ${QT5_LIBRARIES} ${VISUALLEAKDETECTOR_LIBRARIES})


    
    set(FILES_TO_TRANSLATE ${plugin_SOURCES} ${plugin_HEADERS} ${plugin_UI})
    PLUGIN_TRANSLATION(QM_FILES ${target_name} ${UPDATE_TRANSLATIONS} "${EXISTING_TRANSLATION_FILES}" ITOM_LANGUAGES "${FILES_TO_TRANSLATE}")
    

    # COPY SECTION
    set(COPY_SOURCES "")
    set(COPY_DESTINATIONS "")
    ADD_PLUGINLIBRARY_TO_COPY_LIST(${target_name} COPY_SOURCES COPY_DESTINATIONS)

    if(WIN32)
        set(iso54362xmlNAME iso5436-2-xml${iso54362LIBSUFFIX})
        
        if(BUILD_SHARED_LIBS)
            #LIST(APPEND COPY_SOURCES "${CMAKE_CURRENT_BINARY_DIR}/x3plib/src/zlib/contrib/minizip/$<CONFIGURATION>/${ZLIBNAME}${CMAKE_DEBUG_POSTFIX}.dll")
            #LIST(APPEND COPY_DESTINATIONS "${ITOM_APP_DIR}/lib")
            
            if(CMAKE_CL_64)
                set(iso54352_target "iso5436-2-xml64")
            else()
                set(iso54352_target "iso5436-2-xml")
            endif(CMAKE_CL_64)

            LIST(APPEND COPY_SOURCES "$<TARGET_FILE:${iso54352_target}>")
            #LIST(APPEND COPY_SOURCES "${CMAKE_CURRENT_BINARY_DIR}/x3plib/src/ISO5436_2_XML/$<CONFIGURATION>/${iso54362xmlNAME}${CMAKE_DEBUG_POSTFIX}.dll")
            LIST(APPEND COPY_DESTINATIONS "${ITOM_APP_DIR}/lib")
            
            LIST(APPEND COPY_SOURCES "${XERCESC_BINARY}")
            LIST(APPEND COPY_DESTINATIONS "${ITOM_APP_DIR}/lib")
        endif(BUILD_SHARED_LIBS)   

    endif(WIN32)

    LIST(APPEND COPY_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/x3plib/src/ISO5436_2_XML/iso5436_2.xsd")
    LIST(APPEND COPY_DESTINATIONS "${ITOM_APP_DIR}/lib")
    
    PLUGIN_DOCUMENTATION(${target_name} x3pio)

    ADD_QM_FILES_TO_COPY_LIST(${target_name} QM_FILES COPY_SOURCES COPY_DESTINATIONS)
    POST_BUILD_COPY_FILES(${target_name} COPY_SOURCES COPY_DESTINATIONS)

else(XERCESC_FOUND AND XSD_FOUND)
    message(WARNING "xerces or xsd could not be found. ${target_name} will not be build.")
endif(XERCESC_FOUND AND XSD_FOUND)
