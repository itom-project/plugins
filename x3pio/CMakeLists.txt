SET (target_name x3pio)

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

find_package(ITOM_SDK REQUIRED)
include("${ITOM_SDK_DIR}/ItomBuildMacros.cmake")
FIND_PACKAGE_QT(ON Widgets Xml LinguistTools)

find_package(OpenCV COMPONENTS core REQUIRED)
find_package(Xerces)
find_package(Xsd)
find_package(VisualLeakDetector QUIET)

IF(XERCESC_FOUND AND XSD_FOUND)
    
    ADD_SUBDIRECTORY(x3plib/src)
    
    FIND_PATH(ISO5436_2_PATH iso5436_2.xsd
      PATHS $ENV{OPENGPS_DIR}
      ./x3plib/src/ISO5436_2_XML
      ./x3p
      ../ISO5436_2_XML
      ../x3p
    )

    IF(BUILD_SHARED_LIBS)
       IF (LINUX)
          MESSAGE(STATUS "Build shared")
            FIND_PATH(ISO5436_2_LIB_PATH libiso5436-2-xml.so
              PATHS $ENV{OPENGPS_DIR}
              ./x3plib/src/ISO5436_2_XML
              ./x3p
              ../ISO5436_2_XML
              ../x3p
              ../../build
              ../../build/ISO5436_2_XML
              ../../build/x3p
            )
       ENDIF(LINUX)
    ELSE(BUILD_SHARED_LIBS)
    MESSAGE(STATUS "Build static")
      IF (LINUX)
         FIND_PATH(ISO5436_2_LIB_PATH libiso5436-2-xml.a
           PATHS $ENV{OPENGPS_DIR}
           ./x3plib/src/ISO5436_2_XML
           ./x3p
           ../ISO5436_2_XML
           ../x3p
           ../../build
           ../../build/ISO5436_2_XML
           ../../build/x3p
         )
       ENDIF(LINUX)
    ENDIF(BUILD_SHARED_LIBS)

    INCLUDE_DIRECTORIES(
        "x3plib/include"
        #"x3plib/include/opengps/cxx"
        ${CMAKE_CURRENT_BINARY_DIR}
        ${CMAKE_CURRENT_BINARY_DIR}/x3plib/src/ISO5436_2_XML/
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${QT_QTCORE_INCLUDE_DIR}
        ${OpenCV_DIR}/include
        ${ITOM_SDK_INCLUDE_DIR}
        ${XSD_INCLUDE_DIR}
        ${XERCESC_INCLUDE}
        ${VISUALLEAKDETECTOR_INCLUDE_DIR}
    )

    add_definitions(-DUNICODE -D_UNICODE -DISO5436_2_LIBRARY)

    include("${ITOM_SDK_DIR}/ItomBuildMacros.cmake")

    ADD_DEFINITIONS(${QT_DEFINITIONS})
    #IF (BUILD_UNICODE)
    #    ADD_DEFINITIONS(-DUNICODE -D_UNICODE)
    #ENDIF (BUILD_UNICODE)
    ADD_DEFINITIONS(-DCMAKE)

    IF(VISUALLEAKDETECTOR_FOUND AND VISUALLEAKDETECTOR_ENABLED)
        ADD_DEFINITIONS(-DVISUAL_LEAK_DETECTOR_CMAKE)
    ENDIF(VISUALLEAKDETECTOR_FOUND AND VISUALLEAKDETECTOR_ENABLED)

    # enable some qt stuff
    SET (QT_USE_QTXML TRUE)

    # default build types are None, Debug, Release, RelWithDebInfo and MinRelSize
    IF (DEFINED CMAKE_BUILD_TYPE)
        SET(CMAKE_BUILD_TYPE ${CMAKE_BUILD_TYPE} CACHE STRING "Choose the type of build, options are: None(CMAKE_CXX_FLAGS or CMAKE_C_FLAGS used) Debug Release RelWithDebInfo MinSizeRel.")
    ELSE(CMAKE_BUILD_TYPE)
        SET (CMAKE_BUILD_TYPE Debug CACHE STRING "Choose the type of build, options are: None(CMAKE_CXX_FLAGS or CMAKE_C_FLAGS used) Debug Release RelWithDebInfo MinSizeRel.")
    ENDIF (DEFINED CMAKE_BUILD_TYPE)

    INCLUDE_DIRECTORIES(
        ${CMAKE_CURRENT_BINARY_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${QT_QTCORE_INCLUDE_DIR}
        ${OpenCV_DIR}/include
        ${ITOM_SDK_INCLUDE_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}/x3plib/include
        ${XSD_INCLUDE_DIR}/include
    )

    IF (DEFINED MSVC_VERSION)
      # Library postfix/ prefix for different vs version
      #   1300 = VS  7.0
      #   1400 = VS  8.0
      #   1500 = VS  9.0
      #   1600 = VS 10.0
      IF (MSVC_VERSION EQUAL 1300)
        SET(XSD_LIB_POSTFIX "_vc70")
        SET(XSD_LIBPATH_VERS_POSTFIX "vc-7.1/")
      ELSEIF (MSVC_VERSION EQUAL 1400)
        SET(XSD_LIB_POSTFIX "_vc80")
        SET(XSD_LIBPATH_VERS_POSTFIX "vc-8.0/")  
      ELSEIF (MSVC_VERSION EQUAL 1500)
        SET(XSD_LIB_POSTFIX "_vc90")
        SET(XSD_LIBPATH_VERS_POSTFIX "vc-9.0/")  
      ELSEIF (MSVC_VERSION EQUAL 1600)
        SET(XSD_LIB_POSTFIX "_vc100")
        SET(XSD_LIBPATH_VERS_POSTFIX "vc-10.0/")  
      ELSE (MSVC_VERSION EQUAL 1300)
        # since we don't knwo wether we are on windows or not, we just undefined and see what happens
        UNSET(XSDLIB_PATH_POSTFIX)
      ENDIF (MSVC_VERSION EQUAL 1300)

      # Wiora: Set 64 bit target dir (currently this is windows only. How does this work on linux/mac?)
      IF (BUILD_SHARED_LIBS)  
         IF (CMAKE_CL_64)
            SET (XSD_LIBPATH_POSTFIX lib64/)
          ELSE (CMAKE_CL_64)
            SET (XSD_LIBPATH_POSTFIX lib/)
          ENDIF (CMAKE_CL_64)
          SET(XSD_LIBPATH_POSTFIX ${XSD_LIBPATH_POSTFIX}${XSD_LIBPATH_VERS_POSTFIX})
      ELSE (BUILD_SHARED_LIBS)
          if(CMAKE_CL_64)
            SET (XSD_LIBPATH_POSTFIX lib64/)
          ELSE (CMAKE_CL_64)
            SET (XSD_LIBPATH_POSTFIX lib/)
          ENDIF (CMAKE_CL_64)
          SET(XSD_LIBPATH_POSTFIX ${XSD_LIBPATH_POSTFIX}${XSD_LIBPATH_VERS_POSTFIX})
      ENDIF (BUILD_SHARED_LIBS)

    ELSE(DEFINED MSVC_VERSION)
      SET(XSD_LIB_PATH_POSTFIX "")
      SET(XSD_LIB_POSTFIX "")
    ENDIF (DEFINED MSVC_VERSION)

    LINK_DIRECTORIES(
         ${OpenCV_DIR}/lib
         ${XSD_INCLUDE_DIR}/${XSD_LIBPATH_POSTFIX}
         ${CMAKE_CURRENT_BINARY_DIR}/x3plib/src/zlib/contrib/minizip
    )

    if (NOT QT5_FOUND)
      INCLUDE(${QT_USE_FILE})
    endif (NOT QT5_FOUND)

    set(plugin_HEADERS
        ${ITOM_SDK_INCLUDE_DIR}/common/addInInterface.h
        ${ITOM_SDK_INCLUDE_DIR}/common/sharedStructures.h
        ${ITOM_SDK_INCLUDE_DIR}/common/sharedStructuresQt.h
        ${ITOM_SDK_INCLUDE_DIR}/common/typeDefs.h
        ${CMAKE_CURRENT_SOURCE_DIR}/x3pio.h
        ${CMAKE_CURRENT_SOURCE_DIR}/pluginVersion.h
    )

    if (NOT QT5_FOUND)
      QT4_WRAP_CPP_ITOM(plugin_HEADERS_MOC ${plugin_HEADERS})
    endif (NOT QT5_FOUND)

    set(plugin_ui)

    if (QT5_FOUND)
	QT5_WRAP_UI(plugin_ui_MOC ${plugin_ui})
    else (QT5_FOUND)
	QT4_WRAP_UI_ITOM(plugin_ui_MOC ${plugin_ui})
    endif (QT5_FOUND)

    set(plugin_SOURCES 
        ${CMAKE_CURRENT_SOURCE_DIR}/x3pio.cpp
    )

    #Add version information to the plugIn-dll unter MSVC
    if(MSVC)
        list(APPEND plugin_SOURCES ${ITOM_SDK_INCLUDE_DIR}/../pluginLibraryVersion.rc)
    endif(MSVC)    
    
    file (GLOB EXISTING_TRANSLATION_FILES "translation/*.ts")
    #handle translations END STEP 1

    ADD_LIBRARY(${target_name} ${LIBRARY_TYPE} ${plugin_SOURCES} ${plugin_HEADERS} ${plugin_HEADERS_MOC} ${plugin_ui_MOC} ${plugin_rcc_MOC} ${EXISTING_TRANSLATION_FILES})

    IF (LINUX)
	set(ZLIBNAME "minizip")
    ELSE (LINUX)
	set(ZLIBNAME "zlibwapi")
    ENDIF(LINUX)

    IF(BUILD_SHARED_LIBS)
    ELSE(BUILD_SHARED_LIBS)
	SET(iso54362LIBSUFFIX S)
    ENDIF(BUILD_SHARED_LIBS)

    if(CMAKE_CL_64)
        SET(iso54362LIBSUFFIX ${iso54362LIBSUFFIX}64)
    endif(CMAKE_CL_64)

    #handle translations BEGIN STEP 1
    set (FILES_TO_TRANSLATE ${plugin_SOURCES} ${plugin_HEADERS} ${plugin_ui})
    set (TRANSLATION_OUTPUT_FILES)
    set (TRANSLATIONS_FILES)

    if (QT5_FOUND)
	TARGET_LINK_LIBRARIES(${target_name} ${OpenCV_LIBS} ${ITOM_SDK_dataobject_LIBRARY} ${XERCESC_LIBRARY} ${zlibname} iso5436-2-xml${iso54362LIBSUFFIX} ${VISUALLEAKDETECTOR_LIBRARIES} ${ITOM_SDK_itomCommonLib_LIBRARY} ${ITOM_SDK_itomCommonQtLib_LIBRARY})
	qt5_use_modules(${target_name} Widgets UiTools Xml)
    else (QT5_FOUND)
	TARGET_LINK_LIBRARIES(${target_name} ${QT_LIBRARIES} ${QT_QTUITOOLS_LIBRARY} ${QT_QTXML_LIBRARY} ${OpenCV_LIBS} ${ITOM_SDK_dataobject_LIBRARY} ${XERCESC_LIBRARY} ${zlibname} iso5436-2-xml${iso54362LIBSUFFIX} ${VISUALLEAKDETECTOR_LIBRARIES} ${ITOM_SDK_itomCommonLib_LIBRARY} ${ITOM_SDK_itomCommonQtLib_LIBRARY})
    endif (QT5_FOUND)
    ADD_DEPENDENCIES(${target_name} x3plib)

    #handle translations BEGIN STEP 2
    if (UPDATE_TRANSLATIONS)
	if (QT5_FOUND)
	    QT5_CREATE_TRANSLATION(TRANSLATION_OUTPUT_FILES TRANSLATIONS_FILES ${target_name} ITOM_LANGUAGES ${FILES_TO_TRANSLATE} ${TRANSLATIONS_FILES})
	else (QT5_FOUND)
	    QT4_CREATE_TRANSLATION_ITOM(TRANSLATION_OUTPUT_FILES TRANSLATIONS_FILES ${target_name} ITOM_LANGUAGES ${FILES_TO_TRANSLATE} ${TRANSLATIONS_FILES})
	endif (QT5_FOUND)
        
        add_custom_target (_${target_name}_translation DEPENDS ${TRANSLATION_OUTPUT_FILES})
        add_dependencies(${target_name} ${target_name}_translation)
        
	if (QT5_FOUND)
	    QT5_ADD_TRANSLATION(QM_FILES "${CMAKE_CURRENT_BINARY_DIR}/translation" ${target_name} ${TRANSLATIONS_FILES})
	else (QT5_FOUND)
	    QT4_ADD_TRANSLATION_ITOM(QM_FILES "${CMAKE_CURRENT_BINARY_DIR}/translation" ${target_name} ${TRANSLATIONS_FILES})
	endif (QT5_FOUND)
    else (UPDATE_TRANSLATIONS)
      if (QT5_FOUND)
	  QT5_ADD_TRANSLATION(QM_FILES "${CMAKE_CURRENT_BINARY_DIR}/translation" ${target_name} ${EXISTING_TRANSLATION_FILES})
      else (QT5_FOUND)
	  QT4_ADD_TRANSLATION_ITOM(QM_FILES "${CMAKE_CURRENT_BINARY_DIR}/translation" ${target_name} ${EXISTING_TRANSLATION_FILES})
      endif (QT5_FOUND)
    endif (UPDATE_TRANSLATIONS)
    #handle translations END STEP 2
    

    # COPY SECTION
    set(COPY_SOURCES "")
    set(COPY_DESTINATIONS "")
    ADD_PLUGINLIBRARY_TO_COPY_LIST(${target_name} COPY_SOURCES COPY_DESTINATIONS)

    IF (WIN32)
        SET(iso54362xmlNAME iso5436-2-xml${iso54362LIBSUFFIX})
        
        IF(BUILD_SHARED_LIBS)
            #LIST(APPEND COPY_SOURCES "${CMAKE_CURRENT_BINARY_DIR}/x3plib/src/zlib/contrib/minizip/$<CONFIGURATION>/${ZLIBNAME}${CMAKE_DEBUG_POSTFIX}.dll")
            #LIST(APPEND COPY_DESTINATIONS "${ITOM_APP_DIR}/lib")
            
            if(CMAKE_CL_64)
                SET(iso54352_target "iso5436-2-xml64")
            else()
                SET(iso54352_target "iso5436-2-xml")
            endif(CMAKE_CL_64)

            LIST(APPEND COPY_SOURCES "$<TARGET_FILE:${iso54352_target}>")
            #LIST(APPEND COPY_SOURCES "${CMAKE_CURRENT_BINARY_DIR}/x3plib/src/ISO5436_2_XML/$<CONFIGURATION>/${iso54362xmlNAME}${CMAKE_DEBUG_POSTFIX}.dll")
            LIST(APPEND COPY_DESTINATIONS "${ITOM_APP_DIR}/lib")
        ENDIF(BUILD_SHARED_LIBS)   

    ENDIF(WIN32)

    LIST(APPEND COPY_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/x3plib/src/ISO5436_2_XML/iso5436_2.xsd")
    LIST(APPEND COPY_DESTINATIONS "${ITOM_APP_DIR}/lib")
    
    PLUGIN_DOCUMENTATION(${target_name} x3pio)

    if (QT5_FOUND)
	
    else (QT5_FOUND)
	ADD_QM_FILES_TO_COPY_LIST(${target_name} QM_FILES COPY_SOURCES COPY_DESTINATIONS)
    endif (QT5_FOUND)
    POST_BUILD_COPY_FILES(${target_name} COPY_SOURCES COPY_DESTINATIONS)

ELSE(XERCESC_FOUND AND XSD_FOUND)
    message(WARNING "xerces or xsd could not be found. ${target_name} will not be build.")
ENDIF(XERCESC_FOUND AND XSD_FOUND)