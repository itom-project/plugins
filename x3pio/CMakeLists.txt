cmake_minimum_required(VERSION 3.12...3.24)

set(target_name x3pio)

option(BUILD_SHARED_LIBS "Build shared library." ON)
option(BUILD_TARGET64 "Build for 64 bit target if set to ON or 32 bit if set to OFF." ON)

#this is to automatically detect the SDK subfolder of the itom build directory.
if(NOT EXISTS ${ITOM_SDK_DIR})
    find_path(ITOM_SDK_DIR "cmake/itom_sdk.cmake"
    HINTS "$ENV{ITOM_SDK_ROOT}"
          "${CMAKE_CURRENT_BINARY_DIR}/../itom/SDK"
    DOC "Path of SDK subfolder of itom root (build) directory")
endif(NOT EXISTS ${ITOM_SDK_DIR})

if(NOT EXISTS ${ITOM_SDK_DIR})
    message(FATAL_ERROR "ITOM_SDK_DIR is invalid. Provide itom SDK directory path first")
endif(NOT EXISTS ${ITOM_SDK_DIR})

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR} ${ITOM_SDK_DIR}/cmake)

find_package(ITOM_SDK COMPONENTS dataobject itomCommonLib itomCommonQtLib REQUIRED)
find_package(Xerces QUIET)
find_package(XSD)

include(ItomBuildMacros)
itom_init_cmake_policy(3.12)
itom_init_plugin_library(${target_name}) #Start the project, init compiler settings and set default configurations for plugins
itom_find_package_qt(ON Core Widgets Xml LinguistTools)

if(XERCESC_FOUND AND XSD_FOUND)

    add_subdirectory(x3plib/src)

    find_path(ISO5436_2_PATH iso5436_2.xsd
      PATHS $ENV{OPENGPS_DIR}
      ./x3plib/src/ISO5436_2_XML
      ./x3p
      ../ISO5436_2_XML
      ../x3p
    )

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
        )

    add_definitions(-DISO5436_2_LIBRARY)

    # see https://stackoverflow.com/questions/48882439/how-to-restore-auto-ptr-in-visual-studio-c17
    add_definitions(-D_HAS_AUTO_PTR_ETC=1)

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
        else()
            # since we don't knwo wether we are on windows or not, we just undefined and see what happens
            unset(XSDLIB_PATH_POSTFIX)
        endif()

        # Wiora: Set 64 bit target dir (currently this is windows only. How does this work on linux/mac?)
        if(CMAKE_CL_64)
            set(XSD_LIBPATH_POSTFIX lib64/)
        else(CMAKE_CL_64)
            set(XSD_LIBPATH_POSTFIX lib/)
        endif(CMAKE_CL_64)

        set(XSD_LIBPATH_POSTFIX ${XSD_LIBPATH_POSTFIX} ${XSD_LIBPATH_VERS_POSTFIX})


    else(DEFINED MSVC_VERSION)
        set(XSD_LIB_PATH_POSTFIX "")
        set(XSD_LIB_POSTFIX "")
    endif(DEFINED MSVC_VERSION)

    link_directories(
         ${OpenCV_DIR}/lib
         ${XSD_INCLUDE_DIR}/${XSD_LIBPATH_POSTFIX}
         ${CMAKE_CURRENT_BINARY_DIR}/x3plib/src/zlib/contrib/minizip
    )


    set(PLUGIN_HEADERS
        ${CMAKE_CURRENT_SOURCE_DIR}/x3pio.h
        ${CMAKE_CURRENT_SOURCE_DIR}/pluginVersion.h
        ${CMAKE_CURRENT_BINARY_DIR}/gitVersion.h
    )

    set(PLUGIN_UI)

    set(PLUGIN_SOURCES
        ${CMAKE_CURRENT_SOURCE_DIR}/x3pio.cpp
    )

    #Add version information to the plugIn-dll unter MSVC
    if(MSVC)
        list(APPEND PLUGIN_SOURCES ${ITOM_SDK_INCLUDE_DIR}/../pluginLibraryVersion.rc)
    endif()



    add_library(${target_name} SHARED ${PLUGIN_SOURCES} ${PLUGIN_HEADERS})

    if(LINUX)
        set(ZLIBNAME "minizip")
    else(LINUX)
        set(ZLIBNAME "zlibwapi")
    endif(LINUX)

    set(iso54362LIBSUFFIX "")

    if(CMAKE_CL_64)
        set(iso54362LIBSUFFIX ${iso54362LIBSUFFIX}64)
    endif(CMAKE_CL_64)

    # Qt: enable all automoc, autouic and autorcc.
    set_target_properties(${target_name} PROPERTIES AUTOMOC ON AUTORCC ON AUTOUIC ON)

    target_link_libraries(${target_name} ${ITOM_SDK_LIBRARIES} ${XERCESC_LIBRARY} ${zlibname} iso5436-2-xml${iso54362LIBSUFFIX} ${QT5_LIBRARIES} ${VISUALLEAKDETECTOR_LIBRARIES})



    set(FILES_TO_TRANSLATE ${PLUGIN_SOURCES} ${PLUGIN_HEADERS} ${PLUGIN_UI})
    itom_library_translation(QM_FILES TARGET ${target_name} FILES_TO_TRANSLATE ${FILES_TO_TRANSLATE})


    # COPY SECTION
    set(COPY_SOURCES "")
    set(COPY_DESTINATIONS "")
    itom_add_pluginlibrary_to_copy_list(${target_name} COPY_SOURCES COPY_DESTINATIONS)

    if(WIN32)
        set(iso54362xmlNAME iso5436-2-xml${iso54362LIBSUFFIX})

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

    endif(WIN32)

    LIST(APPEND COPY_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/x3plib/src/ISO5436_2_XML/iso5436_2.xsd")
    LIST(APPEND COPY_DESTINATIONS "${ITOM_APP_DIR}/lib")

    itom_configure_plugin_documentation(${target_name} x3pio)

    itom_add_plugin_qm_files_to_copy_list(${target_name} QM_FILES COPY_SOURCES COPY_DESTINATIONS)
    itom_post_build_copy_files(${target_name} COPY_SOURCES COPY_DESTINATIONS)

else(XERCESC_FOUND AND XSD_FOUND)
    message(SEND_ERROR "Xerces or xsd could not be found. ${target_name} will not be build.")
endif(XERCESC_FOUND AND XSD_FOUND)
