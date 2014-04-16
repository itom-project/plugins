SET (target_name PclTools)

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
FIND_PACKAGE_QT(ON Widgets LinguistTools)

find_package(ITOM_SDK COMPONENTS dataobject pointcloud itomCommonLib itomCommonQtLib REQUIRED)

IF(ITOM_SDK_PCL_SUPPORT)
    find_package(OpenCV REQUIRED)
    find_package(VTK QUIET)
    
    IF(VTK_FOUND)
        SET(VTK_INSTALL_EXPORT_NAME 1) #avoid that vtk directly links to an absolute Qt-library, that is located somewhere on their computer configuration.
        find_package(PCL 1.5.1 REQUIRED COMPONENTS common io kdtree search features segmentation filters surface) #io
        find_package(VisualLeakDetector QUIET)

        include("${ITOM_SDK_DIR}/ItomBuildMacros.cmake")

        ADD_DEFINITIONS(${QT_DEFINITIONS})
        IF (BUILD_UNICODE)
            ADD_DEFINITIONS(-DUNICODE -D_UNICODE)
        ENDIF (BUILD_UNICODE)
        ADD_DEFINITIONS(-DCMAKE)

        IF(VISUALLEAKDETECTOR_FOUND AND VISUALLEAKDETECTOR_ENABLED)
            ADD_DEFINITIONS(-DVISUAL_LEAK_DETECTOR_CMAKE)
        ENDIF(VISUALLEAKDETECTOR_FOUND AND VISUALLEAKDETECTOR_ENABLED)

        # enable some qt stuff
        if (NOT QT5_FOUND)
            SET (QT_USE_QTXML TRUE)
        endif (NOT QT5_FOUND)

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
            ${PCL_INCLUDE_DIRS}
            ${ITOM_SDK_INCLUDE_DIR}
            ${VISUALLEAKDETECTOR_INCLUDE_DIR}
            ${EIGEN_INCLUDE_DIR}
            ${FLANN_INCLUDE_DIRS}
            ${Boost_INCLUDE_DIR}
        )

        LINK_DIRECTORIES(
            ${VTK_DIR} #this linker dir is necessary, since VTK_INSTALL_EXPORT_NAME is set to 1 above, such that VTK does not directly include all its dependencies.
            ${VTK_LIBRARY_DIRS}
        )

        if (NOT QT5_FOUND)
            INCLUDE(${QT_USE_FILE})
        endif (NOT QT5_FOUND)

        set(plugin_HEADERS
            ${CMAKE_CURRENT_SOURCE_DIR}/pclTools.h
            ${CMAKE_CURRENT_SOURCE_DIR}/random_sample_corrected.h
            ${CMAKE_CURRENT_SOURCE_DIR}/pcl/filters/cylinder_clipper3D.h
            ${CMAKE_CURRENT_SOURCE_DIR}/xyzFormat.h
            ${CMAKE_CURRENT_SOURCE_DIR}/pluginVersion.h
        )

        if (QT5_FOUND)
            QT5_WRAP_CPP(plugin_HEADERS_MOC ${plugin_HEADERS})
        else (QT5_FOUND)
            QT4_WRAP_CPP_ITOM(plugin_HEADERS_MOC ${plugin_HEADERS})
        endif (QT5_FOUND)

        set(plugin_ui
        )

        if (QT5_FOUND)
            QT5_WRAP_UI(plugin_ui_MOC ${plugin_ui})
        else (QT5_FOUND)
            QT4_WRAP_UI_ITOM(plugin_ui_MOC ${plugin_ui})
        endif (QT5_FOUND)

        set(plugin_SOURCES 
            ${CMAKE_CURRENT_SOURCE_DIR}/pclTools.cpp
            ${CMAKE_CURRENT_SOURCE_DIR}/random_sample_corrected.cpp
            ${CMAKE_CURRENT_SOURCE_DIR}/pcl/filters/impl/cylinder_clipper3D.hpp
        )

        #Add version information to the plugIn-dll unter MSVC
        if(MSVC)
            list(APPEND plugin_SOURCES ${ITOM_SDK_INCLUDE_DIR}/../pluginLibraryVersion.rc)
        endif(MSVC)

        file (GLOB EXISTING_TRANSLATION_FILES "translation/*.ts")
        #handle translations END STEP 1

        ADD_LIBRARY(${target_name} ${LIBRARY_TYPE} ${plugin_SOURCES} ${plugin_HEADERS} ${plugin_HEADERS_MOC} ${plugin_ui_MOC} ${plugin_rcc_MOC} ${EXISTING_TRANSLATION_FILES})
        
        if (QT5_FOUND)
            TARGET_LINK_LIBRARIES(${target_name} ${OpenCV_LIBS} ${VISUALLEAKDETECTOR_LIBRARIES} ${ITOM_SDK_LIBRARIES} ${PCL_LIBRARIES})
            
            qt5_use_modules(${target_name} Widgets UiTools)
        else (QT5_FOUND)
            TARGET_LINK_LIBRARIES(${target_name} ${QT_LIBRARIES} ${OpenCV_LIBS} ${VISUALLEAKDETECTOR_LIBRARIES} ${ITOM_SDK_LIBRARIES} ${PCL_LIBRARIES})
        endif (QT5_FOUND)

        set (FILES_TO_TRANSLATE ${plugin_SOURCES} ${plugin_HEADERS} ${plugin_UI})
    PLUGIN_TRANSLATION(QM_FILES ${target_name} ${UPDATE_TRANSLATIONS} "${EXISTING_TRANSLATION_FILES}" ITOM_LANGUAGES "${FILES_TO_TRANSLATE}")

        PLUGIN_DOCUMENTATION(${target_name} pclTools)

        # COPY SECTION
        set(COPY_SOURCES "")
        set(COPY_DESTINATIONS "")
        ADD_PLUGINLIBRARY_TO_COPY_LIST(${target_name} COPY_SOURCES COPY_DESTINATIONS)
        ADD_QM_FILES_TO_COPY_LIST(${target_name} QM_FILES COPY_SOURCES COPY_DESTINATIONS)
        POST_BUILD_COPY_FILES(${target_name} COPY_SOURCES COPY_DESTINATIONS)
        
    ELSE(VTK_FOUND)
        message(WARNING "${target_name} will not be build since package VTK could not be found. Indicate VTK_DIR.")
    ENDIF(VTK_FOUND)

ELSE(ITOM_SDK_PCL_SUPPORT)
    message(WARNING "${target_name} will not be build since itom has been compiled without PCL support")
ENDIF(ITOM_SDK_PCL_SUPPORT)