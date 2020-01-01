SET (target_name PclTools)

project(${target_name})

message(STATUS "\n--------------- PLUGIN ${target_name} ---------------")

cmake_minimum_required(VERSION 2.8)

OPTION(BUILD_UNICODE "Build with unicode charset if set to ON, else multibyte charset." ON)
OPTION(BUILD_SHARED_LIBS "Build shared library." ON)
OPTION(BUILD_TARGET64 "Build for 64 bit target if set to ON or 32 bit if set to OFF." ON)
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

#SET(VTK_INSTALL_EXPORT_NAME 1) #avoid that vtk directly links to an absolute Qt-library, that is located somewhere on their computer configuration.
find_package(ITOM_SDK REQUIRED)
include("${ITOM_SDK_DIR}/ItomBuildMacros.cmake")
FIND_PACKAGE_QT(ON Widgets LinguistTools OpenGL)

IF(ITOM_SDK_PCL_SUPPORT)
    find_package(ITOM_SDK COMPONENTS dataobject pointcloud itomCommonLib itomCommonQtLib REQUIRED)
    find_package(OpenCV COMPONENTS core REQUIRED)
    find_package(VTK QUIET)
    
    IF(VTK_FOUND)
        
        find_package(PCL 1.5.1 REQUIRED COMPONENTS common io kdtree search features segmentation filters surface recognition)
        find_package(VisualLeakDetector QUIET)
        
        find_file(PCL_HAS_SURFACE_NURBS "nurbs_data.h" PATHS ${PCL_INCLUDE_DIRS} PATH_SUFFIXES "pcl/surface/on_nurbs")
        IF (PCL_HAS_SURFACE_NURBS)
            ADD_DEFINITIONS(-DPCLHASSURFACENURBS -D_PCLHASSURFACENURBS)
        ELSE()
            MESSAGE(STATUS "PCL has been build without the surface_nurbs module, therefore not all filters can be build")
        ENDIF()

        include("${ITOM_SDK_DIR}/ItomBuildMacros.cmake")

        ADD_DEFINITIONS(${QT_DEFINITIONS})
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
        
        # default build types are None, Debug, Release, RelWithDebInfo and MinRelSize
        IF (DEFINED CMAKE_BUILD_TYPE)
            SET(CMAKE_BUILD_TYPE ${CMAKE_BUILD_TYPE} CACHE STRING "Choose the type of build, options are: None(CMAKE_CXX_FLAGS or CMAKE_C_FLAGS used) Debug Release RelWithDebInfo MinSizeRel.")
        ELSE(CMAKE_BUILD_TYPE)
            SET (CMAKE_BUILD_TYPE Debug CACHE STRING "Choose the type of build, options are: None(CMAKE_CXX_FLAGS or CMAKE_C_FLAGS used) Debug Release RelWithDebInfo MinSizeRel.")
        ENDIF (DEFINED CMAKE_BUILD_TYPE)

        message(STATUS "VTK_INC: " ${VTK_INCLUDE_DIR})
        INCLUDE_DIRECTORIES(
            ${CMAKE_CURRENT_BINARY_DIR}
            ${CMAKE_CURRENT_SOURCE_DIR}
            ${QT_QTCORE_INCLUDE_DIR}
            ${OpenCV_DIR}/include
            ${PCL_INCLUDE_DIRS}
            ${BOOST_INCLUDE_DIRS}
            ${ITOM_SDK_INCLUDE_DIRS}
            ${VISUALLEAKDETECTOR_INCLUDE_DIR}
            ${EIGEN_INCLUDE_DIR}
            ${FLANN_INCLUDE_DIRS}
            ${Boost_INCLUDE_DIR}
        )

        LINK_DIRECTORIES(
            ${VTK_DIR} #this linker dir is necessary, since VTK_INSTALL_EXPORT_NAME is set to 1 above, such that VTK does not directly include all its dependencies.
            ${VTK_LIBRARY_DIRS}
            ${BOOST_LIBRARY_DIRS}
        )

        set(plugin_HEADERS
            ${CMAKE_CURRENT_SOURCE_DIR}/pclTools.h
            ${CMAKE_CURRENT_SOURCE_DIR}/random_sample_corrected.h
            ${CMAKE_CURRENT_SOURCE_DIR}/pcl/filters/cylinder_clipper3D.h
            ${CMAKE_CURRENT_SOURCE_DIR}/xyzFormat.h
            ${CMAKE_CURRENT_SOURCE_DIR}/pluginVersion.h
            ${CMAKE_CURRENT_BINARY_DIR}/gitVersion.h
        )

        set(plugin_UI
        )
        
        set(plugin_RCC
        )

        set(plugin_SOURCES 
            ${CMAKE_CURRENT_SOURCE_DIR}/pclTools.cpp
            ${CMAKE_CURRENT_SOURCE_DIR}/implIncludes.cpp
            ${CMAKE_CURRENT_SOURCE_DIR}/pclModelFit.cpp
            ${CMAKE_CURRENT_SOURCE_DIR}/pclModelFitGeneric.cpp
            ${CMAKE_CURRENT_SOURCE_DIR}/pclModelFitGenericImpl.cpp
            ${CMAKE_CURRENT_SOURCE_DIR}/pclModelFitGenericCircleImpl.cpp
            ${CMAKE_CURRENT_SOURCE_DIR}/pclModelFitGenericConeImpl.cpp
            ${CMAKE_CURRENT_SOURCE_DIR}/pclModelFitGenericCylinderImpl.cpp
            ${CMAKE_CURRENT_SOURCE_DIR}/pclModelFitGenericPlaneImpl.cpp
            ${CMAKE_CURRENT_SOURCE_DIR}/pclModelFitGenericSphereImpl.cpp
            ${CMAKE_CURRENT_SOURCE_DIR}/pclNurbs.cpp
            ${CMAKE_CURRENT_SOURCE_DIR}/pclProjectInliers.cpp
            ${CMAKE_CURRENT_SOURCE_DIR}/random_sample_corrected.cpp
            ${CMAKE_CURRENT_SOURCE_DIR}/pcl/filters/impl/cylinder_clipper3D.hpp
        )

        #Add version information to the plugIn-dll unter MSVC
        if(MSVC)
            list(APPEND plugin_SOURCES ${ITOM_SDK_INCLUDE_DIR}/../pluginLibraryVersion.rc)
        endif(MSVC)
        
        if (MSVC)
            set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /bigobj")
        endif()

        if (QT5_FOUND)
            #if automoc if OFF, you also need to call QT5_WRAP_CPP here
            QT5_WRAP_UI(plugin_UI_MOC ${plugin_UI})
            QT5_ADD_RESOURCES(plugin_RCC_MOC ${plugin_RCC})
        else (QT5_FOUND)
            MESSAGE(SEND_ERROR "Currently only Qt5 is supported")
        endif (QT5_FOUND)

        file (GLOB EXISTING_TRANSLATION_FILES "translation/*.ts")
        #handle translations END STEP 1

        SET(VTK_LIBS "")
        IF("${VTK_MAJOR_VERSION}" EQUAL 5)
        foreach(__comps ${VTK_LIBRARIES})
            set(VTK_LIBS ${VTK_LIBS} debug ${__comps}-gd optimized ${__comps})
        endforeach(__comps)
        ENDIF()

        ADD_LIBRARY(${target_name} ${LIBRARY_TYPE} ${plugin_SOURCES} ${plugin_HEADERS} ${plugin_HEADERS_MOC} ${plugin_UI_MOC} ${plugin_RCC_MOC} ${EXISTING_TRANSLATION_FILES})
        
        TARGET_LINK_LIBRARIES(${target_name} ${QT_LIBRARIES} ${OpenCV_LIBS} ${QT5_LIBRARIES} ${VISUALLEAKDETECTOR_LIBRARIES} ${ITOM_SDK_LIBRARIES} ${PCL_LIBRARIES} ${VTK_LIBS})

        IF (QT5_FOUND AND CMAKE_VERSION VERSION_LESS 3.0.2)
            qt5_use_modules(${target_name} ${QT_COMPONENTS})
        ENDIF (QT5_FOUND AND CMAKE_VERSION VERSION_LESS 3.0.2)

        #translation
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