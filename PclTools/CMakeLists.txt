set(target_name PclTools)

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


#set(VTK_INSTALL_EXPORT_NAME 1) #avoid that vtk directly links to an absolute Qt-library, that is located somewhere on their computer configuration.
find_package(ITOM_SDK REQUIRED)
FIND_PACKAGE_QT(ON Widgets LinguistTools OpenGL)

if(ITOM_SDK_PCL_SUPPORT)
    find_package(ITOM_SDK COMPONENTS dataobject pointcloud itomCommonLib itomCommonQtLib REQUIRED)
    find_package(OpenCV COMPONENTS core REQUIRED)
    find_package(VTK QUIET)
    
    if(VTK_FOUND)
        
        find_package(PCL 1.5.1 REQUIRED COMPONENTS common io kdtree search features segmentation filters surface recognition)
        find_package(VisualLeakDetector QUIET)
        
        find_file(PCL_HAS_SURFACE_NURBS "nurbs_data.h" PATHS ${PCL_INCLUDE_DIRS} PATH_SUFFIXES "pcl/surface/on_nurbs")
        if(PCL_HAS_SURFACE_NURBS)
            add_definitions(-DPCLHASSURFACENURBS -D_PCLHASSURFACENURBS)
        else()
            message(STATUS "PCL has been build without the surface_nurbs module, therefore not all filters can be build")
        endif()

        include("${ITOM_SDK_DIR}/ItomBuildMacros.cmake")

        
    
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

        message(STATUS "VTK_INC: " ${VTK_INCLUDE_DIR})
        include_directories(
            ${CMAKE_CURRENT_BINARY_DIR}
            ${CMAKE_CURRENT_SOURCE_DIR}
            #            #${OpenCV_DIR}/include
            ${PCL_INCLUDE_DIRS}
            #${BOOST_INCLUDE_DIRS}
            ${ITOM_SDK_INCLUDE_DIRS}
            ${VISUALLEAKDETECTOR_INCLUDE_DIR}
            #${EIGEN_INCLUDE_DIR}
            #${FLANN_INCLUDE_DIRS}
            #${Boost_INCLUDE_DIR}
        )

        link_directories(
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
        endif()
        
        if(MSVC)
            set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /bigobj")
        endif()

        if(QT5_FOUND)
            #if automoc if OFF, you also need to call QT5_WRAP_CPP here
            QT5_WRAP_UI(plugin_UI_MOC ${plugin_UI})
            QT5_ADD_RESOURCES(plugin_RCC_MOC ${plugin_RCC})
        else()
            message(SEND_ERROR "Currently only Qt5 is supported")
        endif()

        file(GLOB EXISTING_TRANSLATION_FILES "translation/*.ts")
        #handle translations END STEP 1

        set(VTK_LIBS "")
        if("${VTK_MAJOR_VERSION}" EQUAL 5)
        foreach(__comps ${VTK_LIBRARIES})
            set(VTK_LIBS ${VTK_LIBS} debug ${__comps}-gd optimized ${__comps})
        endforeach(__comps)
        endif()

        add_library(${target_name} ${plugin_SOURCES} ${plugin_HEADERS} ${plugin_HEADERS_MOC} ${plugin_UI_MOC} ${plugin_RCC_MOC} ${EXISTING_TRANSLATION_FILES})
        
        target_link_libraries(${target_name} ${OpenCV_LIBS} ${QT5_LIBRARIES} ${VISUALLEAKDETECTOR_LIBRARIES} ${ITOM_SDK_LIBRARIES} ${PCL_LIBRARIES} ${VTK_LIBS})

        #translation
        set(FILES_TO_TRANSLATE ${plugin_SOURCES} ${plugin_HEADERS} ${plugin_UI})
        PLUGIN_TRANSLATION(QM_FILES ${target_name} ${UPDATE_TRANSLATIONS} "${EXISTING_TRANSLATION_FILES}" ITOM_LANGUAGES "${FILES_TO_TRANSLATE}")

        PLUGIN_DOCUMENTATION(${target_name} pclTools)

        # COPY SECTION
        set(COPY_SOURCES "")
        set(COPY_DESTINATIONS "")
        ADD_PLUGINLIBRARY_TO_COPY_LIST(${target_name} COPY_SOURCES COPY_DESTINATIONS)
        ADD_QM_FILES_TO_COPY_LIST(${target_name} QM_FILES COPY_SOURCES COPY_DESTINATIONS)
        POST_BUILD_COPY_FILES(${target_name} COPY_SOURCES COPY_DESTINATIONS)
        
    else(VTK_FOUND)
        message(WARNING "${target_name} will not be build since package VTK could not be found. Indicate VTK_DIR.")
    endif(VTK_FOUND)

else(ITOM_SDK_PCL_SUPPORT)
    message(WARNING "${target_name} will not be build since itom has been compiled without PCL support")
endif(ITOM_SDK_PCL_SUPPORT)