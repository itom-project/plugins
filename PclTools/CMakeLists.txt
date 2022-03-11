set(target_name PclTools)


cmake_minimum_required(VERSION 3.1...3.15)

option(BUILD_TARGET64 "Build for 64 bit target if set to ON or 32 bit if set to OFF." ON)
set(ITOM_SDK_DIR NOTFOUND CACHE PATH "path of SDK subfolder of itom root (build) directory")

#this is to automatically detect the SDK subfolder of the itom build directory.
find_path(ITOM_SDK_DIR "cmake/itom_sdk.cmake"
    HINTS "C:/itom/build/itom/SDK"
          "${CMAKE_CURRENT_BINARY_DIR}/../../itom/SDK"
    DOC "path of SDK subfolder of itom root (build) directory")

if(NOT ITOM_SDK_DIR)
    message(SEND_ERROR "ITOM_SDK_DIR is invalid. Provide itom SDK directory path first")
endif()

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR} ${ITOM_SDK_DIR}/cmake)

include(ItomBuildMacros)
itom_init_cmake_policy(3.12)
itom_init_plugin_library(${target_name}) #Start the project, init compiler settings and set default configurations for plugins


#set(VTK_INSTALL_EXPORT_NAME 1) #avoid that vtk directly links to an absolute Qt-library, that is located somewhere on their computer configuration.
find_package(ITOM_SDK REQUIRED)
itom_find_package_qt(ON Widgets LinguistTools OpenGL)

if(ITOM_SDK_PCL_SUPPORT)
    find_package(ITOM_SDK COMPONENTS dataobject pointcloud itomCommonLib itomCommonQtLib REQUIRED)
    find_package(OpenCV COMPONENTS core REQUIRED)

    if (NOT VTK_FOUND)
      find_package(VTK  COMPONENTS vtkCommonCore QUIET)
      if (NOT VTK_FOUND)
        message("Skipping ${PROJECT_NAME}: ${VTK_NOT_FOUND_MESSAGE}")
        return ()
      endif()
    endif()

    if(VTK_FOUND)
        
        find_package(PCL 1.5.1 REQUIRED COMPONENTS common io kdtree search features segmentation filters surface recognition)
            
        find_file(PCL_HAS_SURFACE_NURBS "nurbs_data.h" PATHS ${PCL_INCLUDE_DIRS} PATH_SUFFIXES "pcl/surface/on_nurbs")
        if(PCL_HAS_SURFACE_NURBS)
            add_definitions(-DPCLHASSURFACENURBS -D_PCLHASSURFACENURBS)
        else()
            message(STATUS "PCL has been build without the surface_nurbs module, therefore not all filters can be build")
        endif()

        message(STATUS "VTK_INC: " ${VTK_INCLUDE_DIR})
        message(STATUS "Found VTK version: ${VTK_VERSION}")
        if (VTK_VERSION VERSION_LESS "8.90.0")
          # old system
            find_package(VTK QUIET)
            set(VTK_LIBS "")
            if("${VTK_MAJOR_VERSION}" EQUAL 5)
            foreach(__comps ${VTK_LIBRARIES})
                set(VTK_LIBS ${VTK_LIBS} debug ${__comps}-gd optimized ${__comps})
            endforeach(__comps)
            endif()
        else()
          # new system
          find_package(VTK COMPONENTS
              CommonColor
              CommonCore
              CommonDataModel
              FiltersSources
              IOImage
              RenderingCore
              RenderingOpenGL2
              IOXML
              IOXMLParser
              GUISupportQt
              GUISupportQtSQL          
              RenderingQt
              ViewsQt
          )
        endif()
        message(STATUS "VTK Libraries: ${VTK_LIBRARIES}")

        include_directories(
            ${CMAKE_CURRENT_BINARY_DIR}
            ${CMAKE_CURRENT_SOURCE_DIR}
            ${PCL_INCLUDE_DIRS}
            ${ITOM_SDK_INCLUDE_DIRS}
        )

        link_directories(
            ${VTK_DIR} #this linker dir is necessary, since VTK_INSTALL_EXPORT_NAME is set to 1 above, such that VTK does not directly include all its dependencies.
            ${VTK_LIBRARY_DIRS}
            ${BOOST_LIBRARY_DIRS}
        )

        set(PLUGIN_HEADERS
            ${CMAKE_CURRENT_SOURCE_DIR}/pclTools.h
            ${CMAKE_CURRENT_SOURCE_DIR}/random_sample_corrected.h
            ${CMAKE_CURRENT_SOURCE_DIR}/pcl/filters/cylinder_clipper3D.h
            ${CMAKE_CURRENT_SOURCE_DIR}/xyzFormat.h
            ${CMAKE_CURRENT_SOURCE_DIR}/pluginVersion.h
            ${CMAKE_CURRENT_BINARY_DIR}/gitVersion.h
        )

        set(PLUGIN_UI
        )
        
        set(PLUGIN_RCC
        )

        set(PLUGIN_SOURCES 
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
            list(APPEND PLUGIN_SOURCES ${ITOM_SDK_INCLUDE_DIR}/../pluginLibraryVersion.rc)
        endif()
        
        if(MSVC)
            set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /bigobj")
        endif()
        

        add_library(${target_name} SHARED ${PLUGIN_SOURCES} ${PLUGIN_HEADERS} ${PLUGIN_UI} ${PLUGIN_RCC})
        
        # Qt: enable all automoc, autouic and autorcc.
        set_target_properties(${target_name} PROPERTIES AUTOMOC ON AUTORCC ON AUTOUIC ON)

        target_link_libraries(${target_name} ${OpenCV_LIBS} ${QT5_LIBRARIES} ${VISUALLEAKDETECTOR_LIBRARIES} ${ITOM_SDK_LIBRARIES} ${PCL_LIBRARIES} ${VTK_LIBRARIES})

        #translation
        set(FILES_TO_TRANSLATE ${PLUGIN_SOURCES} ${PLUGIN_HEADERS} ${PLUGIN_UI})
        itom_library_translation(QM_FILES TARGET ${target_name} FILES_TO_TRANSLATE ${FILES_TO_TRANSLATE})

        itom_configure_plugin_documentation(${target_name} pclTools)

        # COPY SECTION
        set(COPY_SOURCES "")
        set(COPY_DESTINATIONS "")
        itom_add_pluginlibrary_to_copy_list(${target_name} COPY_SOURCES COPY_DESTINATIONS)
        itom_add_plugin_qm_files_to_copy_list(${target_name} QM_FILES COPY_SOURCES COPY_DESTINATIONS)
        itom_post_build_copy_files(${target_name} COPY_SOURCES COPY_DESTINATIONS)
        
    else(VTK_FOUND)
        message(WARNING "${target_name} will not be build since package VTK could not be found. Indicate VTK_DIR.")
    endif(VTK_FOUND)

else(ITOM_SDK_PCL_SUPPORT)
    message(WARNING "${target_name} will not be build since itom has been compiled without PCL support")
endif(ITOM_SDK_PCL_SUPPORT)