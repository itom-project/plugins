cmake_minimum_required(VERSION 3.12...3.24)

set(target_name PCOCamera)

message(STATUS "Project ${target_name} (${CMAKE_CURRENT_BINARY_DIR})")

option(BUILD_TARGET64 "Build for 64 bit target if set to ON or 32 bit if set to OFF." ON)
set(PCO_Camera_BUILD_API_VERSION "1.24 or newer" CACHE STRING "Expected API version")
set_property(CACHE PCO_Camera_BUILD_API_VERSION PROPERTY STRINGS "1.24 or newer" "<1.24" "really old")

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

set(PCO_SDK_DIR "" CACHE PATH "path of the pco.sdk. Must contain subfolders like bin,bin64,include,lib,lib64")

if(NOT PCO_SDK_DIR)
	set(PCO_SDK_DIR $ENV{PCO_SDK_ROOT})
endif(NOT PCO_SDK_DIR)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR} ${ITOM_SDK_DIR}/cmake)

find_package(ITOM_SDK COMPONENTS dataobject itomCommonLib itomCommonQtLib itomWidgets REQUIRED)
find_package(OpenCV COMPONENTS core REQUIRED) #if you require openCV indicate all components that are required (e.g. core, imgproc...)

include(ItomBuildMacros)
itom_init_cmake_policy(3.12)
itom_init_plugin_library(${target_name}) #Start the project, init compiler settings and set default configurations for plugins
itom_find_package_qt(ON Core LinguistTools Widgets)

if(BUILD_TARGET64)
    find_library(PCO_SDK_SC2_CAM_LIB SC2_Cam PATHS ${PCO_SDK_DIR} PATH_SUFFIXES lib64)
    find_file(PCO_SDK_LIBRARY SC2_Cam.dll PATHS ${PCO_SDK_DIR} PATH_SUFFIXES bin64)
else(BUILD_TARGET64)
    find_library(PCO_SDK_SC2_CAM_LIB SC2_Cam PATHS ${PCO_SDK_DIR} PATH_SUFFIXES lib)
    find_file(PCO_SDK_LIBRARY SC2_Cam.dll PATHS ${PCO_SDK_DIR} PATH_SUFFIXES bin)
endif(BUILD_TARGET64)

find_path(PCO_SDK_INCLUDE_DIR SC2_CamExport.h PATHS ${PCO_SDK_DIR} PATH_SUFFIXES include)

if(PCO_SDK_SC2_CAM_LIB AND PCO_SDK_LIBRARY AND PCO_SDK_INCLUDE_DIR)

    if(PCO_Camera_BUILD_API_VERSION VERSION_EQUAL "1.24 or newer")
        add_definitions(-DUSE_API_1_24)
    else(PCO_Camera_BUILD_API_VERSION VERSION_EQUAL "really old")
        add_definitions(-DPCO_SDK_OLD)
    endif(PCO_Camera_BUILD_API_VERSION VERSION_EQUAL "1.24 or newer")

    include_directories(
        ${CMAKE_CURRENT_BINARY_DIR} #build directory of this plugin (recommended)
        ${CMAKE_CURRENT_SOURCE_DIR} #source directory of this plugin (recommended)
        ${ITOM_SDK_INCLUDE_DIRS}     #include directory of the itom SDK (recommended)
        ${ITOM_SDK_INCLUDE_DIR}/itomWidgets #include this subfolder if you use any widgets from itomWidgets
        #add further include directories here
        ${PCO_SDK_INCLUDE_DIR}
    )

    link_directories(
        #add all linker directories
    )

    set(PLUGIN_HEADERS
        ${CMAKE_CURRENT_SOURCE_DIR}/dialogPCOCamera.h
        ${CMAKE_CURRENT_SOURCE_DIR}/dockWidgetPCOCamera.h
        ${CMAKE_CURRENT_SOURCE_DIR}/PCOCamera.h
        ${CMAKE_CURRENT_SOURCE_DIR}/pluginVersion.h
        ${CMAKE_CURRENT_BINARY_DIR}/gitVersion.h
        #add further header files (absolute pathes e.g. using CMAKE_CURRENT_SOURCE_DIR)
    )

    set(PLUGIN_SOURCES
        ${CMAKE_CURRENT_SOURCE_DIR}/dialogPCOCamera.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/dockWidgetPCOCamera.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/PCOCamera.cpp
        #add further source files here
    )

    #Append rc file to the source files for adding information about the plugin
    # to the properties of the DLL under Visual Studio.
    if(MSVC)
        list(APPEND PLUGIN_SOURCES ${ITOM_SDK_INCLUDE_DIR}/../pluginLibraryVersion.rc)
    endif()

    set(PLUGIN_UI
        ${CMAKE_CURRENT_SOURCE_DIR}/dialogPCOCamera.ui
        ${CMAKE_CURRENT_SOURCE_DIR}/dockWidgetPCOCamera.ui
    )

    set(PLUGIN_RCC
        #add absolute pathes to any *.qrc resource files here
    )

    add_library(${target_name} SHARED ${PLUGIN_SOURCES} ${PLUGIN_HEADERS} ${PLUGIN_UI} ${PLUGIN_RCC})

    # Qt: enable all automoc, autouic and autorcc.
    set_target_properties(${target_name} PROPERTIES AUTOMOC ON AUTORCC ON AUTOUIC ON)

    target_link_libraries(${target_name} ${OpenCV_LIBS} ${ITOM_SDK_LIBRARIES} ${QT5_LIBRARIES} ${VISUALLEAKDETECTOR_LIBRARIES} ${PCO_SDK_SC2_CAM_LIB} delayimp)

    set_target_properties(${target_name} PROPERTIES LINK_FLAGS "/DELAYLOAD:SC2_Cam.dll")

    set(FILES_TO_TRANSLATE ${PLUGIN_SOURCES} ${PLUGIN_HEADERS} ${PLUGIN_UI})
    itom_library_translation(QM_FILES TARGET ${target_name} FILES_TO_TRANSLATE ${FILES_TO_TRANSLATE})

    itom_configure_plugin_documentation(${target_name} pcoCamera)

    set(COPY_SOURCES "")
    set(COPY_DESTINATIONS "")

    itom_add_pluginlibrary_to_copy_list(${target_name} COPY_SOURCES COPY_DESTINATIONS)
    itom_add_plugin_qm_files_to_copy_list(${target_name} QM_FILES COPY_SOURCES COPY_DESTINATIONS)

    itom_post_build_copy_files(${target_name} COPY_SOURCES COPY_DESTINATIONS)
    itom_post_build_copy_files_to_lib_folder(${target_name} PCO_SDK_LIBRARY)

else(PCO_SDK_SC2_CAM_LIB AND PCO_SDK_LIBRARY AND PCO_SDK_INCLUDE_DIR)
    message(SEND_ERROR "PCO_SDK_DIR directory is not given or no valid pco.sdk directory. ${target_name} will not be build.")
endif(PCO_SDK_SC2_CAM_LIB AND PCO_SDK_LIBRARY AND PCO_SDK_INCLUDE_DIR)
