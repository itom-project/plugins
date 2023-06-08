cmake_minimum_required(VERSION 3.12...3.24)

set(target_name ThorlabsDCxCam)

#################################################################
# Input elements for CMake GUI (Checkboxes, Pathes, Strings...)
#################################################################
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

set(THORLABS_DCxCAMERASUPPORT_DEVELOP_DIRECTORY "" CACHE PATH "path to develop directory of Thorlabs / Scientific Imaging / DCx Camera Support")

if(NOT THORLABS_DCxCAMERASUPPORT_DEVELOP_DIRECTORY)
	set(THORLABS_DCxCAMERASUPPORT_DEVELOP_DIRECTORY $ENV{THORLABS_DCXCAM_ROOT})
endif(NOT THORLABS_DCxCAMERASUPPORT_DEVELOP_DIRECTORY)

if(THORLABS_DCxCAMERASUPPORT_DEVELOP_DIRECTORY)
    if(WIN32)
        find_path( THORLABS_DCxCAMERA_INCLUDE_DIR uc480.h PATHS ${THORLABS_DCxCAMERASUPPORT_DEVELOP_DIRECTORY} PATH_SUFFIXES Include)

        if(BUILD_TARGET64)
            find_file( THORLABS_DCxCAMERA_LIBRARY uc480_64.lib PATHS ${THORLABS_DCxCAMERASUPPORT_DEVELOP_DIRECTORY} PATH_SUFFIXES Lib)
        else(BUILD_TARGET64)
            find_file( THORLABS_DCxCAMERA_LIBRARY uc480.lib PATHS ${THORLABS_DCxCAMERASUPPORT_DEVELOP_DIRECTORY} PATH_SUFFIXES Lib)
        endif(BUILD_TARGET64)
    endif(WIN32)
endif(THORLABS_DCxCAMERASUPPORT_DEVELOP_DIRECTORY)

if(THORLABS_DCxCAMERA_INCLUDE_DIR)

    set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR} ${ITOM_SDK_DIR}/cmake)

    find_package(ITOM_SDK COMPONENTS dataobject itomCommonLib itomCommonQtLib itomWidgets REQUIRED)
    find_package(OpenCV COMPONENTS core REQUIRED) #if you require openCV indicate all components that are required (e.g. core, imgproc...)

    include(ItomBuildMacros)
    itom_init_cmake_policy(3.12)
    itom_init_plugin_library(${target_name}) #Start the project, init compiler settings and set default configurations for plugins
    itom_find_package_qt(ON Core LinguistTools Widgets)

    include_directories(
        ${CMAKE_CURRENT_BINARY_DIR} #build directory of this plugin (recommended)
        ${CMAKE_CURRENT_SOURCE_DIR} #source directory of this plugin (recommended)
        ${ITOM_SDK_INCLUDE_DIRS}     #include directory of the itom SDK (recommended)
        #add further include directories here
        ${Tools_SOURCE_DIR}
        ${THORLABS_DCxCAMERA_INCLUDE_DIR}
    )

    link_directories(
        #add all linker directories
    )

    set(PLUGIN_HEADERS
        ${CMAKE_CURRENT_SOURCE_DIR}/dialogThorlabsDCxCam.h
        ${CMAKE_CURRENT_SOURCE_DIR}/dockWidgetThorlabsDCxCam.h
        ${CMAKE_CURRENT_SOURCE_DIR}/thorlabsDCxCam.h
        ${CMAKE_CURRENT_SOURCE_DIR}/thorlabsDCxCamInterface.h
        ${CMAKE_CURRENT_SOURCE_DIR}/pluginVersion.h
        ${CMAKE_CURRENT_BINARY_DIR}/gitVersion.h
        #add further header files (absolute pathes e.g. using CMAKE_CURRENT_SOURCE_DIR)
    )

    set(PLUGIN_SOURCES
        ${CMAKE_CURRENT_SOURCE_DIR}/dialogThorlabsDCxCam.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/dockWidgetThorlabsDCxCam.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/thorlabsDCxCam.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/thorlabsDCxCamInterface.cpp
        #add further source files here
    )

    if(MSVC)
        list(APPEND PLUGIN_SOURCES ${ITOM_SDK_INCLUDE_DIR}/../pluginLibraryVersion.rc)
    endif()

    set(PLUGIN_UI
        ${CMAKE_CURRENT_SOURCE_DIR}/dialogThorlabsDCxCam.ui
        ${CMAKE_CURRENT_SOURCE_DIR}/dockWidgetThorlabsDCxCam.ui
    )

    set(PLUGIN_RCC
        #add absolute pathes to any *.qrc resource files here
    )

    add_library(${target_name} SHARED ${PLUGIN_SOURCES} ${PLUGIN_HEADERS} ${PLUGIN_UI} ${PLUGIN_RCC})


    set_target_properties(${target_name} PROPERTIES AUTOMOC ON AUTORCC ON AUTOUIC ON)

    target_link_libraries(${target_name} ${OpenCV_LIBS} ${ITOM_SDK_LIBRARIES} ${QT5_LIBRARIES} ${VISUALLEAKDETECTOR_LIBRARIES} ${THORLABS_DCxCAMERA_LIBRARY} delayimp)

    set(FILES_TO_TRANSLATE ${PLUGIN_SOURCES} ${PLUGIN_HEADERS} ${PLUGIN_UI})
    itom_library_translation(QM_FILES TARGET ${target_name} FILES_TO_TRANSLATE ${FILES_TO_TRANSLATE})

    itom_configure_plugin_documentation(${target_name} ThorlabsDCxCam)

    set(COPY_SOURCES "")
    set(COPY_DESTINATIONS "")

    itom_add_pluginlibrary_to_copy_list(${target_name} COPY_SOURCES COPY_DESTINATIONS)
    itom_add_plugin_qm_files_to_copy_list(${target_name} QM_FILES COPY_SOURCES COPY_DESTINATIONS)

    itom_post_build_copy_files(${target_name} COPY_SOURCES COPY_DESTINATIONS)

else(THORLABS_DCxCAMERA_INCLUDE_DIR)
    message(SEND_ERROR "uc480.h for plugin ${target_name} could not be found. ${target_name} will not be build. Please properly indicate THORLABS_DCxCAMERASUPPORT_DEVELOP_DIRECTORY.")
endif(THORLABS_DCxCAMERA_INCLUDE_DIR)
