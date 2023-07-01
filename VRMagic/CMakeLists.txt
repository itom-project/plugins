cmake_minimum_required(VERSION 3.12...3.24)

set(target_name VRMagic)

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

find_package(ITOM_SDK COMPONENTS dataobject itomCommonLib itomCommonQtLib itomWidgets REQUIRED)

include(ItomBuildMacros)
itom_init_cmake_policy(3.12)
itom_init_plugin_library(${target_name}) #Start the project, init compiler settings and set default configurations for plugins
itom_find_package_qt(ON Core Widgets Xml LinguistTools)

if(NOT VRMAGIC_INCLUDE_DIR)
	if(WIN32)
		find_path( VRMAGIC_INCLUDE_DIR vrmusbcam2.h PATHS "$ENV{CommonProgramFiles}\\VRmagic\\VRmUsbCamSDK" "$ENV{VRMAGIC_ROOT}" PATH_SUFFIXES include DOC "VRmagic include directory (must contain the file vrmusbcam2.h). Usually in %CommonProgramFiles%/VRmagic/VRmUsbCamSDK/include.")
	else()
		find_path( VRMAGIC_INCLUDE_DIR vrmusbcam2.h PATH_SUFFIXES include DOC "VRmagic include directory (must contain the file vrmusbcam2.h).")
	endif()
endif(NOT VRMAGIC_INCLUDE_DIR)

if(VRMAGIC_INCLUDE_DIR)
    get_filename_component(VRMAGIC_SDK_DIR ${VRMAGIC_INCLUDE_DIR} DIRECTORY)


    include_directories(
        ${CMAKE_CURRENT_BINARY_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${ITOM_SDK_INCLUDE_DIRS}
        ${ITOM_SDK_INCLUDE_DIR}/itomWidgets
		${VRMAGIC_INCLUDE_DIR}
    )

    if(BUILD_TARGET64)
        link_directories(
            ${VRMAGIC_SDK_DIR}/lib/x64
        )
    else(BUILD_TARGET64)
        link_directories(
            ${VRMAGIC_SDK_DIR}/lib/x86
        )
    endif(BUILD_TARGET64)

    set(PLUGIN_HEADERS
        ${CMAKE_CURRENT_SOURCE_DIR}/dialogVRMagic.h
        ${CMAKE_CURRENT_SOURCE_DIR}/dockWidgetVRMagic.h
        ${CMAKE_CURRENT_SOURCE_DIR}/VRMagic.h
        ${CMAKE_CURRENT_SOURCE_DIR}/pluginVersion.h
        ${CMAKE_CURRENT_BINARY_DIR}/gitVersion.h
    )

    set(PLUGIN_UI
        ${CMAKE_CURRENT_SOURCE_DIR}/dialogVRMagic.ui
        ${CMAKE_CURRENT_SOURCE_DIR}/dockWidgetVRMagic.ui
    )

    set(PLUGIN_SOURCES
        ${CMAKE_CURRENT_SOURCE_DIR}/dialogVRMagic.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/dockWidgetVRMagic.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/VRMagic.cpp
    )

    set(PLUGIN_RCC
    )

    if(MSVC)
        list(APPEND PLUGIN_SOURCES ${ITOM_SDK_INCLUDE_DIR}/../pluginLibraryVersion.rc)
    endif()

    add_library(${target_name} SHARED ${PLUGIN_SOURCES} ${PLUGIN_HEADERS} ${PLUGIN_UI} ${PLUGIN_RCC})

    # Qt: enable all automoc, autouic and autorcc.
    set_target_properties(${target_name} PROPERTIES AUTOMOC ON AUTORCC ON AUTOUIC ON)

    target_link_libraries(${target_name} ${ITOM_SDK_LIBRARIES} vrmusbcam2 ${QT5_LIBRARIES} ${VISUALLEAKDETECTOR_LIBRARIES})

    #translation
    set(FILES_TO_TRANSLATE ${PLUGIN_SOURCES} ${PLUGIN_HEADERS} ${PLUGIN_UI})
    itom_library_translation(QM_FILES TARGET ${target_name} FILES_TO_TRANSLATE ${FILES_TO_TRANSLATE})

    #documentation
    itom_configure_plugin_documentation(${target_name} VRMagic)

    # COPY SECTION
    set(COPY_SOURCES "")
    set(COPY_DESTINATIONS "")
    itom_add_pluginlibrary_to_copy_list(${target_name} COPY_SOURCES COPY_DESTINATIONS)
    itom_add_plugin_qm_files_to_copy_list(${target_name} QM_FILES COPY_SOURCES COPY_DESTINATIONS)
    itom_post_build_copy_files(${target_name} COPY_SOURCES COPY_DESTINATIONS)

    if(BUILD_TARGET64)
        file(GLOB dlls "${VRMAGIC_SDK_DIR}/lib/x64/*.dll")
    else(BUILD_TARGET64)
        file(GLOB dlls "${VRMAGIC_SDK_DIR}/lib/x86/*.dll")
    endif(BUILD_TARGET64)
    itom_post_build_copy_files_to_lib_folder(${target_name} dlls)

else(VRMAGIC_INCLUDE_DIR)
    message(SEND_ERROR "${target_name}-SDK directory could not be found. ${target_name} will not be build")
endif(VRMAGIC_INCLUDE_DIR)
