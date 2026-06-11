cmake_minimum_required(VERSION 3.12...3.29)
set(target_name AndorSDK3)

message(STATUS "Project ${target_name} (${CMAKE_CURRENT_BINARY_DIR})")

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

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR} ${ITOM_SDK_DIR}/cmake)

find_package(ITOM_SDK COMPONENTS dataobject itomCommonLib itomCommonQtLib itomWidgets REQUIRED)
find_package(OpenCV COMPONENTS core REQUIRED) #if you require openCV indicate all components that are required (e.g. core, imgproc...)

include(ItomBuildMacros)
itom_init_cmake_policy(3.12)
itom_init_plugin_library(${target_name}) #Start the project, init compiler settings and set default configurations for plugins
itom_find_package_qt(ON Core LinguistTools Widgets)

if(MSVC)
    # Qt disables the native wchar_t type, do it too to avoid linking issues
    # well that changed in qt5 so go back to built-in type
    if(QT5_FOUND)
        set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Zc:wchar_t" )
    else()
        set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Zc:wchar_t-" )
    endif()
endif()

find_path(ANDOR_SDK3_DIR "atcore.h" HINTS "C:/Program Files/Andor SDK3")
if(BUILD_TARGET64)
    if(MSVC)
        FIND_LIBRARY(ANDOR_SDK3_LIBRARY "atcorem" HINTS ${ANDOR_SDK3_DIR})
    else()
        FIND_LIBRARY(ANDOR_SDK3_LIBRARY "atcore" HINTS ${ANDOR_SDK3_DIR})
    endif()
    find_file(ANDOR_SDK3_BINARY "atcore.dll"  PATHS ${ANDOR_SDK3_DIR} NO_DEFAULT_PATH)
    find_file(ANDOR_SDK3_BINARY2 "atcl_bitflow.dll"  PATHS ${ANDOR_SDK3_DIR} NO_DEFAULT_PATH)
    find_file(ANDOR_SDK3_BINARY3 "atdevregcam.dll"  PATHS ${ANDOR_SDK3_DIR} NO_DEFAULT_PATH)
else(BUILD_TARGET64)
    if(MSVC)
        FIND_LIBRARY(ANDOR_SDK3_LIBRARY "atcorem" HINTS ${ANDOR_SDK3_DIR}/win32)
    else()
        FIND_LIBRARY(ANDOR_SDK3_LIBRARY "atcore" HINTS ${ANDOR_SDK3_DIR}/win32)
    endif()
    find_file(ANDOR_SDK3_BINARY "atcore.dll"  PATHS ${ANDOR_SDK3_DIR}/win32 NO_DEFAULT_PATH)
    find_file(ANDOR_SDK3_BINARY2 "atcl_bitflow.dll"  PATHS ${ANDOR_SDK3_DIR}/win32 NO_DEFAULT_PATH)
    find_file(ANDOR_SDK3_BINARY3 "atdevregcam.dll"  PATHS ${ANDOR_SDK3_DIR}/win32 NO_DEFAULT_PATH)
endif(BUILD_TARGET64)

if(ANDOR_SDK3_DIR)

    include_directories(
        ${CMAKE_CURRENT_BINARY_DIR} #build directory of this plugin (recommended)
        ${CMAKE_CURRENT_SOURCE_DIR} #source directory of this plugin (recommended)
        ${ITOM_SDK_INCLUDE_DIRS}    #include directory of the itom SDK (recommended)
        #add further include directories here
        ${ANDOR_SDK3_DIR}
    )

    link_directories(
    )

    set(PLUGIN_HEADERS
        ${CMAKE_CURRENT_SOURCE_DIR}/DialogAndorSDK3.h
        ${CMAKE_CURRENT_SOURCE_DIR}/DockWidgetAndorSDK3.h
        ${CMAKE_CURRENT_SOURCE_DIR}/AndorSDK3.h
        ${CMAKE_CURRENT_SOURCE_DIR}/AndorSDK3Interface.h
        ${CMAKE_CURRENT_BINARY_DIR}/pluginVersion.h
        ${CMAKE_CURRENT_BINARY_DIR}/gitVersion.h
        #add further header files (absolute paths e.g. using CMAKE_CURRENT_SOURCE_DIR)
    )

    set(PLUGIN_SOURCES
        ${CMAKE_CURRENT_SOURCE_DIR}/DialogAndorSDK3.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/DockWidgetAndorSDK3.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/AndorSDK3.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/AndorSDK3Interface.cpp
        #add further source files here
    )

    if(MSVC)
        list(APPEND PLUGIN_SOURCES ${ITOM_SDK_INCLUDE_DIR}/../pluginLibraryVersion.rc)
    endif()

    set(PLUGIN_UI
        ${CMAKE_CURRENT_SOURCE_DIR}/DialogAndorSDK3.ui
        ${CMAKE_CURRENT_SOURCE_DIR}/DockWidgetAndorSDK3.ui
    )

	#Define Version in pluginVersion.h
	configure_file( ${CMAKE_CURRENT_SOURCE_DIR}/pluginVersion.h.in ${CMAKE_CURRENT_BINARY_DIR}/pluginVersion.h)

    set(PLUGIN_RCC
        #add absolute paths to any *.qrc resource files here
    )

    add_library(${target_name} SHARED ${PLUGIN_SOURCES} ${PLUGIN_HEADERS} ${PLUGIN_UI} ${PLUGIN_RCC})

    set_target_properties(${target_name} PROPERTIES AUTOMOC ON AUTORCC ON AUTOUIC ON)

    target_link_libraries(${target_name} ${OpenCV_LIBS} ${ITOM_SDK_LIBRARIES} ${QT5_LIBRARIES} ${VISUALLEAKDETECTOR_LIBRARIES} ${ANDOR_SDK3_LIBRARY})

    set(FILES_TO_TRANSLATE ${PLUGIN_SOURCES} ${PLUGIN_HEADERS} ${PLUGIN_UI})
    itom_library_translation(QM_FILES TARGET ${target_name} FILES_TO_TRANSLATE ${FILES_TO_TRANSLATE})

    itom_configure_plugin_documentation(${target_name} AndorSDK3)

    set(COPY_SOURCES "")
    set(COPY_DESTINATIONS "")

    itom_add_pluginlibrary_to_copy_list(${target_name} COPY_SOURCES COPY_DESTINATIONS)
    itom_add_plugin_qm_files_to_copy_list(${target_name} QM_FILES COPY_SOURCES COPY_DESTINATIONS)

    itom_post_build_copy_files(${target_name} COPY_SOURCES COPY_DESTINATIONS)

    if(WIN32)
        itom_post_build_copy_files_to_lib_folder(${target_name} ANDOR_SDK3_BINARY)
        itom_post_build_copy_files_to_lib_folder(${target_name} ANDOR_SDK3_BINARY2)
        itom_post_build_copy_files_to_lib_folder(${target_name} ANDOR_SDK3_BINARY3)
    endif(WIN32)

else()
    message(SEND_ERROR "${target_name} directory could not be found. ${target_name} will not be build")
endif()
