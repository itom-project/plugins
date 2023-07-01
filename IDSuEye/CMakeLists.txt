cmake_minimum_required(VERSION 3.12...3.24)

set(target_name IDSuEye)

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

if(NOT EXISTS ${IDS_DEVELOP_DIRECTORY})
    if(EXISTS $ENV{IDS_UEYE_ROOT})
        set(IDS_DEVELOP_DIRECTORY "$ENV{IDS_UEYE_ROOT}/develop" CACHE PATH "path to develop directory of IDS SDK")
    else(EXISTS $ENV{IDS_UEYE_ROOT})
        set(IDS_DEVELOP_DIRECTORY "IDS-UEYE_DIR-NOTFOUND" CACHE PATH "path to develop directory of IDS SDK")
    endif(EXISTS $ENV{IDS_UEYE_ROOT})
endif(NOT EXISTS ${IDS_DEVELOP_DIRECTORY})

if(IDS_DEVELOP_DIRECTORY)
    if(WIN32)
        message(STATUS "IDS SDK is located in ${IDS_DEVELOP_DIRECTORY}")
        set(IDS_INCLUDE_DIRECTORY "${IDS_DEVELOP_DIRECTORY}/include" )
        find_file( IDS_HEADER_FILE ueye.h PATHS ${IDS_INCLUDE_DIRECTORY})
    else(WIN32)
        find_path(IDS_INCLUDE_DIRECTORY
        NAMES ueye.h
        PATHS /usr/local/include/
              /usr/include)
        find_file(IDS_HEADER_FILE ueye.h PATHS ${IDS_INCLUDE_DIRECTORY})
    endif(WIN32)
endif(IDS_DEVELOP_DIRECTORY)

if(IDS_HEADER_FILE)
    if(WIN32)
        set( DELAY_IMP_LIB "delayimp")
        if(IDS_DEVELOP_DIRECTORY)
            if(BUILD_TARGET64)
                find_library(IDS_LIBRARY "ueye_api_64" PATHS ${IDS_DEVELOP_DIRECTORY} PATH_SUFFIXES Lib)
            else(BUILD_TARGET64)
                find_library(IDS_LIBRARY "ueye_api" PATHS ${IDS_DEVELOP_DIRECTORY} PATH_SUFFIXES Lib)
            endif(BUILD_TARGET64)
        else(IDS_DEVELOP_DIRECTORY)
            set(IDS_LIB_DIRECTORY "${CMAKE_CURRENT_LIST_DIR}/IDSDevLibs" )
            if(BUILD_TARGET64)
                set( IDS_LIBRARY ${IDS_LIB_DIRECTORY}/Lib/uEye_api_64.lib )
            else(BUILD_TARGET64)
                set( IDS_LIBRARY ${IDS_LIB_DIRECTORY}/Lib/uEye_api.lib )
            endif(BUILD_TARGET64)
        endif(IDS_DEVELOP_DIRECTORY)
    else(WIN32)
        set( DELAY_IMP_LIB )
        if(IDS_DEVELOP_DIRECTORY)
        else(IDS_DEVELOP_DIRECTORY)
            if(BUILD_TARGET64)
                FIND_LIBRARY(IDS_LIBRARY NAMES ueye_api64 libueye_api64
                    PATHS /usr/local/lib64/
                          /usr/local/lib/
                          /usr/lib/
                          /usr/lib/x86_64-linux-gnu/
                          /usr/lib64/
                )
            else(BUILD_TARGET64)
                FIND_LIBRARY(IDS_LIBRARY NAMES ueye_api libueye_api
                    PATHS /usr/local/lib/
                          /usr/local/lib64
                          /usr/lib/i386-linux-gnu/
                          /usr/lib/
                          /usr/lib64/
                )
            endif(BUILD_TARGET64)
        endif(IDS_DEVELOP_DIRECTORY)
    endif(WIN32)

    include_directories(
        ${CMAKE_CURRENT_BINARY_DIR} #build directory of this plugin (recommended)
        ${CMAKE_CURRENT_SOURCE_DIR} #source directory of this plugin (recommended)
        ${ITOM_SDK_INCLUDE_DIRS}    #include directory of the itom SDK (recommended)
        #add further include directories here
        ${Tools_SOURCE_DIR}
        ${IDS_INCLUDE_DIRECTORY}
    )

    link_directories(
    )

    set(PLUGIN_HEADERS
        ${CMAKE_CURRENT_SOURCE_DIR}/DialogIDS.h
        ${CMAKE_CURRENT_SOURCE_DIR}/DockWidgetIDS.h
        ${CMAKE_CURRENT_SOURCE_DIR}/IDSuEye.h
        ${CMAKE_CURRENT_SOURCE_DIR}/IDSInterface.h
        ${CMAKE_CURRENT_SOURCE_DIR}/pluginVersion.h
        ${CMAKE_CURRENT_BINARY_DIR}/gitVersion.h
        #add further header files (absolute pathes e.g. using CMAKE_CURRENT_SOURCE_DIR)
    )

    set(PLUGIN_SOURCES
        ${CMAKE_CURRENT_SOURCE_DIR}/DialogIDS.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/DockWidgetIDS.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/IDSuEye.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/IDSInterface.cpp
        #add further source files here
    )

    #Append rc file to the source files for adding information about the plugin
    # to the properties of the DLL under Visual Studio.
    if(MSVC)
        list(APPEND PLUGIN_SOURCES ${ITOM_SDK_INCLUDE_DIR}/../pluginLibraryVersion.rc)
    endif()

    set(PLUGIN_UI
        ${CMAKE_CURRENT_SOURCE_DIR}/dialogIDS.ui
        ${CMAKE_CURRENT_SOURCE_DIR}/dockWidgetIDS.ui
    )

    set(PLUGIN_RCC
        #add absolute pathes to any *.qrc resource files here
    )

    add_library(${target_name} SHARED ${PLUGIN_SOURCES} ${PLUGIN_HEADERS} ${PLUGIN_UI} ${PLUGIN_RCC})

    set_target_properties(${target_name} PROPERTIES AUTOMOC ON AUTORCC ON AUTOUIC ON)

    target_link_libraries(${target_name} ${OpenCV_LIBS} ${ITOM_SDK_LIBRARIES} ${QT5_LIBRARIES} ${VISUALLEAKDETECTOR_LIBRARIES} ${IDS_LIBRARY} ${DELAY_IMP_LIB})

    set(FILES_TO_TRANSLATE ${PLUGIN_SOURCES} ${PLUGIN_HEADERS} ${PLUGIN_UI})
    itom_library_translation(QM_FILES TARGET ${target_name} FILES_TO_TRANSLATE ${FILES_TO_TRANSLATE})

    itom_configure_plugin_documentation(${target_name} IDSuEyeCamera)

    set(COPY_SOURCES "")
    set(COPY_DESTINATIONS "")

    itom_add_pluginlibrary_to_copy_list(${target_name} COPY_SOURCES COPY_DESTINATIONS)
    itom_add_plugin_qm_files_to_copy_list(${target_name} QM_FILES COPY_SOURCES COPY_DESTINATIONS)

    itom_post_build_copy_files(${target_name} COPY_SOURCES COPY_DESTINATIONS)

else(IDS_HEADER_FILE)
    message(SEND_ERROR "ueye.h for plugin ${target_name} could not be found. ${target_name} will not be build.")
endif(IDS_HEADER_FILE)
