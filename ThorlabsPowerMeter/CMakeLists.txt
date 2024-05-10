cmake_minimum_required(VERSION 3.12...3.29)
set(target_name ThorlabsPowerMeter)

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

if(NOT EXISTS ${THORLABS_PM100D_VISA_DIR})
    if(EXISTS $ENV{NI_VISA_ROOT})
        set(THORLABS_PM100D_VISA_DIR "$ENV{NI_VISA_ROOT}" CACHE PATH "path to the install directory of Thorlabs PM100D VISA (e.g. C:/Program Files/IVI Foundation/VISA/Win64 for x64)")
    else(EXISTS $ENV{NI_VISA_ROOT})
        set(THORLABS_PM100D_VISA_DIR "AVTVIMBA_INCLUDE_DIR_NOTFOUND" CACHE PATH "path to the install directory of Thorlabs PM100D VISA (e.g. C:/Program Files/IVI Foundation/VISA/Win64 for x64)")
    endif(EXISTS $ENV{NI_VISA_ROOT})
endif(NOT EXISTS ${THORLABS_PM100D_VISA_DIR})

set(THORLABS_POWER_METER_API_VERSION "1.1.0" CACHE STRING "Expected API version: 1.0.2 stands for the legacy software 'Thorlabs Power Meter', while 1.1.0 stands for the current software 'Thorlabs Optical Power Meter'")
set_property(CACHE THORLABS_POWER_METER_API_VERSION PROPERTY STRINGS 1.0.2 1.1.0)

if(THORLABS_PM100D_VISA_DIR)
    if(WIN32)
        if(THORLABS_POWER_METER_API_VERSION VERSION_EQUAL "1.1.0")
            find_path( THORLABS_PM100D_VISA_INCLUDE_DIR NAMES "TLPM.h" PATHS ${THORLABS_PM100D_VISA_DIR} PATH_SUFFIXES "Include")
        else(THORLABS_POWER_METER_API_VERSION VERSION_EQUAL "1.1.0")
            find_path( THORLABS_PM100D_VISA_INCLUDE_DIR NAMES "PM100D.h" PATHS ${THORLABS_PM100D_VISA_DIR} PATH_SUFFIXES "Include")
        endif(THORLABS_POWER_METER_API_VERSION VERSION_EQUAL "1.1.0")

        if(BUILD_TARGET64)
            if(THORLABS_POWER_METER_API_VERSION VERSION_EQUAL "1.1.0")
                find_library (THORLABS_TLPM_VISA_LIBRARY "TLPM_64" PATHS ${THORLABS_PM100D_VISA_DIR} PATH_SUFFIXES "Lib_x64/msc")
            else(THORLABS_POWER_METER_API_VERSION VERSION_EQUAL "1.1.0")
                find_library (THORLABS_PM100D_VISA_LIBRARY "PM100D_64" PATHS ${THORLABS_PM100D_VISA_DIR} PATH_SUFFIXES "Lib_x64/msc")
                find_library(THORLABS_VISA_LIBRARY "visa64" PATHS ${THORLABS_PM100D_VISA_DIR} PATH_SUFFIXES "Lib_x64/msc")
            endif(THORLABS_POWER_METER_API_VERSION VERSION_EQUAL "1.1.0")
        else(BUILD_TARGET64)
            if(THORLABS_POWER_METER_API_VERSION VERSION_EQUAL "1.1.0")
                find_library (THORLABS_TLPM_VISA_LIBRARY "TLPM_32" PATHS ${THORLABS_PM100D_VISA_DIR} PATH_SUFFIXES "lib/msc")
            else(THORLABS_POWER_METER_API_VERSION VERSION_EQUAL "1.1.0")
                find_library (THORLABS_PM100D_VISA_LIBRARY "PM100D_32" PATHS ${THORLABS_PM100D_VISA_DIR} PATH_SUFFIXES "lib/msc")
                find_library(THORLABS_VISA_LIBRARY "visa32" PATHS ${THORLABS_PM100D_VISA_DIR} PATH_SUFFIXES "lib/msc")
            endif(THORLABS_POWER_METER_API_VERSION VERSION_EQUAL "1.1.0")
        endif(BUILD_TARGET64)
    endif(WIN32)
endif(THORLABS_PM100D_VISA_DIR)

if(THORLABS_PM100D_VISA_INCLUDE_DIR)

    set(CMAKE_INCLUDE_CURRENT_DIR ON)
    set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR} ${ITOM_SDK_DIR}/cmake)

    find_package(ITOM_SDK COMPONENTS dataobject itomCommonLib itomCommonQtLib itomWidgets REQUIRED)

    include(ItomBuildMacros)
    itom_init_cmake_policy(3.12)
    itom_init_plugin_library(${target_name}) #Start the project, init compiler settings and set default configurations for plugins
    itom_find_package_qt(ON Core LinguistTools Widgets)

    if(THORLABS_POWER_METER_API_VERSION VERSION_EQUAL "1.1.0")
        add_definitions(-DUSE_API_1_1_0)
    else()
        add_definitions(-DUSE_API_1_0_2)
    endif()

    include_directories(
        ${CMAKE_CURRENT_BINARY_DIR} #build directory of this plugin (recommended)
        ${CMAKE_CURRENT_SOURCE_DIR} #source directory of this plugin (recommended)
        ${ITOM_SDK_INCLUDE_DIRS}    #include directory of the itom SDK (recommended)
        ${THORLABS_PM100D_VISA_INCLUDE_DIR}
        #add further include directories here
    )

    link_directories()

    set(PLUGIN_HEADERS
        ${CMAKE_CURRENT_SOURCE_DIR}/thorlabsPowerMeter.h
        ${CMAKE_CURRENT_SOURCE_DIR}/dialogThorlabsPowerMeter.h
        ${CMAKE_CURRENT_SOURCE_DIR}/dockWidgetThorlabsPowerMeter.h
        ${CMAKE_CURRENT_SOURCE_DIR}/pluginVersion.h
        ${CMAKE_CURRENT_BINARY_DIR}/gitVersion.h
        #${CMAKE_CURRENT_SOURCE_DIR}/PM100source.h
        #add further header files (absolute paths e.g. using CMAKE_CURRENT_SOURCE_DIR)
    )

    set(PLUGIN_SOURCES
        ${CMAKE_CURRENT_SOURCE_DIR}/dialogThorlabsPowerMeter.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/dockWidgetThorlabsPowerMeter.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/thorlabsPowerMeter.cpp
        #${CMAKE_CURRENT_SOURCE_DIR}/PM100source.cpp
        #add further source files here
    )

    if(MSVC)
        list(APPEND PLUGIN_SOURCES ${ITOM_SDK_INCLUDE_DIR}/../pluginLibraryVersion.rc)
    endif()

    set(PLUGIN_UI
        ${CMAKE_CURRENT_SOURCE_DIR}/dialogThorlabsPowerMeter.ui
        ${CMAKE_CURRENT_SOURCE_DIR}/dockWidgetThorlabsPowerMeter.ui
    )

    set(PLUGIN_RCC
        #add absolute paths to any *.qrc resource files here
    )

    #add all (generated) header and source files to the library (these files are compiled then)
    add_library(${target_name} SHARED ${PLUGIN_SOURCES} ${PLUGIN_HEADERS} ${PLUGIN_UI} ${PLUGIN_RCC})

    # Qt: enable all automoc, autouic and autorcc.
    set_target_properties(${target_name} PROPERTIES AUTOMOC ON AUTORCC ON AUTOUIC ON)

    if(THORLABS_POWER_METER_API_VERSION VERSION_EQUAL "1.1.0")
        target_link_libraries(${target_name} ${ITOM_SDK_LIBRARIES} ${QT5_LIBRARIES} ${VISUALLEAKDETECTOR_LIBRARIES} ${THORLABS_TLPM_VISA_LIBRARY})
    else(THORLABS_POWER_METER_API_VERSION VERSION_EQUAL "1.1.0")
        target_link_libraries(${target_name} ${ITOM_SDK_LIBRARIES} ${QT5_LIBRARIES} ${VISUALLEAKDETECTOR_LIBRARIES} ${THORLABS_VISA_LIBRARY} ${THORLABS_PM100D_VISA_LIBRARY})
    endif(THORLABS_POWER_METER_API_VERSION VERSION_EQUAL "1.1.0")

    set(FILES_TO_TRANSLATE ${PLUGIN_SOURCES} ${PLUGIN_HEADERS} ${PLUGIN_UI})
    itom_library_translation(QM_FILES TARGET ${target_name} FILES_TO_TRANSLATE ${FILES_TO_TRANSLATE})

    itom_configure_plugin_documentation(${target_name} thorlabsPowerMeter)

    set(COPY_SOURCES "")
    set(COPY_DESTINATIONS "")

    itom_add_pluginlibrary_to_copy_list(${target_name} COPY_SOURCES COPY_DESTINATIONS)
    itom_add_plugin_qm_files_to_copy_list(${target_name} QM_FILES COPY_SOURCES COPY_DESTINATIONS)

    itom_post_build_copy_files(${target_name} COPY_SOURCES COPY_DESTINATIONS)

else()
    message(SEND_ERROR "PM100D.h for plugin ${target_name} could not be found. ${target_name} will not be build. Please properly indicate THORLABS_PM100D_VISA_DIR.")
endif()
