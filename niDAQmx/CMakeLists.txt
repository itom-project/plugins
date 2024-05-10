cmake_minimum_required(VERSION 3.12...3.29)
set(target_name niDAQmx) #->replace PCI6115 by the name of your plugin (one word)

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
find_package(NIDAQmx)

itom_find_package_qt(ON Core LinguistTools Widgets)

include(ItomBuildMacros)
itom_init_cmake_policy(3.12)
itom_init_plugin_library(${target_name}) #Start the project, init compiler settings and set default configurations for plugins

if(NIDAQMX_FOUND)
    include_directories(
        ${CMAKE_CURRENT_BINARY_DIR} #build directory of this plugin (recommended)
        ${CMAKE_CURRENT_SOURCE_DIR} #source directory of this plugin (recommended)
        ${ITOM_SDK_INCLUDE_DIRS}    #include directory of the itom SDK (recommended)
        ${NIDAQMX_INCLUDE_DIR}
        #add further include directories here
    )

    link_directories(
        #add all linker directories
    )

	set(PLUGIN_HEADERS
		${CMAKE_CURRENT_SOURCE_DIR}/NI-DAQmx.h
		${CMAKE_CURRENT_SOURCE_DIR}/dialogNI-DAQmx.h
		${CMAKE_CURRENT_SOURCE_DIR}/dockWidgetNI-DAQmx.h
		${CMAKE_CURRENT_BINARY_DIR}/pluginVersion.h
        ${CMAKE_CURRENT_BINARY_DIR}/gitVersion.h
        ${CMAKE_CURRENT_SOURCE_DIR}/NI-PeripheralClasses.h
        ${CMAKE_CURRENT_SOURCE_DIR}/NI-DAQmxError.h
        #add further header files (absolute paths e.g. using CMAKE_CURRENT_SOURCE_DIR)
    )

    set(PLUGIN_SOURCES
        ${CMAKE_CURRENT_SOURCE_DIR}/dialogNI-DAQmx.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/dockWidgetNI-DAQmx.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/NI-DAQmx.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/NI-PeripheralClasses.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/NI-DAQmxError.cpp
        #add further source files here
    )

	#Define Version in pluginVersion.h
	configure_file( ${CMAKE_CURRENT_SOURCE_DIR}/pluginVersion.h.in ${CMAKE_CURRENT_BINARY_DIR}/pluginVersion.h)

    if(MSVC)
        list(APPEND PLUGIN_SOURCES ${ITOM_SDK_INCLUDE_DIR}/../pluginLibraryVersion.rc)
    endif(MSVC)

    set(PLUGIN_UI
        "${CMAKE_CURRENT_SOURCE_DIR}/dialogNI-DAQmx.ui"
        "${CMAKE_CURRENT_SOURCE_DIR}/dockWidgetNI-DAQmx.ui"
    )

    set(PLUGIN_RCC
        #add absolute paths to any *.qrc resource files here
        ${CMAKE_CURRENT_SOURCE_DIR}/resource.qrc
    )

    #add all (generated) header and source files to the library (these files are compiled then)
    add_library(${target_name} SHARED ${PLUGIN_SOURCES} ${PLUGIN_HEADERS} ${PLUGIN_UI} ${PLUGIN_RCC})

    # Qt: enable all automoc, autouic and autorcc.
    set_target_properties(${target_name} PROPERTIES AUTOMOC ON AUTORCC ON AUTOUIC ON)

    target_link_libraries(${target_name} ${QT5_LIBRARIES} ${ITOM_SDK_LIBRARIES} ${VISUALLEAKDETECTOR_LIBRARIES} ${NIDAQMX_LIBRARY})

    #translation
    set(FILES_TO_TRANSLATE ${PLUGIN_SOURCES} ${PLUGIN_HEADERS} ${PLUGIN_UI})
    itom_library_translation(QM_FILES TARGET ${target_name} FILES_TO_TRANSLATE ${FILES_TO_TRANSLATE})

    # the target-name differs from the plugin name of this plugin!
    itom_configure_plugin_documentation(${target_name} niDAQmx "NI-DAQmx")

    set(COPY_SOURCES "")
    set(COPY_DESTINATIONS "")

    itom_add_pluginlibrary_to_copy_list(${target_name} COPY_SOURCES COPY_DESTINATIONS)
    itom_add_plugin_qm_files_to_copy_list(${target_name} QM_FILES COPY_SOURCES COPY_DESTINATIONS)

    itom_post_build_copy_files(${target_name} COPY_SOURCES COPY_DESTINATIONS)

else(NIDAQMX_FOUND)
    message(SEND_ERROR "${target_name}-API directory could not be found. ${target_name} will not be build")
endif(NIDAQMX_FOUND)
