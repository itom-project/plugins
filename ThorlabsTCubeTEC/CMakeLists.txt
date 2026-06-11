cmake_minimum_required(VERSION 3.12...3.29)
set(target_name ThorlabsTCubeTEC)

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

set(THORLABS_KINESIS_DIRECTORY "" CACHE PATH "path to the install directory of Thorlabs Kinesis (e.g. C:/Program Files/Thorlabs/Kinesis for x64)")

if(NOT THORLABS_KINESIS_DIRECTORY)
    set(THORLABS_KINESIS_DIRECTORY $ENV{THORLABS_KINESIS_ROOT})
endif(NOT THORLABS_KINESIS_DIRECTORY)

set(THORLABS_KINESIS_TEC_LIBNAME "Thorlabs.MotionControl.TCube.TEC")

option(BUILD_TARGET64 "Build for 64 bit target if set to ON or 32 bit if set to OFF." ON)

if(THORLABS_KINESIS_DIRECTORY)
    if(WIN32)
        find_path( THORLABS_KINESIS_TEC_INCLUDE_DIR "${THORLABS_KINESIS_TEC_LIBNAME}.h" PATHS ${THORLABS_KINESIS_DIRECTORY})
        find_library (THORLABS_KINESIS_TEC_LIBRARY ${THORLABS_KINESIS_TEC_LIBNAME} PATHS ${THORLABS_KINESIS_DIRECTORY})
    endif(WIN32)
endif(THORLABS_KINESIS_DIRECTORY)

if(THORLABS_KINESIS_TEC_INCLUDE_DIR)

    set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR} ${ITOM_SDK_DIR}/cmake)
    set(CMAKE_INCLUDE_CURRENT_DIR ON)

    find_package(ITOM_SDK COMPONENTS dataobject itomCommonLib itomCommonQtLib itomWidgets REQUIRED)

    include(ItomBuildMacros)
    itom_init_cmake_policy(3.12)
    itom_init_plugin_library(${target_name})
    itom_find_package_qt(ON Core Widgets LinguistTools)

    include_directories(
        ${CMAKE_CURRENT_BINARY_DIR} #build directory of this plugin (recommended)
        ${CMAKE_CURRENT_SOURCE_DIR} #source directory of this plugin (recommended)
        ${ITOM_SDK_INCLUDE_DIRS}    #include directory of the itom SDK (recommended) as well as necessary 3rd party directories (e.g. from OpenCV)
        ${THORLABS_KINESIS_TEC_INCLUDE_DIR}
    )

    link_directories(
        #add all linker directories
    )

    set(PLUGIN_HEADERS
        ${CMAKE_CURRENT_SOURCE_DIR}/ThorlabsTCubeTEC.h
        ${CMAKE_CURRENT_SOURCE_DIR}/dockWidgetThorlabsTCubeTEC.h
        ${CMAKE_CURRENT_BINARY_DIR}/pluginVersion.h
        ${CMAKE_CURRENT_BINARY_DIR}/gitVersion.h #provided by the init script, contains currently checked out git tag
        #add further header files (absolute paths e.g. using CMAKE_CURRENT_SOURCE_DIR)
    )

    set(PLUGIN_SOURCES
        ${CMAKE_CURRENT_SOURCE_DIR}/dockWidgetThorlabsTCubeTEC.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/ThorlabsTCubeTEC.cpp
        #add further source files here
    )

    #Append rc file to the source files for adding information about the plugin
    # to the properties of the DLL under Visual Studio.
    if(MSVC)
        list(APPEND PLUGIN_SOURCES ${ITOM_SDK_INCLUDE_DIR}/../pluginLibraryVersion.rc)
    endif()

    set(PLUGIN_UI
        ${CMAKE_CURRENT_SOURCE_DIR}/dockWidgetThorlabsTCubeTEC.ui
    )

    set(PLUGIN_RCC
        #add absolute paths to any *.qrc resource files here
    )

    #Define Version in pluginVersion.h
    configure_file( ${CMAKE_CURRENT_SOURCE_DIR}/pluginVersion.h.in ${CMAKE_CURRENT_BINARY_DIR}/pluginVersion.h)

    #add all (generated) header and source files to the library (these files are compiled then)
    add_library(${target_name} SHARED
        ${PLUGIN_SOURCES}
        ${PLUGIN_HEADERS}
        ${PLUGIN_UI}
        ${PLUGIN_RCC}
        )

    # Qt: enable all automoc, autouic and autorcc. Autouic file will be disabled for all files that are processed manually by itom_qt_wrap_ui.
    set_target_properties(${target_name} PROPERTIES AUTOMOC ON AUTORCC ON AUTOUIC ON)


    target_link_libraries(${target_name} ${ITOM_SDK_LIBRARIES} ${QT5_LIBRARIES} ${VISUALLEAKDETECTOR_LIBRARIES} ${THORLABS_KINESIS_TEC_LIBRARY})

    set(FILES_TO_TRANSLATE ${PLUGIN_SOURCES} ${PLUGIN_HEADERS} ${PLUGIN_UI} ${PLUGIN_RCC})
    itom_library_translation(
        QM_FILES
        TARGET ${target_name}
        FILES_TO_TRANSLATE ${FILES_TO_TRANSLATE}
        )

    itom_configure_plugin_documentation(${target_name} ThorlabsTCubeTEC)


    set(COPY_SOURCES "")
    set(COPY_DESTINATIONS "")

    itom_add_pluginlibrary_to_copy_list(${target_name} COPY_SOURCES COPY_DESTINATIONS)
    itom_add_plugin_qm_files_to_copy_list(${target_name} QM_FILES COPY_SOURCES COPY_DESTINATIONS)

    itom_post_build_copy_files(${target_name} COPY_SOURCES COPY_DESTINATIONS)

    set(BINARY_FILES
        "${THORLABS_KINESIS_DIRECTORY}/Thorlabs.MotionControl.TCube.TEC.dll"
        "${THORLABS_KINESIS_DIRECTORY}/Thorlabs.MotionControl.DeviceManager.dll"
        "${THORLABS_KINESIS_DIRECTORY}/ftd2xx.dll"
        )

    itom_post_build_copy_files_to_lib_folder(${target_name} BINARY_FILES)

else(THORLABS_KINESIS_TEC_INCLUDE_DIR)
    message(SEND_ERROR "${THORLABS_KINESIS_TEC_LIBNAME}.h for plugin ${target_name} could not be found. ${target_name} will not be build. Please properly indicate THORLABS_KINESIS_DIRECTORY.")
endif(THORLABS_KINESIS_TEC_INCLUDE_DIR)
