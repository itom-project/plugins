cmake_minimum_required(VERSION 3.12...3.29)
set(target_name ThorlabsISM)

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
set(THORLABS_KINESIS_DIRECTORY "" CACHE PATH "path to the install directory of Thorlabs Kinesis (e.g. C:/Program Files/Thorlabs/Kinesis for x64)")

if(NOT THORLABS_KINESIS_DIRECTORY)
    set(THORLABS_KINESIS_DIRECTORY $ENV{THORLABS_KINESIS_ROOT})
endif(NOT THORLABS_KINESIS_DIRECTORY)

set(THORLABS_KINESIS_LIBNAME "Thorlabs.MotionControl.IntegratedStepperMotors")

if(THORLABS_KINESIS_DIRECTORY)
    if(WIN32)
        find_path( THORLABS_KINESIS_INCLUDE_DIR "${THORLABS_KINESIS_LIBNAME}.h" PATHS ${THORLABS_KINESIS_DIRECTORY})
        find_library (THORLABS_KINESIS_LIBRARY ${THORLABS_KINESIS_LIBNAME} PATHS ${THORLABS_KINESIS_DIRECTORY})
    endif()
endif(THORLABS_KINESIS_DIRECTORY)

if(THORLABS_KINESIS_INCLUDE_DIR)
    set(CMAKE_INCLUDE_CURRENT_DIR ON)
    set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR} ${ITOM_SDK_DIR}/cmake)

    find_package(ITOM_SDK COMPONENTS dataobject itomCommonLib itomCommonQtLib itomWidgets REQUIRED)

    include(ItomBuildMacros)
    itom_init_cmake_policy(3.12)
    itom_init_plugin_library(${target_name}) #Start the project, init compiler settings and set default configurations for plugins
    itom_find_package_qt(ON Widgets Xml LinguistTools)

    include_directories(
        ${CMAKE_CURRENT_BINARY_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}/Mpusbapi
        ${ITOM_SDK_INCLUDE_DIRS}
        ${THORLABS_KINESIS_INCLUDE_DIR}
    )

    set(PLUGIN_HEADERS
        ${CMAKE_CURRENT_SOURCE_DIR}/dialogThorlabsISM.h
        ${CMAKE_CURRENT_SOURCE_DIR}/dockWidgetThorlabsISM.h
        ${CMAKE_CURRENT_SOURCE_DIR}/ThorlabsISM.h
        ${CMAKE_CURRENT_BINARY_DIR}/pluginVersion.h
        ${CMAKE_CURRENT_BINARY_DIR}/gitVersion.h
    )

    set(PLUGIN_UI
        ${CMAKE_CURRENT_SOURCE_DIR}/dialogThorlabsISM.ui
        ${CMAKE_CURRENT_SOURCE_DIR}/dockWidgetThorlabsISM.ui
    )

    set(PLUGIN_SOURCES
        ${CMAKE_CURRENT_SOURCE_DIR}/dialogThorlabsISM.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/dockWidgetThorlabsISM.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/ThorlabsISM.cpp
    )

    set(PLUGIN_RCC
        #add absolute paths to any *.qrc resource files here
    )

    #Add version information to the plugIn-dll under MSVC
    if(MSVC)
        list(APPEND PLUGIN_SOURCES ${ITOM_SDK_INCLUDE_DIR}/../pluginLibraryVersion.rc)
    endif()

    #Define Version in pluginVersion.h
    configure_file( ${CMAKE_CURRENT_SOURCE_DIR}/pluginVersion.h.in ${CMAKE_CURRENT_BINARY_DIR}/pluginVersion.h)

    add_library(${target_name} SHARED ${PLUGIN_SOURCES} ${PLUGIN_HEADERS} ${PLUGIN_UI} ${PLUGIN_RCC})

    # Qt: enable all automoc, autouic and autorcc.
    set_target_properties(${target_name} PROPERTIES AUTOMOC ON AUTORCC ON AUTOUIC ON)

    target_link_libraries(${target_name} ${OpenCV_LIBS} ${ITOM_SDK_LIBRARIES} ${QT5_LIBRARIES} ${VISUALLEAKDETECTOR_LIBRARIES} ${THORLABS_KINESIS_LIBRARY})

    #translation
    set(FILES_TO_TRANSLATE ${PLUGIN_SOURCES} ${PLUGIN_HEADERS} ${PLUGIN_UI})
    itom_library_translation(QM_FILES TARGET ${target_name} FILES_TO_TRANSLATE ${FILES_TO_TRANSLATE})

    itom_configure_plugin_documentation(${target_name} ThorlabsISM)

    # COPY SECTION
    set(COPY_SOURCES "")
    set(COPY_DESTINATIONS "")
    itom_add_pluginlibrary_to_copy_list(${target_name} COPY_SOURCES COPY_DESTINATIONS)
    itom_add_plugin_qm_files_to_copy_list(${target_name} QM_FILES COPY_SOURCES COPY_DESTINATIONS)
    itom_post_build_copy_files(${target_name} COPY_SOURCES COPY_DESTINATIONS)

    set(BINARY_FILES
        "${THORLABS_KINESIS_DIRECTORY}/Thorlabs.MotionControl.IntegratedStepperMotors.dll"
        "${THORLABS_KINESIS_DIRECTORY}/Thorlabs.MotionControl.DeviceManager.dll"
        "${THORLABS_KINESIS_DIRECTORY}/ftd2xx.dll"
        )

    itom_post_build_copy_files_to_lib_folder(${target_name} BINARY_FILES)

else()
    message(SEND_ERROR "${THORLABS_KINESIS_LIBNAME}.h for plugin ${target_name} could not be found. ${target_name} will not be build. Please properly indicate THORLABS_KINESIS_DIRECTORY.")
endif()
