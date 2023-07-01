cmake_minimum_required(VERSION 3.12...3.24)

set(target_name MeasurementComputing)

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

include(ItomBuildMacros)
itom_init_cmake_policy(3.12)
itom_init_plugin_library(${target_name}) #Start the project, init compiler settings and set default configurations for plugins
itom_find_package_qt(ON Core LinguistTools Widgets)

if(BUILD_TARGET64)
   set(dll_name "cbw64.dll")
   set(bit_dest "64")
else(BUILD_TARGET64)
   set(dll_name "cbw32.dll")
   set(bit_dest "32")
endif(BUILD_TARGET64)

find_path(MeasurementComputing_DAQ_C_SDK_DIR cbw.h PATHS "C:/Users/Public/Documents/Measurement Computing/DAQ/C" DOC "Directory of the DAQ C library, must contain the file cbw.h (e.g. C:/Users/Public/Documents/Measurement Computing/DAQ/C)")
find_file(MeasurementComputing_DAQ_BINARY ${dll_name} PATHS "C:/Program Files/InstaCal" "D:/Programme/InstaCal" "C:/Program Files (x86)/Measurement Computing/DAQ" $ENV{MCDAQ_ROOT} DOC "Library cbw32.dll or cbw64.dll that is found in the directory of the InstaCal application.")

if(MeasurementComputing_DAQ_C_SDK_DIR AND MeasurementComputing_DAQ_BINARY)
    set(MeasurementComputing_LIB debug cbw${bit_dest} optimized cbw${bit_dest}  CACHE STRING "" FORCE)

    include_directories(
        ${CMAKE_CURRENT_BINARY_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${ITOM_SDK_INCLUDE_DIRS}
        ${MeasurementComputing_DAQ_C_SDK_DIR}
    )

    link_directories(
        ${USBADDA_LIB_DIR}
        ${MeasurementComputing_DAQ_C_SDK_DIR}
    )

    set(PLUGIN_HEADERS
        ${CMAKE_CURRENT_SOURCE_DIR}/MeasurementComputing.h
        ${CMAKE_CURRENT_SOURCE_DIR}/pluginVersion.h
        ${CMAKE_CURRENT_BINARY_DIR}/gitVersion.h
        ${CMAKE_CURRENT_SOURCE_DIR}/dialogMeasurementComputing.h
    )

    set(PLUGIN_UI
        ${CMAKE_CURRENT_SOURCE_DIR}/dialogMeasurementComputing.ui
    )

    set(PLUGIN_SOURCES
        ${CMAKE_CURRENT_SOURCE_DIR}/MeasurementComputing.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/dialogMeasurementComputing.cpp
    )

    set(PLUGIN_RCC
        #add absolute pathes to any *.qrc resource files here
    )

    #Add version information to the plugIn-dll unter MSVC
    if(MSVC)
        list(APPEND PLUGIN_SOURCES ${ITOM_SDK_INCLUDE_DIR}/../pluginLibraryVersion.rc)
    endif()

    add_library(${target_name} SHARED ${PLUGIN_SOURCES} ${PLUGIN_HEADERS} ${PLUGIN_UI} ${PLUGIN_RCC})

    # Qt: enable all automoc, autouic and autorcc.
    set_target_properties(${target_name} PROPERTIES AUTOMOC ON AUTORCC ON AUTOUIC ON)

    if(MSVC)
        #delayload cbw32.dll or cbw64.dll
        set_target_properties (${target_name} PROPERTIES LINK_FLAGS "/DELAYLOAD:cbw${bit_dest}.dll")
        target_link_libraries(${target_name} ${ITOM_SDK_LIBRARIES} ${MeasurementComputing_LIB} ${QT5_LIBRARIES} ${VISUALLEAKDETECTOR_LIBRARIES} Delayimp.lib)
    else(MSVC)
        target_link_libraries(${target_name} ${ITOM_SDK_LIBRARIES} ${MeasurementComputing_LIB} ${QT5_LIBRARIES} ${VISUALLEAKDETECTOR_LIBRARIES})
    endif()

    set(FILES_TO_TRANSLATE ${PLUGIN_SOURCES} ${PLUGIN_HEADERS} ${PLUGIN_UI})
    itom_library_translation(QM_FILES TARGET ${target_name} FILES_TO_TRANSLATE ${FILES_TO_TRANSLATE})

    #documentation
    itom_configure_plugin_documentation(${target_name} MeasurementComputing)

    # COPY SECTION
    set(COPY_SOURCES "")
    set(COPY_DESTINATIONS "")
    itom_add_pluginlibrary_to_copy_list(${target_name} COPY_SOURCES COPY_DESTINATIONS)
    itom_add_plugin_qm_files_to_copy_list(${target_name} QM_FILES COPY_SOURCES COPY_DESTINATIONS)
    itom_post_build_copy_files(${target_name} COPY_SOURCES COPY_DESTINATIONS)

    # copy libs to itoms lib folder
    itom_post_build_copy_files_to_lib_folder(${target_name} MeasurementComputing_DAQ_BINARY)

else()
    message(SEND_ERROR "C-SDK directory of Measurement Computing DAQ or install directory of InstaCal application could not be found. ${target_name} will not be build")
endif()
