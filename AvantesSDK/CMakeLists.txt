cmake_minimum_required(VERSION 3.12...3.29)

if(WIN32)

    set(target_name AvantesSDK)

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
    itom_find_package_qt(ON Widgets UiTools Xml LinguistTools)


    if(NOT EXISTS ${AVASPEC_DIR})
        if(EXISTS $ENV{AVASPEC_ROOT})
            set(AVASPEC_DIR "$ENV{AVASPEC_ROOT}" CACHE PATH "path to Avantes AvaSpec SDK directory")
        else(EXISTS $ENV{AVASPEC_ROOT})
            set(AVASPEC_DIR "AVASPEC_DIR_NOTFOUND" CACHE PATH "path to Avantes AvaSpec SDK directory")
        endif(EXISTS $ENV{AVASPEC_ROOT})
    endif(NOT EXISTS ${AVASPEC_DIR})


    if(EXISTS ${AVASPEC_DIR})
        set(AVASPEC_INCLUDE_DIR "${AVASPEC_DIR}/examples/Qt_demo_full")
        find_file( AVASPEC_HEADER_FILE "avaspec.h" PATHS ${AVASPEC_INCLUDE_DIR})
    endif(EXISTS ${AVASPEC_DIR})

    find_file(AVASPEC_BINARY "avaspecx64.dll"  PATHS ${AVASPEC_DIR})

    # D:\3rdParty\AvaSpec\examples\Qt_demo_full
    set(AVASPEC_LIBDIR_HINT "${AVASPEC_DIR}/examples/Qt_demo_full")
    find_library(AVASPEC_LIBRARY NAMES "avaspecx64" PATHS ${AVASPEC_LIBDIR_HINT} NO_DEFAULT_PATH)

    if(AVASPEC_HEADER_FILE AND AVASPEC_BINARY AND AVASPEC_LIBRARY)

        include_directories(
            ${CMAKE_CURRENT_BINARY_DIR}
            ${CMAKE_CURRENT_SOURCE_DIR}
            ${ITOM_SDK_INCLUDE_DIRS}
        )

        set(PLUGIN_HEADERS
            ${CMAKE_CURRENT_SOURCE_DIR}/avantesSDK.h
            ${CMAKE_CURRENT_SOURCE_DIR}/dialogAvantesSDK.h
            ${CMAKE_CURRENT_SOURCE_DIR}/dockWidgetAvantesSDK.h
            ${CMAKE_CURRENT_SOURCE_DIR}/pluginVersion.h
            ${CMAKE_CURRENT_BINARY_DIR}/gitVersion.h
        )

        set(PLUGIN_SOURCES
            ${CMAKE_CURRENT_SOURCE_DIR}/avantesSDK.cpp
            ${CMAKE_CURRENT_SOURCE_DIR}/dialogAvantesSDK.cpp
            ${CMAKE_CURRENT_SOURCE_DIR}/dockWidgetAvantesSDK.cpp
        )

        set(PLUGIN_UI
            ${CMAKE_CURRENT_SOURCE_DIR}/dialogAvantesSDK.ui
            ${CMAKE_CURRENT_SOURCE_DIR}/dockWidgetAvantesSDK.ui
        )

        set(PLUGIN_RCC
            #add absolute pathes to any *.qrc resource files here
        )

        #Add version information to the plugIn-dll unter MSVC
        if(MSVC)
            list(APPEND PLUGIN_SOURCES ${ITOM_SDK_INCLUDE_DIR}/../pluginLibraryVersion.rc)
        endif()

        message(STATUS "AVASPEC_HEADER_FILE: ${AVASPEC_HEADER_FILE}")
        message(STATUS "AVASPEC_BINARY: ${AVASPEC_BINARY}")
        message(STATUS "AVASPEC_LIBRARY: ${AVASPEC_LIBRARY}")

        add_library(${target_name} SHARED ${PLUGIN_SOURCES} ${PLUGIN_HEADERS} ${PLUGIN_UI} ${PLUGIN_RCC})

        add_library(acaspec_imported SHARED IMPORTED) # or STATIC instead of SHARED
        set_target_properties(acaspec_imported PROPERTIES
            IMPORTED_LOCATION ${AVASPEC_BINARY}
            IMPORTED_IMPLIB ${AVASPEC_LIBRARY}
            INTERFACE_INCLUDE_DIRECTORIES ${AVASPEC_INCLUDE_DIR})

        # Qt: enable all automoc, autouic and autorcc.
        set_target_properties(${target_name} PROPERTIES AUTOMOC ON AUTORCC ON AUTOUIC ON)

        target_link_libraries(${target_name} ${ITOM_SDK_LIBRARIES} ${QT5_LIBRARIES} ${VISUALLEAKDETECTOR_LIBRARIES} acaspec_imported)

        #translation
        set(FILES_TO_TRANSLATE ${PLUGIN_SOURCES} ${PLUGIN_HEADERS} ${PLUGIN_UI})
        itom_library_translation(QM_FILES TARGET ${target_name} FILES_TO_TRANSLATE ${FILES_TO_TRANSLATE})

        #documentation
        itom_configure_plugin_documentation(${target_name} avantesSDK)

        # COPY SECTION
        set(COPY_SOURCES "")
        set(COPY_DESTINATIONS "")
        itom_add_pluginlibrary_to_copy_list(${target_name} COPY_SOURCES COPY_DESTINATIONS)

        itom_add_plugin_qm_files_to_copy_list(${target_name} QM_FILES COPY_SOURCES COPY_DESTINATIONS)
        itom_post_build_copy_files(${target_name} COPY_SOURCES COPY_DESTINATIONS)

        itom_post_build_copy_files_to_lib_folder(${target_name} AVASPEC_BINARY)

    else(AVASPEC_HEADER_FILE AND AVASPEC_BINARY AND AVASPEC_LIBRARY)
        message(SEND_ERROR "${target_name}-API directory could not be found. ${target_name} will not be build")
    endif(AVASPEC_HEADER_FILE AND AVASPEC_BINARY AND AVASPEC_LIBRARY)

else(WIN32)
    message(SEND_ERROR "${target_name}-API only supports Windows Applications.")
endif(WIN32)