set(target_name AVTVimba)

message(STATUS "Project ${target_name} (${CMAKE_CURRENT_BINARY_DIR})")

cmake_minimum_required(VERSION 3.1...3.15)

option(BUILD_TARGET64 "Build for 64 bit target if set to ON or 32 bit if set to OFF." ON)
set(ITOM_SDK_DIR NOTFOUND CACHE PATH "path of SDK subfolder of itom root (build) directory")

find_path(ITOM_SDK_DIR "cmake/itom_sdk.cmake"
    HINTS "C:/itom/build/itom/SDK"
          "${CMAKE_CURRENT_BINARY_DIR}/../../itom/SDK"
    DOC "path of SDK subfolder of itom root (build) directory")

if(NOT ITOM_SDK_DIR)
    message(SEND_ERROR "ITOM_SDK_DIR is invalid. Provide itom SDK directory path first")
endif()

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR} ${ITOM_SDK_DIR}/cmake)

find_package(ITOM_SDK COMPONENTS dataobject itomCommonLib itomCommonQtLib itomWidgets REQUIRED)

include(ItomBuildMacros)
itom_init_cmake_policy(3.12)
itom_init_plugin_library(${target_name}) #Start the project, init compiler settings and set default configurations for plugins
itom_find_package_qt(ON Core LinguistTools Widgets)

if(NOT EXISTS ${AVTVIMBA_INCLUDE_DIR})
    if(EXISTS $ENV{AVTVIMBA_ROOT})
        set(AVTVIMBA_INCLUDE_DIR "$ENV{AVTVIMBA_ROOT}" CACHE PATH "path to AVT Vimba SDK directory")
    else(EXISTS $ENV{AVTVIMBA_ROOT})
        set(AVTVIMBA_INCLUDE_DIR "AVTVIMBA_INCLUDE_DIR_NOTFOUND" CACHE PATH "path to AVT Vimba SDK directory")
    endif(EXISTS $ENV{AVTVIMBA_ROOT})
endif(NOT EXISTS ${AVTVIMBA_INCLUDE_DIR})

if(EXISTS ${AVTVIMBA_INCLUDE_DIR})
    find_file( AVT_VIMBA_HEADER_FILE VimbaCPP.h PATHS ${AVTVIMBA_INCLUDE_DIR})
endif(EXISTS ${AVTVIMBA_INCLUDE_DIR})

if(AVT_VIMBA_HEADER_FILE)
    
    message(STATUS "Hello AVT")

    set(AVTVIMBA_API_DIR ${AVTVIMBA_INCLUDE_DIR}/../ CACHE PATH "Base directory of VimbaCPP SDK")

    include_directories(
        ${CMAKE_CURRENT_BINARY_DIR} #build directory of this plugin (recommended)
        ${CMAKE_CURRENT_SOURCE_DIR} #source directory of this plugin (recommended)
        ${ITOM_SDK_INCLUDE_DIRS}    #include directory of the itom SDK (recommended) 
        ${AVTVIMBA_INCLUDE_DIR}/../..
        #add further include directories here
    )

    link_directories(
        #add all linker directories
    )

    set(PLUGIN_HEADERS
        ${CMAKE_CURRENT_SOURCE_DIR}/avtVimba.h
        ${CMAKE_CURRENT_SOURCE_DIR}/dialogAvtVimba.h
        ${CMAKE_CURRENT_SOURCE_DIR}/dockWidgetAvtVimba.h
        ${CMAKE_CURRENT_SOURCE_DIR}/avtEnums.h
        ${CMAKE_CURRENT_SOURCE_DIR}/pluginVersion.h
        ${CMAKE_CURRENT_BINARY_DIR}/gitVersion.h
        #add further header files (absolute pathes e.g. using CMAKE_CURRENT_SOURCE_DIR)
    )

    set(PLUGIN_SOURCES
        ${CMAKE_CURRENT_SOURCE_DIR}/dialogAvtVimba.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/dockWidgetAvtVimba.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/avtVimba.cpp
        #add further source files here
    )

    if(MSVC)
        list(APPEND PLUGIN_SOURCES ${ITOM_SDK_INCLUDE_DIR}/../pluginLibraryVersion.rc)
    endif()

    set(PLUGIN_UI
        ${CMAKE_CURRENT_SOURCE_DIR}/dialogAvtVimba.ui
        ${CMAKE_CURRENT_SOURCE_DIR}/dockWidgetAvtVimba.ui
    )

    set(PLUGIN_RCC
        #add absolute pathes to any *.qrc resource files here
    )
    
    if(BUILD_TARGET64)
        set(AVT_LIBDIR_HINT "${AVTVIMBA_API_DIR}/lib/win64")
        set(AVT_BINDIR_HINT "${AVTVIMBA_API_DIR}/bin/win64")

    else(BUILD_TARGET64)
        set(AVT_LIBDIR_HINT "${AVTVIMBA_API_DIR}/lib/win32")
        set(AVT_BINDIR_HINT "${AVTVIMBA_API_DIR}/bin/win32")

    endif(BUILD_TARGET64)
    find_library(AVTVIMBA_LIBRARY NAMES "VimbaCPP"  PATHS ${AVT_LIBDIR_HINT} NO_DEFAULT_PATH)

    
    if(AVTVIMBA_LIBRARY)
        message(STATUS "AVTVimba library found at ${AVTVIMBA_LIBRARY}")
    else(AVTVIMBA_LIBRARY)
        message(FATAL "AVTVimba library not found")
    endif(AVTVIMBA_LIBRARY)
    
    if(WIN32)
        find_file(AVTVIMBA_BINARY_C "VimbaC.dll"  PATHS ${AVT_BINDIR_HINT} NO_DEFAULT_PATH)
        find_file(AVTVIMBA_BINARY_CPP "VimbaCPP.dll"  PATHS ${AVT_BINDIR_HINT} NO_DEFAULT_PATH)
    endif(WIN32)
    
        
    if(AVTVIMBA_BINARY_C)
        message(STATUS "AVTVimba binary C found at ${AVTVIMBA_BINARY_C}")
    else(AVTVIMBA_BINARY_C)
        message(FATAL "AVTVimba binary C not found")
    endif(AVTVIMBA_BINARY_C)

    add_library(${target_name} SHARED ${PLUGIN_SOURCES} ${PLUGIN_HEADERS} ${PLUGIN_UI} ${PLUGIN_RCC})

    set_target_properties(${target_name} PROPERTIES AUTOMOC ON AUTORCC ON AUTOUIC ON)
    
    target_link_libraries(${target_name} ${ITOM_SDK_LIBRARIES} ${QT5_LIBRARIES} ${VISUALLEAKDETECTOR_LIBRARIES} ${AVTVIMBA_LIBRARY})
    
    set(FILES_TO_TRANSLATE ${PLUGIN_SOURCES} ${PLUGIN_HEADERS} ${PLUGIN_UI})
    itom_library_translation(QM_FILES TARGET ${target_name} FILES_TO_TRANSLATE ${FILES_TO_TRANSLATE})

    itom_configure_plugin_documentation(${target_name} avtVimba)

    set(COPY_SOURCES "")
    set(COPY_DESTINATIONS "")

    itom_add_pluginlibrary_to_copy_list(${target_name} COPY_SOURCES COPY_DESTINATIONS)
    itom_add_plugin_qm_files_to_copy_list(${target_name} QM_FILES COPY_SOURCES COPY_DESTINATIONS)

    itom_post_build_copy_files(${target_name} COPY_SOURCES COPY_DESTINATIONS)

    itom_post_build_copy_files_to_lib_folder(${target_name} AVTVIMBA_BINARY_C)
    itom_post_build_copy_files_to_lib_folder(${target_name} AVTVIMBA_BINARY_CPP)
    
else(AVT_VIMBA_HEADER_FILE)
    message(SEND_ERROR "${target_name}-API directory could not be found. ${target_name} will not be build")    
endif(AVT_VIMBA_HEADER_FILE)
