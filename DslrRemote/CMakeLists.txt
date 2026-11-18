cmake_minimum_required(VERSION 3.12...3.29)
set(target_name DslrRemote)

option(BUILD_UNICODE "Build with unicode charset if set to ON, else multibyte charset." ON)
option(BUILD_SHARED_LIBS "Build shared library." ON)
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

find_package(ITOM_SDK COMPONENTS dataobject itomCommonLib itomCommonQtLib REQUIRED)

include(ItomBuildMacros)
itom_init_cmake_policy(3.12)
itom_init_plugin_library(${target_name}) #Start the project, init compiler settings and set default configurations for plugins
itom_find_package_qt(ON Core Widgets LinguistTools)

if(BUILD_SHARED_LIBS)
    set(LIBRARY_TYPE SHARED)
else(BUILD_SHARED_LIBS)
    set(LIBRARY_TYPE STATIC)
endif(BUILD_SHARED_LIBS)


if(WIN32)
    add_subdirectory(regex)
    add_subdirectory(libtool)
    add_subdirectory(libgphoto2)
    add_subdirectory(libgphoto2_port)
    add_subdirectory(camlibs)
else()
    find_package(GPhoto REQUIRED)
endif()

if(WIN32 OR LIBGPHOTO_FOUND)
    if(BUILD_UNICODE)
            endif(BUILD_UNICODE)
    add_definitions(-DCMAKE)

    message(STATUS ${CMAKE_CURRENT_BINARY_DIR})

    include("CreateEndianHeader.cmake")
    include_directories(
        ${CMAKE_CURRENT_BINARY_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}
if(WIN32)
    ${CMAKE_CURRENT_SOURCE_DIR}/libgphoto2_port
else(WIN32)
    ${LIBGPHOTO_INCLUDE_DIR}
endif(WIN32)
        ${ITOM_SDK_INCLUDE_DIRS}
        )

    set(PLUGIN_HEADERS
        ${CMAKE_CURRENT_BINARY_DIR}/pluginVersion.h
        ${CMAKE_CURRENT_BINARY_DIR}/gitVersion.h
        ${CMAKE_CURRENT_SOURCE_DIR}/DslrRemote.h
    )

    set(PLUGIN_UI
    )

    set(PLUGIN_SOURCES
        ${CMAKE_CURRENT_SOURCE_DIR}/DslrRemote.cpp
    )

    #Define Version in pluginVersion.h
    configure_file( ${CMAKE_CURRENT_SOURCE_DIR}/pluginVersion.h.in ${CMAKE_CURRENT_BINARY_DIR}/pluginVersion.h)

    set(PLUGIN_RCC
        #add absolute paths to any *.qrc resource files here
    )

    #Add version information to the plugIn-dll under MSVC
    if(MSVC)
        list(APPEND PLUGIN_SOURCES ${ITOM_SDK_INCLUDE_DIR}/../pluginLibraryVersion.rc)
    endif()

    add_library(${target_name} ${LIBRARY_TYPE} ${PLUGIN_SOURCES} ${PLUGIN_HEADERS})

    # Qt: enable all automoc, autouic and autorcc.
    set_target_properties(${target_name} PROPERTIES AUTOMOC ON AUTORCC ON AUTOUIC ON)

if(WIN32)
    set(LTDLLIB optimized ./libtool/$(Configuration)/libltdl debug ./libtool/$(Configuration)/libltdld)
    set(REGEXLIB optimized ./regex/$(Configuration)/regex debug ./regex/$(Configuration)/regexd)
    set(LIBGPHOTO2 optimized ./libgphoto2/$(Configuration)/libgphoto2 debug ./libgphoto2/$(Configuration)/libgphoto2d)
    set(LIBGPHOTO2PORT optimized ./libgphoto2_port/$(Configuration)/libgphoto2_port debug ./libgphoto2_port/$(Configuration)/libgphoto2_portd)

    target_link_libraries(${target_name} ${ITOM_SDK_LIBRARIES} ${QT5_LIBRARIES} ${VISUALLEAKDETECTOR_LIBRARIES} ${QT_LIBRARIES} ${LTDLLIB} ${REGEXLIB} ${LIBGPHOTO2} ${LIBGPHOTO2PORT})
else()
    target_link_libraries(${target_name} ${ITOM_SDK_LIBRARIES} ${QT5_LIBRARIES} ${VISUALLEAKDETECTOR_LIBRARIES} ${QT_LIBRARIES} ${LIBGPHOTO_LIBRARIES})
endif()

add_dependencies(${target_name} LibUSB)

if(WIN32)
    add_dependencies(${target_name} regex)
    add_dependencies(${target_name} libltdl)
    add_dependencies(${target_name} libgphoto2_port)
    add_dependencies(${target_name} libgphoto2)
endif()

    #translation
    set(FILES_TO_TRANSLATE ${PLUGIN_SOURCES} ${PLUGIN_HEADERS} ${PLUGIN_UI})
    itom_library_translation(QM_FILES TARGET ${target_name} FILES_TO_TRANSLATE ${FILES_TO_TRANSLATE})

    #documentation
    itom_configure_plugin_documentation(${target_name} DslrRemote)

    # COPY SECTION
    set(COPY_SOURCES "")
    set(COPY_DESTINATIONS "")
    #LIST(APPEND COPY_SOURCES "$<TARGET_FILE:dcraw>")
    #LIST(APPEND COPY_DESTINATIONS "${ITOM_APP_DIR}/lib")
    itom_add_pluginlibrary_to_copy_list(${target_name} COPY_SOURCES COPY_DESTINATIONS)
    itom_add_plugin_qm_files_to_copy_list(${target_name} QM_FILES COPY_SOURCES COPY_DESTINATIONS)
    itom_post_build_copy_files(${target_name} COPY_SOURCES COPY_DESTINATIONS)

else(WIN32 OR LIBGPHOTO_FOUND)
    message(SEND_ERROR "LIBGPHOTO_INCLUDE_DIR directory could not be found. ${target_name} will not be build.")
endif()
