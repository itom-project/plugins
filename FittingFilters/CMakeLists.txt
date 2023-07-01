cmake_minimum_required(VERSION 3.12...3.24)

set(target_name FittingFilters)

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

find_package(ITOM_SDK COMPONENTS dataobject itomCommonLib itomCommonQtLib REQUIRED)
find_package(OpenCV COMPONENTS core imgproc REQUIRED)
find_package(LAPACKE)

include(ItomBuildMacros)
itom_init_cmake_policy(3.12)
itom_init_plugin_library(${target_name}) #Start the project, init compiler settings and set default configurations for plugins
itom_find_package_qt(ON Widgets Xml LinguistTools)

find_package( OpenMP QUIET)
if(OPENMP_FOUND)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

include_directories(
    ${CMAKE_CURRENT_BINARY_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${ITOM_SDK_INCLUDE_DIRS}
)

set(PLUGIN_HEADERS
    ${CMAKE_CURRENT_SOURCE_DIR}/fittingfilters.h
    ${CMAKE_CURRENT_SOURCE_DIR}/numericalRecipes.h
    ${CMAKE_CURRENT_SOURCE_DIR}/pluginVersion.h
    ${CMAKE_CURRENT_BINARY_DIR}/gitVersion.h
)

if(${LAPACKE_FOUND})
    set(PLUGIN_HEADERS ${PLUGIN_HEADERS}
        ${LAPACKE_INCLUDE_DIR}/lapacke.h
    )

    include_directories(
        ${LAPACKE_INCLUDE_DIR}
    )

    add_definitions(-DLAPACKE -DADD_ -DHAVE_LAPACK_CONFIG_H -DLAPACK_COMPLEX_STRUCTURE )
endif()

set(PLUGIN_UI
)

set(PLUGIN_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/fittingfilters.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/polyfit2d.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/polyfitweighted2d.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/numericalRecipes.cpp
)

#Add version information to the plugIn-dll unter MSVC
if(MSVC)
    list(APPEND PLUGIN_SOURCES ${ITOM_SDK_INCLUDE_DIR}/../pluginLibraryVersion.rc)
endif()




add_library(${target_name} SHARED ${PLUGIN_SOURCES} ${PLUGIN_HEADERS} ${PLUGIN_UI} ${PLUGIN_RCC})

# Qt: enable all automoc, autouic and autorcc.
set_target_properties(${target_name} PROPERTIES AUTOMOC ON AUTORCC ON AUTOUIC ON)

target_link_libraries(${target_name} ${ITOM_SDK_LIBRARIES} ${OpenCV_LIBS} ${QT5_LIBRARIES} ${VISUALLEAKDETECTOR_LIBRARIES} ${LAPACKE_LIBRARIES})



#translation
set(FILES_TO_TRANSLATE ${PLUGIN_SOURCES} ${PLUGIN_HEADERS} ${PLUGIN_UI})
itom_library_translation(QM_FILES TARGET ${target_name} FILES_TO_TRANSLATE ${FILES_TO_TRANSLATE})

itom_configure_plugin_documentation(${target_name} fittingFilters)

# COPY SECTION
set(COPY_SOURCES "")
set(COPY_DESTINATIONS "")

message(STATUS "${LAPACKE_FOUND}  - ${LAPACKE_RUNTIME_LIBRARIES} - ${LAPACKE_LIBRARIES}")
if(WIN32)
    if(${LAPACKE_FOUND})
    foreach(__LIB ${LAPACKE_RUNTIME_LIBRARIES})
        itom_post_build_copy_files_to_lib_folder(${target_name} __LIB)
        message(STATUS ${__LIB})
    endforeach(__LIB)
    endif(${LAPACKE_FOUND})
endif(WIN32)

itom_add_pluginlibrary_to_copy_list(${target_name} COPY_SOURCES COPY_DESTINATIONS)
itom_add_plugin_qm_files_to_copy_list(${target_name} QM_FILES COPY_SOURCES COPY_DESTINATIONS)
itom_post_build_copy_files(${target_name} COPY_SOURCES COPY_DESTINATIONS)
