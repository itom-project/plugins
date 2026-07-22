cmake_minimum_required(VERSION 3.12...3.29)
set(target_name OpenCVFilters)

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

# Require minimal OpenCV modules and probe optional ones (OpenCV 5 may not provide
# every module; enable source files conditionally).
find_package(OpenCV COMPONENTS core imgproc REQUIRED)
# Probe optional modules; QUIET so CMake won't fail if they're missing
find_package(OpenCV COMPONENTS calib3d flann features2d QUIET)

# Configure which optional modules are present
set(OPENCV_HAVE_CALIB3D OFF)
set(OPENCV_HAVE_FEATURES2D OFF)
set(OPENCV_HAVE_FLANN OFF)
if(TARGET OpenCV::calib3d)
    set(OPENCV_HAVE_CALIB3D ON)
endif()
if(TARGET OpenCV::features2d)
    set(OPENCV_HAVE_FEATURES2D ON)
endif()
if(TARGET OpenCV::flann)
    set(OPENCV_HAVE_FLANN ON)
endif()

# Build list of OpenCV targets to link
set(OPENCV_TARGETS OpenCV::core OpenCV::imgproc)
if(OPENCV_HAVE_CALIB3D)
    list(APPEND OPENCV_TARGETS OpenCV::calib3d)
endif()
if(OPENCV_HAVE_FEATURES2D)
    list(APPEND OPENCV_TARGETS OpenCV::features2d)
endif()
if(OPENCV_HAVE_FLANN)
    list(APPEND OPENCV_TARGETS OpenCV::flann)
endif()

# Prefer imported OpenCV targets when available; otherwise fall back to
# the legacy `${OpenCV_LIBS}` variable. Also add include dirs if provided.
if(TARGET OpenCV::core)
    set(OPENCV_LINK_LIBS ${OPENCV_TARGETS})
else()
    if(DEFINED OpenCV_LIBS)
        set(OPENCV_LINK_LIBS ${OpenCV_LIBS})
    else()
        set(OPENCV_LINK_LIBS "")
    endif()
endif()

if(DEFINED OpenCV_INCLUDE_DIRS)
    include_directories(${OpenCV_INCLUDE_DIRS})
endif()

include(ItomBuildMacros)
itom_init_cmake_policy(3.12)
itom_init_plugin_library(${target_name}) #Start the project, init compiler settings and set default configurations for plugins
itom_find_package_qt(ON Widgets Xml LinguistTools)

include_directories(
    ${CMAKE_CURRENT_BINARY_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${ITOM_SDK_INCLUDE_DIRS}
)

link_directories(
)

set(PLUGIN_HEADERS
    ${CMAKE_CURRENT_SOURCE_DIR}/OpenCVFilters.h
    ${CMAKE_CURRENT_SOURCE_DIR}/itomCvConversions.h
    ${CMAKE_CURRENT_BINARY_DIR}/pluginVersion.h
    ${CMAKE_CURRENT_BINARY_DIR}/gitVersion.h
)

set(PLUGIN_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/OpenCVFilters.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/itomCvConversions.cpp
)

# Add optional source files only if their OpenCV modules are available
if(OPENCV_HAVE_CALIB3D)
    list(APPEND PLUGIN_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/calib3d.cpp)
else()
    message(WARNING "OpenCV calib3d module not found; skipping calib3d-based filters")
endif()
if(OPENCV_HAVE_FEATURES2D)
    list(APPEND PLUGIN_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/features2d.cpp)
else()
    message(WARNING "OpenCV features2d module not found; skipping features2d-based filters")
endif()

#Define Version in pluginVersion.h
configure_file( ${CMAKE_CURRENT_SOURCE_DIR}/pluginVersion.h.in ${CMAKE_CURRENT_BINARY_DIR}/pluginVersion.h)

set(PLUGIN_UI
)

set(PLUGIN_RCC
)

#Add version information to the plugIn-dll under MSVC
if(MSVC)
    list(APPEND PLUGIN_SOURCES ${ITOM_SDK_INCLUDE_DIR}/../pluginLibraryVersion.rc)
endif()

add_library(${target_name} SHARED ${PLUGIN_SOURCES} ${PLUGIN_HEADERS} ${PLUGIN_UI} ${PLUGIN_RCC})

# Qt: enable all automoc, autouic and autorcc.
set_target_properties(${target_name} PROPERTIES AUTOMOC ON AUTORCC ON AUTOUIC ON)

target_link_libraries(${target_name} ${ITOM_SDK_LIBRARIES} ${OPENCV_LINK_LIBS} ${QT5_LIBRARIES} ${VISUALLEAKDETECTOR_LIBRARIES})

#translation
set(FILES_TO_TRANSLATE ${PLUGIN_SOURCES} ${PLUGIN_HEADERS} ${PLUGIN_UI})
itom_library_translation(QM_FILES TARGET ${target_name} FILES_TO_TRANSLATE ${FILES_TO_TRANSLATE})

itom_configure_plugin_documentation(${target_name} openCVFilters "OpenCV-Filters")

# COPY SECTION
set(COPY_SOURCES "")
set(COPY_DESTINATIONS "")
itom_add_pluginlibrary_to_copy_list(${target_name} COPY_SOURCES COPY_DESTINATIONS)
itom_add_plugin_qm_files_to_copy_list(${target_name} QM_FILES COPY_SOURCES COPY_DESTINATIONS)
itom_post_build_copy_files(${target_name} COPY_SOURCES COPY_DESTINATIONS)
