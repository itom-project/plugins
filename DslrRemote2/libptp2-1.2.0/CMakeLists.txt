cmake_minimum_required(VERSION 3.12...3.24)

set(target_name libptp2)

option(BUILD_UNICODE "Build with unicode charset if set to ON, else multibyte charset." ON)
option(BUILD_SHARED_LIBS "Build shared library." ON)
option(BUILD_TARGET64 "Build for 64 bit target if set to ON or 32 bit if set to OFF." ON)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR})

if(BUILD_SHARED_LIBS)
    set(LIBRARY_TYPE SHARED)
else(BUILD_SHARED_LIBS)
    set(LIBRARY_TYPE STATIC)
endif(BUILD_SHARED_LIBS)


if(LibUSB_INCLUDE_DIRS)

    if(BUILD_UNICODE)
            endif(BUILD_UNICODE)
    add_definitions(-DPTP_DLL)




    message(STATUS ${CMAKE_CURRENT_BINARY_DIR})

    include_directories(
        ${CMAKE_CURRENT_BINARY_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}/src
        )

    set(PLUGIN_HEADERS
        ${CMAKE_CURRENT_SOURCE_DIR}/src/libptp-endian.h
        ${CMAKE_CURRENT_SOURCE_DIR}/src/libptp-stdint.h
        ${CMAKE_CURRENT_SOURCE_DIR}/src/ptp.h
        #${CMAKE_CURRENT_SOURCE_DIR}/src/ptpcam.h
    )

    set(PLUGIN_SOURCES
        ${CMAKE_CURRENT_SOURCE_DIR}/src/properties.c
        ${CMAKE_CURRENT_SOURCE_DIR}/src/ptp.c
        #${CMAKE_CURRENT_SOURCE_DIR}/myusb.c
        #${CMAKE_CURRENT_SOURCE_DIR}/ptpcam.c
        #${CMAKE_CURRENT_SOURCE_DIR}/src/ptp-pack.c
    )

    set_source_files_properties( ${PLUGIN_SOURCES} PROPERTIES LANGUAGE CXX )

    add_library(${target_name} ${LIBRARY_TYPE} ${PLUGIN_SOURCES} ${PLUGIN_HEADERS})

    if(WIN32)
        target_link_libraries(${target_name} ${VISUALLEAKDETECTOR_LIBRARIES} wsock32)
    else(WIN32)
        target_link_libraries(${target_name} ${VISUALLEAKDETECTOR_LIBRARIES})
    endif(WIN32)

    LIST(APPEND COPY_SOURCES "$<TARGET_FILE:${target_name}>")
    LIST(APPEND COPY_DESTINATIONS "${ITOM_APP_DIR}/lib")
    itom_post_build_copy_files(${target_name} COPY_SOURCES COPY_DESTINATIONS)

else(LibUSB_INCLUDE_DIRS)
    message(WARNING "LibUSB_INCLUDE_DIRS directory could not be found. ${target_name} will not be build.")
endif(LibUSB_INCLUDE_DIRS)
