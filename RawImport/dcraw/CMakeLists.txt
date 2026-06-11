cmake_minimum_required(VERSION 3.12...3.29)
set(target_name dcraw)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR} ${ITOM_SDK_DIR}/cmake)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

include_directories(
    ${CMAKE_CURRENT_BINARY_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}
)

set(PLUGIN_HEADERS
    ${CMAKE_CURRENT_SOURCE_DIR}/dcraw.h
)

set(PLUGIN_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/dcraw.c
)

add_executable(${target_name} ${PLUGIN_SOURCES} ${PLUGIN_HEADERS})

if(WIN32)
    target_link_libraries(${target_name})
else(WIN32)
    target_link_libraries(${target_name} m)
endif(WIN32)
