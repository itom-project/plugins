cmake_minimum_required(VERSION 3.12...3.24)

set(target_name libmodbus_library)

project(${target_name})

message(STATUS "-------- LIBMODBUS 3.1.2 LIBRARY ----------")

option(BUILD_LIBMODBUS_SHARED "Build libmodbus as shared library (DLL) or static library (must be OFF)" OFF)

if(BUILD_LIBMODBUS_SHARED)
    message(SEND_ERROR "Currently only BUILD_LIBMODBUS_SHARED=OFF is supported")
    set(LIBRARY_TYPE SHARED)
    add_definitions(-DDLLBUILD)
else(BUILD_LIBMODBUS_SHARED)
    add_definitions(-DLIBMODBUSSTATIC)
    set(LIBRARY_TYPE STATIC)

    #if itomCommon is static, add -fPIC as compiler flag for linux
    if(UNIX)
      set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")
      set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fPIC")
    endif(UNIX)
endif(BUILD_LIBMODBUS_SHARED)

if(WIN32)
    set(LIBRARIES "Ws2_32")
endif(WIN32)

#configure_file(
#    ${CMAKE_CURRENT_SOURCE_DIR}/src/modbus-version.h.in
#    ${CMAKE_CURRENT_BINARY_DIR}/modbus-version.h
#    )

configure_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/config.h.in
    ${CMAKE_CURRENT_BINARY_DIR}/config.h
    )

include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_BINARY_DIR}
)

set(libmodbus_HEADERS
    ${CMAKE_CURRENT_SOURCE_DIR}/src/modbus-version.h
    ${CMAKE_CURRENT_BINARY_DIR}/config.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/modbus.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/modbus-private.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/modbus-rtu.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/modbus-rtu-private.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/modbus-tcp.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/modbus-tcp-private.h
)

set(libmodbus_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/src/modbus.c
    ${CMAKE_CURRENT_SOURCE_DIR}/src/modbus-data.c
    ${CMAKE_CURRENT_SOURCE_DIR}/src/modbus-rtu.c
    ${CMAKE_CURRENT_SOURCE_DIR}/src/modbus-tcp.c
)

add_library(${target_name} ${LIBRARY_TYPE} ${libmodbus_SOURCES} ${libmodbus_HEADERS})

target_link_libraries(${target_name} ${LIBRARIES})

#
#if("${CMAKE_SIZEOF_VOID_P}" EQUAL "8")
#    message(STATUS "Target is 64 bits")
#    if(WIN32)
#        set(DEST lib/win64)
#    else(WIN32)
#        set(DEST lib/linux64)
#    endif(WIN32)
#else("${CMAKE_SIZEOF_VOID_P}" EQUAL "8")
#    message(STATUS "Target is 32 bits")
#    if(WIN32)
#        set(DEST lib/win32)
#    else(WIN32)
#        set(DEST lib/linux32)
#    endif(WIN32)
#endif("${CMAKE_SIZEOF_VOID_P}" EQUAL "8")

#if(WIN32)
#    set(DEST win32)
#else(WIN32)
#endif(WIN32)
#INSTALL(TARGETS ${target_name} DESTINATION ${DEST} INCLUDES DESTINATION src)
#foreach(ITEM ${libmodbus_HEADERS})
#  install(FILES ${ITEM} DESTINATION "include")
#endforeach()
