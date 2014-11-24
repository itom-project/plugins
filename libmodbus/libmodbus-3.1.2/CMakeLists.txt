SET (target_name libmodbus_library)
project(${target_name}) 

cmake_minimum_required(VERSION 2.8)
 
OPTION(BUILD_UNICODE "Build with unicode charset if set to ON, else multibyte charset." ON)
OPTION(BUILD_LIBMODBUS_SHARED "Build libmodbus as shared library (DLL) or static library (must be OFF)" OFF)
SET (CMAKE_DEBUG_POSTFIX "d" CACHE STRING "Adds a postfix for debug-built libraries.")

IF(BUILD_LIBMODBUS_SHARED)
    MESSAGE(SEND_ERROR "Currently only BUILD_LIBMODBUS_SHARED=OFF is supported")
    SET(LIBRARY_TYPE SHARED)
    ADD_DEFINITIONS(-DDLLBUILD)
ELSE(BUILD_LIBMODBUS_SHARED)
    ADD_DEFINITIONS(-DLIBMODBUSSTATIC)
    SET(LIBRARY_TYPE STATIC)
    
    #if itomCommon is static, add -fPIC as compiler flag for linux
    IF(UNIX)
      set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")
    ENDIF(UNIX)
ENDIF(BUILD_LIBMODBUS_SHARED)

IF(WIN32)
    SET(LIBRARIES "Ws2_32")
ENDIF(WIN32)

IF (BUILD_UNICODE)
    ADD_DEFINITIONS(-DUNICODE -D_UNICODE)
ENDIF (BUILD_UNICODE)

#configure_file(
#    ${CMAKE_CURRENT_SOURCE_DIR}/src/modbus-version.h.in
#    ${CMAKE_CURRENT_BINARY_DIR}/modbus-version.h
#    )

configure_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/config.h.in
    ${CMAKE_CURRENT_BINARY_DIR}/config.h
    )
    
INCLUDE_DIRECTORIES(
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

TARGET_LINK_LIBRARIES(${target_name} ${LIBRARIES})

#
#if("${CMAKE_SIZEOF_VOID_P}" EQUAL "8")
#    message(STATUS "Target is 64 bits")
#    if (WIN32)
#        SET(DEST lib/win64)
#    else (WIN32)
#        SET(DEST lib/linux64)
#    endif(WIN32)
#else("${CMAKE_SIZEOF_VOID_P}" EQUAL "8")
#    message(STATUS "Target is 32 bits")
#    if (WIN32)         
#        SET(DEST lib/win32)
#    else (WIN32)
#        SET(DEST lib/linux32)
#    endif(WIN32)
#endif("${CMAKE_SIZEOF_VOID_P}" EQUAL "8")

#IF(WIN32)
#    SET(DEST win32)
#ELSE(WIN32)
#ENDIF(WIN32)
#INSTALL(TARGETS ${target_name} DESTINATION ${DEST} INCLUDES DESTINATION src)
#foreach(ITEM ${libmodbus_HEADERS})
#  install(FILES ${ITEM} DESTINATION "include")
#endforeach()