cmake_minimum_required(VERSION 3.12...3.29)
project(zlibwapi)

set(LIBRARY_TYPE STATIC)

#if minizip is static, add -fPIC as compiler flag for linux
if(UNIX)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fPIC")
endif(UNIX)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR})

if(WIN32)
   add_definitions(-D_CRT_NONSTDC_NO_DEPRECATE -D_CRT_SECURE_NO_DEPRECATE -D_CRT_NONSTDC_NO_WARNINGS -DZLIB_WINAPI)
endif()

# default build types are None, Debug, Release, RelWithDebInfo and MinRelSize
if(DEFINED CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE ${CMAKE_BUILD_TYPE} CACHE STRING "Choose the type of build, options are: None(CMAKE_CXX_FLAGS or CMAKE_C_FLAGS used) Debug Release RelWithDebInfo MinSizeRel.")
else()
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build, options are: None(CMAKE_CXX_FLAGS or CMAKE_C_FLAGS used) Debug Release RelWithDebInfo MinSizeRel.")
endif()

include_directories( "${PROJECT_SOURCE_DIR}"
      "${PROJECT_SOURCE_DIR}/../../"
)

#if(NOT CMAKE_ASM_MASM_COMPILER_WORKS)
#    message(FATAL_ERROR "No assembler found!")
#endif()
if(MSVC)
    enable_language(ASM_MASM)
    add_definitions(-DASMV -DASMINF)

    if(CMAKE_CL_64)
        add_custom_command(OUTPUT inffasx64.obj
        COMMAND ${CMAKE_ASM_MASM_COMPILER} -c
            ${CMAKE_CURRENT_SOURCE_DIR}/../masmx64/inffasx64.asm
        DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/../masmx64/inffasx64.asm
        COMMENT "generate inffasx64.obj")

        add_custom_command(OUTPUT gvmat64.obj
        COMMAND ${CMAKE_ASM_MASM_COMPILER} -c
            ${CMAKE_CURRENT_SOURCE_DIR}/../masmx64/gvmat64.asm
        DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/../masmx64/gvmat64.asm
        COMMENT "generate gvmat64.obj")

        set(libSrcs
            #${CMAKE_CURRENT_SOURCE_DIR}/../masmx64/inffasx64.asm
            #${CMAKE_CURRENT_SOURCE_DIR}/../masmx64/gvmat64.asm
            inffasx64.obj
            gvmat64.obj
            ${CMAKE_CURRENT_SOURCE_DIR}/../masmx64/inffas8664.c
        )
    else(CMAKE_CL_64)
        add_custom_command(OUTPUT inffas32.obj
        COMMAND ${CMAKE_ASM_MASM_COMPILER} -c
            ${CMAKE_CURRENT_SOURCE_DIR}/../masmx86/inffas32.asm
        DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/../masmx86/inffas32.asm
        COMMENT "generate inffas32.obj")

        add_custom_command(OUTPUT match686.obj
        COMMAND ${CMAKE_ASM_MASM_COMPILER} -c
            ${CMAKE_CURRENT_SOURCE_DIR}/../masmx86/match686.asm
        DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/../masmx86/match686.asm
        COMMENT "generate match686.obj")

        set(libSrcs
            #${CMAKE_CURRENT_SOURCE_DIR}/../masmx86/inffas32.asm
            #${CMAKE_CURRENT_SOURCE_DIR}/../masmx86/match686.asm
            inffas32.obj
            match686.obj
        )
    endif(CMAKE_CL_64)
else(MSVC)
    #enable_language(ASM_MASM)
    message(STATUS "MINIZIP: No MSVC: therefore no assembler compilation enabled")
endif()

set(libSrcs
        ${libSrcs}
        ${CMAKE_CURRENT_SOURCE_DIR}/ioapi.c
        ${CMAKE_CURRENT_SOURCE_DIR}/unzip.c
        ${CMAKE_CURRENT_SOURCE_DIR}/zip.c
        ${CMAKE_CURRENT_SOURCE_DIR}/mztools.c
)

if(WIN32)
  set(libSrcs
    ${libSrcs}
        ${CMAKE_CURRENT_SOURCE_DIR}/../../adler32.c
        ${CMAKE_CURRENT_SOURCE_DIR}/../../compress.c
        ${CMAKE_CURRENT_SOURCE_DIR}/../../crc32.c
        ${CMAKE_CURRENT_SOURCE_DIR}/../../deflate.c
        ${CMAKE_CURRENT_SOURCE_DIR}/../../gzclose.c
        ${CMAKE_CURRENT_SOURCE_DIR}/../../gzlib.c
        ${CMAKE_CURRENT_SOURCE_DIR}/../../gzread.c
        ${CMAKE_CURRENT_SOURCE_DIR}/../../gzwrite.c
        ${CMAKE_CURRENT_SOURCE_DIR}/../../infback.c
        ${CMAKE_CURRENT_SOURCE_DIR}/../../inffast.c
        ${CMAKE_CURRENT_SOURCE_DIR}/../../inflate.c
        ${CMAKE_CURRENT_SOURCE_DIR}/../../inftrees.c
        ${CMAKE_CURRENT_SOURCE_DIR}/../../minigzip.c
        ${CMAKE_CURRENT_SOURCE_DIR}/../../trees.c
        ${CMAKE_CURRENT_SOURCE_DIR}/../../uncompr.c
        ${CMAKE_CURRENT_SOURCE_DIR}/../../zutil.c
        ${CMAKE_CURRENT_SOURCE_DIR}/iowin32.c
#        ${CMAKE_CURRENT_SOURCE_DIR}/../vstudio/vc10/zlibvc.def
  )
else()
#  set(libSrcs
#    ${libSrcs}
#    ${CMAKE_CURRENT_SOURCE_DIR}/minizip.c
#    ${CMAKE_CURRENT_SOURCE_DIR}/miniunz.c
#  )
endif()

add_library(zlibwapi ${LIBRARY_TYPE} ${libSrcs})

if(DEFINED MSVC_VERSION)
  set_target_properties(zlibwapi PROPERTIES COMPILE_FLAGS "/Zc:wchar_t")
endif()


install(TARGETS  zlibwapi
       RUNTIME DESTINATION bin
       ARCHIVE DESTINATION iso5436-2-xml/lib
    )
