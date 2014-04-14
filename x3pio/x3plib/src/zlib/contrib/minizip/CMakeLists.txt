project(zlibwapi)

cmake_minimum_required(VERSION 2.8)


#IF(BUILD_SHARED_LIBS)
#  SET(LIBRARY_TYPE SHARED)
#  add_definitions(-DZLIB_DLL )
#ELSE(BUILD_SHARED_LIBS)
    #always compile as static library
    SET(LIBRARY_TYPE STATIC) 
#ENDIF(BUILD_SHARED_LIBS)

#if minizip is static, add -fPIC as compiler flag for linux
IF(UNIX)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fPIC")
ENDIF(UNIX)

SET (CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR})

add_definitions(-DUNICODE -D_UNICODE)
if (WIN32)
   add_definitions(-D_CRT_NONSTDC_NO_DEPRECATE -D_CRT_SECURE_NO_DEPRECATE -D_CRT_NONSTDC_NO_WARNINGS -DZLIB_WINAPI)
endif()



# default build types are None, Debug, Release, RelWithDebInfo and MinRelSize
IF (DEFINED CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE ${CMAKE_BUILD_TYPE} CACHE STRING "Choose the type of build, options are: None(CMAKE_CXX_FLAGS or CMAKE_C_FLAGS used) Debug Release RelWithDebInfo MinSizeRel.")
ELSE(CMAKE_BUILD_TYPE)
  SET (CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build, options are: None(CMAKE_CXX_FLAGS or CMAKE_C_FLAGS used) Debug Release RelWithDebInfo MinSizeRel.")
ENDIF (DEFINED CMAKE_BUILD_TYPE)

INCLUDE_DIRECTORIES( "${PROJECT_SOURCE_DIR}"
      "${PROJECT_SOURCE_DIR}/../../"
)



#IF(NOT CMAKE_ASM_MASM_COMPILER_WORKS)
#    MESSAGE(FATAL_ERROR "No assembler found!")
#ENDIF()
IF (MSVC)
    ENABLE_LANGUAGE(ASM_MASM)
    add_definitions(-DASMV -DASMINF)
    
    IF(CMAKE_CL_64)
        ADD_CUSTOM_COMMAND(OUTPUT inffasx64.obj
        COMMAND ${CMAKE_ASM_MASM_COMPILER} -c
            ${CMAKE_CURRENT_SOURCE_DIR}/../masmx64/inffasx64.asm
        DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/../masmx64/inffasx64.asm
        COMMENT "generate inffasx64.obj") 

        ADD_CUSTOM_COMMAND(OUTPUT gvmat64.obj
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
    ELSE(CMAKE_CL_64)
        ADD_CUSTOM_COMMAND(OUTPUT inffas32.obj
        COMMAND ${CMAKE_ASM_MASM_COMPILER} -c
            ${CMAKE_CURRENT_SOURCE_DIR}/../masmx86/inffas32.asm
        DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/../masmx86/inffas32.asm
        COMMENT "generate inffas32.obj") 

        ADD_CUSTOM_COMMAND(OUTPUT match686.obj
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
    ENDIF(CMAKE_CL_64)
ELSE (MSVC)
    #ENABLE_LANGUAGE(ASM_MASM)
    MESSAGE(STATUS "MINIZIP: No MSVC: therefore no assembler compilation enabled")
ENDIF(MSVC) 




set(libSrcs 
        ${libSrcs}
        ${CMAKE_CURRENT_SOURCE_DIR}/ioapi.c
        ${CMAKE_CURRENT_SOURCE_DIR}/unzip.c
        ${CMAKE_CURRENT_SOURCE_DIR}/zip.c
        ${CMAKE_CURRENT_SOURCE_DIR}/mztools.c
)

if (WIN32)
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

if (DEFINED MSVC_VERSION)
  set_target_properties(zlibwapi PROPERTIES COMPILE_FLAGS "/Zc:wchar_t")
endif ()


install(TARGETS  zlibwapi 
       RUNTIME DESTINATION bin
       ARCHIVE DESTINATION iso5436-2-xml/lib
    )


