# #########################################################################
# #########################################################################
# CMake FIND file for the proprietary NIDAQmx Windows library (NIDAQmx).
#
# Try to find NIDAQmx
# Once done this will define
# NIDAQMX_FOUND - System has NIDAQmx
# NIDAQMX_LIBRARY - The NIDAQmx library
# NIDAQMX_INCLUDE_DIR - The NIDAQmx include file
# #########################################################################
# #########################################################################
# #########################################################################
# Useful variables

IF( WIN32 )

    if ( CMAKE_SIZEOF_VOID_P EQUAL 4 )
      list(APPEND NIDAQMX_DIR 
        "C:/Program Files (x86)/National Instruments/NI-DAQ/DAQmx ANSI C Dev" 
        "C:/Program Files (x86)/National Instruments/Shared/ExternalCompilerSupport/C")
      SET(SUFFIXES "lib/msvc"
            "lib32/msvc")
    else ( CMAKE_SIZEOF_VOID_P EQUAL 4 )
      list(APPEND NIDAQMX_DIR 
        "C:/Program Files/National Instruments/NI-DAQ/DAQmx ANSI C Dev" 
        "C:/Program Files (x86)/National Instruments/Shared/ExternalCompilerSupport/C")
      SET(SUFFIXES "lib/msvc"
            "lib64/msvc")
    endif ( CMAKE_SIZEOF_VOID_P EQUAL 4 )

    # Find installed library using CMake functions
    find_library(NIDAQMX_LIBRARY
        NAMES "NIDAQmx"
        PATHS ${NIDAQMX_DIR}
        PATH_SUFFIXES ${SUFFIXES})

    find_path(NIDAQMX_INCLUDE_DIR
        NAMES "NIDAQmx.h"
        PATHS ${NIDAQMX_DIR}
        PATH_SUFFIXES "include")

    # Handle the QUIETLY and REQUIRED arguments and set NIDAQMX_FOUND to TRUE if all listed variables are TRUE
    include(FindPackageHandleStandardArgs)
    find_package_handle_standard_args(NIDAQmx DEFAULT_MSG NIDAQMX_LIBRARY NIDAQMX_INCLUDE_DIR)
    # #########################################################################

    # #########################################################################
    mark_as_advanced(NIDAQMX_LIBRARY NIDAQMX_INCLUDE_DIR)
    # #########################################################################

ENDIF( WIN32 )

IF( UNIX )

if( EXISTS "/etc/centos-release" )
   file(READ "/etc/centos-release" centos-release)
   string(FIND centos_release "CentOS Linux release 7.5" match)
   string (COMPARE NOTEQUAL match "-1" success)
   if( success )
      if( EXISTS "/usr/share/ni-daqmx/nidaqmx.version" )
         file(READ "/usr/share/ni-daqmx/nidaqmx.version" nidaqmx-version)
         string (FIND nidaqmx-version "18.1" match)
         string (COMPARE NOTEQUAL match "-1" success)
         if( success )
            if( EXISTS "/usr/include/NIDAQmx.h" )
               set(NIDAQMX_INCLUDE_DIR "/usr/include" CACHE PATH "NIDAQMX include directory" FORCE)
            endif()
            if( EXISTS "/usr/lib/x86_64-linux-gnu/libnidaqmx.so" )
               set(NIDAQMX_LIBRARY "/usr/lib/x86_64-linux-gnu/libnidaqmx.so" CACHE FILEPATH "NIDAQMX library path" FORCE)
            endif()
            if( (DEFINED NIDAQMX_INCLUDE_DIR) AND (DEFINED NIDAQMX_LIBRARY) )
               mark_as_advanced(NIDAQMX_LIBRARY NIDAQMX_INCLUDE_DIR)
               set(NIDAQMX_FOUND ON CACHE BOOL "NIDAQMX 18.1 is installed on machine running Centos 7.5" FORCE)
            endif()
         endif()
      endif()
   endif()
endif()

if( NOT DEFINED NIDAQMX_FOUND )
   message(FATAL_ERROR "For Linux, this plugin only works on Centos 7.5 with NIDAQMX version 18.1")
endif()

ENDIF( UNIX )


