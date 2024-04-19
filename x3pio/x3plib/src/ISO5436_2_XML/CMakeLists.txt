cmake_minimum_required(VERSION 3.12...3.29)
project(ISO5436_2)

option(BUILD_X3P_ISO5436_SHARED "Build ISO5436-XML library as shared library (default: ON)." ON)

if(BUILD_X3P_ISO5436_SHARED)
  set(LIBRARY_TYPE SHARED)
  add_definitions(-DZLIB_DLL -D_DLL -DBUILD_ISO5436_2_XML_DLL -DZLIB_WINAPI)
  # for ZLIB_WINAPI definition, see
  # see http://stackoverflow.com/questions/5424549/unresolved-externals-despite-linking-in-zlib-lib
else()
  set(LIBRARY_TYPE STATIC)
  add_definitions(-D_LIB)
  set(iso54362LIBSUFFIX S)
endif()

if(CMAKE_CL_64)
    set(iso54362LIBSUFFIX ${iso54362LIBSUFFIX}64)
endif(CMAKE_CL_64)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR})

set(iso54362xmlNAME "iso5436-2-xml${iso54362LIBSUFFIX}")

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR} ${ITOM_SDK_DIR}/cmake)

find_package(Xerces REQUIRED)
find_package(XSD REQUIRED)

if(NOT EXISTS ${CMAKE_CURRENT_BINARY_DIR}/opengps/cxx/iso5436_2_xsd.cxx)
    file(WRITE "${CMAKE_CURRENT_BINARY_DIR}/opengps/cxx/iso5436_2_xsd.cxx" "//this file will be overwritten by XSD_SCHEMA" )
endif()
if(NOT EXISTS ${CMAKE_CURRENT_BINARY_DIR}/opengps/cxx/iso5436_2_xsd.hxx)
    file(WRITE "${CMAKE_CURRENT_BINARY_DIR}/opengps/cxx/iso5436_2_xsd.hxx" "//this file will be overwritten by XSD_SCHEMA" )
endif()

XSD_SCHEMA(OUTPATH ${PROJECT_SOURCE_DIR}/iso5436_2.xsd  --prologue-file ${PROJECT_SOURCE_DIR}/xsd_Licence_Header.c --generate-doxygen --generate-ostream --generate-serialization --char-type wchar_t --generate-comparison --generate-from-base-ctor --namespace-map http://www.opengps.eu/2008/ISO5436_2=OpenGPS::Schemas::ISO5436_2 --export-symbol _OPENGPS_EXPORT --cxx-suffix \"_xsd.cxx\" --hxx-suffix \"_xsd.hxx\" --output-dir ${CMAKE_CURRENT_BINARY_DIR}/opengps/cxx) #${PROJECT_SOURCE_DIR}/../../include/opengps/cxx/)

# Subversion is needed to generate version.h from version.in
find_package(Subversion)
if(NOT SUBVERSION_FOUND)
   find_package(TortoiseSVN REQUIRED)
   message(STATUS "Using TortoiseSVN from:")
   message(STATUS "   " ${TortoiseSVN_EXECUTABLE})
   set(Subversion_SVN_EXECUTABLE ${TortoiseSVN_EXECUTABLE})
endif(NOT SUBVERSION_FOUND)
# Find native ZLIB package
# find_package(ZLIB REQUIRED)


add_definitions(-DUNICODE -D_UNICODE -DISO5436_2_LIBRARY -DBUILD_ISO5436_2_XML  -DXSD_FP_ALL_DIGITS)
if(WIN32 AND NOT BUILD_X3P_ISO5436_SHARED)
   add_definitions(-DZLIB_WINAPI)
endif()

include_directories( "${PROJECT_SOURCE_DIR}/../../include"
      "${PROJECT_SOURCE_DIR}/../../../../include/opengps/cxx"
      "${PROJECT_SOURCE_DIR}/../zlib"
      "${PROJECT_SOURCE_DIR}/../zlib/contrib/minizip"
      "${XSD_INCLUDE_DIR}"
      "${XERCESC_INCLUDE}"
)

link_directories(
     "${CMAKE_CURRENT_BINARY_DIR}/../zlib"
     "${CMAKE_CURRENT_BINARY_DIR}/../zlib/contrib/minizip"
)

## Generate the version include file from the .in file
#if(GENVERSION)
#   if(WIN32)
#      # Find subwcrev executable
#      get_filename_component(Subversion_PATH "${Subversion_SVN_EXECUTABLE}" PATH)
#      set(Subwcrev_EXECUTABLE "${Subversion_PATH}/subwcrev.exe" )
#      message(STATUS Subwcrev found: ${Subwcrev_EXECUTABLE})
#      # subwcrev $(ProjectDir) $(InputDir)/cxx/Version.in $(InputDir)/cxx/Version.h
#      add_custom_command(OUTPUT "${CMAKE_CURRENT_SOURCE_DIR}/cxx/version.h"
#                          PRE_BUILD
#                          COMMAND "${Subwcrev_EXECUTABLE}"
#                          ARGS "${PROJECT_SOURCE_DIR}"
#                               "${CMAKE_CURRENT_SOURCE_DIR}/cxx/version.in"
#                               "${CMAKE_CURRENT_SOURCE_DIR}/cxx/version.h"
#                          MAIN_DEPENDENCY "${CMAKE_CURRENT_SOURCE_DIR}/cxx/version.in"
#                          DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}"
#                          COMMENT generating cxx/version.h )
#   endif(WIN32)
#   if(UNIX)
#      # on mac and linux we need the included replacement script
#      add_custom_command(OUTPUT "${CMAKE_CURRENT_SOURCE_DIR}/cxx/version.h"
#                          PRE_BUILD
#                          COMMAND "${PROJECT_SOURCE_DIR}/../tools/subwcrev.sh"
#                          ARGS "${PROJECT_SOURCE_DIR}"
#                               "${CMAKE_CURRENT_SOURCE_DIR}/cxx/version.in"
#                               "${CMAKE_CURRENT_SOURCE_DIR}/cxx/version.h"
#                          MAIN_DEPENDENCY "${CMAKE_CURRENT_SOURCE_DIR}/cxx/version.in"
#                          DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}"
#                          COMMENT generating cxx/version.h )
#   endif(UNIX)
#endif(GENVERSION)

## Now publish the version file to public include dir
#add_custom_command(OUTPUT "${CMAKE_CURRENT_SOURCE_DIR}/../../include/opengps/cxx/version.h"
#                    PRE_BUILD
#                    COMMAND "${CMAKE_COMMAND}" -E copy
#                    ARGS "${CMAKE_CURRENT_SOURCE_DIR}/cxx/version.h"
#                         "${CMAKE_CURRENT_SOURCE_DIR}/../../include/opengps/cxx/"
#                    MAIN_DEPENDENCY "${CMAKE_CURRENT_SOURCE_DIR}/cxx/version.h"
#                    DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/cxx/version.h"
#                    COMMENT publishing cxx/version.h )


set(iso54362xml_HEADER
        ${CMAKE_CURRENT_SOURCE_DIR}/c/data_point_c.hxx
        ${CMAKE_CURRENT_SOURCE_DIR}/c/iso5436_2_handle_c.hxx
        ${CMAKE_CURRENT_SOURCE_DIR}/c/messages_c.hxx
        ${CMAKE_CURRENT_SOURCE_DIR}/c/point_iterator_c.hxx
        ${CMAKE_CURRENT_SOURCE_DIR}/c/point_vector_c.hxx
        ${CMAKE_CURRENT_SOURCE_DIR}/cxx/version.h
        ${CMAKE_CURRENT_SOURCE_DIR}/cxx/binary_lsb_point_vector_reader_context.hxx
        ${CMAKE_CURRENT_SOURCE_DIR}/cxx/binary_lsb_point_vector_writer_context.hxx
        ${CMAKE_CURRENT_SOURCE_DIR}/cxx/binary_msb_point_vector_reader_context.hxx
        ${CMAKE_CURRENT_SOURCE_DIR}/cxx/binary_msb_point_vector_writer_context.hxx
        ${CMAKE_CURRENT_SOURCE_DIR}/cxx/binary_point_vector_reader_context.hxx
        ${CMAKE_CURRENT_SOURCE_DIR}/cxx/binary_point_vector_writer_context.hxx
        ${CMAKE_CURRENT_SOURCE_DIR}/cxx/data_point_impl.hxx
        ${CMAKE_CURRENT_SOURCE_DIR}/cxx/data_point_parser.hxx
        ${CMAKE_CURRENT_SOURCE_DIR}/cxx/double_data_point_parser.hxx
        ${CMAKE_CURRENT_SOURCE_DIR}/cxx/double_point_buffer.hxx
        ${CMAKE_CURRENT_SOURCE_DIR}/cxx/environment.hxx
        ${CMAKE_CURRENT_SOURCE_DIR}/cxx/float_data_point_parser.hxx
        ${CMAKE_CURRENT_SOURCE_DIR}/cxx/float_point_buffer.hxx
        ${CMAKE_CURRENT_SOURCE_DIR}/cxx/inline_validity.hxx
        ${CMAKE_CURRENT_SOURCE_DIR}/cxx/int16_data_point_parser.hxx
        ${CMAKE_CURRENT_SOURCE_DIR}/cxx/int16_point_buffer.hxx
        ${CMAKE_CURRENT_SOURCE_DIR}/cxx/int32_data_point_parser.hxx
        ${CMAKE_CURRENT_SOURCE_DIR}/cxx/int32_point_buffer.hxx
        ${CMAKE_CURRENT_SOURCE_DIR}/cxx/iso5436_2_container.hxx
        ${CMAKE_CURRENT_SOURCE_DIR}/cxx/missing_data_point_parser.hxx
        ${CMAKE_CURRENT_SOURCE_DIR}/cxx/point_buffer.hxx
        ${CMAKE_CURRENT_SOURCE_DIR}/cxx/point_validity_provider.hxx
        ${CMAKE_CURRENT_SOURCE_DIR}/cxx/point_vector_iostream.hxx
        ${CMAKE_CURRENT_SOURCE_DIR}/cxx/point_vector_parser.hxx
        ${CMAKE_CURRENT_SOURCE_DIR}/cxx/point_vector_parser_builder.hxx
        ${CMAKE_CURRENT_SOURCE_DIR}/cxx/point_vector_proxy.hxx
        ${CMAKE_CURRENT_SOURCE_DIR}/cxx/point_vector_proxy_context.hxx
        ${CMAKE_CURRENT_SOURCE_DIR}/cxx/point_vector_proxy_context_list.hxx
        ${CMAKE_CURRENT_SOURCE_DIR}/cxx/point_vector_proxy_context_matrix.hxx
        ${CMAKE_CURRENT_SOURCE_DIR}/cxx/point_vector_reader_context.hxx
        ${CMAKE_CURRENT_SOURCE_DIR}/cxx/point_vector_writer_context.hxx
        ${CMAKE_CURRENT_SOURCE_DIR}/cxx/stdafx.hxx
        ${CMAKE_CURRENT_SOURCE_DIR}/cxx/valid_buffer.hxx
        ${CMAKE_CURRENT_SOURCE_DIR}/cxx/version.in
        ${CMAKE_CURRENT_SOURCE_DIR}/cxx/vector_buffer.hxx
        ${CMAKE_CURRENT_SOURCE_DIR}/cxx/vector_buffer_builder.hxx
        ${CMAKE_CURRENT_SOURCE_DIR}/cxx/win32_environment.hxx
        ${CMAKE_CURRENT_SOURCE_DIR}/cxx/linux_environment.hxx
        ${CMAKE_CURRENT_SOURCE_DIR}/cxx/xml_point_vector_reader_context.hxx
        ${CMAKE_CURRENT_SOURCE_DIR}/cxx/xml_point_vector_writer_context.hxx
        ${CMAKE_CURRENT_SOURCE_DIR}/cxx/zip_stream_buffer.hxx
          ${CMAKE_CURRENT_SOURCE_DIR}/xyssl/md5.h
        )
set(opengps_HDRS
        ${CMAKE_CURRENT_SOURCE_DIR}/../../include/opengps/data_point.h
        ${CMAKE_CURRENT_SOURCE_DIR}/../../include/opengps/data_point_type.h
        ${CMAKE_CURRENT_SOURCE_DIR}/../../include/opengps/info.h
        ${CMAKE_CURRENT_SOURCE_DIR}/../../include/opengps/iso5436_2.h
        ${CMAKE_CURRENT_SOURCE_DIR}/../../include/opengps/messages.h
        ${CMAKE_CURRENT_SOURCE_DIR}/../../include/opengps/opengps.h
        ${CMAKE_CURRENT_SOURCE_DIR}/../../include/opengps/point_iterator.h
        ${CMAKE_CURRENT_SOURCE_DIR}/../../include/opengps/point_vector.h
    )
set(opengps_CXXHDRS
        ${CMAKE_CURRENT_SOURCE_DIR}/../../include/opengps/cxx/data_point.hxx
        ${CMAKE_CURRENT_SOURCE_DIR}/../../include/opengps/cxx/exceptions.hxx
        ${CMAKE_CURRENT_SOURCE_DIR}/../../include/opengps/cxx/info.hxx
        ${CMAKE_CURRENT_SOURCE_DIR}/../../include/opengps/cxx/iso5436_2.hxx
        ${CMAKE_CURRENT_SOURCE_DIR}/../../include/opengps/cxx/iso5436_2_handle.hxx
        ${CMAKE_CURRENT_BINARY_DIR}/opengps/cxx/iso5436_2_xsd.hxx
        ${CMAKE_CURRENT_SOURCE_DIR}/../../include/opengps/cxx/opengps.hxx
        ${CMAKE_CURRENT_SOURCE_DIR}/../../include/opengps/cxx/point_iterator.hxx
        ${CMAKE_CURRENT_SOURCE_DIR}/../../include/opengps/cxx/point_vector.hxx
        ${CMAKE_CURRENT_SOURCE_DIR}/../../include/opengps/cxx/point_vector_base.hxx
        ${CMAKE_CURRENT_SOURCE_DIR}/../../include/opengps/cxx/string.hxx
        ${CMAKE_CURRENT_SOURCE_DIR}/../../include/opengps/cxx/version.h
        )
set(iso54362xml_SRCS
        ${CMAKE_CURRENT_SOURCE_DIR}/xsd_Licence_Header.c
        ${CMAKE_CURRENT_SOURCE_DIR}/c/data_point_c.cxx
        ${CMAKE_CURRENT_SOURCE_DIR}/c/iso5436_2_c.cxx
        ${CMAKE_CURRENT_SOURCE_DIR}/c/messages_c.cxx
        ${CMAKE_CURRENT_SOURCE_DIR}/c/point_iterator_c.cxx
        ${CMAKE_CURRENT_SOURCE_DIR}/c/point_vector_c.cxx
        ${CMAKE_CURRENT_SOURCE_DIR}/cxx/binary_lsb_point_vector_reader_context.cxx
        ${CMAKE_CURRENT_SOURCE_DIR}/cxx/binary_lsb_point_vector_writer_context.cxx
        ${CMAKE_CURRENT_SOURCE_DIR}/cxx/binary_msb_point_vector_reader_context.cxx
        ${CMAKE_CURRENT_SOURCE_DIR}/cxx/binary_msb_point_vector_writer_context.cxx
        ${CMAKE_CURRENT_SOURCE_DIR}/cxx/binary_point_vector_reader_context.cxx
        ${CMAKE_CURRENT_SOURCE_DIR}/cxx/binary_point_vector_writer_context.cxx
        ${CMAKE_CURRENT_SOURCE_DIR}/cxx/data_point.cxx
        ${CMAKE_CURRENT_SOURCE_DIR}/cxx/data_point_impl.cxx
        ${CMAKE_CURRENT_SOURCE_DIR}/cxx/data_point_proxy.cxx
        ${CMAKE_CURRENT_SOURCE_DIR}/cxx/double_data_point_parser.cxx
        ${CMAKE_CURRENT_SOURCE_DIR}/cxx/double_point_buffer.cxx
        ${CMAKE_CURRENT_SOURCE_DIR}/cxx/environment.cxx
        ${CMAKE_CURRENT_SOURCE_DIR}/cxx/exceptions.cxx
        ${CMAKE_CURRENT_SOURCE_DIR}/cxx/float_data_point_parser.cxx
        ${CMAKE_CURRENT_SOURCE_DIR}/cxx/float_point_buffer.cxx
        ${CMAKE_CURRENT_SOURCE_DIR}/cxx/info.cxx
        ${CMAKE_CURRENT_SOURCE_DIR}/cxx/inline_validity.cxx
        ${CMAKE_CURRENT_SOURCE_DIR}/cxx/int16_data_point_parser.cxx
        ${CMAKE_CURRENT_SOURCE_DIR}/cxx/int16_point_buffer.cxx
        ${CMAKE_CURRENT_SOURCE_DIR}/cxx/int32_data_point_parser.cxx
        ${CMAKE_CURRENT_SOURCE_DIR}/cxx/int32_point_buffer.cxx
        ${CMAKE_CURRENT_SOURCE_DIR}/cxx/iso5436_2.cxx
        ${CMAKE_CURRENT_SOURCE_DIR}/cxx/iso5436_2_container.cxx
        ${CMAKE_CURRENT_SOURCE_DIR}/cxx/missing_data_point_parser.cxx
        ${CMAKE_CURRENT_SOURCE_DIR}/cxx/point_buffer.cxx
        ${CMAKE_CURRENT_SOURCE_DIR}/cxx/point_iterator.cxx
        ${CMAKE_CURRENT_SOURCE_DIR}/cxx/point_validity_provider.cxx
        ${CMAKE_CURRENT_SOURCE_DIR}/cxx/point_vector.cxx
        ${CMAKE_CURRENT_SOURCE_DIR}/cxx/point_vector_iostream.cxx
        ${CMAKE_CURRENT_SOURCE_DIR}/cxx/point_vector_parser.cxx
        ${CMAKE_CURRENT_SOURCE_DIR}/cxx/point_vector_parser_builder.cxx
        ${CMAKE_CURRENT_SOURCE_DIR}/cxx/point_vector_proxy.cxx
        ${CMAKE_CURRENT_SOURCE_DIR}/cxx/point_vector_proxy_context.cxx
        ${CMAKE_CURRENT_SOURCE_DIR}/cxx/point_vector_proxy_context_list.cxx
        ${CMAKE_CURRENT_SOURCE_DIR}/cxx/point_vector_proxy_context_matrix.cxx
        ${CMAKE_CURRENT_SOURCE_DIR}/cxx/string.cxx
        ${CMAKE_CURRENT_SOURCE_DIR}/cxx/valid_buffer.cxx
        ${CMAKE_CURRENT_SOURCE_DIR}/cxx/vector_buffer.cxx
        ${CMAKE_CURRENT_SOURCE_DIR}/cxx/vector_buffer_builder.cxx
        ${CMAKE_CURRENT_SOURCE_DIR}/cxx/win32_environment.cxx
        ${CMAKE_CURRENT_SOURCE_DIR}/cxx/linux_environment.cxx
        ${CMAKE_CURRENT_SOURCE_DIR}/cxx/xml_point_vector_reader_context.cxx
        ${CMAKE_CURRENT_SOURCE_DIR}/cxx/xml_point_vector_writer_context.cxx
        ${CMAKE_CURRENT_SOURCE_DIR}/cxx/zip_stream_buffer.cxx
        ${CMAKE_CURRENT_BINARY_DIR}/opengps/cxx/iso5436_2_xsd.cxx
        ${CMAKE_CURRENT_SOURCE_DIR}/xyssl/md5.c
)

add_library(${iso54362xmlNAME} ${LIBRARY_TYPE} ${iso54362xml_SRCS} ${iso54362xml_HEADER} ${opengps_HDRS} ${opengps_CXXHDRS})

if(LINUX)
   set(ZLIBNAME "minizip")
else(LINUX)
   set(ZLIBNAME "zlibwapi")
endif(LINUX)

target_link_libraries(${iso54362xmlNAME} ${XERCESC_LIBRARY} ${ZLIBNAME})

install(TARGETS  ${iso54362xmlNAME}
       RUNTIME DESTINATION bin
       LIBRARY DESTINATION iso5436-2-xml/lib
        ARCHIVE DESTINATION iso5436-2-xml/lib
    )
install(FILES ${opengps_HDRS} DESTINATION iso5436-2-xml/include/opengps)
install(FILES ${opengps_CXXHDRS} DESTINATION iso5436-2-xml/include/opengps/cxx)

if(MSVC)
  set_target_properties(${iso54362xmlNAME}  PROPERTIES COMPILE_FLAGS "/Zc:wchar_t")
endif()


set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "iso5436-2-xml")
set(CPACK_PACKAGE_VENDOR "opengps")


set(CPACK_PACKAGE_VERSION_MAJOR "1")
set(CPACK_PACKAGE_VERSION_MINOR "0")
set(CPACK_PACKAGE_VERSION_PATCH "0")


if(WIN32 AND NOT UNIX)
    set(CPACK_GENERATOR ZIP)
endif(WIN32 AND NOT UNIX)


include(CPack)
