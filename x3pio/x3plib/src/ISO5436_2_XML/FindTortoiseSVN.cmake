# - Extract information from a subversion working copy using TortoiseSVN
# The module defines the following variables:
#  TortoiseSVN_EXECUTABLE - path to svn command line client
#  TortoiseSVN_FOUND - true if the command line client was found
# If the client executable is found the macro
#  TortoiseSVN_SubWCREV(<dir> <var-prefix>)
# is defined to extract information of a subversion working copy at
# a given location. The macro defines the following variables:
#  <var-prefix>_WC_REVISION - current revision
#  <var-prefix>_WC_INFO - output of command `SubWCRev <dir>'
# Example usage:
#  FIND_PACKAGE(TortoiseSVN)
#  if(TortoiseSVN_FOUND)
#    TortoiseSVN_WC_INFO(${PROJECT_SOURCE_DIR} Project)
#    message("Current revision is ${Project_WC_REVISION}")
#  endif(TortoiseSVN_FOUND)
#
#------------------------------------------------------------------------------
#  Modifications:
#    Kathleen Bonnell, Wed Apr 14 16:15:23 MST 2010
#    Use "SubWCRev WorkingCopyPath SrcVersionFile DstVersionFile" version of
#    command to extract revision number.  More consistent results this way.
#------------------------------------------------------------------------------

set(TortoiseSVN_FOUND FALSE)

find_program(TortoiseSVN_EXECUTABLE SubWCRev
  DOC "Tortoise SVN client")
mark_as_advanced(TortoiseSVN_EXECUTABLE)

if(TortoiseSVN_EXECUTABLE)
  set(TortoiseSVN_FOUND TRUE)

  MACRO(TortoiseSVN_WC_INFO dir prefix)
    file(WRITE "${VISIT_SOURCE_DIR}/svnrev.in" "$WCREV$")

    EXECUTE_PROCESS(COMMAND ${TortoiseSVN_EXECUTABLE} "." "${VISIT_SOURCE_DIR}/svnrev.in" "${VISIT_SOURCE_DIR}/svnrev"
      WORKING_DIRECTORY ${VISIT_SOURCE_DIR}
      OUTPUT_VARIABLE TortoiseSVN_info_output
      ERROR_VARIABLE TortoiseSVN_info_error
      RESULT_VARIABLE TortoiseSVN_info_result
      OUTPUT_STRIP_TRAILING_WHITESPACE)

    if(NOT ${TortoiseSVN_info_result} EQUAL 0)
        message(SEND_ERROR "Command \"${TortoiseSVN_EXECUTABLE} ${dir}\" failed with output:\n${TortoiseSVN_info_error}")
    else(NOT ${TortoiseSVN_info_result} EQUAL 0)
        file(STRINGS ${VISIT_SOURCE_DIR}/svnrev ${prefix}_WC_REVISION)
    endif(NOT ${TortoiseSVN_info_result} EQUAL 0)
    file(REMOVE ${VISIT_SOURCE_DIR}/svnrev.in ${VISIT_SOURCE_DIR}/svnrev)
  ENDMACRO(TortoiseSVN_WC_INFO)

endif(TortoiseSVN_EXECUTABLE)

if(NOT TortoiseSVN_FOUND)
  if(NOT TortoiseSVN_FIND_QUIETLY)
    message(STATUS "TortoiseSVN was not found.")
  else(NOT TortoiseSVN_FIND_QUIETLY)
    if(TortoiseSVN_FIND_REQUIRED)
      message(FATAL_ERROR "TortoiseSVN was not found.")
    endif(TortoiseSVN_FIND_REQUIRED)
  endif(NOT TortoiseSVN_FIND_QUIETLY)
endif(NOT TortoiseSVN_FOUND)
