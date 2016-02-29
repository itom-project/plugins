# - Find libgphoto
# Find libpghoto includes and library
# This module defines
# LIBGHPOTO_INCLUDE_DIR, where to find libgphotoXXX.h, etc.
# LIBGPHOTO_LIBRARIES, the libraries needed to use libgphoto.
# LIBGPHOTO_FOUND, If false, do not try to use libgphoto.
# also defined, but not for general use are
# LIBGPHOTO_LIBRARY, where to find the libgphoto library.

SET(LIBGPHOTO_FOUND false)

find_path(LIBGPHOTO_DIR gphoto2.h PATHS /usr/local/include /usr/local/include/gphoto2 /usr/include /usr/include/gphoto2 /opt/local/lib /opt/local/lib/gphoto2 DOC "Root directory of libgphoto")
FIND_PATH(LIBGPHOTO_INCLUDE_DIR gphoto2.h PATHS /usr/local/include /usr/local/include/gphoto /usr/include /usr/include/gphoto2 /opt/local/lib /opt/local/lib/gphoto2 ${LIBGPHOTO_DIR})

FIND_LIBRARY(LIBGPHOTO_LIBRARY NAMES gphoto2 libgphoto2 PATHS /usr/lib /usr/local/lib /opt/locala/lib ${LIBGPHOTO_DIR})
FIND_LIBRARY(LIBGPHOTO_PORT_LIBRARY NAMES gphoto2_port libgphoto2_port PATHS /usr/lib /usr/local/lib /opt/locala/lib ${LIBGPHOTO_DIR})

IF (LIBGPHOTO_LIBRARY AND LIBGPHOTO_PORT_LIBRARY AND LIBGPHOTO_INCLUDE_DIR)
    SET(LIBGPHOTO_LIBRARIES ${LIBGPHOTO_LIBRARY} ${LIBGPHOTO_PORT_LIBRARY})
    SET(LIBGPHOTO_FOUND true)
ELSE (LIBGPHOTO_LIBRARY AND LIBGPHOTO_PORT_LIBRARY AND LIBGPHOTO_INCLUDE_DIR)
    SET(LIBGPHOTO_FOUND false)
    SET(LIBGPHOTO_LIBRARIES "")
ENDIF (LIBGPHOTO_LIBRARY AND LIBGPHOTO_PORT_LIBRARY AND LIBGPHOTO_INCLUDE_DIR)


IF (LIBGPHOTO_FOUND)
   IF (NOT LIBGPHOTO_FIND_QUIETLY)
      MESSAGE(STATUS "Found libgphoto: ${LIBGPHOTO_LIBRARIES}")
   ENDIF (NOT LIBGPHOTO_FIND_QUIETLY)
ELSE (LIBGPHOTO_FOUND)
   IF (LIBGPHOTO_FIND_REQUIRED)
      MESSAGE(FATAL_ERROR "Could not find libgphoto library")
   ENDIF (LIBGPHOTO_FIND_REQUIRED)
ENDIF (LIBGPHOTO_FOUND)

mark_as_advanced(LIBGPHOTO_LIBRARY)
