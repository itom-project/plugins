#ifndef CONFIG_H
#define CONFIG_H

#define _CRT_SECURE_NO_WARNINGS // turn off some warnings

#define _GPHOTO2_INTERNAL_CODE // guess here it is legal to define that
#ifdef _MSC_VER
#  if _MSC_VER < 1900
#    define __func__ __FUNCTION__
#  endif
#endif

#define GETTEXT_PACKAGE 0
#ifdef WIN32
#define OS_WINDOWS
#endif

# if _MSC_VER < 1900
# ifndef snprintf
#  define snprintf		_snprintf
# endif
# endif

#endif
