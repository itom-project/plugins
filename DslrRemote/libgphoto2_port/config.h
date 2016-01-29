#ifndef CONFIG_H
#define CONFIG_H

#define _CRT_SECURE_NO_WARNINGS // turn off some warnings

#define _GPHOTO2_INTERNAL_CODE // guess here it is legal to define that
#define __func__  __FUNCTION__

#define GETTEXT_PACKAGE 0
#ifdef WIN32
#define OS_WINDOWS
#endif

# ifndef snprintf
#  define snprintf		_snprintf
# endif

#endif