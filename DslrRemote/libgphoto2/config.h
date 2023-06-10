#ifndef CONFIG_H
#define CONFIG_H

#define _CRT_SECURE_NO_WARNINGS // turn off some warnings

#include <stdio.h>
#include <assert.h>
#include <io.h>
#include <Windows.h>

/*
#if 1
    #define GP_LOG_E( ERRMSG, ... ) fprintf(stderr, ERRMSG, __VA_ARGS__)
    #define GP_LOG_D( DBGMSG, ... ) fprintf(stdout, DBGMSG, __VA_ARGS__)
    #define C_PARAMS_MSG( COND, PARAMSMSG ) if (!COND) fprintf(stdout, PARAMSMSG)
    #define C_PARAMS( VAL ) \
        if ( !(VAL) ) assert(sprintf("NULL Pointer parameter in %s %s", __FUNCTION__, __FILE__))
    #define C_MEM( VAL ) \
        if ( !(VAL) ) assert(sprintf("Failed to allocate memory in %s %s", __FUNCTION__, __FILE__))
#else
    #define GP_LOG_E( ERRMSG, ... )
    #define GP_LOG_D( DBGMSG, ... )
    #define C_PARAMS_MSG( COND, PARAMSMSG ) COND
    #define C_PARAMS( VAL ) VAL
    #define C_MEM( VAL ) VAL
#endif
*/

#define __func__  __FUNCTION__
#define lseek _lseek
#define ftruncate _chsize
#define ssize_t unsigned int
#define snprintf _snprintf
#define GETTEXT_PACKAGE 0
#undef bindtextdomain
#define bindtextdomain(Domain,Directory) (Domain) // we just do not use it as we do not have it ;-)
#undef bind_textdomain_codeset
#define bind_textdomain_codeset(GETTEXT_PACKAGE, CODESET) 0
#undef interface

#define _GPHOTO2_INTERNAL_CODE // we need this as otherwise gpi_vsnprintf and gp_system_mkdir are undefined - is this intentional?

#ifndef uint8_t
	#define uint8_t uint8
#endif
#ifndef uint16_t
	#define uint16_t uint16
#endif
#ifndef uint32_t
	#define uint32_t uint32
#endif

#define inline __inline
#ifndef usleep
	#define usleep(MUS) Sleep(MUS / 1000)
#endif

#ifndef setenv
inline int setenv(char const *name, char const *value, int overwrite)
{
	char buffer[255];
	int const err = snprintf(buffer, sizeof(buffer), "%s=%s", name, value);
	if (err < 0 || err >= sizeof(buffer))
		return -1;
	return _putenv(buffer);
}
#endif

#ifndef unsetenv
	#define unsetenv
#endif


#endif
