WRITE_FILE("${CMAKE_CURRENT_BINARY_DIR}/gphoto2-endian.h"
	"/* This file is generated automatically by configure */\n\
/* It is valid only for the system type ${effective_target} */\n\
\n\
#ifndef __BYTEORDER_H\n\
#define __BYTEORDER_H\n\
")

include(TestBigEndian)
TEST_BIG_ENDIAN(ENDIANNESS)

if(ENDIANNESS)
# big endian
WRITE_FILE("${CMAKE_CURRENT_BINARY_DIR}/gphoto2-endian.h"
	"/* No other byte swapping functions are available on this big-endian system */\n\
#define swap16(x)	((uint16_t)(((x) << 8) | ((uint16_t)(x) >> 8)))\n\
#define swap32(x)	((uint32_t)(((uint32_t)(x) << 24) & 0xff000000UL | \\\n\
				    ((uint32_t)(x) << 8)  & 0x00ff0000UL | \\\n\
				    ((x) >> 8)  & 0x0000ff00UL | \\\n\
				    ((x) >> 24) & 0x000000ffUL))\n\
#define swap64(x) ((uint64_t)(((uint64_t)(x) << 56) & 0xff00000000000000ULL | \\\n\
			      ((uint64_t)(x) << 40) & 0x00ff000000000000ULL | \\\n\
			      ((uint64_t)(x) << 24) & 0x0000ff0000000000ULL | \\\n\
			      ((uint64_t)(x) << 8)  & 0x000000ff00000000ULL | \\\n\
			      ((x) >> 8)  & 0x00000000ff000000ULL | \\\n\
			      ((x) >> 24) & 0x0000000000ff0000ULL | \\\n\
			      ((x) >> 40) & 0x000000000000ff00ULL | \\\n\
			      ((x) >> 56) & 0x00000000000000ffULL))\n\
\n" APPEND)
else(ENDIANNESS)
# little endian
WRITE_FILE("${CMAKE_CURRENT_BINARY_DIR}/gphoto2-endian.h"
	"/* Use these as generic byteswapping macros on this little endian system */\n\
/* on windows we might not have ntohs / ntohl without including winsock.dll,\n\
 * so use generic macros */\n\
#ifdef __HAVE_NTOHL\n\
# define swap16(x)	htons(x)\n\
# define swap32(x)	htonl(x)\n\
#else\n\
# define swap16(x)	((uint16_t)(((x) << 8) | ((uint16_t)(x) >> 8)))\n\
# define swap32(x)	((uint32_t)((((uint32_t)(x) << 24) & 0xff000000UL) | \\\n\
				    (((uint32_t)(x) << 8)  & 0x00ff0000UL) | \\\n\
				    (((x) >> 8)  & 0x0000ff00UL) | \\\n\
				    (((x) >> 24) & 0x000000ffUL)))\n\
#endif\n\
/* No optimized 64 bit byte swapping macro is available */\n\
#define swap64(x) ((uint64_t)((((uint64_t)(x) << 56) & 0xff00000000000000ULL) | \\\n\
			      (((uint64_t)(x) << 40) & 0x00ff000000000000ULL) | \\\n\
			      (((uint64_t)(x) << 24) & 0x0000ff0000000000ULL) | \\\n\
			      (((uint64_t)(x) << 8)  & 0x000000ff00000000ULL) | \\\n\
			      (((x) >> 8)  & 0x00000000ff000000ULL) | \\\n\
			      (((x) >> 24) & 0x0000000000ff0000ULL) | \\\n\
			      (((x) >> 40) & 0x000000000000ff00ULL) | \\\n\
			      (((x) >> 56) & 0x00000000000000ffULL)))\n\
\n" APPEND)

endif(ENDIANNESS)

WRITE_FILE("${CMAKE_CURRENT_BINARY_DIR}/gphoto2-endian.h"
	"/* The byte swapping macros have the form: */\n\
/*   EENN[a]toh or htoEENN[a] where EE is be (big endian) or */\n\
/* le (little-endian), NN is 16 or 32 (number of bits) and a, */\n\
/* if present, indicates that the endian side is a pointer to an */\n\
/* array of uint8_t bytes instead of an integer of the specified length. */\n\
/* h refers to the host's ordering method. */\n\
\n\
/* So, to convert a 32-bit integer stored in a buffer in little-endian */\n\
/* format into a uint32_t usable on this machine, you could use: */\n\
/*   uint32_t value = le32atoh(&buf[3]); */\n\
/* To put that value back into the buffer, you could use: */\n\
/*   htole32a(&buf[3], value); */\n\
\n\
/* Define aliases for the standard byte swapping macros */\n\
/* Arguments to these macros must be properly aligned on natural word */\n\
/* boundaries in order to work properly on all architectures */\n\
#ifndef htobe16\n\
# ifdef __HAVE_NTOHL\n\
#  define htobe16(x) htons(x)\n\
# else\n\
#  ifdef WORDS_BIGENDIAN\n\
#   define htobe16(x) (x)\n\
#  else\n\
#   define htobe16(x) swap16(x)\n\
#  endif\n\
# endif\n\
#endif\n\
#ifndef htobe32\n\
# ifdef __HAVE_NTOHL\n\
#  define htobe32(x) htonl(x)\n\
# else\n\
#  ifdef WORDS_BIGENDIAN\n\
#   define htobe32(x) (x)\n\
#  else\n\
#   define htobe32(x) swap32(x)\n\
#  endif\n\
# endif\n\
#endif\n\
#ifndef be16toh\n\
# define be16toh(x) htobe16(x)\n\
#endif\n\
#ifndef be32toh\n\
# define be32toh(x) htobe32(x)\n\
#endif\n\
\n\
#define HTOBE16(x) (x) = htobe16(x)\n\
#define HTOBE32(x) (x) = htobe32(x)\n\
#define BE32TOH(x) (x) = be32toh(x)\n\
#define BE16TOH(x) (x) = be16toh(x)\n\
\n" APPEND)

if(ENDIANNESS)
# big endian
WRITE_FILE("${CMAKE_CURRENT_BINARY_DIR}/gphoto2-endian.h"
	"/* Define our own extended byte swapping macros for big-endian machines */\n\
#ifndef htole16\n\
# define htole16(x)      swap16(x)\n\
#endif\n\
#ifndef htole32\n\
# define htole32(x)      swap32(x)\n\
#endif\n\
#ifndef le16toh\n\
# define le16toh(x)      swap16(x)\n\
#endif\n\
#ifndef le32toh\n\
# define le32toh(x)      swap32(x)\n\
#endif\n\
#ifndef le64toh\n\
# define le64toh(x)      swap64(x)\n\
#endif\n\
\n\
#ifndef htobe64\n\
# define htobe64(x)      (x)\n\
#endif\n\
#ifndef be64toh\n\
# define be64toh(x)      (x)\n\
#endif\n\
\n\
#define HTOLE16(x)      (x) = htole16(x)\n\
#define HTOLE32(x)      (x) = htole32(x)\n\
#define LE16TOH(x)      (x) = le16toh(x)\n\
#define LE32TOH(x)      (x) = le32toh(x)\n\
#define LE64TOH(x)      (x) = le64toh(x)\n\
\n\
#define HTOBE64(x)      (void) (x)\n\
#define BE64TOH(x)      (void) (x)\n\
\n" APPEND)

else(ENDIANNESS)
# little endian
WRITE_FILE("${CMAKE_CURRENT_BINARY_DIR}/gphoto2-endian.h"
	"/* On little endian machines, these macros are null */\n\
#ifndef htole16\n\
# define htole16(x)      (x)\n\
#endif\n\
#ifndef htole32\n\
# define htole32(x)      (x)\n\
#endif\n\
#ifndef htole64\n\
# define htole64(x)      (x)\n\
#endif\n\
#ifndef le16toh\n\
# define le16toh(x)      (x)\n\
#endif\n\
#ifndef le32toh\n\
# define le32toh(x)      (x)\n\
#endif\n\
#ifndef le64toh\n\
# define le64toh(x)      (x)\n\
#endif\n\
\n\
#define HTOLE16(x)      (void) (x)\n\
#define HTOLE32(x)      (void) (x)\n\
#define HTOLE64(x)      (void) (x)\n\
#define LE16TOH(x)      (void) (x)\n\
#define LE32TOH(x)      (void) (x)\n\
#define LE64TOH(x)      (void) (x)\n\
\n\
/* These don't have standard aliases */\n\
#ifndef htobe64\n\
# define htobe64(x)      swap64(x)\n\
#endif\n\
#ifndef be64toh\n\
# define be64toh(x)      swap64(x)\n\
#endif\n\
\n\
#define HTOBE64(x)      (x) = htobe64(x)\n\
#define BE64TOH(x)      (x) = be64toh(x)\n\
\n" APPEND)
endif(ENDIANNESS)

WRITE_FILE("${CMAKE_CURRENT_BINARY_DIR}/gphoto2-endian.h"
	"/* Define the C99 standard length-specific integer types */\n\
#include <stdint.h>\n\
\n" APPEND)

message(STATUS "PROC-TYPE: " ${CMAKE_SYSTEM_PROCESSOR})
# this is for x86 platforms only but we simply include for the time being until the test
# for the platform is included
WRITE_FILE("${CMAKE_CURRENT_BINARY_DIR}/gphoto2-endian.h"
	"/* Here are some macros to create integers from a byte array */\n\
/* These are used to get and put integers from/into a uint8_t array */\n\
/* with a specific endianness.  This is the most portable way to generate */\n\
/* and read messages to a network or serial device.  Each member of a */\n\
/* packet structure must be handled separately. */\n\
\n\
/* The i386 and compatibles can handle unaligned memory access, */\n\
/* so use the optimized macros above to do this job */\n\
#ifndef be16atoh\n\
# define be16atoh(x)     be16toh(*(uint16_t*)(x))\n\
#endif\n\
#ifndef be32atoh\n\
# define be32atoh(x)     be32toh(*(uint32_t*)(x))\n\
#endif\n\
#ifndef be64atoh\n\
# define be64atoh(x)     be64toh(*(uint64_t*)(x))\n\
#endif\n\
#ifndef le16atoh\n\
# define le16atoh(x)     le16toh(*(uint16_t*)(x))\n\
#endif\n\
#ifndef le32atoh\n\
# define le32atoh(x)     le32toh(*(uint32_t*)(x))\n\
#endif\n\
#ifndef le64atoh\n\
# define le64atoh(x)     le64toh(*(uint64_t*)(x))\n\
#endif\n\
\n\
#ifndef htob16a\n\
# define htobe16a(a,x)   *(uint16_t*)(a) = htobe16(x)\n\
#endif\n\
#ifndef htobe32a\n\
# define htobe32a(a,x)   *(uint32_t*)(a) = htobe32(x)\n\
#endif\n\
#ifndef htobe64a\n\
# define htobe64a(a,x)   *(uint64_t*)(a) = htobe64(x)\n\
#endif\n\
#ifndef htole16a\n\
# define htole16a(a,x)   *(uint16_t*)(a) = htole16(x)\n\
#endif\n\
#ifndef htole32a\n\
# define htole32a(a,x)   *(uint32_t*)(a) = htole32(x)\n\
#endif\n\
#ifndef htole64a\n\
# define htole64a(a,x)   *(uint64_t*)(a) = htole64(x)\n\
#endif\n\
\n" APPEND)

# guess this is ambigious need to take a look at it ...
WRITE_FILE("${CMAKE_CURRENT_BINARY_DIR}/gphoto2-endian.h"
	"/* Here are some macros to create integers from a byte array */\n\
/* These are used to get and put integers from/into a uint8_t array */\n\
/* with a specific endianness.  This is the most portable way to generate */\n\
/* and read messages to a network or serial device.  Each member of a */\n\
/* packet structure must be handled separately. */\n\
\n\
/* Non-optimized but portable macros */\n\
/*#define be16atoh(x)     ((uint16_t)(((x)[0]<<8)|(x)[1]))\n\
#define be32atoh(x)     ((uint32_t)(((x)[0]<<24)|((x)[1]<<16)|((x)[2]<<8)|(x)[3]))\n\
#define be64atoh_x(x,off,shift) 	(((uint64_t)((x)[off]))<<shift)\n\
#define be64atoh(x)     ((uint64_t)(be64atoh_x(x,0,56)|be64atoh_x(x,1,48)|be64atoh_x(x,2,40)| \\\n\
        be64atoh_x(x,3,32)|be64atoh_x(x,4,24)|be64atoh_x(x,5,16)|be64atoh_x(x,6,8)|((x)[7])))\n\
#define le16atoh(x)     ((uint16_t)(((x)[1]<<8)|(x)[0]))\n\
#define le32atoh(x)     ((uint32_t)(((x)[3]<<24)|((x)[2]<<16)|((x)[1]<<8)|(x)[0]))\n\
#define le64atoh_x(x,off,shift) (((uint64_t)(x)[off])<<shift)\n\
#define le64atoh(x)     ((uint64_t)(le64atoh_x(x,7,56)|le64atoh_x(x,6,48)|le64atoh_x(x,5,40)| \\\n\
        le64atoh_x(x,4,32)|le64atoh_x(x,3,24)|le64atoh_x(x,2,16)|le64atoh_x(x,1,8)|((x)[0])))\n\
\n\
#define htobe16a(a,x)   (a)[0]=(uint8_t)((x)>>8), (a)[1]=(uint8_t)(x)\n\
#define htobe32a(a,x)   (a)[0]=(uint8_t)((x)>>24), (a)[1]=(uint8_t)((x)>>16), \\\n\
        (a)[2]=(uint8_t)((x)>>8), (a)[3]=(uint8_t)(x)\n\
#define htobe64a(a,x)   (a)[0]=(uint8_t)((x)>>56), (a)[1]=(uint8_t)((x)>>48), \\\n\
        (a)[2]=(uint8_t)((x)>>40), (a)[3]=(uint8_t)((x)>>32), \\\n\
        (a)[4]=(uint8_t)((x)>>24), (a)[5]=(uint8_t)((x)>>16), \\\n\
        (a)[6]=(uint8_t)((x)>>8), (a)[7]=(uint8_t)(x)\n\
#define htole16a(a,x)   (a)[1]=(uint8_t)((x)>>8), (a)[0]=(uint8_t)(x)\n\
#define htole32a(a,x)   (a)[3]=(uint8_t)((x)>>24), (a)[2]=(uint8_t)((x)>>16), \\\n\
        (a)[1]=(uint8_t)((x)>>8), (a)[0]=(uint8_t)(x)\n\
#define htole64a(a,x)   (a)[7]=(uint8_t)((x)>>56), (a)[6]=(uint8_t)((x)>>48), \\\n\
        (a)[5]=(uint8_t)((x)>>40), (a)[4]=(uint8_t)((x)>>32), \\\n\
        (a)[3]=(uint8_t)((x)>>24), (a)[2]=(uint8_t)((x)>>16), \\\n\
        (a)[1]=(uint8_t)((x)>>8), (a)[0]=(uint8_t)(x)\n\
*/\n" APPEND)

WRITE_FILE("${CMAKE_CURRENT_BINARY_DIR}/gphoto2-endian.h" "#endif /*__BYTEORDER_H*/" APPEND)
