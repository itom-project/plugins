#ifndef DRAW_H
#define DRAW_H

#define NODEPS

#ifdef WIN32
#define getc_unlocked _fgetc_nolock
#define fseeko _fseeki64
#define ftello _ftelli64
#endif

#endif //DRAW_H
