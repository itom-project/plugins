/* =============================================================
 *
 * CNV_A.H
 *
 * Header file for PCOCNV.DLL Author MB
 *
 * ================================================================== */

// @ver1.000

// defines for Lut's ...
// local functions

//--------------------

#ifndef CNV_S_H
#define CNV_S_H

#ifndef TRUE
#define TRUE 1
#endif

#ifndef FALSE
#define FALSE 0
#endif

#ifndef SENSICAM_TYPES
#define SENSICAM_TYPES
typedef unsigned char byte;            /* 8-bit  */
typedef unsigned short word;           /* 16-bit */
typedef unsigned int dword;            /* 32-bit */
#endif

#define MAX_LUT 60
#define MAX_OUTSIZE 256
#define MAX_BITPIX 16
#define MAX_TYPES 2


struct bw_lut
{
  int min;
  int max;
  int typ;
  int mid;
  int min_out;
  int max_out;
  int size;
  void *ptr;
};

typedef struct bw_lut BWLUT;


struct color_lut
{
  BWLUT red;
  BWLUT green;
  BWLUT blue;
};

struct pseudo_lut
{
  BWLUT bw;
  int psize;
  byte *red_ptr;
  byte *green_ptr;
  byte *blue_ptr;
};

typedef struct color_lut COLORLUT;
typedef struct pseudo_lut PSEUDOLUT;


#endif                                 /* CNV_S_H */
