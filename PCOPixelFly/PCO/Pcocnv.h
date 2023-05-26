/*
// The following ifdef block is the standard way of creating macros which make exporting
// from a DLL simpler. All files within this DLL are compiled with the PCOCNV_EXPORTS
// symbol defined on the command line. this symbol should not be defined on any project
// that uses this DLL. This way any other project whose source files include this file see
// PCOCNV_API functions as being imported from a DLL, wheras this DLL sees symbols
// defined with this macro as being exported.
#ifdef PCOCNV_EXPORTS
#define PCOCNV_API __declspec(dllexport)
#else
#define PCOCNV_API __declspec(dllimport)
#endif
*/
#define PCOCNV_API

#ifdef __cplusplus
extern "C" {
#endif  /* __cplusplus */


#include "cnv_s.h"


PCOCNV_API BWLUT* CREATE_BWLUT(int bitpix, int min_out, int max_out);
//creates a structure BWLUT
//allocates memory for the structure and for 2^bitpix bytes
//bitpix: bits per pixel of the picture data (i.e. 12)
//min_out: lowest value of output in table  0...254
//max_out: highest value of output in table 1...255

PCOCNV_API int DELETE_BWLUT(BWLUT *lut);
//delete the LUT
//free all allocated memory

PCOCNV_API COLORLUT* CREATE_COLORLUT(int bitpix, int min_out, int max_out);
//creates a structure COLORLUT, which exists of three BWLUT's
//one for each color red, green and blue
//allocates memory for the structure and for 2^bitpix bytes
//bitpix: bits per pixel of the picture data (i.e. 12)
//min_out: lowest value of output in table  0...254
//max_out: highest value of output in table 1...255

PCOCNV_API int DELETE_COLORLUT(COLORLUT *lut);
//delete the LUT
//free all allocated memory

PCOCNV_API PSEUDOLUT* CREATE_PSEUDOLUT(int bitpix, int min_out, int max_out);
//creates a structure PSEUDOLUT, which exists of one BWLUT
//and one table for each color red, green and blue
//allocates memory for the structure and for 3 * ( 2^bitpix ) bytes
//bitpix: bits per pixel of the picture data (i.e. 12)
//min_out: lowest value of output in table  0...254
//max_out: highest value of output in table 1...255

PCOCNV_API int DELETE_PSEUDOLUT(PSEUDOLUT *lut);
//delete the LUT
//free all allocated memory

PCOCNV_API void CONVERT_SET(BWLUT *lut,int min,int max,int typ);
//set the range within which the data of the picture is
//to be converted into 8bit data.
//New values for the table are calculated
//lut: BWLUT to set
//min: minimal value of input 0...2^bitpix-2
//max: maximal value of input 1...2^bitpix-1
//condition: min<max
//typ: 0x00 = linear table
//     0x01 = logarithmic table
//     0x80 = invers linear table
//     0x81 = invers logarithmic table



PCOCNV_API int LOAD_PSEUDO_LUT(int format,char *filename,PSEUDOLUT *plut);
//load the three pseudolut color tables of plut
//from the file filename
//which includes data in the following formats

//plut:   PSEUDOLUT to write data in
//filename: name of file with data
//format: 0 = binary 256*RGB
//        1 = binary 256*R,256*G,256*R
//        2 = ASCII  256*RGB
//        3 = ASCII  256*R,256*G,256*R





PCOCNV_API int CONV_BUF_12TO8(int mode, int width,int height, word *b12, byte *b8,BWLUT *lut);
//convert picture data in b12 to 8bit data in b8
//through table in structure of BWLUT
//mode:   0       = normal
//        bit0: 1 = flip
//        bit3: 1 = mirror
//width:  width of picture
//height: height of picture
//b12:    pointer to picture data array
//b8:     pointer to byte data array
//lut:    pointer to structure of typ BWLUT

PCOCNV_API int CONV_BUF_12TOCOL(int mode, int width, int height, word *gb12, byte *gb8,COLORLUT *clut);
//convert picture data in b12 to 24bit or 32bit data in b8
//through tables in structure COLORLUT
//mode:   0       = normal to 24bit BGR
//        bit0: 1 = flip
//        bit1: 1 = 32bit BGR0
//        bit3: 1 = mirror
//        bit5: 1 = low average
//width:  width of picture
//height: height of picture
//b12:    pointer to picture data array
//b8:     pointer to byte data array (RGBx)
//lut:    pointer to structure of typ COLORLUT

PCOCNV_API int CONV_BUF_12TOPSEUDO(int mode, int width, int height, word *gb12, byte *gb8,PSEUDOLUT *plut);
//convert picture data in b12 to 24bit or 32bit data in b8
//through tables in structure PSEUDOLUT
//mode:   0       = normal to 24bit BGR
//        bit0: 1 = flip
//        bit1: 1 = 32bit BGR0
//        bit3: 1 = mirror

//width:  width of picture
//height: height of picture
//b12:    pointer to picture data array
//b8:     pointer to byte data array (RGBx)
//lut:    pointer to structure of typ PSEUDOLUT


#ifdef __cplusplus
}
#endif
