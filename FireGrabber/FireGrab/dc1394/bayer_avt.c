/*
 * 1394-Based Digital Camera Control Library
 *
 * AVT Bayer pattern decoding functions
 *
 * Copyright (C) 2010 Allied Vision Technologies GmbH
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#include <stdint.h>
#include <dc1394/types.h>
#include <dc1394/log.h>
#include <string.h>

#ifndef MIN
  #define MIN(a,b) ((a) < (b) ? (a) : (b))
#endif
#ifndef MAX
  #define MAX(a,b) ((a) < (b) ? (b) : (a))
#endif

//! Structure for accessing data in the YUV 4:4:4 format (YUV)
typedef struct
{
    uint8_t Y;   //!< Luma
    uint8_t U;   //!< U
    uint8_t V;   //!< V
} S_YUV444;

//! Structure for accessing data in the YUV 4:2:2 format (UYVY)
typedef struct
{
    uint8_t U;   //!< the U part for both pixels
    uint8_t Y0;  //!< the intensity of the first pixel
    uint8_t V;   //!< the V part for both pixels
    uint8_t Y1;  //!< the intensity of the second pixel
}  S_YUV422;

//! Structure for accessing data in the 24 bit RGB format
typedef struct
{
    uint8_t R;   //!< red
    uint8_t G;   //!< green
    uint8_t B;   //!< blue
}  S_RGB8;

////////////////////////////////////////////////////////////////////////////////
// Convert raw mode with 8 bit values
////////////////////////////////////////////////////////////////////////////////

void dc1394_avt_Raw8ToYUV444(uint8_t *pDst1, const uint8_t *pSrc, uint32_t XSize, uint32_t YSize, dc1394color_filter_t bayerPattern)
{
    const uint8_t    *pBuf    =   pSrc;
    const uint8_t    *pR,*pB,*pG0,*pG1; // G0:the green pixel in the same column as the red pixel
    uint32_t         i,j;
    S_YUV444 *pDst         = (S_YUV444*) pDst1;
    uint32_t uColorOffsetY = (uint32_t) bayerPattern/2;
    uint32_t uColorOffsetX = (uint32_t) bayerPattern%2;
    int16_t U;
    int16_t V;
    uint8_t G;


    for(i=uColorOffsetY; i<YSize+uColorOffsetY-1; i++,pBuf+=XSize)
    {
        switch((i&1)+2*uColorOffsetX)
        {
        case 3:
            pB  = pBuf;
            pG0 = pB+1;
            pG1 = pB+XSize;
            pR  = pG1+1;
            break;
        case 2:
            pG1 = pBuf;
            pR  = pG1+1;
            pB  = pG1+XSize;
            pG0 = pB+1;
            break;
        case 1:
            pG0 = pBuf;
            pB  = pG0+1;
            pR  = pG0+XSize;
            pG1 = pR+1;
            break;
        case 0:
        default:
            pR  = pBuf;
            pG1 = pR+1;
            pG0 = pR+XSize;
            pB  = pG0+1;
            break;
        }

        // Go through all pixels
        for(j=uColorOffsetX;j<XSize+uColorOffsetX-1;j++)
        {
            G       = (uint8_t)(((uint16_t)*pG0+*pG1)/2);
            U       = ((*pB)<<7) - 43* *pR - 85* G;
            V       = ((*pR)<<7) -107* G - 21* *pB;
            pDst->Y = (uint8_t)(( 77* *pR +150* G + 29* *pB) >>8);
            pDst->U = (uint8_t)((U>>8) + 128);
            pDst->V = (uint8_t)((V>>8) + 128);

            pDst++;

            if(j&1)
            {
                pB  +=2;
                pG1 +=2;
            }
            else
            {
                pR  +=2;
                pG0 +=2;
            }
        }
        pDst->Y = (pDst-1)->Y;
        pDst->U = (pDst-1)->U;
        pDst->V = (pDst-1)->V;

        pDst++;
    }

    memcpy(pDst,pDst-XSize,XSize*sizeof(S_YUV444));
}


/*
 * Inplace YUV444->YUV422 conversion with horizontal averaging.
 * A 4x1 averaging window is used.
 * Image width is expected to be a multiple of 2.
 * The resulting data is not packed in YUV 422 format, data format is still YUV444.
 * However, valid U and V components are stored only for pixels with even X-coordinate.
 */
void dc1394_avt_YUV_averaging_4(uint8_t *pDst, uint32_t XSize, uint32_t YSize)
{
    S_YUV444 *dst = (S_YUV444*) pDst;

    S_YUV444 *pLineEnd = dst + XSize - 4;
    S_YUV444 *pFrameEnd = dst + (XSize*YSize) - 2;

    for(;;)
    {
        dst[0].U = (uint8_t) ( ( (uint16_t) dst[0].U + dst[1].U + dst[2].U + dst[3].U ) / 4 );
        dst[0].V = (uint8_t) ( ( (uint16_t) dst[0].V + dst[1].V + dst[2].V + dst[3].V ) / 4 );

        if( dst == pLineEnd )
        {
            /* a smaller averaging window is used for the last column */
            dst += 2;
            dst[0].U = (uint8_t) ( ( (uint16_t) dst[0].U + dst[1].U ) / 2 );
            dst[0].V = (uint8_t) ( ( (uint16_t) dst[0].V + dst[1].V ) / 2 );
            if( dst == pFrameEnd )
            {
                break;
            }
            pLineEnd += XSize;
        }
        dst += 2;
    }
}


/*
 * Inplace YUV444->YUV422 conversion with horizontal averaging.
 * A 2x1 averaging window is used.
 * Image width is expected to be a multiple of 2.
 * The resulting data is not packed in YUV 422 format, data format is still YUV444.
 * However, valid U and V components are stored only for pixels with even X-coordinate.
 */
void dc1394_avt_YUV_averaging_2(void *pDst, uint32_t XSize, uint32_t YSize)

{

    S_YUV444        *dst    = (S_YUV444*) ( pDst);

    S_YUV444        *pFrameEnd = dst+(XSize*YSize);

    for(;;)
    {
        dst[0].U = (uint8_t) ( ( (uint16_t) dst[0].U + dst[1].U ) / 2 );
        dst[0].V = (uint8_t) ( ( (uint16_t) dst[0].V + dst[1].V ) / 2 );
        dst += 2;
        if( dst >= pFrameEnd )
        {
            break;
        }
    }
}


/*
 * Inplace YUV444->YUV422 conversion with horizontal averaging.
 * A 4x2 averaging window is used.
 * Image width is expected to be a multiple of 2.
 * The resulting data is not packed in YUV 422 format, data format is still YUV444.
 * However, valid U and V components are stored only for pixels with even X-coordinate.
 */
void dc1394_avt_YUV_averaging_42(uint8_t *pDst, uint32_t XSize, uint32_t YSize)
{
    S_YUV444 *pYUV         = (S_YUV444 *) pDst;
    S_YUV444 *pYUV_NextRow = pYUV + XSize;
    S_YUV444 *pLineEnd     = pYUV + XSize - 4;
    S_YUV444 *pFrameEnd    = pYUV + (XSize*(YSize-1));

    /* special treatment for first pixel */
    pYUV[0].U = (uint8_t) ( ( (uint16_t) pYUV[0].U + pYUV[1].U + pYUV[2].U +
                              pYUV_NextRow[0].U + pYUV_NextRow[1].U + pYUV_NextRow[2].U + 3 ) / 6 );
    pYUV[0].V = (uint8_t) ( ( (uint16_t) pYUV[0].V + pYUV[1].V + pYUV[2].V +
                              pYUV_NextRow[0].V + pYUV_NextRow[1].V + pYUV_NextRow[2].V + 3 ) / 6 );

    for (;;)
    {
        /* main loop - standard processing */
        pYUV += 2;
        pYUV_NextRow += 2;
        pYUV[0].U = (uint8_t) ( ( (uint16_t) pYUV[-1].U + pYUV[0].U + pYUV[1].U + pYUV[2].U +
                                  pYUV_NextRow[-1].U + pYUV_NextRow[0].U + pYUV_NextRow[1].U + pYUV_NextRow[2].U + 4 ) / 8 );
        pYUV[0].V = (uint8_t) ( ( (uint16_t) pYUV[-1].V + pYUV[0].V + pYUV[1].V + pYUV[2].V +
                                  pYUV_NextRow[-1].V + pYUV_NextRow[0].V + pYUV_NextRow[1].V + pYUV_NextRow[2].V + 4 ) / 8 );

        if( pYUV == pLineEnd )
        {
            /* special treatment for last pixel of each row */
            pYUV += 2;
            pYUV_NextRow += 2;
            pYUV[0].U = (uint8_t) ( ( (uint16_t) pYUV[-1].U + pYUV[0].U + pYUV[1].U +
                                  pYUV_NextRow[-1].U + pYUV_NextRow[0].U + pYUV_NextRow[1].U + 3 ) / 6 );
            pYUV[0].V = (uint8_t) ( ( (uint16_t) pYUV[-1].V + pYUV[0].V + pYUV[1].V  +
                                  pYUV_NextRow[-1].V + pYUV_NextRow[0].V + pYUV_NextRow[1].V + 3 ) / 6 );
            pLineEnd += XSize;

            pYUV += 2;
            pYUV_NextRow += 2;
            if ( pYUV == pFrameEnd )
            {
                break;
            }

            /* special treatment for first pixel of each row */
            pYUV[0].U = (uint8_t) ( ( (uint16_t) pYUV[0].U + pYUV[1].U + pYUV[2].U +
                              pYUV_NextRow[0].U + pYUV_NextRow[1].U + pYUV_NextRow[2].U + 3 ) / 6 );
            pYUV[0].V = (uint8_t) ( ( (uint16_t) pYUV[0].V + pYUV[1].V + pYUV[2].V +
                              pYUV_NextRow[0].V + pYUV_NextRow[1].V + pYUV_NextRow[2].V + 3 ) / 6 );
        }
    }

    /* special treatment for last line */
    pYUV[0].U = (uint8_t) ( ( (uint16_t) pYUV[0].U + pYUV[1].U + pYUV[2].U + 1 ) / 3 );
    pYUV[0].V = (uint8_t) ( ( (uint16_t) pYUV[0].V + pYUV[1].V + pYUV[2].V + 1 ) / 3 );
    for (;;)
    {
        pYUV += 2;
        pYUV[0].U = (uint8_t) ( ( (uint16_t) pYUV[-1].U + pYUV[0].U + pYUV[1].U + pYUV[2].U + 2 ) / 4 );
        pYUV[0].V = (uint8_t) ( ( (uint16_t) pYUV[-1].V + pYUV[0].V + pYUV[1].V + pYUV[2].V + 2 ) / 4 );
        if( pYUV == pLineEnd )
        {
            /* special treatment for last pixel */
            pYUV += 2;
            pYUV[0].U = (uint8_t) ( ( (uint16_t) pYUV[-1].U + pYUV[0].U + pYUV[1].U + 1 ) / 3 );
            pYUV[0].V = (uint8_t) ( ( (uint16_t) pYUV[-1].V + pYUV[0].V + pYUV[1].V + 1 ) / 3 );
            break;
        }
    }
}



/*
 * Inplace YUV422->RGB conversion.
 * Image width is expected to be a multiple of 2.
 * The input data is expected to be in "YUV422 unpacked" format as produced by
 * dc1394_avt_YUV_averaging_2, dc1394_avt_YUV_averaging_4, dc1394_avt_YUV_averaging_42.
 */
void dc1394_avt_YUV422UnpackedToRGB(uint8_t *pDst, uint32_t nPixelCount)
{
    S_RGB8 *rgb = (S_RGB8*) (pDst);
    S_RGB8 *pFrameEnd = rgb + nPixelCount - 2;
    for(;;)
    {
        S_YUV444 *yuv = (S_YUV444*) rgb;
        uint8_t Y0 = yuv[0].Y;
        uint8_t U  = yuv[0].U;
        uint8_t V  = yuv[0].V;
        uint8_t Y1 = yuv[1].Y;

        const int32_t vr      = (int16_t) ( (float) (V-128) * 1.4022 );
        const int32_t uvg     = (int16_t) ( (float) (V-128) * (-0.7144) ) +
                                 (int16_t) ( (float) (U-128) * (-0.3457) );
        const int32_t ub      = (int16_t) ( (float) (U-128) * 1.7710 );

        rgb[0].R = MAX( MIN( (int32_t) Y0 + vr , 255 ), 0);
        rgb[0].G = MAX( MIN( (int32_t) Y0 + uvg, 255 ), 0);
        rgb[0].B = MAX( MIN( (int32_t) Y0 + ub , 255 ), 0);
        rgb[1].R = MAX( MIN( (int32_t) Y1 + vr , 255 ), 0);
        rgb[1].G = MAX( MIN( (int32_t) Y1 + uvg, 255 ), 0);
        rgb[1].B = MAX( MIN( (int32_t) Y1 + ub , 255 ), 0);

        if( rgb == pFrameEnd )
        {
            break;
        }
        rgb += 2;
    }
}

void dc1394_avt_Raw8ToRGB_YUV422(uint8_t *restrict pDst, const uint8_t *restrict pSrc, uint32_t XSize, uint32_t YSize, dc1394color_filter_t bayerPattern)
{
    dc1394_avt_Raw8ToYUV444(pDst, pSrc, XSize, YSize, bayerPattern);
    dc1394_avt_YUV_averaging_2( pDst, XSize, YSize);
    dc1394_avt_YUV422UnpackedToRGB(pDst, XSize*YSize );
}

void dc1394_avt_Raw8ToRGB_LCAA(uint8_t *restrict pDst, const uint8_t *restrict pSrc, uint32_t XSize, uint32_t YSize, dc1394color_filter_t bayerPattern)
{
    dc1394_avt_Raw8ToYUV444(pDst, pSrc, XSize, YSize, bayerPattern);
    dc1394_avt_YUV_averaging_4(pDst, XSize, YSize );
    dc1394_avt_YUV422UnpackedToRGB(pDst, XSize*YSize);
}

void dc1394_avt_Raw8ToRGB_LCAAV(uint8_t *restrict pDst, const uint8_t *restrict pSrc, uint32_t XSize, uint32_t YSize, dc1394color_filter_t bayerPattern)
{
    dc1394_avt_Raw8ToYUV444(pDst, pSrc, XSize, YSize, bayerPattern);
    dc1394_avt_YUV_averaging_42( pDst, XSize, YSize);
    dc1394_avt_YUV422UnpackedToRGB(pDst, XSize*YSize);
}
