#include <new>
#include <stdio.h>
#include <windows.h>
#include <Mfapi.h>


#include "RawImage.h"


RawImage::RawImage(unsigned int size): ri_new(0), ri_pixels(NULL)
{
    ri_size = size;

    ri_pixels = new unsigned char[size];

    memset((void *)ri_pixels,0,ri_size);
}

bool RawImage::isNew()
{
    return (ri_new > 0);
}

unsigned int RawImage::getSize()
{
    return ri_size;
}

RawImage::~RawImage(void)
{
    delete []ri_pixels;

    ri_pixels = NULL;
}

long RawImage::CreateInstance(RawImage **ppRImage,unsigned int size)
{
    *ppRImage = new (std::nothrow) RawImage(size);

    if (ppRImage == NULL)
    {
        return E_OUTOFMEMORY;
    }
    return S_OK;
}

void RawImage::setCopy(const BYTE * pSampleBuffer)
{
    memcpy(ri_pixels, pSampleBuffer, ri_size);

    ri_new = 1;
}

void RawImage::fastCopy(const BYTE * pSampleBuffer)
{
#if _WIN64
    memcpy(ri_pixels, pSampleBuffer, ri_size);
#else
    int *bsrc = (int *)pSampleBuffer;

    int *dst = (int *)ri_pixels;

    unsigned int buffersize = ri_size/4;

    _asm
    {
        mov ESI, bsrc

        mov EDI, dst

        mov ECX, buffersize

        cld

        rep movsd
    }
#endif

    ri_new = 1;
}

unsigned char * RawImage::getpPixels()
{
    ri_new = 0;

    return ri_pixels;
}
