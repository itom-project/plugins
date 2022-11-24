#pragma once

#include <QAtomicInteger>

typedef unsigned char BYTE;

class RawImage
{
public:
    ~RawImage(void);

        // Function of creation of the instance of the class
    static long CreateInstance(RawImage **ppRImage,unsigned int size);

    void setCopy(const BYTE * pSampleBuffer);
    
    void fastCopy(const BYTE * pSampleBuffer);

    unsigned char * getpPixels();

    bool isNew();

    unsigned int getSize();

private:
    
    QAtomicInt ri_new;

    unsigned int ri_size;

    unsigned char *ri_pixels;
    
    RawImage(unsigned int size);

};

