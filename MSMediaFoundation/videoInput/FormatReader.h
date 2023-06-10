#pragma once

#include <guiddef.h>

#include "VideoInput.h"

struct IMFMediaType;

/// Class for parsing info from IMFMediaType into the local MediaType
class FormatReader
{
public:
    static MediaType Read(IMFMediaType *pType);
    ~FormatReader(void);
private:
    FormatReader(void);
};
