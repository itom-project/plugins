#pragma once

#include <qsharedpointer.h>
#include "DebugPrintOut.h"
#include "VideoDevices.h"

struct IMFAttributes;

// Class for creating of Media Foundation context
class Media_Foundation
{
public:
    Media_Foundation(QSharedPointer<DebugPrintOut> debugPrintOut);

    virtual ~Media_Foundation(void);

    //static Media_Foundation& getInstance();

    bool buildListOfDevices(QSharedPointer<VideoDevices> videoDevices);

private: 
        
    

    QSharedPointer<DebugPrintOut> m_debugPrintOut;

};

