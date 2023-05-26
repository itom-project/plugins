#pragma once

#include <windows.h>
#include <qmutex.h>
#include <qsharedpointer.h>
#include "DebugPrintOut.h"

unsigned int WINAPI MainThreadFunction( LPVOID lpParam );

class ImageGrabber;

struct IMFMediaSource;

typedef void(*emergensyStopEventCallback)(int, void *);

/// Class for controlling of thread of the grabbing raw data from video device
class ImageGrabberThread
{
    friend unsigned int WINAPI MainThreadFunction( LPVOID lpParam );

public:
    ~ImageGrabberThread(void);

    static HRESULT CreateInstance(ImageGrabberThread **ppIGT, IMFMediaSource *pSource, unsigned int deviceID, QSharedPointer<DebugPrintOut> debugPrintOut);

    void start();

    void stop();

    void setEmergencyStopEvent(void *userData, void(*func)(int, void *));

    ImageGrabber *getImageGrabber();

    QMutex runMutex;

protected:

    virtual void run();

private:

    ImageGrabberThread(IMFMediaSource *pSource, unsigned int deviceID, QSharedPointer<DebugPrintOut> debugPrintOut);

    uintptr_t m_igtHandle;

    unsigned int m_igtThreadIdArray;

    ImageGrabber *m_pIgtImageGrabber;

    emergensyStopEventCallback m_igtFunc;

    void *m_igtUserData;

    bool m_igtStop;

    unsigned int m_igtDeviceID;

    QSharedPointer<DebugPrintOut> m_debugPrintOut;

};
