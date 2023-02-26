#pragma once

#include <map>
#include <vector>
#include <string>
#include <qsharedpointer.h>

#include "DebugPrintOut.h"
#include "VideoInput.h"

struct IMFActivate;

struct IMFMediaSource;

struct IMFMediaType;

class ImageGrabberThread;

class RawImage;

typedef std::wstring String;

typedef std::vector<int> vectorNum;

typedef std::map<String, vectorNum> SUBTYPEMap;

typedef std::map<UINT64, SUBTYPEMap> FrameRateMap;

typedef void(*emergensyStopEventCallback)(int, void *);

/// Class for controlling of video device
class VideoDevice
{

public:
    VideoDevice(void);
    ~VideoDevice(void);

    void closeDevice();
    
    CamParameters getParameters();

    void setParameters(CamParameters parameters);

    void setEmergencyStopEvent(void *userData, void(*func)(int, void *));
    
    long readInfoOfDevice(IMFActivate *pActivate, unsigned int Num);
        
    wchar_t *getName();

    size_t getCountFormats();

    unsigned int getWidth();
    
    unsigned int getHeight();

    MediaType getFormat(unsigned int id);
    
    bool setupDevice(unsigned int w, unsigned int h, unsigned int idealFramerate = 0);

    bool setupDevice(unsigned int id);

    bool isDeviceSetup();

    bool isDeviceMediaSource();
    
    bool isDeviceRawDataSource();

    bool isFrameNew();

    IMFMediaSource *getMediaSource();

    RawImage *getRawImageOut();

private:

    long enumerateCaptureFormats(IMFMediaSource *pSource);

    long setDeviceFormat(IMFMediaSource *pSource, unsigned long dwFormatIndex);

    void buildLibraryofTypes();

    int findType(unsigned int size, unsigned int frameRate = 0);

    long resetDevice(IMFActivate *pActivate);

    long initDevice();

    long checkDevice(IMFAttributes *pAttributes, IMFActivate **pDevice);

    enum typeLock
    {
        MediaSourceLock,

        RawDataLock,

        OpenLock

    } vd_LockOut;
    
    wchar_t *m_pFriendlyName;

    ImageGrabberThread *m_pImGrTh;

    CamParameters m_prevParameters;

    unsigned int m_width;

    unsigned int m_height;

    unsigned int m_currentNumber;

    bool m_isSetuped;
        
    std::map<UINT64, FrameRateMap> m_captureFormats;
    
    std::vector<MediaType> m_currentFormats;

    IMFMediaSource *m_pSource;

    emergensyStopEventCallback m_func;

    void *m_userData;
    
    QSharedPointer<DebugPrintOut> m_debugPrintOut;
    

};

