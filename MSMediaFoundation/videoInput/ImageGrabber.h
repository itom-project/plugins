#pragma once

#include <mfidl.h>
#include <qelapsedtimer.h>
#include <qsharedpointer.h>
#include "DebugPrintOut.h"

struct IMFMediaSource;

class RawImage;

// Class for grabbing image from video stream
class ImageGrabber : public IMFSampleGrabberSinkCallback
{
public:
    ~ImageGrabber();

    HRESULT initImageGrabber(IMFMediaSource *pSource, GUID VideoFormat);

    HRESULT startGrabbing(void);

    void stopGrabbing();

    RawImage *getRawImage();

    // Function of creation of the instance of the class
    static HRESULT CreateInstance(ImageGrabber **ppIG,unsigned int deviceID, QSharedPointer<DebugPrintOut> debugPrintOut);

private:

    bool ig_RIE;

    bool ig_Close;

    long m_cRef;

    unsigned int ig_DeviceID;

    IMFMediaSource *ig_pSource;

    IMFMediaSession *ig_pSession;

    IMFTopology *ig_pTopology;

    RawImage *ig_RIFirst;

    RawImage *ig_RISecond;

    RawImage *ig_RIOut;

    QElapsedTimer m_stopTimer;

    QSharedPointer<DebugPrintOut> m_debugPrintOut;

    ImageGrabber(unsigned int deviceID, QSharedPointer<DebugPrintOut> debugPrintOut);

    HRESULT CreateTopology(IMFMediaSource *pSource, IMFActivate *pSinkActivate, IMFTopology **ppTopo);

    HRESULT AddSourceNode(
    IMFTopology *pTopology,
    IMFMediaSource *pSource,
    IMFPresentationDescriptor *pPD,
    IMFStreamDescriptor *pSD,
    IMFTopologyNode **ppNode);

    HRESULT AddOutputNode(
    IMFTopology *pTopology,
    IMFActivate *pActivate,
    DWORD dwId,
    IMFTopologyNode **ppNode);

    // IUnknown methods
    STDMETHODIMP QueryInterface(REFIID iid, void** ppv);
    STDMETHODIMP_(ULONG) AddRef();
    STDMETHODIMP_(ULONG) Release();

    // IMFClockStateSink methods
    STDMETHODIMP OnClockStart(MFTIME hnsSystemTime, LONGLONG llClockStartOffset);
    STDMETHODIMP OnClockStop(MFTIME hnsSystemTime);
    STDMETHODIMP OnClockPause(MFTIME hnsSystemTime);
    STDMETHODIMP OnClockRestart(MFTIME hnsSystemTime);
    STDMETHODIMP OnClockSetRate(MFTIME hnsSystemTime, float flRate);

    // IMFSampleGrabberSinkCallback methods
    STDMETHODIMP OnSetPresentationClock(IMFPresentationClock* pClock);
    STDMETHODIMP OnProcessSample(REFGUID guidMajorMediaType, DWORD dwSampleFlags,
        LONGLONG llSampleTime, LONGLONG llSampleDuration, const BYTE * pSampleBuffer,
        DWORD dwSampleSize);
    STDMETHODIMP OnShutdown();


};
