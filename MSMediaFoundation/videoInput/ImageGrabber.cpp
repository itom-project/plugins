#include <new>
#include <windows.h>
#include <mfapi.h>
#include <mfidl.h>
#include <mfobjects.h>
#include <Mferror.h>
#include <Shlwapi.h>
#include <stdio.h>

#include <qdebug.h>


#include "ImageGrabber.h"
#include "Common.h"
#include "DebugPrintOut.h"
#include "RawImage.h"
#include "FormatReader.h"


#pragma comment(lib, "mfplat")
#pragma comment(lib, "mf")
#pragma comment(lib, "mfuuid")
#pragma comment(lib, "Shlwapi")


#define CHECK_HR(x) if (FAILED(x)) { goto done; }

//---------------------------------------------------------------------------------------
ImageGrabber::ImageGrabber(unsigned int deviceID, QSharedPointer<DebugPrintOut> debugPrintOut):
    m_cRef(1),
    ig_DeviceID(deviceID),
    ig_pSource(NULL),
    ig_pSession(NULL),
    ig_pTopology(NULL),
    ig_RIE(true),
    ig_Close(false),
    ig_RIFirst(NULL),
    ig_RISecond(NULL),
    ig_RIOut(NULL),
    m_debugPrintOut(debugPrintOut)
{
}

//---------------------------------------------------------------------------------------
ImageGrabber::~ImageGrabber(void)
{
    if (ig_pSession)
    {
        ig_pSession->Shutdown();
    }

    //SafeRelease(&ig_pSession);

    //SafeRelease(&ig_pTopology);

    delete ig_RIFirst;
    ig_RIFirst = NULL;
    delete ig_RISecond;
    ig_RISecond = NULL;

    m_debugPrintOut->printOut("IMAGEGRABBER VideoDevice %i: Destroying instance of the ImageGrabber class \n", ig_DeviceID);

}

HRESULT ImageGrabber::initImageGrabber(IMFMediaSource *pSource, GUID VideoFormat)
{
    IMFActivate *pSinkActivate = NULL;
    IMFMediaType *pType = NULL;

    IMFPresentationDescriptor *pPD = NULL;
    IMFStreamDescriptor *pSD = NULL;
    IMFMediaTypeHandler *pHandler = NULL;
    IMFMediaType *pCurrentType = NULL;

    HRESULT hr = S_OK;
    MediaType MT;

     // Clean up.
    if (ig_pSession)
    {
        ig_pSession->Shutdown();
    }

    SafeRelease(&ig_pSession);
    SafeRelease(&ig_pTopology);

    ig_pSource = pSource;
    hr = pSource->CreatePresentationDescriptor(&pPD);
    DWORD cTypes = 0;

    if (FAILED(hr))
    {
        goto err;
    }

    BOOL fSelected;
    hr = pPD->GetStreamDescriptorByIndex(0, &fSelected, &pSD);

    if (FAILED(hr))
    {
        goto err;
    }

    hr = pSD->GetMediaTypeHandler(&pHandler);

    if (FAILED(hr))
    {
        goto err;
    }

    hr = pHandler->GetMediaTypeCount(&cTypes);

    if (FAILED(hr))
    {
        goto err;
    }

    if (cTypes > 0)
    {
        hr = pHandler->GetCurrentMediaType(&pCurrentType);

        if (FAILED(hr))
        {
            goto err;
        }

        MT = FormatReader::Read(pCurrentType);
    }

err:
    SafeRelease(&pPD);
    SafeRelease(&pSD);
    SafeRelease(&pHandler);
    SafeRelease(&pCurrentType);

    unsigned int sizeRawImage = 0;

    if (VideoFormat == MFVideoFormat_RGB24)
    {
        sizeRawImage = MT.MF_MT_FRAME_SIZE * 3;
    }
    else
    if (VideoFormat == MFVideoFormat_RGB32)
    {
        sizeRawImage = MT.MF_MT_FRAME_SIZE * 4;
    }

    CHECK_HR(hr = RawImage::CreateInstance(&ig_RIFirst, sizeRawImage));

    CHECK_HR(hr = RawImage::CreateInstance(&ig_RISecond, sizeRawImage));

    ig_RIOut = ig_RISecond;


    // Configure the media type that the Sample Grabber will receive.
    // Setting the major and subtype is usually enough for the topology loader
    // to resolve the topology.

    CHECK_HR(hr = MFCreateMediaType(&pType));
    CHECK_HR(hr = pType->SetGUID(MF_MT_MAJOR_TYPE, MFMediaType_Video));
    CHECK_HR(hr = pType->SetGUID(MF_MT_SUBTYPE, VideoFormat));

    // Create the sample grabber sink.
    CHECK_HR(hr = MFCreateSampleGrabberSinkActivate(pType, this, &pSinkActivate));

    // To run as fast as possible, set this attribute (requires Windows 7):
    CHECK_HR(hr = pSinkActivate->SetUINT32(MF_SAMPLEGRABBERSINK_IGNORE_CLOCK, TRUE));

    // Create the Media Session.

    CHECK_HR(hr = MFCreateMediaSession(NULL, &ig_pSession));

    // Create the topology.
    CHECK_HR(hr = CreateTopology(pSource, pSinkActivate, &ig_pTopology));

done:

    // Clean up.
    if (FAILED(hr))
    {
        if (ig_pSession)
        {
            ig_pSession->Shutdown();
        }

        SafeRelease(&ig_pSession);
        SafeRelease(&ig_pTopology);
    }

    SafeRelease(&pSinkActivate);
    SafeRelease(&pType);

    return hr;
}

void ImageGrabber::stopGrabbing()
{
    if (ig_pSession)
    {
        HRESULT hr = ig_pSession->Stop();
        if (hr != S_OK)
        {
            qDebug() << "could not stop";
        }

        m_stopTimer.start();
    }


    m_debugPrintOut->printOut("IMAGEGRABBER VideoDevice %i: Stopping of of grabbing of images\n", ig_DeviceID);
}

HRESULT ImageGrabber::startGrabbing(void)
{
    IMFMediaEvent *pEvent = NULL;

    PROPVARIANT var;
    PropVariantInit(&var);

    HRESULT hr = S_OK;
    HRESULT hrStatus = S_OK;
    MediaEventType met;

    CHECK_HR(hr = ig_pSession->SetTopology(0, ig_pTopology));
    CHECK_HR(hr = ig_pSession->Start(&GUID_NULL, &var));

    m_debugPrintOut->printOut("IMAGEGRABBER VideoDevice %i: Start Grabbing of the images\n", ig_DeviceID);

    while (1)
    {
        if (!ig_pSession)
        {
            break;
        }

        hr = ig_pSession->GetEvent(MF_EVENT_FLAG_NO_WAIT, &pEvent);

        if (hr == MF_E_NO_EVENTS_AVAILABLE)
        {
            if (m_stopTimer.isValid() == true && m_stopTimer.elapsed() > 2000)
            {
                qDebug() << "MSMediaFoundation MediaSession did not sent a stop event. Stopping is forced...";
                hr = S_OK;
                goto done;
            }

            Sleep(100);
            continue;
        }

        if (!SUCCEEDED(hr))
        {
            hr = S_OK;
            goto done;
        }

        hr = pEvent->GetStatus(&hrStatus);
        if (!SUCCEEDED(hr))
        {
            hr = S_OK;
            goto done;
        }

        hr = pEvent->GetType(&met);
        if (!SUCCEEDED(hr))
        {
            hr = S_OK;

            goto done;
        }

        if (met == MESessionEnded)
        {
            m_debugPrintOut->printOut("IMAGEGRABBER VideoDevice %i: MESessionEnded \n", ig_DeviceID);

            ig_pSession->Stop();
            break;
        }

        if (met == MESessionStopped)
        {
            m_debugPrintOut->printOut("IMAGEGRABBER VideoDevice %i: MESessionStopped \n", ig_DeviceID);
            break;
        }


        /*if (met == MEVideoCaptureDeviceRemoved)
        {
            m_debugPrintOut->printOut("IMAGEGRABBER VideoDevice %i: MEVideoCaptureDeviceRemoved \n", ig_DeviceID);

            break;
        }*/

        SafeRelease(&pEvent);
    }

    m_debugPrintOut->printOut("IMAGEGRABBER VideoDevice %i: Finish startGrabbing \n", ig_DeviceID);

done:
    SafeRelease(&pEvent);

    SafeRelease(&ig_pSession);

    SafeRelease(&ig_pTopology);

    return hr;
}

HRESULT ImageGrabber::CreateTopology(IMFMediaSource *pSource, IMFActivate *pSinkActivate, IMFTopology **ppTopo)
{
    IMFTopology *pTopology = NULL;
    IMFPresentationDescriptor *pPD = NULL;
    IMFStreamDescriptor *pSD = NULL;
    IMFMediaTypeHandler *pHandler = NULL;
    IMFTopologyNode *pNode1 = NULL;
    IMFTopologyNode *pNode2 = NULL;

    HRESULT hr = S_OK;
    DWORD cStreams = 0;

    CHECK_HR(hr = MFCreateTopology(&pTopology));
    CHECK_HR(hr = pSource->CreatePresentationDescriptor(&pPD));
    CHECK_HR(hr = pPD->GetStreamDescriptorCount(&cStreams));

    for (DWORD i = 0; i < cStreams; i++)
    {
        // In this example, we look for audio streams and connect them to the sink.

        BOOL fSelected = FALSE;
        GUID majorType;

        CHECK_HR(hr = pPD->GetStreamDescriptorByIndex(i, &fSelected, &pSD));
        CHECK_HR(hr = pSD->GetMediaTypeHandler(&pHandler));
        CHECK_HR(hr = pHandler->GetMajorType(&majorType));

        if (majorType == MFMediaType_Video && fSelected)
        {
            CHECK_HR(hr = AddSourceNode(pTopology, pSource, pPD, pSD, &pNode1));
            CHECK_HR(hr = AddOutputNode(pTopology, pSinkActivate, 0, &pNode2));
            CHECK_HR(hr = pNode1->ConnectOutput(0, pNode2, 0));
            break;
        }
        else
        {
            CHECK_HR(hr = pPD->DeselectStream(i));
        }
        SafeRelease(&pSD);
        SafeRelease(&pHandler);
    }

    *ppTopo = pTopology;
    (*ppTopo)->AddRef();

done:
    SafeRelease(&pTopology);
    SafeRelease(&pNode1);
    SafeRelease(&pNode2);
    SafeRelease(&pPD);
    SafeRelease(&pSD);
    SafeRelease(&pHandler);
    return hr;
}

HRESULT ImageGrabber::AddSourceNode(
    IMFTopology *pTopology,           // Topology.
    IMFMediaSource *pSource,          // Media source.
    IMFPresentationDescriptor *pPD,   // Presentation descriptor.
    IMFStreamDescriptor *pSD,         // Stream descriptor.
    IMFTopologyNode **ppNode)         // Receives the node pointer.
{
    IMFTopologyNode *pNode = NULL;

    HRESULT hr = S_OK;
    CHECK_HR(hr = MFCreateTopologyNode(MF_TOPOLOGY_SOURCESTREAM_NODE, &pNode));
    CHECK_HR(hr = pNode->SetUnknown(MF_TOPONODE_SOURCE, pSource));
    CHECK_HR(hr = pNode->SetUnknown(MF_TOPONODE_PRESENTATION_DESCRIPTOR, pPD));
    CHECK_HR(hr = pNode->SetUnknown(MF_TOPONODE_STREAM_DESCRIPTOR, pSD));
    CHECK_HR(hr = pTopology->AddNode(pNode));

    // Return the pointer to the caller.
    *ppNode = pNode;
    (*ppNode)->AddRef();

done:
    SafeRelease(&pNode);
    return hr;
}

HRESULT ImageGrabber::AddOutputNode(
    IMFTopology *pTopology,     // Topology.
    IMFActivate *pActivate,     // Media sink activation object.
    DWORD dwId,                 // Identifier of the stream sink.
    IMFTopologyNode **ppNode)   // Receives the node pointer.
{
    IMFTopologyNode *pNode = NULL;

    HRESULT hr = S_OK;
    CHECK_HR(hr = MFCreateTopologyNode(MF_TOPOLOGY_OUTPUT_NODE, &pNode));
    CHECK_HR(hr = pNode->SetObject(pActivate));
    CHECK_HR(hr = pNode->SetUINT32(MF_TOPONODE_STREAMID, dwId));
    CHECK_HR(hr = pNode->SetUINT32(MF_TOPONODE_NOSHUTDOWN_ON_REMOVE, FALSE));
    CHECK_HR(hr = pTopology->AddNode(pNode));

    // Return the pointer to the caller.
    *ppNode = pNode;
    (*ppNode)->AddRef();

done:
    SafeRelease(&pNode);
    return hr;
}



HRESULT ImageGrabber::CreateInstance(ImageGrabber **ppIG, unsigned int deviceID, QSharedPointer<DebugPrintOut> debugPrintOut)
{
    *ppIG = new (std::nothrow) ImageGrabber(deviceID, debugPrintOut);

    if (ppIG == NULL)
    {
        return E_OUTOFMEMORY;
    }

    debugPrintOut->printOut("IMAGEGRABBER VideoDevice %i: Creating instance of ImageGrabber\n", deviceID);

    return S_OK;
}

STDMETHODIMP ImageGrabber::QueryInterface(REFIID riid, void** ppv)
{
    // Creation tab of shifting interfaces from start of this class
    static const QITAB qit[] =
    {

        QITABENT(ImageGrabber, IMFSampleGrabberSinkCallback),
        QITABENT(ImageGrabber, IMFClockStateSink),
        { 0 }
    };
    return QISearch(this, qit, riid, ppv);
}

STDMETHODIMP_(ULONG) ImageGrabber::AddRef()
{
    return InterlockedIncrement(&m_cRef);
}

STDMETHODIMP_(ULONG) ImageGrabber::Release()
{
    ULONG cRef = InterlockedDecrement(&m_cRef);
    if (cRef == 0)
    {
        delete this;
    }
    return cRef;
}

STDMETHODIMP ImageGrabber::OnClockStart(MFTIME hnsSystemTime, LONGLONG llClockStartOffset)
{
    return S_OK;
}

STDMETHODIMP ImageGrabber::OnClockStop(MFTIME hnsSystemTime)
{
    return S_OK;
}

STDMETHODIMP ImageGrabber::OnClockPause(MFTIME hnsSystemTime)
{
    return S_OK;
}

STDMETHODIMP ImageGrabber::OnClockRestart(MFTIME hnsSystemTime)
{
    return S_OK;
}

STDMETHODIMP ImageGrabber::OnClockSetRate(MFTIME hnsSystemTime, float flRate)
{
    return S_OK;
}

STDMETHODIMP ImageGrabber::OnSetPresentationClock(IMFPresentationClock* pClock)
{
    return S_OK;
}

STDMETHODIMP ImageGrabber::OnProcessSample(REFGUID guidMajorMediaType, DWORD dwSampleFlags,
    LONGLONG llSampleTime, LONGLONG llSampleDuration, const BYTE * pSampleBuffer,
    DWORD dwSampleSize)
{
    //qDebug() << "OnProcessSample" << ((double)llSampleTime/10000.0) << (double)llSampleDuration/10000.0 << dwSampleSize;

    if (ig_RIE)
    {
        ig_RIFirst->fastCopy(pSampleBuffer);

        ig_RIOut = ig_RIFirst;
    }
    else
    {
        ig_RISecond->fastCopy(pSampleBuffer);

        ig_RIOut = ig_RISecond;
    }

    ig_RIE = !ig_RIE;


    return S_OK;
}

STDMETHODIMP ImageGrabber::OnShutdown()
{
    return S_OK;
}

RawImage *ImageGrabber::getRawImage()
{
    return ig_RIOut;
}
