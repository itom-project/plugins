#include <mfobjects.h>
#include <mfidl.h>
#include <mfapi.h>
#include <string.h>

#include <Strmif.h>
#include "vfwmsgs.h" //only for error message


#include "VideoDevice.h"
#include "FormatReader.h"
#include "Common.h"
#include "DebugPrintOut.h"
#include "ImageGrabberThread.h"
#include "ImageGrabber.h"
#include "RawImage.h"


#pragma comment(lib, "Strmiids")
VideoDevice::VideoDevice(void):
    m_isSetuped(false),
    vd_LockOut(OpenLock),
    m_pFriendlyName(NULL),
    m_width(0),
    m_height(0),
    m_pSource(NULL),
    m_func(NULL),
    m_userData(NULL),
    m_debugPrintOut(new DebugPrintOut())
{

}

void VideoDevice::setParameters(CamParameters parameters)
{
    if (m_isSetuped)
    {
        if (m_pSource)
        {
            unsigned int shift = sizeof(Parameter);

            Parameter *pParameter = (Parameter *)(&parameters);

            Parameter *pPrevParameter = (Parameter *)(&m_prevParameters);

            IAMVideoProcAmp *pProcAmp = NULL;
            HRESULT hr = m_pSource->QueryInterface(IID_PPV_ARGS(&pProcAmp));

            if (SUCCEEDED(hr))
            {
                for(unsigned int i = 0; i < 10; i++)
                {
                    if (pPrevParameter[i].CurrentValue != pParameter[i].CurrentValue || pPrevParameter[i].Flag != pParameter[i].Flag)
                        hr = pProcAmp->Set(VideoProcAmp_Brightness + i, pParameter[i].CurrentValue, pParameter[i].Flag);

                }

                pProcAmp->Release();
            }

            IAMCameraControl *pProcControl = NULL;
            hr = m_pSource->QueryInterface(IID_PPV_ARGS(&pProcControl));

            if (SUCCEEDED(hr))
            {
                for(unsigned int i = 0; i < 7; i++)
                {
                    if (pPrevParameter[10 + i].CurrentValue != pParameter[10 + i].CurrentValue || pPrevParameter[10 + i].Flag != pParameter[10 + i].Flag)
                        hr = pProcControl->Set(CameraControl_Pan+i, pParameter[10 + i].CurrentValue, pParameter[10 + i].Flag);
                }

                pProcControl->Release();
            }

            m_prevParameters = parameters;
        }
    }
}

CamParameters VideoDevice::getParameters()
{
    CamParameters out;
    long flag;

    if (m_isSetuped)
    {
        if (m_pSource)
        {
            unsigned int shift = sizeof(Parameter);

            Parameter *pParameter = (Parameter *)(&out);

            IAMVideoProcAmp *pProcAmp = NULL;
            HRESULT hr = m_pSource->QueryInterface(IID_PPV_ARGS(&pProcAmp));

            if (SUCCEEDED(hr))
            {
                for(unsigned int i = 0; i < 10; i++)
                {
                    Parameter temp;

                    hr = pProcAmp->GetRange(VideoProcAmp_Brightness + i, &temp.Min, &temp.Max, &temp.Step, &temp.Default, &temp.Flag);

                    if (SUCCEEDED(hr))
                    {
                        flag = temp.Flag;
                        if (SUCCEEDED(pProcAmp->Get(VideoProcAmp_Brightness + i, &temp.CurrentValue, &temp.Flag)) == false)
                        {
                            temp.CurrentValue = temp.Default;
                            temp.Flag = flag;
                        }

                        temp.Available = true;

                        pParameter[i] = temp;
                    }
                    else if (hr == E_PROP_ID_UNSUPPORTED)
                    {
                        pParameter[i].Available = false;
                    }
                    else
                    {
                        pParameter[i].Available = false;
                    }
                }

                pProcAmp->Release();
            }

            IAMCameraControl *pProcControl = NULL;
            hr = m_pSource->QueryInterface(IID_PPV_ARGS(&pProcControl));

            if (SUCCEEDED(hr))
            {
                for(unsigned int i = 0; i < 7; i++)
                {
                    Parameter temp;

                    hr = pProcControl->GetRange(CameraControl_Pan + i, &temp.Min, &temp.Max, &temp.Step, &temp.Default, &temp.Flag);

                    if (SUCCEEDED(hr))
                    {
                        flag = temp.Flag;
                        if (SUCCEEDED(pProcControl->Get(CameraControl_Pan + i, &temp.CurrentValue, &temp.Flag)) == false)
                        {
                            temp.CurrentValue = temp.Default;
                            temp.Flag = flag;
                        }

                        temp.Available = true;
                        pParameter[10 + i] = temp;
                    }
                    else if (hr == E_PROP_ID_UNSUPPORTED)
                    {
                        pParameter[10 + i].Available = false;
                    }
                    else
                    {
                        pParameter[10 + i].Available = false;
                    }
                }

                pProcControl->Release();
            }
        }
    }

    return out;
}

long VideoDevice::resetDevice(IMFActivate *pActivate)
{
    HRESULT hr = -1;

    m_currentFormats.clear();

    if (m_pFriendlyName)
    {
        CoTaskMemFree(m_pFriendlyName);
    }

    m_pFriendlyName = NULL;

    if (pActivate)
    {
        IMFMediaSource *pSource = NULL;

        hr = pActivate->GetAllocatedString(
                MF_DEVSOURCE_ATTRIBUTE_FRIENDLY_NAME,
                &m_pFriendlyName,
                NULL
                );


        hr = pActivate->ActivateObject(
            __uuidof(IMFMediaSource),
            (void**)&pSource
            );

        if (SUCCEEDED(hr))
        {
            enumerateCaptureFormats(pSource);

            buildLibraryofTypes();

            pSource->Shutdown();

            SafeRelease(&pSource);
        }

        if (FAILED(hr))
        {
            m_pFriendlyName = NULL;

            m_debugPrintOut->printOut("VideoDevice %i: IMFMediaSource interface cannot be created \n", m_currentNumber);
        }
    }

    return hr;
}

long VideoDevice::readInfoOfDevice(IMFActivate *pActivate, unsigned int Num)
{
    HRESULT hr = -1;

    m_currentNumber = Num;

    hr = resetDevice(pActivate);

    return hr;
}

long VideoDevice::checkDevice(IMFAttributes *pAttributes, IMFActivate **pDevice)
{
    HRESULT hr = S_OK;

    IMFActivate **ppDevices = NULL;

    UINT32 count;

    wchar_t *newFriendlyName = NULL;

    hr = MFEnumDeviceSources(pAttributes, &ppDevices, &count);

    if (SUCCEEDED(hr))
    {
        if (count > 0)
        {
            if (count > m_currentNumber)
            {
                hr = ppDevices[m_currentNumber]->GetAllocatedString(
                MF_DEVSOURCE_ATTRIBUTE_FRIENDLY_NAME,
                &newFriendlyName,
                NULL
                );

                if (SUCCEEDED(hr))
                {
                    if (wcscmp(newFriendlyName, m_pFriendlyName) != 0)
                    {
                        m_debugPrintOut->printOut("VideoDevice %i: Chosen device cannot be found \n", m_currentNumber);

                        hr = -1;

                        pDevice = NULL;
                    }
                    else
                    {
                        *pDevice = ppDevices[m_currentNumber];

                        (*pDevice)->AddRef();
                    }
                }
                else
                {
                    m_debugPrintOut->printOut("VideoDevice %i: Name of device cannot be gotten \n", m_currentNumber);
                }

            }
            else
            {
                m_debugPrintOut->printOut("VideoDevice %i: Number of devices more than current number of the device \n", m_currentNumber);

                hr = -1;
            }

            for(UINT32 i = 0; i < count; i++)
            {
                SafeRelease(&ppDevices[i]);
            }

            SafeRelease(ppDevices);
        }
        else
            hr = -1;
    }
    else
    {
        m_debugPrintOut->printOut("VideoDevice %i: List of DeviceSources cannot be enumerated \n", m_currentNumber);
    }

    return hr;
}

long VideoDevice::initDevice()
{
    HRESULT hr = -1;

    IMFAttributes *pAttributes = NULL;

    IMFActivate * vd_pActivate= NULL;

    CoInitialize(NULL);

    hr = MFCreateAttributes(&pAttributes, 1);

    if (SUCCEEDED(hr))
    {
        hr = pAttributes->SetGUID(
            MF_DEVSOURCE_ATTRIBUTE_SOURCE_TYPE,
            MF_DEVSOURCE_ATTRIBUTE_SOURCE_TYPE_VIDCAP_GUID
            );
    }

    if (SUCCEEDED(hr))
    {
        hr = checkDevice(pAttributes, &vd_pActivate);

        if (SUCCEEDED(hr) && vd_pActivate)
        {
            if (m_pSource)
            {
                m_pSource->Shutdown();
            }

            SafeRelease(&m_pSource);

            hr = vd_pActivate->ActivateObject(
                __uuidof(IMFMediaSource),
                (void**)&m_pSource
                );

            if (SUCCEEDED(hr))
            {

            }

            SafeRelease(&vd_pActivate);
        }
        else
        {
            m_debugPrintOut->printOut("VideoDevice %i: Device there is not \n", m_currentNumber);
        }
    }
    else
    {

        m_debugPrintOut->printOut("VideoDevice %i: The attribute of video cameras cannot be getting \n", m_currentNumber);

    }

    SafeRelease(&pAttributes);

    return hr;
}

MediaType VideoDevice::getFormat(unsigned int id)
{
    if (id < m_currentFormats.size())
    {
        return m_currentFormats[id];
    }
    else return MediaType();

}

size_t VideoDevice::getCountFormats()
{
    return m_currentFormats.size();
}

void VideoDevice::setEmergencyStopEvent(void *userData, void(*func)(int, void *))
{
    m_func = func;

    m_userData = userData;
}

void VideoDevice::closeDevice()
{
    if (m_isSetuped)
    {
        m_isSetuped = false;

        m_pSource->Stop();

        m_pSource->Shutdown();

        SafeRelease(&m_pSource);

        if (vd_LockOut == RawDataLock)
        {
            m_pImGrTh->stop();

            if (m_pImGrTh->runMutex.tryLock(3000))
            {
                m_pImGrTh->runMutex.unlock();
            }
            else
            {
                Sleep(500);
            }

            delete m_pImGrTh;
        }

        m_pImGrTh = NULL;

        vd_LockOut = OpenLock;

        m_debugPrintOut->printOut("VideoDevice %i: Device is stopped \n", m_currentNumber);
    }
}

unsigned int VideoDevice::getWidth()
{
    if (m_isSetuped)
        return m_width;
    else
        return 0;
}

unsigned int VideoDevice::getHeight()
{
    if (m_isSetuped)
        return m_height;
    else
        return 0;
}

IMFMediaSource *VideoDevice::getMediaSource()
{
    IMFMediaSource *out = NULL;

    if (vd_LockOut == OpenLock)
    {
        vd_LockOut = MediaSourceLock;

        out = m_pSource;
    }

    return out;
}

int VideoDevice::findType(unsigned int size, unsigned int frameRate)
{
    if (m_captureFormats.size() == 0)
        return 0;

    FrameRateMap FRM = m_captureFormats[size];

    if (FRM.size() == 0)
        return 0;

    unsigned int frameRateMax = 0;  SUBTYPEMap STMMax;

    if (frameRate == 0)
    {
        std::map<UINT64, SUBTYPEMap>::iterator f = FRM.begin();

        for(; f != FRM.end(); f++)
        {
             if ((*f).first >= frameRateMax)
             {
                 frameRateMax = (*f).first;

                 STMMax = (*f).second;
             }
        }

    }
    else
    {
        std::map<UINT64, SUBTYPEMap>::iterator f = FRM.begin();

        for(; f != FRM.end(); f++)
        {
             if ((*f).first >= frameRateMax)
             {
                 if (frameRate > (*f).first)
                 {
                     frameRateMax = (*f).first;

                     STMMax = (*f).second;
                 }
             }
        }
    }

    if (STMMax.size() == 0)
        return 0;


    std::map<String, vectorNum>::iterator S = STMMax.begin();

    vectorNum VN = (*S).second;

    if (VN.size() == 0)
        return 0;

    return VN[0];

}

void VideoDevice::buildLibraryofTypes()
{
    unsigned int size;

    unsigned int framerate;

    std::vector<MediaType>::iterator i = m_currentFormats.begin();

    int count = 0;

    for(; i != m_currentFormats.end(); i++)
    {
        size = (*i).MF_MT_FRAME_SIZE;

        framerate = (*i).MF_MT_FRAME_RATE;

        FrameRateMap FRM = m_captureFormats[size];

        SUBTYPEMap STM = FRM[framerate];

        String subType((*i).pMF_MT_SUBTYPEName);

        vectorNum VN = STM[subType];

        VN.push_back(count);

        STM[subType] = VN;

        FRM[framerate] = STM;

        m_captureFormats[size] = FRM;

        count++;
    }
}

long VideoDevice::setDeviceFormat(IMFMediaSource *pSource, unsigned long  dwFormatIndex)
{
    IMFPresentationDescriptor *pPD = NULL;
    IMFStreamDescriptor *pSD = NULL;
    IMFMediaTypeHandler *pHandler = NULL;
    IMFMediaType *pType = NULL;

    HRESULT hr = pSource->CreatePresentationDescriptor(&pPD);
    if (FAILED(hr))
    {
        goto done;
    }

    BOOL fSelected;
    hr = pPD->GetStreamDescriptorByIndex(0, &fSelected, &pSD);
    if (FAILED(hr))
    {
        goto done;
    }

    hr = pSD->GetMediaTypeHandler(&pHandler);
    if (FAILED(hr))
    {
        goto done;
    }

    hr = pHandler->GetMediaTypeByIndex((DWORD)dwFormatIndex, &pType);
    if (FAILED(hr))
    {
        goto done;
    }

    hr = pHandler->SetCurrentMediaType(pType);

done:
    SafeRelease(&pPD);
    SafeRelease(&pSD);
    SafeRelease(&pHandler);
    SafeRelease(&pType);
    return hr;
}

bool VideoDevice::isDeviceSetup()
{
    return m_isSetuped;
}

RawImage * VideoDevice::getRawImageOut()
{
    if (!m_isSetuped) return NULL;

    if (m_pImGrTh)
            return m_pImGrTh->getImageGrabber()->getRawImage();
    else
    {
        m_debugPrintOut->printOut("VideoDevice %i: The instance of ImageGrabberThread class does not exist  \n", m_currentNumber);
    }
    return NULL;
}

bool VideoDevice::isFrameNew()
{
    if (!m_isSetuped) return false;

    if (vd_LockOut == RawDataLock || vd_LockOut == OpenLock)
    {
        if (vd_LockOut == OpenLock)
        {
            vd_LockOut = RawDataLock;

            HRESULT hr = ImageGrabberThread::CreateInstance(&m_pImGrTh, m_pSource, m_currentNumber, m_debugPrintOut);

            if (FAILED(hr))
            {
                m_debugPrintOut->printOut("VideoDevice %i: The instance of ImageGrabberThread class cannot be created.\n", m_currentNumber);

                return false;
            }

            m_pImGrTh->setEmergencyStopEvent(m_userData, m_func);

            m_pImGrTh->start();

            return true;
        }

        if (m_pImGrTh)
        {
            return m_pImGrTh->getImageGrabber()->getRawImage()->isNew();
        }

    }

    return false;
}

bool VideoDevice::isDeviceMediaSource()
{
    if (vd_LockOut == MediaSourceLock) return true;

    return false;
}

bool VideoDevice::isDeviceRawDataSource()
{
    if (vd_LockOut == RawDataLock) return true;

    return false;
}

bool VideoDevice::setupDevice(unsigned int id)
{
    if (!m_isSetuped)
    {
        HRESULT hr = -1;

        hr = initDevice();

        if (SUCCEEDED(hr))
        {
            m_width = m_currentFormats[id].width;

            m_height = m_currentFormats[id].height;

            hr = setDeviceFormat(m_pSource, (DWORD) id);

            m_isSetuped = (SUCCEEDED(hr));

            if (m_isSetuped)
                m_debugPrintOut->printOut("\n\nVideoDevice %i: Device is setuped \n", m_currentNumber);

            m_prevParameters = getParameters();

            return m_isSetuped;
        }
        else
        {
            m_debugPrintOut->printOut("VideoDevice %i: Interface IMFMediaSource cannot be got \n", m_currentNumber);

            return false;
        }
    }
    else
    {
        m_debugPrintOut->printOut("VideoDevice %i: Device is setuped already \n", m_currentNumber);

        return false;
    }
}

bool VideoDevice::setupDevice(unsigned int w, unsigned int h, unsigned int idealFramerate)
{
    unsigned int id = findType(w * h, idealFramerate);

    return setupDevice(id);
}

wchar_t *VideoDevice::getName()
{
    return m_pFriendlyName;
}

VideoDevice::~VideoDevice(void)
{
    closeDevice();

    if (m_pSource)
    {
        m_pSource->Shutdown();
    }

    SafeRelease(&m_pSource);

    if (m_pFriendlyName)
    {
        CoTaskMemFree(m_pFriendlyName);
    }
}

long VideoDevice::enumerateCaptureFormats(IMFMediaSource *pSource)
{
    IMFPresentationDescriptor *pPD = NULL;
    IMFStreamDescriptor *pSD = NULL;
    IMFMediaTypeHandler *pHandler = NULL;
    IMFMediaType *pType = NULL;
    DWORD cTypes = 0;

    HRESULT hr = pSource->CreatePresentationDescriptor(&pPD);
    if (FAILED(hr))
    {
        goto done;
    }

    BOOL fSelected;
    hr = pPD->GetStreamDescriptorByIndex(0, &fSelected, &pSD);
    if (FAILED(hr))
    {
        goto done;
    }

    hr = pSD->GetMediaTypeHandler(&pHandler);
    if (FAILED(hr))
    {
        goto done;
    }

    hr = pHandler->GetMediaTypeCount(&cTypes);
    if (FAILED(hr))
    {
        goto done;
    }

    for (DWORD i = 0; i < cTypes; i++)
    {
        hr = pHandler->GetMediaTypeByIndex(i, &pType);

        if (FAILED(hr))
        {
            goto done;
        }

        MediaType MT = FormatReader::Read(pType);

        m_currentFormats.push_back(MT);

        SafeRelease(&pType);
    }

done:
    SafeRelease(&pPD);
    SafeRelease(&pSD);
    SafeRelease(&pHandler);
    SafeRelease(&pType);
    return hr;
}
