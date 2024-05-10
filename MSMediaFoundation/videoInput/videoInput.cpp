#include "VideoInput.h"
#include "Media_Foundation.h"
#include "VideoDevices.h"
#include "VideoDevice.h"
#include "DebugPrintOut.h"
#include "RawImage.h"

template <class T> void SafeRelease(T *ppT)
{
    if (ppT)
    {
        delete (ppT);
        ppT = NULL;
    }
}

Parameter::Parameter()
{
    CurrentValue = 0;

    Min = 0;

    Max = 0;

    Step = 0;

    Default = 0;

    Flag = 0;

    Available = false;
}

MediaType::MediaType()
{
    pMF_MT_AM_FORMAT_TYPEName = NULL;

    pMF_MT_MAJOR_TYPEName = NULL;

    pMF_MT_SUBTYPEName = NULL;

    Clear();
}

MediaType::~MediaType()
{
    Clear();
}

void MediaType::Clear()
{

    MF_MT_FRAME_SIZE = 0;

    height = 0;

    width = 0;

    MF_MT_YUV_MATRIX = 0;

    MF_MT_VIDEO_LIGHTING = 0;

    MF_MT_DEFAULT_STRIDE = 0;

    MF_MT_VIDEO_CHROMA_SITING = 0;

    MF_MT_FIXED_SIZE_SAMPLES = 0;

    MF_MT_VIDEO_NOMINAL_RANGE = 0;

    MF_MT_FRAME_RATE = 0;

    MF_MT_FRAME_RATE_low = 0;

    MF_MT_PIXEL_ASPECT_RATIO = 0;

    MF_MT_PIXEL_ASPECT_RATIO_low = 0;

    MF_MT_ALL_SAMPLES_INDEPENDENT = 0;

    MF_MT_FRAME_RATE_RANGE_MIN = 0;

    MF_MT_FRAME_RATE_RANGE_MIN_low = 0;

    MF_MT_SAMPLE_SIZE = 0;

    MF_MT_VIDEO_PRIMARIES = 0;

    MF_MT_INTERLACE_MODE = 0;

    MF_MT_FRAME_RATE_RANGE_MAX = 0;

    MF_MT_FRAME_RATE_RANGE_MAX_low = 0;

    memset(&MF_MT_MAJOR_TYPE, 0, sizeof(GUID));

    memset(&MF_MT_AM_FORMAT_TYPE, 0, sizeof(GUID));

    memset(&MF_MT_SUBTYPE, 0, sizeof(GUID));
}

//--------------------------------------------------------------------------------
VideoInput::VideoInput(QSharedPointer<DebugPrintOut> debugPrintOut):
    m_accessToDevices(false),
    m_debugPrintOut(debugPrintOut)
{
    m_videoDevices = QSharedPointer<VideoDevices>(new VideoDevices(debugPrintOut));

    m_mediaFoundation = QSharedPointer<Media_Foundation>(new Media_Foundation(debugPrintOut));

    m_debugPrintOut->printOut("\n***** VideoInput LIBRARY - 2013 (Author: Evgeny Pereguda) *****\n\n");

    updateListOfDevices();

    if (!m_accessToDevices)
    {
        m_debugPrintOut->printOut("INITIALIZATION: There is not any suitable video device\n");
    }
}

//--------------------------------------------------------------------------------
void VideoInput::updateListOfDevices()
{
    m_accessToDevices = m_mediaFoundation->buildListOfDevices(m_videoDevices);

    if (!m_accessToDevices)
    {
        m_debugPrintOut->printOut("UPDATING: There is not any suitable video device\n");
    }
}

//--------------------------------------------------------------------------------
VideoInput::~VideoInput(void)
{
    m_debugPrintOut->printOut("\n***** CLOSE VideoInput LIBRARY - 2013 *****\n\n");
}

//--------------------------------------------------------------------------------
IMFMediaSource *VideoInput::getMediaSource(unsigned int deviceID)
{
    if (m_accessToDevices)
    {
        VideoDevice * VD = m_videoDevices->getDevice(deviceID);

        if (VD)
        {
            IMFMediaSource *out = VD->getMediaSource();

            if (!out)
            {
                m_debugPrintOut->printOut("VideoDevice %i: There is not any suitable IMFMediaSource interface\n", deviceID);
            }

            return out;
        }
    }
    else
    {
        m_debugPrintOut->printOut("VideoDevice(s): There is not any suitable video device\n");
    }

    return NULL;
}

//--------------------------------------------------------------------------------
bool VideoInput::setupDevice(unsigned int deviceID, unsigned int id)
{
    if (m_accessToDevices)
    {
        VideoDevice * VD = m_videoDevices->getDevice(deviceID);

        if (VD)
        {
            bool out = VD->setupDevice(id);

            if (!out)
            {
                m_debugPrintOut->printOut("VideoDevice %i: This device cannot be started\n", deviceID);
            }

            return out;
        }
    }
    else
    {
        m_debugPrintOut->printOut("VideoDevice(s): There is not any suitable video device\n");
    }

    return false;
}

//--------------------------------------------------------------------------------
bool VideoInput::setupDevice(unsigned int deviceID, unsigned int w, unsigned int h, unsigned int idealFramerate)
{
    if (m_accessToDevices)
    {
        VideoDevice * VD = m_videoDevices->getDevice(deviceID);

        if (VD)
        {
            bool out = VD->setupDevice(w, h, idealFramerate);

            if (!out)
            {
                m_debugPrintOut->printOut("VideoDevice %i: this device cannot be started\n", deviceID);
            }

            return out;
        }
    }
    else
    {
        m_debugPrintOut->printOut("VideoDevice(s): There is not any suitable video device\n", deviceID);
    }

    return false;
}

//--------------------------------------------------------------------------------
MediaType VideoInput::getFormat(unsigned int deviceID, unsigned int id)
{
    if (m_accessToDevices)
    {
        VideoDevice * VD = m_videoDevices->getDevice(deviceID);

        if (VD)
        {
            return VD->getFormat(id);
        }
    }
    else
    {
        m_debugPrintOut->printOut("VideoDevice(s): There is not any suitable video device\n");
    }

    return MediaType();
}

//--------------------------------------------------------------------------------
bool VideoInput::isDeviceSetup(unsigned int deviceID)
{
    if (m_accessToDevices)
    {
        VideoDevice * VD = m_videoDevices->getDevice(deviceID);

        if (VD)
        {
            return VD->isDeviceSetup();
        }
    }
    else
    {
        m_debugPrintOut->printOut("VideoDevice(s): There is not any suitable video device\n");
    }

    return false;
}

//--------------------------------------------------------------------------------
bool VideoInput::isDeviceMediaSource(unsigned int deviceID)
{
    if (m_accessToDevices)
    {
        VideoDevice * VD = m_videoDevices->getDevice(deviceID);

        if (VD)
        {
            return VD->isDeviceMediaSource();
        }
    }
    else
    {
        m_debugPrintOut->printOut("Device(s): There is not any suitable video device\n");
    }

    return false;
}

//--------------------------------------------------------------------------------
bool VideoInput::isDeviceRawDataSource(unsigned int deviceID)
{
    if (m_accessToDevices)
    {
        VideoDevice * VD = m_videoDevices->getDevice(deviceID);

        if (VD)
        {
            return VD->isDeviceRawDataSource();
        }
    }
    else
    {
        m_debugPrintOut->printOut("VideoDevice(s): There is not any suitable video device\n");
    }

    return false;
}

//--------------------------------------------------------------------------------
bool VideoInput::isFrameNew(unsigned int deviceID)
{
    if (m_accessToDevices)
    {
        if (!isDeviceSetup(deviceID))
        {
            if (isDeviceMediaSource(deviceID))
                return false;
        }

        VideoDevice * VD = m_videoDevices->getDevice(deviceID);

        if (VD)
        {
            return VD->isFrameNew();
        }
    }
    else
    {
        m_debugPrintOut->printOut("VideoDevice(s): There is not any suitable video device\n");
    }

    return false;
}

//--------------------------------------------------------------------------------
size_t VideoInput::getCountFormats(unsigned int deviceID)
{
    if (m_accessToDevices)
    {
        VideoDevice * VD = m_videoDevices->getDevice(deviceID);

        if (VD)
        {
            return VD->getCountFormats();
        }
    }
    else
    {
        m_debugPrintOut->printOut("VideoDevice(s): There is not any suitable video device\n");
    }

    return 0;
}

//--------------------------------------------------------------------------------
void VideoInput::closeAllDevices()
{
    for (unsigned int i = 0; i < m_videoDevices->getCount(); i++)
    {
        closeDevice(i);
    }
}

//--------------------------------------------------------------------------------
void VideoInput::setParameters(unsigned int deviceID, CamParameters parameters)
{
    if (m_accessToDevices)
    {
        VideoDevice *VD = m_videoDevices->getDevice(deviceID);

        if (VD)
        {
            VD->setParameters(parameters);
        }
    }
    else
    {
        m_debugPrintOut->printOut("VideoDevice(s): There is not any suitable video device\n");
    }
}

//--------------------------------------------------------------------------------
CamParameters VideoInput::getParameters(unsigned int deviceID)
{
    CamParameters out;

    if (m_accessToDevices)
    {
        VideoDevice *VD = m_videoDevices->getDevice(deviceID);

        if (VD)
        {
            out = VD->getParameters();
        }
    }
    else
    {
        m_debugPrintOut->printOut("VideoDevice(s): There is not any suitable video device\n");
    }

    return out;
}

//--------------------------------------------------------------------------------
void VideoInput::closeDevice(unsigned int deviceID)
{
    if (m_accessToDevices)
    {
        VideoDevice *VD = m_videoDevices->getDevice(deviceID);

        if (VD)
        {
            VD->closeDevice();
        }
    }
    else
    {
        m_debugPrintOut->printOut("VideoDevice(s): There is not any suitable video device\n");
    }
}

//--------------------------------------------------------------------------------
unsigned int VideoInput::getWidth(unsigned int deviceID)
{
    if (m_accessToDevices)
    {
        VideoDevice * VD = m_videoDevices->getDevice(deviceID);

        if (VD)
        {
            return VD->getWidth();
        }
    }
    else
    {
        m_debugPrintOut->printOut("VideoDevice(s): There is not any suitable video device\n");
    }

    return 0;
}

//--------------------------------------------------------------------------------
unsigned int VideoInput::getHeight(unsigned int deviceID)
{
    if (m_accessToDevices)
    {
        VideoDevice * VD = m_videoDevices->getDevice(deviceID);

        if (VD)
        {
            return VD->getHeight();
        }
    }
    else
    {
        m_debugPrintOut->printOut("VideoDevice(s): There is not any suitable video device\n");
    }

    return 0;
}

//--------------------------------------------------------------------------------
//! Return name of available video device
/*!
    \param deviceID is the index of the device
    \param s a constant character pointer.
    \return name or "Empty" if it does not exist
    \sa VideoDevice::getName, VideoDevices::getDevice
*/
const wchar_t *VideoInput::getNameVideoDevice(unsigned int deviceID)
{
    if (m_accessToDevices)
    {
        VideoDevice * VD = m_videoDevices->getDevice(deviceID);

        if (VD)
        {
            return VD->getName();
        }
    }
    else
    {
        m_debugPrintOut->printOut("VideoDevice(s): There is not any suitable video device\n");
    }

    return L"Empty";
}

//--------------------------------------------------------------------------------
//! A normal member taking two arguments and returning an integer value.
/*!
    \param a an integer argument.
    \param s a constant character pointer.
    \return The test results
    \sa Test(), ~Test(), testMeToo() and publicVar()
*/
unsigned int VideoInput::listDevices(bool silent)
{
    size_t out = 0;

    if (m_accessToDevices)
    {
        out = m_videoDevices->getCount();

        if (!silent) {
            m_debugPrintOut->printOut("\nVideoInput SPY MODE!\n\n");
        }

        if (!silent) {
            m_debugPrintOut->printOut("SETUP: Looking For Capture Devices\n");
        }

        for(size_t i = 0; i < out; i++)
        {
            if (!silent)
            {
                m_debugPrintOut->printOut("SETUP: %i) %s \n", i, getNameVideoDevice(i));
            }
        }

        if (!silent)
        {
            m_debugPrintOut->printOut("SETUP: %i Device(s) found\n\n", out);
        }


    }
    else
    {
        m_debugPrintOut->printOut("VideoDevice(s): There is not any suitable video device\n");
    }

    return out;
}

//--------------------------------------------------------------------------------
bool VideoInput::isDevicesAcceable()
{
    return m_accessToDevices;
}

//--------------------------------------------------------------------------------
void VideoInput::setVerbose(bool state)
{
    return m_debugPrintOut->setVerbose(state);
}

//--------------------------------------------------------------------------------
void VideoInput::setEmergencyStopEvent(unsigned int deviceID, void *userData, void(*func)(int, void *))
{
    if (m_accessToDevices)
    {
        if (func)
        {
            VideoDevice * VD = m_videoDevices->getDevice(deviceID);

            if (VD)
            {
                VD->setEmergencyStopEvent(userData, func);
            }
        }
    }
    else
    {
        m_debugPrintOut->printOut("VideoDevice(s): There is not any suitable video device\n");
    }
}

//--------------------------------------------------------------------------------
bool VideoInput::getPixels(unsigned int deviceID, unsigned char * dstBuffer, bool flipRedAndBlue, bool flipImage)
{
    bool success = false;

    unsigned int bytes = 3;

    if (m_accessToDevices)
    {
        if (isDeviceRawDataSource(deviceID))
        {
            RawImage *RIOut = m_videoDevices->getDevice(deviceID)->getRawImageOut();

            if (RIOut)
            {
                unsigned int height    = m_videoDevices->getDevice(deviceID)->getHeight();
                unsigned int width    = m_videoDevices->getDevice(deviceID)->getWidth();

                unsigned int size = bytes * width * height;

                if (size == RIOut->getSize())
                {
                    processPixels(RIOut->getpPixels(), dstBuffer, width, height, bytes, flipRedAndBlue, flipImage);

                    success = true;
                }
                else
                {
                    m_debugPrintOut->printOut("ERROR: GetPixels() - bufferSizes do not match!\n");
                }
            }
            else
            {
                m_debugPrintOut->printOut("ERROR: GetPixels() - Unable to grab frame for device %i\n", deviceID);
            }
        }
    }
    else
    {
        m_debugPrintOut->printOut("VideoDevice(s): There is not any suitable video device\n");
    }

    return success;
}

//--------------------------------------------------------------------------------
void VideoInput::processPixels(unsigned char * src, unsigned char * dst, unsigned int width, unsigned int height, unsigned int bpp, bool bRGB, bool bFlip)
{

    unsigned int widthInBytes = width * bpp;

    unsigned int numBytes = widthInBytes * height;

    unsigned int numInts = numBytes >> 2;

    unsigned int widthInInts = widthInBytes >> 2;

    if (!bRGB)
    {

        int x = 0;
        int y = 0;

#ifndef _WIN64
        int* dstInt = NULL;
        int* srcInt = NULL;
#endif

        if (bFlip)
        {
            for (unsigned int y = 0; y < height; ++y)
            {
#if _WIN64
    memcpy(dst + (y * widthInBytes), src + ( (height -y -1) * widthInBytes), widthInBytes);
#else
                dstInt = (int *)(dst + (y * widthInBytes));

                srcInt = (int *)(src + ( (height -y -1) * widthInBytes));

                _asm
                {
                    mov ESI, srcInt

                    mov EDI, dstInt

                    mov ECX, widthInInts

                    cld

                    rep movsd
                }
#endif
            }

        }
        else
        {
#if _WIN64
    memcpy(dst, src, numBytes);
#else
            _asm
            {
                mov ESI, src

                mov EDI, dst

                mov ECX, numInts

                cld

                rep movsd
            }
#endif
        }
    }
    else
    {
        if (bFlip)
        {

            int x = 0;
            int y = (height - 1) * widthInBytes;
            src += y;

            for (unsigned int i = 0; i < numBytes; i+=3)
            {
                if (x >= width)
                {
                    x = 0;
                    src -= widthInBytes*2;
                }

                *dst = *(src+2);
                dst++;

                *dst = *(src+1);
                dst++;

                *dst = *src;
                dst++;

                src+=3;
                x++;
            }
        }
        else
        {
            for(unsigned int i = 0; i < numBytes; i += 3)
            {
                *dst = *(src+2);
                dst++;

                *dst = *(src+1);
                dst++;

                *dst = *src;
                dst++;

                src+=3;
            }
        }
    }
}
