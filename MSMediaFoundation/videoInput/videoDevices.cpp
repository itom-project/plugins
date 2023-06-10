#include <Mfobjects.h>
#include <Mfidl.h>


#include "VideoDevices.h"
#include "VideoDevice.h"
#include "DebugPrintOut.h"
#include "Common.h"

//!  Class for all video devices
/*!
  The singleton instance of this class contains all found and available video devices
*/

//--------------------------------------------------------
VideoDevices::VideoDevices(QSharedPointer<DebugPrintOut> debugPrintOut):
    m_debugPrintOut(debugPrintOut)
{
}

//--------------------------------------------------------
VideoDevices::~VideoDevices(void)
{
    clearDevices();
}

//--------------------------------------------------------
void VideoDevices::clearDevices()
{
    std::vector<VideoDevice *>::iterator i = m_devices.begin();

    for (; i != m_devices.end(); ++i)
    {
        delete (*i);
    }

    m_devices.clear();
}


//--------------------------------------------------------
VideoDevice * VideoDevices::getDevice(unsigned int i)
{
    if (i >= m_devices.size())
    {
        return NULL;
    }

    if (i < 0)
    {
        return NULL;
    }

    return m_devices[i];
}

//---------------------------------------------------------------------
long VideoDevices::initDevices(IMFAttributes *pAttributes)
{
    HRESULT hr = S_OK;

    IMFActivate **ppDevices = NULL;

    clearDevices();

    UINT32 count;

    hr = MFEnumDeviceSources(pAttributes, &ppDevices, &count);

    if (SUCCEEDED(hr))
    {
        if (count > 0)
        {
            for(UINT32 i = 0; i < count; i++)
            {
                VideoDevice *vd = new VideoDevice;

                vd->readInfoOfDevice(ppDevices[i], i);

                m_devices.push_back(vd);

                SafeRelease(&ppDevices[i]);
            }

            SafeRelease(ppDevices);
        }
        else
            hr = -1;
    }
    else
    {
        m_debugPrintOut->printOut("VideoDevices: The instances of the VideoDevice class cannot be created\n");
    }

    return hr;
}

//--------------------------------------------------------
size_t VideoDevices::getCount()
{
    return m_devices.size();
}
