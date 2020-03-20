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

VideoDevices::VideoDevices(void): count(0)
{
}

void VideoDevices::clearDevices()
{
	std::vector<VideoDevice *>::iterator i = vds_Devices.begin();

	for(; i != vds_Devices.end(); ++i)
		delete (*i);

	vds_Devices.clear();
}

VideoDevices::~VideoDevices(void)
{	
	clearDevices();
}

VideoDevice * VideoDevices::getDevice(unsigned int i)
{
	if(i >= vds_Devices.size())
	{
		return NULL;
	}

	if(i < 0)
	{
		return NULL;
	}

	return vds_Devices[i];
}

long VideoDevices::initDevices(IMFAttributes *pAttributes)
{
	HRESULT hr = S_OK;
		
	IMFActivate **ppDevices = NULL;

	clearDevices();
	
	hr = MFEnumDeviceSources(pAttributes, &ppDevices, &count);

	if (SUCCEEDED(hr))
    {
        if(count > 0)
		{
			for(UINT32 i = 0; i < count; i++)
			{
				VideoDevice *vd = new VideoDevice;

				vd->readInfoOfDevice(ppDevices[i], i);

				vds_Devices.push_back(vd);		

				SafeRelease(&ppDevices[i]);
			}

			SafeRelease(ppDevices);
		}
		else
			hr = -1;
    }
	else
	{
		DebugPrintOut *DPO = &DebugPrintOut::getInstance();

		DPO->printOut(L"VideoDevices: The instances of the VideoDevice class cannot be created\n");
	}

	return hr;
}

size_t VideoDevices::getCount()
{
	return vds_Devices.size();
}

VideoDevices& VideoDevices::getInstance() 
{
	static VideoDevices instance;

	return instance;
}