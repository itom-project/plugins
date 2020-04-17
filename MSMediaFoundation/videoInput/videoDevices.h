#pragma once

#include <basetsd.h>
#include <vector>

#include <qsharedpointer.h>
#include "DebugPrintOut.h"

struct IMFAttributes;

class VideoDevice;

/// Class for managing of list of video devices
class VideoDevices
{
public:
    VideoDevices(QSharedPointer<DebugPrintOut> debugPrintOut);

	~VideoDevices(void);

	long initDevices(IMFAttributes *pAttributes);

	VideoDevice *getDevice(unsigned int i);

	size_t getCount();
			
	void clearDevices();

private:
	
	std::vector<VideoDevice *> m_devices;
		
    QSharedPointer<DebugPrintOut> m_debugPrintOut;
};

