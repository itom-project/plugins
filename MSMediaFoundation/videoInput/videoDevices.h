#pragma once

#include <basetsd.h>
#include <vector>

struct IMFAttributes;

class VideoDevice;

/// Class for managing of list of video devices
class VideoDevices
{
public:
	~VideoDevices(void);

	long initDevices(IMFAttributes *pAttributes);

	static VideoDevices& getInstance();

	VideoDevice *getDevice(unsigned int i);

	size_t getCount();
			
	void clearDevices();

private:
			
    UINT32 count;

	std::vector<VideoDevice *> vds_Devices;
		
	VideoDevices(void);
};

