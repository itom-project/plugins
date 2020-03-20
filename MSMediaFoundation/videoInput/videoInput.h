#pragma once

#include <guiddef.h>


struct IMFMediaSource;

// Structure for collecting info about types of video, which are supported by current video device
struct MediaType
{
	unsigned int MF_MT_FRAME_SIZE;

	unsigned int height;

	unsigned int width;
			
	unsigned int MF_MT_YUV_MATRIX;
	
	unsigned int MF_MT_VIDEO_LIGHTING;
	
	unsigned int MF_MT_DEFAULT_STRIDE;
	
	unsigned int MF_MT_VIDEO_CHROMA_SITING;
	
	GUID MF_MT_AM_FORMAT_TYPE;
	
	wchar_t *pMF_MT_AM_FORMAT_TYPEName;
	
	unsigned int MF_MT_FIXED_SIZE_SAMPLES;
	
	unsigned int MF_MT_VIDEO_NOMINAL_RANGE;
	
	unsigned int MF_MT_FRAME_RATE;

	unsigned int MF_MT_FRAME_RATE_low;
	
	unsigned int MF_MT_PIXEL_ASPECT_RATIO;

	unsigned int MF_MT_PIXEL_ASPECT_RATIO_low;
	
	unsigned int MF_MT_ALL_SAMPLES_INDEPENDENT;
	
	unsigned int MF_MT_FRAME_RATE_RANGE_MIN;
	
	unsigned int MF_MT_FRAME_RATE_RANGE_MIN_low;
	
	unsigned int MF_MT_SAMPLE_SIZE;
	
	unsigned int MF_MT_VIDEO_PRIMARIES;
	
	unsigned int MF_MT_INTERLACE_MODE;
	
	unsigned int MF_MT_FRAME_RATE_RANGE_MAX;
	
	unsigned int MF_MT_FRAME_RATE_RANGE_MAX_low;

	GUID MF_MT_MAJOR_TYPE;
	
	wchar_t *pMF_MT_MAJOR_TYPEName;
	
	GUID MF_MT_SUBTYPE;
	
	wchar_t *pMF_MT_SUBTYPEName;	

	MediaType();
	~MediaType();
	void Clear();
};

// Structure for collecting info about one parameter of current video device
struct Parameter
{
	long CurrentValue;

	long Min;
	
	long Max;
	
	long Step;
	
	long Default; 
	
	long Flag;

    bool Available;

	Parameter();
};

// Structure for collecting info about 17 parameters of current video device
struct CamParameters
{
	    Parameter Brightness;
        Parameter Contrast;
        Parameter Hue;
        Parameter Saturation;
        Parameter Sharpness;
        Parameter Gamma;
        Parameter ColorEnable;
        Parameter WhiteBalance;
        Parameter BacklightCompensation;
        Parameter Gain;


		Parameter Pan;
        Parameter Tilt;
        Parameter Roll;
        Parameter Zoom;
        Parameter Exposure;
        Parameter Iris;
        Parameter Focus;
};

/// The only visiable class for controlling of video devices in format singelton
class VideoInput
{
public:
	virtual ~VideoInput(void);

	// Getting of static instance of VideoInput class
	static VideoInput& getInstance(); 

	// Closing video device with deviceID
	void closeDevice(unsigned int deviceID);
	
	// Setting callback function for emergency events(for example: removing video device with deviceID) with userData
	void setEmergencyStopEvent(unsigned int deviceID, void *userData, void(*func)(int, void *));

	// Closing all devices
	void closeAllDevices();

	// Getting of parameters of video device with deviceID
	CamParameters getParameters(unsigned int deviceID);

	// Setting of parameters of video device with deviceID
	void setParameters(unsigned int deviceID, CamParameters parameters);

	// Getting numbers of existence VideoDevices with listing in consol
	unsigned int listDevices(bool silent = false);
		
	// Getting numbers of formats, which are supported by VideoDevice with deviceID
	size_t getCountFormats(unsigned int deviceID);

	// Getting width of image, which is getting from VideoDevice with deviceID
	unsigned int getWidth(unsigned int deviceID);

	// Getting height of image, which is getting from VideoDevice with deviceID
	unsigned int getHeight(unsigned int deviceID);

	// Getting name of VideoDevice with deviceID
	wchar_t *getNameVideoDevice(unsigned int deviceID);
	
	// Getting interface MediaSource for Media Foundation from VideoDevice with deviceID
	IMFMediaSource *getMediaSource(unsigned int deviceID);
	
	// Getting format with id, which is supported by VideoDevice with deviceID 
	MediaType getFormat(unsigned int deviceID, int unsigned id);

	// Checking of existence of the suitable video devices
	bool isDevicesAcceable();

	// Checking of using the VideoDevice with deviceID
	bool isDeviceSetup(unsigned int deviceID);

	// Checking of using MediaSource from VideoDevice with deviceID
	bool isDeviceMediaSource(unsigned int deviceID);
	
	// Checking of using Raw Data of pixels from VideoDevice with deviceID
	bool isDeviceRawDataSource(unsigned int deviceID);

	// Setting of the state of outprinting info in consol
	void setVerbose(bool state);
	
	// Initialization of video device with deviceID by media type with id
	bool setupDevice(unsigned int deviceID, unsigned int id = 0);

	// Initialization of video device with deviceID by wisth w, height h and fps idealFramerate
	bool setupDevice(unsigned int deviceID, unsigned int w, unsigned int h, unsigned int idealFramerate = 30);

	// Checking of recivig of new frame from video device with deviceID 
	bool isFrameNew(unsigned int deviceID);

	// Writing of Raw Data pixels from video device with deviceID with correction of RedAndBlue flipping flipRedAndBlue and vertical flipping flipImage
	bool getPixels(unsigned int deviceID, unsigned char * pixels, bool flipRedAndBlue = false, bool flipImage = false);
	
private: 

	bool accessToDevices;
	
    VideoInput(void);

	void processPixels(unsigned char * src, unsigned char * dst, unsigned int width, unsigned int height, unsigned int bpp, bool bRGB, bool bFlip);
	
	void updateListOfDevices();
};

