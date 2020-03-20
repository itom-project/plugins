#include <new>
#include <mfapi.h>
#include <process.h>    /* _beginthread, _endthread */

#include "ImageGrabberThread.h"
#include "ImageGrabber.h"
#include "DebugPrintOut.h"

#include <qdebug.h>


//----------------------------------------------------------------------------------------
unsigned int WINAPI MainThreadFunction( LPVOID lpParam )
{
	ImageGrabberThread *pIGT = (ImageGrabberThread *)lpParam;

	pIGT->run();

	return 0; 
}

//----------------------------------------------------------------------------------------
HRESULT ImageGrabberThread::CreateInstance(ImageGrabberThread **ppIGT, IMFMediaSource *pSource, unsigned int deviceID)
{
	DebugPrintOut *DPO = &DebugPrintOut::getInstance();

	*ppIGT = new (std::nothrow) ImageGrabberThread(pSource, deviceID);

    if (ppIGT == NULL)
    {
		DPO->printOut(L"IMAGEGRABBERTHREAD VideoDevice %i: Memory cannot be allocated\n", deviceID);

        return E_OUTOFMEMORY;
    }
	else
		DPO->printOut(L"IMAGEGRABBERTHREAD VideoDevice %i: Creating of the instance of ImageGrabberThread\n", deviceID);
	
    return S_OK;
}

//----------------------------------------------------------------------------------------
ImageGrabberThread::ImageGrabberThread(IMFMediaSource *pSource, unsigned int deviceID): igt_Handle(0), igt_stop(false), igt_pImageGrabber(NULL)
{
	DebugPrintOut *DPO = &DebugPrintOut::getInstance();

	HRESULT hr = ImageGrabber::CreateInstance(&igt_pImageGrabber, deviceID);
		
	igt_DeviceID = deviceID;

	if(SUCCEEDED(hr))
	{
		hr = igt_pImageGrabber->initImageGrabber(pSource, MFVideoFormat_RGB24);

		if(!SUCCEEDED(hr))
		{
			DPO->printOut(L"IMAGEGRABBERTHREAD VideoDevice %i: There is a problem with initialization of the instance of the ImageGrabber class\n", deviceID);
		}
		else
		{
			DPO->printOut(L"IMAGEGRABBERTHREAD VideoDevice %i: Initialization of instance of the ImageGrabber class\n", deviceID);
		}
	}
	else
	{
		DPO->printOut(L"IMAGEGRABBERTHREAD VideoDevice %i There is a problem with creation of the instance of the ImageGrabber class\n", deviceID);
	}
}


//----------------------------------------------------------------------------------------
ImageGrabberThread::~ImageGrabberThread(void)
{
	DebugPrintOut *DPO = &DebugPrintOut::getInstance();

	DPO->printOut(L"IMAGEGRABBERTHREAD VideoDevice %i: Destroing ImageGrabberThread\n", igt_DeviceID);

	delete igt_pImageGrabber;
	igt_pImageGrabber = NULL;
}

//----------------------------------------------------------------------------------------
void ImageGrabberThread::setEmergencyStopEvent(void *userData, void(*func)(int, void *))
{
	if(func)
	{
		igt_func = func;

		igt_userData = userData;
	}
}

//----------------------------------------------------------------------------------------
void ImageGrabberThread::stop()
{
	igt_stop = true;

	if(igt_pImageGrabber)
	{
		igt_pImageGrabber->stopGrabbing();
	}
}

//----------------------------------------------------------------------------------------
void ImageGrabberThread::start()
{
	igt_Handle = _beginthreadex( 
            NULL,                   // default security attributes
            0,                      // use default stack size  
            MainThreadFunction,       // thread function name
            this,          // argument to thread function 
            0,                      // use default creation flags 
            &igt_ThreadIdArray);   // returns the thread identifier 
}

//----------------------------------------------------------------------------------------
void ImageGrabberThread::run()
{
	DebugPrintOut *DPO = &DebugPrintOut::getInstance();

	if(igt_pImageGrabber)
	{
		DPO->printOut(L"IMAGEGRABBERTHREAD VideoDevice %i: Thread for grabbing images is started\n", igt_DeviceID);

		runMutex.lock();
		
		HRESULT hr = igt_pImageGrabber->startGrabbing();

		runMutex.unlock();

		if(!SUCCEEDED(hr))		
		{
			DPO->printOut(L"IMAGEGRABBERTHREAD VideoDevice %i: There is a problem with starting the process of grabbing\n", igt_DeviceID);
		}
		
	}
	else
	{
		DPO->printOut(L"IMAGEGRABBERTHREAD VideoDevice %i The thread is finished without execution of grabbing\n", igt_DeviceID);
	}


	if(!igt_stop)
	{
		DPO->printOut(L"IMAGEGRABBERTHREAD VideoDevice %i: Emergency Stop thread\n", igt_DeviceID);

		if(igt_func)
		{
			igt_func(igt_DeviceID, igt_userData);
		}
	}
	else
    {
		DPO->printOut(L"IMAGEGRABBERTHREAD VideoDevice %i: Finish thread\n", igt_DeviceID);
    }
}

//----------------------------------------------------------------------------------------
ImageGrabber *ImageGrabberThread::getImageGrabber()
{
	return igt_pImageGrabber;
}