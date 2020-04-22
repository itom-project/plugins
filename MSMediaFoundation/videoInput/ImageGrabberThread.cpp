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
HRESULT ImageGrabberThread::CreateInstance(
    ImageGrabberThread **ppIGT, 
    IMFMediaSource *pSource, 
    unsigned int deviceID,
    QSharedPointer<DebugPrintOut> debugPrintOut)
{
	*ppIGT = new (std::nothrow) ImageGrabberThread(pSource, deviceID, debugPrintOut);

    if (ppIGT == NULL)
    {
		debugPrintOut->printOut("IMAGEGRABBERTHREAD VideoDevice %i: Memory cannot be allocated\n", deviceID);

        return E_OUTOFMEMORY;
    }
    else
    {
        debugPrintOut->printOut("IMAGEGRABBERTHREAD VideoDevice %i: Creating of the instance of ImageGrabberThread\n", deviceID);
    }
	
    return S_OK;
}

//----------------------------------------------------------------------------------------
ImageGrabberThread::ImageGrabberThread(IMFMediaSource *pSource, unsigned int deviceID, QSharedPointer<DebugPrintOut> debugPrintOut):
    m_igtHandle(0), 
    m_igtStop(false), 
    m_pIgtImageGrabber(NULL),
    m_debugPrintOut(debugPrintOut)
{
	HRESULT hr = ImageGrabber::CreateInstance(&m_pIgtImageGrabber, deviceID, debugPrintOut);
		
	m_igtDeviceID = deviceID;

	if (SUCCEEDED(hr))
	{
		hr = m_pIgtImageGrabber->initImageGrabber(pSource, MFVideoFormat_RGB24);

		if (!SUCCEEDED(hr))
		{
            m_debugPrintOut->printOut("IMAGEGRABBERTHREAD VideoDevice %i: There is a problem with initialization of the instance of the ImageGrabber class\n", deviceID);
		}
		else
		{
            m_debugPrintOut->printOut("IMAGEGRABBERTHREAD VideoDevice %i: Initialization of instance of the ImageGrabber class\n", deviceID);
		}
	}
	else
	{
        m_debugPrintOut->printOut("IMAGEGRABBERTHREAD VideoDevice %i There is a problem with creation of the instance of the ImageGrabber class\n", deviceID);
	}
}


//----------------------------------------------------------------------------------------
ImageGrabberThread::~ImageGrabberThread(void)
{
    m_debugPrintOut->printOut("IMAGEGRABBERTHREAD VideoDevice %i: Destroying ImageGrabberThread\n", m_igtDeviceID);

	delete m_pIgtImageGrabber;
	m_pIgtImageGrabber = NULL;
}

//----------------------------------------------------------------------------------------
void ImageGrabberThread::setEmergencyStopEvent(void *userData, void(*func)(int, void *))
{
	if (func)
	{
		m_igtFunc = func;

		m_igtUserData = userData;
	}
}

//----------------------------------------------------------------------------------------
void ImageGrabberThread::stop()
{
	m_igtStop = true;

	if (m_pIgtImageGrabber)
	{
		m_pIgtImageGrabber->stopGrabbing();
	}
}

//----------------------------------------------------------------------------------------
void ImageGrabberThread::start()
{
	m_igtHandle = _beginthreadex( 
            NULL,                   // default security attributes
            0,                      // use default stack size  
            MainThreadFunction,       // thread function name
            this,          // argument to thread function 
            0,                      // use default creation flags 
            &m_igtThreadIdArray);   // returns the thread identifier 
}

//----------------------------------------------------------------------------------------
void ImageGrabberThread::run()
{
	if (m_pIgtImageGrabber)
	{
        m_debugPrintOut->printOut("IMAGEGRABBERTHREAD VideoDevice %i: Thread for grabbing images is started\n", m_igtDeviceID);

		runMutex.lock();
		
		HRESULT hr = m_pIgtImageGrabber->startGrabbing();

		runMutex.unlock();

		if (!SUCCEEDED(hr))		
		{
            m_debugPrintOut->printOut("IMAGEGRABBERTHREAD VideoDevice %i: There is a problem with starting the process of grabbing\n", m_igtDeviceID);
		}
		
	}
	else
	{
        m_debugPrintOut->printOut("IMAGEGRABBERTHREAD VideoDevice %i The thread is finished without execution of grabbing\n", m_igtDeviceID);
	}


	if (!m_igtStop)
	{
        m_debugPrintOut->printOut("IMAGEGRABBERTHREAD VideoDevice %i: Emergency Stop thread\n", m_igtDeviceID);

		if (m_igtFunc)
		{
			m_igtFunc(m_igtDeviceID, m_igtUserData);
		}
	}
	else
    {
        m_debugPrintOut->printOut("IMAGEGRABBERTHREAD VideoDevice %i: Finish thread\n", m_igtDeviceID);
    }
}

//----------------------------------------------------------------------------------------
ImageGrabber *ImageGrabberThread::getImageGrabber()
{
	return m_pIgtImageGrabber;
}