
#include <mfapi.h>
#include <mfplay.h>


#include "Media_Foundation.h"
#include "VideoDevices.h"
#include "DebugPrintOut.h"
#include "Common.h"

//----------------------------------------------------------------------
Media_Foundation::Media_Foundation(QSharedPointer<DebugPrintOut> debugPrintOut) :
    m_debugPrintOut(debugPrintOut)
{
	HRESULT hr = MFStartup(MF_VERSION);

	if (!SUCCEEDED(hr))
	{
		m_debugPrintOut->printOut("MEDIA FOUNDATION: It cannot be created!!!\n");
	}
}

//----------------------------------------------------------------------
Media_Foundation::~Media_Foundation(void)
{
	HRESULT hr = MFShutdown();  
	
	if (!SUCCEEDED(hr))
	{
		m_debugPrintOut->printOut("MEDIA FOUNDATION: Resources cannot be released\n");
	}
}

//----------------------------------------------------------------------
bool Media_Foundation::buildListOfDevices(QSharedPointer<VideoDevices> videoDevices)
{	
	HRESULT hr = S_OK;
	
	IMFAttributes *pAttributes = NULL;

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
		hr = videoDevices->initDevices(pAttributes);
    }	
	else
	{
		m_debugPrintOut->printOut("MEDIA FOUNDATION: The access to the video cameras denied\n");
	
	}

	SafeRelease(&pAttributes);

	return (SUCCEEDED(hr));
}