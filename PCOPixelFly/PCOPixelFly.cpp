#include "PCOPixelFly.h"
#include "pluginVersion.h"
#include "common/sharedFunctionsQt.h"

#define PCO_DRIVER_V2 1 //if 1, this can handle both the dll-major version 1 and 2!, if 0: only the dll-major version 1 is supported

#if PCO_DRIVER_V2 == 1
    #include "./PCO/v201_03/Pccam.h"
    #include "./PCO/v201_03/pccamdef.h"
    #include "./PCO/v201_03/camlib.h"
#else
    #include "./PCO/pccam.h"
    #include "./PCO/pccamdef.h"
    #include "./PCO/camlib.h"
#endif

#include <QFile>
#include <qstring.h>
#include <qstringlist.h>
#include <QtCore/QtPlugin>

#include "dockWidgetPCOPixelFly.h"
#include "common/helperCommon.h"

#ifndef linux
    #pragma comment(lib, "Version.lib")
#endif

Q_DECLARE_METATYPE(ito::DataObject)

//int PCOPixelFlyInterface::m_instCounter = 5;  // initialization starts with five due to normal boards are 0..4

static char InitList[5] = {0, 0, 0, 0, 0};  /*!<A map with successfull initialized board (max = 5) */
static char Initnum = 0;    /*!< Number of successfull initialized PCO-Pixelfly-Board */ 

static 	HINSTANCE g_libcam = NULL;	/*!< Handle to the pccam.dll in windows\system32 */
static  HINSTANCE g_libpcocnv = NULL;	/*!< Handle to the pcocnv.dll in .\plugins\PCO */

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal PCOPixelFlyInterface::getAddInInst(ito::AddInBase **addInInst)
{
    PCOPixelFly* newInst = new PCOPixelFly();
    newInst->setBasePlugin(this);
    *addInInst = qobject_cast<ito::AddInBase*>(newInst);

    m_InstList.append(*addInInst);

    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal PCOPixelFlyInterface::closeThisInst(ito::AddInBase **addInInst)
{
    if (*addInInst)
    {
        delete ((PCOPixelFly *)*addInInst);
        int idx = m_InstList.indexOf(*addInInst);
        m_InstList.removeAt(idx);
    }

    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
PCOPixelFlyInterface::PCOPixelFlyInterface()
{
    m_type = ito::typeDataIO | ito::typeGrabber;
    setObjectName("PCOPixelFly");
    
    m_description = QObject::tr("PCO Pixelfly cameras");

    char docstring[] = \
"This plugin connects the grabber family Pixelfly from PCO to itom. It has mainly been tested with the camera 'pixelfly qe', \
that is connected to the computer by the PCO PCI interface board 540. \n\
\n\
Please install first the necessary drivers for the camera and grabber board from www.pco.de. This plugin supports two families of \
drivers. The driver with major version 1 only supports Windows, 32bit operating systems, while the new driver version 2 also operates \
on 64bit Windows systems.";
	m_detaildescription = QObject::tr(docstring);

    m_detaildescription = QObject::tr("Developed for Windows only. Tested with PixelFlyQE.");
    m_author = "W. Lyda, M. Gronle, ITO, University Stuttgart";
    m_version = (PLUGIN_VERSION_MAJOR << 16) + (PLUGIN_VERSION_MINOR << 8) + PLUGIN_VERSION_PATCH;
    m_minItomVer = MINVERSION;
    m_maxItomVer = MAXVERSION;
    m_license = QObject::tr("LGPL / the contained camera SDK belongs to PCO - Computer Optics GmbH");
    m_aboutThis = QObject::tr("N.A.");        
    
    ito::Param paramVal = ito::Param("Board Number", ito::ParamBase::Int, 0, 3, 0, NULL);
    m_initParamsOpt.append(paramVal);
    paramVal = ito::Param("restoreLast", ito::ParamBase::Int, 0, 1, 0, NULL);
    m_initParamsOpt.append(paramVal);

    return;
}

//----------------------------------------------------------------------------------------------------------------------------------
PCOPixelFlyInterface::~PCOPixelFlyInterface()
{
    m_initParamsMand.clear();
    m_initParamsOpt.clear();
}

//----------------------------------------------------------------------------------------------------------------------------------
Q_EXPORT_PLUGIN2(PCOPixelFlyinterface, PCOPixelFlyInterface)

//----------------------------------------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------------------------------------
const ito::RetVal PCOPixelFly::showConfDialog(void)
{
    ito::RetVal retValue(ito::retOk);
    int bitppix_old = 12;
    int binning_old = 0;
    int bitppix_new = 12;
    int binning_new = 0;
    double offset_new = 0.0;

    //dialogPCOPixelFly *confDialog = new dialogPCOPixelFly();
    dialogPCOPixelFly *confDialog = new dialogPCOPixelFly(this);

    connect(this, SIGNAL(parametersChanged(QMap<QString, ito::Param>)), confDialog, SLOT(valuesChanged(QMap<QString, ito::Param>)));
    QMetaObject::invokeMethod(this, "sendParameterRequest");

    if (confDialog->exec())
    {
        disconnect(this, SIGNAL(parametersChanged(QMap<QString, ito::Param>)), confDialog, SLOT(valuesChanged(QMap<QString, ito::Param>)));
        confDialog->sendVals();
    }
    else
    {
        disconnect(this, SIGNAL(parametersChanged(QMap<QString, ito::Param>)), confDialog, SLOT(valuesChanged(QMap<QString, ito::Param>)));
    }
    delete confDialog;

    return retValue;
}
//----------------------------------------------------------------------------------------------------------------------------------
PCOPixelFly::PCOPixelFly() : 
    AddInGrabber(), 
    m_saveParamsOnClose(false), 
    m_hdriver(NULL), 
    m_pBWlut(NULL), 
    m_pictimeout(0), 
    m_nextbuf(0), 
    m_isgrabbing(0),
    m_verticalBinning(0),
    m_horizontalBinning(0),
    m_libraryMajor(0)
{
	//qRegisterMetaType<ito::DataObject>("ito::DataObject");
	//qRegisterMetaType<QMap<QString, ito::Param> >("QMap<QString, ito::Param>");

   ito::Param paramVal("name", ito::ParamBase::String | ito::ParamBase::Readonly | ito::ParamBase::NoAutosave, "PCOPixelFly", NULL);
   m_params.insert(paramVal.getName(), paramVal);
   paramVal = ito::Param("integration_time", ito::ParamBase::Double, 0.000010, 0.065, 0.01, tr("Integrationtime of CCD programmed in s").toAscii().data());
   m_params.insert(paramVal.getName(), paramVal);
   paramVal = ito::Param("frame_time", ito::ParamBase::Double | ito::ParamBase::Readonly, 1/13.0, 1/13.0, 1/13.0, tr("Shortest time between two frames").toAscii().data());
   m_params.insert(paramVal.getName(), paramVal);

   paramVal = ito::Param("gain", ito::ParamBase::Double, 0.0, 1.0, 0.0, tr("Toggle nighvision ON/OFF").toAscii().data());
   m_params.insert(paramVal.getName(), paramVal);
   paramVal = ito::Param("offset", ito::ParamBase::Double, 0.0, 1.0, 0.0, tr("Currently not used").toAscii().data());
   m_params.insert(paramVal.getName(), paramVal);

   paramVal = ito::Param("x0", ito::ParamBase::Int, 0, 1391, 0, tr("Startvalue for ROI").toAscii().data());
   m_params.insert(paramVal.getName(), paramVal);
   paramVal = ito::Param("y0", ito::ParamBase::Int, 0, 1023, 0, tr("Stoppvalue for ROI").toAscii().data());
   m_params.insert(paramVal.getName(), paramVal);
   paramVal = ito::Param("x1", ito::ParamBase::Int, 0, 1391, 1391, tr("Stopvalue for ROI").toAscii().data());
   m_params.insert(paramVal.getName(), paramVal);
   paramVal = ito::Param("y1", ito::ParamBase::Int, 0, 1023, 1023, tr("Stopvalue for ROI").toAscii().data());
   m_params.insert(paramVal.getName(), paramVal);
   paramVal = ito::Param("sizex", ito::ParamBase::Int | ito::ParamBase::Readonly, 1, 1392, 1392, tr("ROI-Size in x").toAscii().data());
   m_params.insert(paramVal.getName(), paramVal);
   paramVal = ito::Param("sizey", ito::ParamBase::Int | ito::ParamBase::Readonly, 1, 1024, 1024, tr("ROI-Size in y").toAscii().data());
   m_params.insert(paramVal.getName(), paramVal);

   paramVal = ito::Param("bpp", ito::ParamBase::Int, 8, 12, 12, tr("Grabdepth in bpp").toAscii().data());
   m_params.insert(paramVal.getName(), paramVal);

   paramVal = ito::Param("binning", ito::ParamBase::Int, 101, 202, 101, tr("Activate 2x2 binning").toAscii().data());
   m_params.insert(paramVal.getName(), paramVal);
   paramVal = ito::Param("trigger_mode", ito::ParamBase::Int, 0x10, 0x41, 0x11, tr("Set Triggermode, currently not implemented").toAscii().data());
   m_params.insert(paramVal.getName(), paramVal);
   paramVal = ito::Param("shift_bits", ito::ParamBase::Int, 0, 4, 0, tr("Shiftbits in 8-bitmode only").toAscii().data());
   m_params.insert(paramVal.getName(), paramVal);
   paramVal = ito::Param("board_number", ito::ParamBase::Int | ito::ParamBase::Readonly | ito::ParamBase::NoAutosave, 0, 4, 0, tr("Number of this board").toAscii().data());
   m_params.insert(paramVal.getName(), paramVal);
   paramVal = ito::Param("driver_version", ito::ParamBase::String | ito::ParamBase::Readonly | ito::ParamBase::NoAutosave, NULL, tr("driver version").toAscii().data());
   m_params.insert(paramVal.getName(), paramVal);

    memset((void*) &(this->m_bufnumber[0]), 0, BUFFERNUMBER*sizeof(int));
    memset((void*) &(this->m_event[0]), 0, BUFFERNUMBER*sizeof(HANDLE));
	memset((void*) &(this->m_waited[0]), 0, BUFFERNUMBER*sizeof(BOOL));
	memset((void*) &(this->m_pAdr[0]), 0, BUFFERNUMBER*sizeof(void*));


    //now create dock widget for this plugin
    DockWidgetPCOPixelFly *myDockWidget = new DockWidgetPCOPixelFly(m_params, getID() );
    connect(this, SIGNAL(parametersChanged(QMap<QString, ito::Param>)), myDockWidget, SLOT(valuesChanged(QMap<QString, ito::Param>)));
    connect(myDockWidget, SIGNAL(GainPropertiesChanged(double)), this, SLOT(GainPropertiesChanged(double)));
	connect(myDockWidget, SIGNAL(OffsetPropertiesChanged(double)), this, SLOT(OffsetPropertiesChanged(double)));
    connect(myDockWidget, SIGNAL(IntegrationPropertiesChanged(double)), this, SLOT(IntegrationPropertiesChanged(double)));

    Qt::DockWidgetAreas areas = Qt::AllDockWidgetAreas;
    QDockWidget::DockWidgetFeatures features = QDockWidget::DockWidgetClosable | QDockWidget::DockWidgetFloatable | QDockWidget::DockWidgetMovable;
    createDockWidget(QString(m_params["name"].getVal<char *>()), features, areas, myDockWidget);

}

//----------------------------------------------------------------------------------------------------------------------------------
PCOPixelFly::~PCOPixelFly()
{
   m_params.clear();
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal PCOPixelFly::PCOLoadLibrary(void)
{
	ito::RetVal retValue = ito::retOk;

	QString version;
    libraryVersionNumber("pccam.dll", version);

	m_params["driver_version"].setVal<char*>( version.toAscii().data() );

    QStringList version2 = version.split(".");
    if(version2.size() > 0)
    {
        bool ok = false;
        int major = version2[0].toInt(&ok);
        if(ok && major > 0)
        {
            m_libraryMajor = major;
        }
    }
	
    if(!g_libcam && !Initnum)   // so no grabber is initialized yet, dll shoud not be loaded
	{

#if UNICODE
        g_libcam = LoadLibrary(L"pccam.dll");
#else
        g_libcam = LoadLibrary("pccam.dll");
#endif
    }

	if (!g_libcam)
	{
		retValue += ito::RetVal(ito::retError, 0, tr("Could not load library \"pccam.dll\"").toAscii().data());
		return retValue;
	}

	initboard = (int(*)(int,HANDLE*))
               GetProcAddress(g_libcam,"INITBOARD");
	if (initboard == NULL)
	{
		retValue += ito::RetVal(ito::retError, 0, tr("Cannot get function INITBOARD").toAscii().data());
	}
	closeboard = (int(*)(HANDLE*))
               GetProcAddress(g_libcam,"CLOSEBOARD");
	if (closeboard == NULL)
	{
		retValue += ito::RetVal(ito::retError, 0, tr("Cannot get function CLOSEBOARD").toAscii().data());
	}
	getboardpar = (int(*)(HANDLE,unsigned int*,int))
               GetProcAddress(g_libcam,"GETBOARDPAR");
	if (getboardpar == NULL)
	{
		retValue += ito::RetVal(ito::retError, 0, tr("Cannot get function GETBOARDPAR").toAscii().data());
	}

    getboardval = NULL;
    if(m_libraryMajor >= 2)
    {
        getboardval = (int(*)(HANDLE,int,void*))
               GetProcAddress(g_libcam,"GETBOARDVAL");
	    if (getboardpar == NULL)
	    {
		    retValue += ito::RetVal(ito::retError, 0, tr("Cannot get function GETBOARDVAL").toAscii().data());
	    }
    }

	getsizes = (int(*)(HANDLE,int*,int*,int*,int*,int*))
               GetProcAddress(g_libcam,"GETSIZES");
	if (getsizes == NULL)
	{
		retValue += ito::RetVal(ito::retError, 0, tr("Cannot get function GETSIZES").toAscii().data());
	}

	setmode = (int(*)(HANDLE,int,int,int,int,int,int,int,int,int))
               GetProcAddress(g_libcam,"SETMODE");
	if (setmode == NULL)
	{
		retValue += ito::RetVal(ito::retError, 0, tr("Cannot get function SETMODE").toAscii().data());
	}
	start_camera = (int(*)(HANDLE))
               GetProcAddress(g_libcam,"START_CAMERA");
	if (start_camera == NULL)
	{
		retValue += ito::RetVal(ito::retError, 0, tr("Cannot get function START_CAMERA").toAscii().data());
	}

	stop_camera = (int(*)(HANDLE))
               GetProcAddress(g_libcam,"STOP_CAMERA");
	if (stop_camera == NULL)
	{
		retValue += ito::RetVal(ito::retError, 0, tr("Cannot get function STOP_CAMERA").toAscii().data());
	}

	trigger_camera = (int(*)(HANDLE))
               GetProcAddress(g_libcam,"TRIGGER_CAMERA");
	if (trigger_camera == NULL)
	{
		retValue += ito::RetVal(ito::retError, 0, tr("Cannot get function TRIGGER_CAMERA").toAscii().data());
	}

	allocate_buffer = (int(*)(HANDLE,int*,int*))
               GetProcAddress(g_libcam,"ALLOCATE_BUFFER");
	if (allocate_buffer == NULL)
	{
		retValue += ito::RetVal(ito::retError, 0, tr("Cannot get function ALLOCATE_BUFFER").toAscii().data());
	}

	free_buffer = (int(*)(HANDLE,int))
               GetProcAddress(g_libcam,"FREE_BUFFER");
	if (free_buffer == NULL)
	{
		retValue += ito::RetVal(ito::retError, 0, tr("Cannot get function FREE_BUFFER").toAscii().data());
	}

	getbuffer_status = (int(*)(HANDLE,int,int,int*,int))
               GetProcAddress(g_libcam,"GETBUFFER_STATUS");
	if (getbuffer_status == NULL)
	{
		retValue += ito::RetVal(ito::retError, 0, tr("Cannot get function GETBUFFER_STATUS").toAscii().data());
	}

	add_buffer_to_list = (int(*)(HANDLE,int,int,int,int))
               GetProcAddress(g_libcam,"ADD_BUFFER_TO_LIST");
	if (add_buffer_to_list == NULL)
	{
		retValue += ito::RetVal(ito::retError, 0, tr("Cannot get function ADD_BUFFER_TO_LIST").toAscii().data());
	}

	remove_buffer_from_list = (int(*)(HANDLE,int))
               GetProcAddress(g_libcam,"REMOVE_BUFFER_FROM_LIST");
	if (remove_buffer_from_list == NULL)
	{
		retValue += ito::RetVal(ito::retError, 0, tr("Cannot get function REMOVE_BUFFER_FROM_LIST").toAscii().data());
	}

	setbuffer_event = (int(*)(HANDLE,int,HANDLE*))
               GetProcAddress(g_libcam,"SETBUFFER_EVENT");
	if (setbuffer_event == NULL)
	{
		retValue += ito::RetVal(ito::retError, 0, tr("Cannot get function SET_BUFFER_EVENT").toAscii().data());
	}

#if _WIN64
	map_buffer_ex = (int(*)(HANDLE,int,int,int,void**))
               GetProcAddress(g_libcam,"MAP_BUFFER_EX");
	if (map_buffer_ex == NULL)
	{
		retValue += ito::RetVal(ito::retError, 0, tr("Cannot get function MAP_BUFFER_EX").toAscii().data());
	}
    map_buffer = NULL;
#else
    map_buffer = (int(*)(HANDLE,int,int,int,DWORD*))
               GetProcAddress(g_libcam,"MAP_BUFFER");
	if (map_buffer == NULL)
	{
		retValue += ito::RetVal(ito::retError, 0, tr("Cannot get function MAP_BUFFER").toAscii().data());
	}
    map_buffer_ex = NULL;
#endif

	unmap_buffer = (int(*)(HANDLE,int))
               GetProcAddress(g_libcam,"UNMAP_BUFFER");
	if (unmap_buffer == NULL)
	{
		retValue += ito::RetVal(ito::retError, 0, tr("Cannot get function UNMAP_BUFFER").toAscii().data());
	}

	if ((retValue != ito::retOk))
	{
        if(!Initnum)
        {
            FreeLibrary(g_libcam);
		    g_libcam = NULL;
        }
        else
        {
            std::cerr << "DLLs not unloaded due to further running grabber instances\n" << std::endl;
        }
		return retValue;
	}

//    if(!g_libpcocnv && !Initnum)   // so no grabber is initialized yet, dll shoud not be loaded
//	{
//#if UNICODE
//	    if ((g_libpcocnv = LoadLibrary(L".\\plugins\\PCOPixelFly\\PCO\\pcocnv.dll")) == NULL)
//#else
//	    if ((g_libpcocnv = LoadLibrary(".\\plugins\\PCOPixelFly\\PCO\\pcocnv.dll")) == NULL)
//#endif
//	    {
//		    retValue += ito::RetVal(ito::retError, 0, tr("LoadLibrary(\"pcocnv.dll\")").toAscii().data());
//	    }
//    }
//
//	create_bwlut = (void *(*)(int,int,int))
//               GetProcAddress(g_libpcocnv,"CREATE_BWLUT");
//	if (create_bwlut == NULL)
//	{
//		retValue += ito::RetVal(ito::retError, 0, tr("Cannot get function CREATE_BWLUT").toAscii().data());
//	}
//	delete_bwlut = (int(*)(void *))
//               GetProcAddress(g_libpcocnv,"DELETE_BWLUT");
//	if (delete_bwlut == NULL)
//	{
//		retValue += ito::RetVal(ito::retError, 0, tr("Cannot get function DELETE_BWLUT").toAscii().data());
//	}
//
//	convert_set = (int(*)(void *,int,int,int))
//              GetProcAddress(g_libpcocnv,"CONVERT_SET");
//	if (convert_set == NULL)
//	{
//		retValue += ito::RetVal(ito::retError, 0, tr("Cannot get function CONVERT_SET").toAscii().data());
//	}
//
//	conv_buf_12to8 = (int(*)(int,int,int,unsigned short*,unsigned char*,void*))
//                 GetProcAddress(g_libpcocnv,"CONV_BUF_12TO8");
//	if (conv_buf_12to8 == NULL)
//	{
//		retValue += ito::RetVal(ito::retError, 0, tr("Cannot get function CONV_BUF_12TO8").toAscii().data());
//	}

	if (retValue != ito::retOk)
	{
		if(Initnum > 0)
        {
            if (g_libcam)
		    {
			    FreeLibrary(g_libcam);
		    }
            g_libcam = NULL;

		    /*if (g_libpcocnv)
		    {
			    FreeLibrary(g_libpcocnv);
		    }
		    g_libpcocnv = NULL;*/
        }
        else
        {
            std::cerr << "DLLs not unloaded due to further running grabber instances\n" << std::endl;
        }
	}

	return retValue;
}

ito::RetVal PCOPixelFly::libraryVersionNumber(const QByteArray &fileName, QString &version)
{
    //source: http://stackoverflow.com/questions/940707/how-do-i-programatically-get-the-version-of-a-dll-or-exe-file
    version = "0.0.0.0";
#ifdef linux
    
    return ito::RetVal(ito::retError,0,tr("the library version can only be fetched on windows systems").toAscii().data());
#else
    DWORD               dwSize              = 0;
    BYTE                *pbVersionInfo      = NULL;
    VS_FIXEDFILEINFO    *pFileInfo          = NULL;
    UINT                puLenFileInfo       = 0;

    #if UNICODE
        wchar_t *pszFilePath = new wchar_t[ fileName.size() + 2];
        int size = fileName.toWCharArray(pszFilePath);
        pszFilePath[size] = '\0';
        delete[] pszFilePath;
    #else
        char *pszFilePath = qstrdup( fileName.data() );
    #endif

    // get the version info for the file requested
    dwSize = GetFileVersionInfoSize( pszFilePath, NULL );
    if ( dwSize == 0 )
    {
        delete[] pszFilePath;
        return ito::RetVal::format(ito::retError,0,"Error in GetFileVersionInfoSize: %d", GetLastError() );
    }

    pbVersionInfo = new BYTE[ dwSize ];

    if ( !GetFileVersionInfo( pszFilePath, 0, dwSize, pbVersionInfo ) )
    {
        delete[] pszFilePath;
        delete[] pbVersionInfo;
        return ito::RetVal::format(ito::retError,0,"Error in GetFileVersionInfo: %d", GetLastError() );
    }

    delete[] pszFilePath;

    if ( !VerQueryValue( pbVersionInfo, TEXT("\\"), (LPVOID*) &pFileInfo, &puLenFileInfo ) )
    {
        delete[] pbVersionInfo;
        return ito::RetVal::format(ito::retError,0,"Error in VerQueryValue: %d", GetLastError() );
    }

    DWORD maj = ( pFileInfo->dwFileVersionMS >> 16 ) & 0xff;
    DWORD min = ( pFileInfo->dwFileVersionMS >> 0 ) & 0xff;
    DWORD build = ( pFileInfo->dwFileVersionLS >>  16 ) & 0xff;
    DWORD revision = ( pFileInfo->dwFileVersionLS >>  0 ) & 0xff;
    version = QString("%1.%2.%3.%4").arg(maj).arg(min).arg(build).arg(revision);
    //// pFileInfo->dwFileVersionMS is usually zero. However, you should check
    //// this if your version numbers seem to be wrong

    //printf( "File Version: %d.%d.%d.%d\n",
    //    ( pFileInfo->dwFileVersionLS >> 24 ) & 0xff,
    //    ( pFileInfo->dwFileVersionLS >> 16 ) & 0xff,
    //    ( pFileInfo->dwFileVersionLS >>  8 ) & 0xff,
    //    ( pFileInfo->dwFileVersionLS >>  0 ) & 0xff
    //    );

    //// pFileInfo->dwProductVersionMS is usually zero. However, you should check
    //// this if your version numbers seem to be wrong

    //printf( "Product Version: %d.%d.%d.%d\n",
    //    ( pFileInfo->dwProductVersionLS >> 24 ) & 0xff,
    //    ( pFileInfo->dwProductVersionLS >> 16 ) & 0xff,
    //    ( pFileInfo->dwProductVersionLS >>  8 ) & 0xff,
    //    ( pFileInfo->dwProductVersionLS >>  0 ) & 0xff
    //    );

    return ito::retOk;
#endif
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal PCOPixelFly::PCOChkError(int errornumber)
{
	ito::RetVal retValue = ito::retOk;
	int i;

    if(errornumber == 0)
    {
        return retValue;
    }

	struct error
	{
		int value;
		const char *text;
	} 

	errors[] =
	{	/* All Errormassages are taken from the PCO-Manual. */
		{	0,      tr("no Error").toAscii().data()},
		{	-1,	    tr("initialization failed, no camera connected").toAscii().data()},
		{	-2,	    tr("timeout in any function").toAscii().data()},
		{	-3,	    tr("function call with wrong parameter").toAscii().data()},
		{	-4,	    tr("cannot locate PCI card or card driver").toAscii().data()},
		{	-5,	    tr("wrong operating system").toAscii().data()},
		{	-6,	    tr("no or wrong driver installed").toAscii().data()},
		{	-7,	    tr("IO function failed").toAscii().data()},
		{	-8,	    tr("reserved").toAscii().data()},
		{	-9,	    tr("invalid camera mode").toAscii().data()},
		{	-10,    tr("reserved").toAscii().data()},
		{	-11,    tr("device is hold by another process").toAscii().data()},
		{	-12,    tr("error in reading or writing data to board").toAscii().data()},
		{	-13,    tr("wrong driver function").toAscii().data()},
		{	-14,    tr("reserved").toAscii().data()},
		{	-101,   tr("timeout in any driver function").toAscii().data()},
		{	-102,   tr("board is hold by an other process").toAscii().data()},
		{	-103,   tr("wrong boardtype").toAscii().data()},
		{	-104,   tr("cannot match processhandle to a board").toAscii().data()},
		{	-105,   tr("failed to init PCI").toAscii().data()},
		{	-106,   tr("no board found").toAscii().data()},
		{	-107,   tr("read configuratuion registers failed").toAscii().data()},
		{	-108,   tr("board has wrong configuration").toAscii().data()},
		{	-109,   tr("memory allocation error").toAscii().data()},
		{	-110,   tr("camera is busy").toAscii().data()},
		{	-111,   tr("board is not idle").toAscii().data()},
		{	-112,   tr("wrong parameter in function cal").toAscii().data()},
		{	-113,   tr("head is disconnected").toAscii().data()},
		{	-114,   tr("head verification failed").toAscii().data()},
		{	-115,   tr("board cannot work with attached head").toAscii().data()},
		{	-116,   tr("board initialisation FPGA failed").toAscii().data()},
		{	-117,   tr("board initialisation NVRAM failed").toAscii().data()},
		{	-120,   tr("not enough IO-buffer space for return values").toAscii().data()},
		{	-121,   tr("not enough IO-buffer space for return values").toAscii().data()},
		{	-122,   tr("Head power is switched off").toAscii().data()},
		{	-130,   tr("picture buffer not prepared for transfer").toAscii().data()},
		{	-131,   tr("picture buffer in use").toAscii().data()},
		{	-132,   tr("picture buffer hold by another process").toAscii().data()},
		{	-133,   tr("picture buffer not found").toAscii().data()},
		{	-134,   tr("picture buffer cannot be freed").toAscii().data()},
		{	-135,   tr("cannot allocate more picture buffer").toAscii().data()},
		{	-136,   tr("no memory left for picture buffer").toAscii().data()},
		{	-137,   tr("memory reserve failed").toAscii().data()},
		{	-138,   tr("memory commit failed").toAscii().data()},
		{	-139,   tr("allocate internal memory LUT failed").toAscii().data()},
		{	-140,   tr("allocate internal memory PAGETAB failed").toAscii().data()},
		{	-148,   tr("event not available").toAscii().data()},
		{	-149,   tr("delete event failed").toAscii().data()},
		{	-156,   tr("enable interrupts failed").toAscii().data()},
		{	-157,   tr("disable interrupts failed").toAscii().data()},
		{	-158,   tr("no interrupt connected to the board").toAscii().data()},
		{	-164,   tr("timeout in DMA").toAscii().data()},
		{	-165,   tr("no dma buffer found").toAscii().data()},
		{	-166,   tr("locking of pages failed").toAscii().data()},
		{	-167,   tr("unlocking of pages failed").toAscii().data()},
		{	-168,   tr("DMA buffersize to smal").toAscii().data()},
		{	-169,   tr("PCI-Bus error in DMA").toAscii().data()},
		{	-170,   tr("DMA is runnig, command not allowed").toAscii().data()},
		{	-228,   tr("get processor failed").toAscii().data()},
		{	-229,   tr("reserved").toAscii().data()},
		{	-230,   tr("wrong processor found").toAscii().data()},
		{	-231,   tr("wrong processor size").toAscii().data()},
		{	-232,   tr("wrong processor device").toAscii().data()},
		{	-233,   tr("read flash failed").toAscii().data()},
		{	-224,   tr("not grabbing").toAscii().data()},
	};

	for(i = 0; i < sizeof(errors) / sizeof(errors[0]); i++)
	{
		if (errors[i].value == errornumber)
		{
			retValue += ito::RetVal(ito::retError, 0, (char*) &(errors[i].text[0]));
			return retValue;
		}
	}

	retValue += ito::RetVal::format(ito::retError, 0, tr("unknown error code of PCO PixelFly (%i)").toAscii().data(), errornumber);
	return retValue;
}
//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal PCOPixelFly::PCORemoveFromList(void)
{
	ito::RetVal retValue = ito::retOk;
	
	int timeout = 100;
	int istatus = 0;
	int iRetCode = 0;

	for (int i = 0; i <BUFFERNUMBER; i++)
	{
		//if the buffer was queued remove it 
		getbuffer_status(this->m_hdriver, this->m_bufnumber[i], 0, &istatus, 4);
#if PCO_DRIVER_V2 == 1
        if (PCC_BUF_STATUS_QUEUED(istatus))
#else
	    if (PCC_BUF_STAT_QUEUED(&istatus))
#endif
		{
			int count = 0;
			while (((iRetCode = remove_buffer_from_list(this->m_hdriver,this->m_bufnumber[i])) != 0) && (count < timeout))
			{
				Sleep(5);
				count += 5;
			}
			//if a DMA-Transfer is running on this buffer we get an error
			if (iRetCode != 0)
			{
				retValue = PCOChkError(iRetCode);
				return retValue;
			}
		}
	}

	return retValue;
}
//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal PCOPixelFly::PCORemoveThisFromListIfError(int bufnr)
{
	ito::RetVal retValue = ito::retOk;
	
	int count = 0;
	int timeout = 100;
	int istatus = 0;
	int iRetCode = -3;

	getbuffer_status(this->m_hdriver, bufnr, 0, &istatus, 4); //getbuffer_status is obsolete in driver version 2 or higher, but still used here.
#if PCO_DRIVER_V2 == 1
    if (PCC_BUF_STATUS_ERROR(istatus))
#else
	if (PCC_BUF_STAT_ERROR(&istatus))
#endif
	{
	
		while(((iRetCode = remove_buffer_from_list(this->m_hdriver,bufnr)) != 0) && (count < timeout))
		{
			Sleep(5);
			count += 5;
		}
		//if a DMA-Transfer is running on this buffer we get an error
		if (iRetCode != 0)
		{
				retValue = PCOChkError(iRetCode);
				return retValue;
		}
	}

	return retValue;
}
//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal PCOPixelFly::PCOAddToList(void)
{
	ito::RetVal retValue = ito::retOk;
	int iRetCode = 0;
	long lactsize = 0;

	int maxxsize = (int)m_params["sizex"].getMax();
	int maxysize = (int)m_params["sizey"].getMax();
	int bitppix = m_params["bpp"].getVal<int>();

 	lactsize = (long)(maxxsize*maxysize*ceil(bitppix/8.0));

	for (int i = 0; i < BUFFERNUMBER; i++)
	{
		iRetCode = add_buffer_to_list(this->m_hdriver, this->m_bufnumber[i], lactsize, 0, 0);
		if (iRetCode != 0)
		{
			retValue = PCOChkError(iRetCode);
			return retValue;
		}

		this->m_waited[i] = 0;
	}
	this->m_nextbuf = 0;

	return retValue;
}	
//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal PCOPixelFly::PCOResetEvents(void)
{
	ito::RetVal retValue = ito::retOk;

	//set Event to nonsignaled state
	for (int i = 0; i < BUFFERNUMBER; i++)
	{
		ResetEvent(this->m_event[i]);
	}

	return retValue;
}
//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal PCOPixelFly::PCOAllocateBuffer(void)
{
	ito::RetVal retValue=ito::retOk;

	int iRetCode = 0;

	int maxxsize = 0;
	int maxysize = 0;
	int bitppix = 0;
	int lsize = 0;
	int i = 0;
	DWORD linadr = 0;
	DWORD handleflag[100] = {0};
	bool valid = false;

	maxxsize = m_params["sizex"].getVal<int>();
	maxysize = m_params["sizey"].getVal<int>();
	bitppix = m_params["bpp"].getVal<int>();

	for (i = 0; i < BUFFERNUMBER; i++)
	{
		this->m_bufnumber[i] = -1;
		lsize = (int)(maxxsize * maxysize * ceil(bitppix / 8.0));
		iRetCode = allocate_buffer(this->m_hdriver, &this->m_bufnumber[i], (int *) &lsize);
		if (iRetCode != 0)
		{
			retValue = this->PCOChkError(iRetCode);
		}
		Sleep(5);
	}

	for (i = 0; i < BUFFERNUMBER; i++)
	{
		if (retValue != ito::retError)
		{	
#if _WIN64
            map_buffer_ex(this->m_hdriver, this->m_bufnumber[i], lsize, 0, &(m_pAdr[i]) );
#else
			iRetCode = map_buffer(this->m_hdriver, this->m_bufnumber[i], lsize, 0, &linadr);
            this->m_pAdr[i] = (void *) linadr;
#endif
			if (iRetCode != 0)
			{
                m_pAdr[i] = NULL;
				retValue = this->PCOChkError(iRetCode);
			}
			
		}
		Sleep(5);
	}
	
	for (i = 0; i < BUFFERNUMBER; i++)
	{
		if (retValue != ito::retError)
		{	
			iRetCode = setbuffer_event(this->m_hdriver, this->m_bufnumber[i], &this->m_event[i]);
			if (iRetCode != 0)
			{
				retValue = this->PCOChkError(iRetCode);
			}
			valid = GetHandleInformation(this->m_event[i], &handleflag[0]);
		}
		Sleep(5);
	}

	this->m_nextbuf = 0;

	return retValue;

}
//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal PCOPixelFly::PCOFreeAllocatedBuffer(void)
{
	ito::RetVal retValue=ito::retOk;
	bool rethandle=true;
	int iRetCode = 0;
	int i = 0;

	for (i = 0; i < BUFFERNUMBER; i++)
	{
		iRetCode = -170;
		while(iRetCode == -170)
		{
			iRetCode = remove_buffer_from_list(this->m_hdriver, this->m_bufnumber[i]);
			Sleep(50);	// Here is something terribly wrong and I do not not what
		} 
	}
	
	for (i = 0; i < BUFFERNUMBER; i++)
	{
		iRetCode = unmap_buffer(this->m_hdriver, this->m_bufnumber[i]);
		if (iRetCode)
		{
			retValue += this->PCOChkError(iRetCode);
		}
		Sleep(50);
	}


	for (i = 0; i<BUFFERNUMBER; i++)
	{
	
		iRetCode = free_buffer(this->m_hdriver, this->m_bufnumber[i]);
		if (iRetCode)
		{
			retValue += this->PCOChkError(iRetCode);
		}
		this->m_pAdr[i]=NULL;
		Sleep(50);
	}


	for (i = 0; i < BUFFERNUMBER; i++)
	{
#ifndef _DEBUG

		if (this->m_event[i] != NULL)
		{
			rethandle = CloseHandle(this->m_event[i]);
		}
#endif
		this->m_event[i]=NULL;
	}

	return retValue;
}
//----------------------------------------------------------------------------------------------------------------------------------
/*!
    \details This method copies the complete tparam of the corresponding parameter to val

    \param [in,out] val  is a input of type::tparam containing name, value and further informations
    \param [in] waitCond is the semaphore (default: NULL), which is released if this method has been terminated
    \return retOk in case that everything is ok, else retError
    \sa ito::tParam, ItomSharedSemaphore
*/
ito::RetVal PCOPixelFly::getParam(QSharedPointer<ito::Param> val, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);
    QString key = val->getName();

    if(key == "")
    {
        retValue += ito::RetVal(ito::retError, 0, tr("name of requested parameter is empty.").toAscii().data());
    }
    else
    {
        QMap<QString, ito::Param>::const_iterator paramIt = m_params.constFind(key);
        if (paramIt != m_params.constEnd())
        {
            *val = paramIt.value();
        }
        else
        {
            retValue += ito::RetVal(ito::retError, 0, tr("parameter not found in m_params.").toAscii().data());
        }
    }

    if (waitCond) 
    {
        waitCond->returnValue = retValue;
        waitCond->release();
    }

   return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
/*!
    \detail This method copies the value of val to to the m_params-parameter and sets the corresponding camera parameters.

    \param [in] val  is a input of type::tparam containing name, value and further informations
    \param [in] waitCond is the semaphore (default: NULL), which is released if this method has been terminated
    \return retOk in case that everything is ok, else retError
    \sa ito::tParam, ItomSharedSemaphore
*/
ito::RetVal PCOPixelFly::setParam(QSharedPointer<ito::ParamBase> val, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);

	int iRetCode = 0;
	int i = 0;
	int maxxsize = 0;
	int maxysize = 0;
	int curxsize = 0;
	int curysize = 0;

	int running = 0;

	int gain = 0;

	int bitppix = 12;
	int bitppix_old = m_params["bpp"].getVal<int>();
	int shift = 0;
	
	int trigger_mode = 0x11;
	int integration_time = 10000;	
	
	QString key = val->getName();

    if(key == "")	// Check if the key is valied
    {
        retValue += ito::RetVal(ito::retError, 0, tr("name of given parameter is empty.").toAscii().data());
    }
    else	// key valid so go on
    {
		QMap<QString, ito::Param>::iterator paramIt = m_params.find(key);	// try to find the parameter in the parameter list

		if (paramIt != m_params.end()) // Okay the camera has this parameter so go on
		{
    
			if (grabberStartedCount())
			{
				running = grabberStartedCount();
				setGrabberStarted(1);
				retValue += this->stopDevice(0);
			}
			else
			{
				setGrabberStarted(1);
				this->stopDevice(0);
			}

            if(paramIt->getFlags() & ito::ParamBase::Readonly)
            {
                retValue += ito::RetVal(ito::retWarning, 0, tr("Parameter is read only, input ignored").toAscii().data());
                goto end;
            }
			else if(val->isNumeric() && paramIt->isNumeric())
			{
				double curval = val->getVal<double>();
				if( curval > paramIt->getMax())
				{
				    retValue += ito::RetVal(ito::retError, 0, tr("New value is larger than parameter range, input ignored").toAscii().data());
                    goto end;
				}
				else if(curval < paramIt->getMin())
				{
				    retValue += ito::RetVal(ito::retError, 0, tr("New value is smaller than parameter range, input ignored").toAscii().data());
                    goto end;
				}
				else 
				{
				    paramIt.value().setVal<double>(curval);
				}
			}
			else if (paramIt->getType() == val->getType())
			{
				retValue += paramIt.value().copyValueFrom( &(*val) );
			}
			else
			{
				retValue += ito::RetVal(ito::retError, 0, tr("Parameter type conflict").toAscii().data());
				goto end;
			}
		
			Sleep(5);

			integration_time = (int)(m_params["integration_time"].getVal<double>()*1000000);
			trigger_mode = m_params["trigger_mode"].getVal<int>();
		
			gain = (int)(m_params["gain"].getVal<double>()+0.5); // Toggle Gain On / Off
			m_params["gain"].setVal<int>(gain);
		
			bitppix = (int) m_params["bpp"].getVal<int>();
			shift = (int) m_params["shift_bits"].getVal<int>();

			if ((strcmp(paramIt.value().getName(),"bpp") == 0) || (strcmp(paramIt.value().getName(),"binning") == 0))
			{

                if((strcmp(paramIt.value().getName(),"binning") == 0))
                {
                    int newbinX = val->getVal<int>()/100;
                    int newbinY = val->getVal<int>()-newbinX *100;

                    int maxbinX = (int)paramIt->getMax()/100;
                    int maxbinY = (int)paramIt->getMax()- maxbinX * 100;

                    int minbinX = (int)paramIt->getMin()/100;
                    int minbinY = (int)paramIt->getMin()- minbinX * 100;

                    if( newbinX > maxbinX)
                    {
                        retValue += ito::RetVal(ito::retError, 0, tr("New value in X is larger than maximal value, input ignored").toAscii().data());
                        goto end;
                    }
                    else if( newbinY > maxbinY)
                    {
                        retValue += ito::RetVal(ito::retError, 0, tr("New value in Y is larger than maximal value, input ignored").toAscii().data());
                        goto end;
                    }
                    else if(newbinX < minbinX)
                    {
                        retValue += ito::RetVal(ito::retError, 0, tr("New value in X is smaller than parameter range, input ignored").toAscii().data());
                        goto end;
                    }
                    else if(newbinY < minbinY)
                    {
                        retValue += ito::RetVal(ito::retError, 0, tr("New value in Y is smaller than parameter range, input ignored").toAscii().data());
                        goto end;
                    }
                    else
                    {
                        m_horizontalBinning =  newbinX - 1;
                        m_verticalBinning =  newbinY - 1;
                        paramIt.value().setVal<int>(newbinX * 100 + newbinY);
                    }
                }

				if ((bitppix != 8) && (bitppix != 12) && (bitppix != 16))
				{
					iRetCode = getsizes(this->m_hdriver, &maxxsize, &maxysize, &curxsize, &curysize, &bitppix);
					if (iRetCode != 0)
					{
						retValue = this->PCOChkError(iRetCode);
						m_params["bpp"].setVal<int>(bitppix_old);
					}
					else
					{
						m_params["bpp"].setVal<int>(bitppix);
					}
				
					retValue += ito::RetVal(ito::retError, 0, tr("Tried to set invalid Bits per Pixe").toAscii().data());
					return retValue;
				}
				if (bitppix > 8)
				{
					shift = 0;
					m_params["shift_bits"].setVal<int>(0);
				}

				retValue += PCOFreeAllocatedBuffer();

				iRetCode = setmode(this->m_hdriver, trigger_mode, 0, integration_time, m_verticalBinning, m_horizontalBinning, gain, 0, bitppix, shift);
				if (iRetCode != 0)
				{
					retValue = this->PCOChkError(iRetCode);
				}

				iRetCode = getsizes(this->m_hdriver, &maxxsize, &maxysize, &curxsize, &curysize, &bitppix);
				if (iRetCode != 0)
				{
					retValue = this->PCOChkError(iRetCode);
				}

				m_params["bpp"].setVal<int>(bitppix);

				maxxsize = (int)(maxxsize /(m_horizontalBinning+1));
				maxysize = (int)(maxysize /(m_verticalBinning+1));

				static_cast<ito::IntMeta*>( m_params["sizex"].getMeta() )->setMax(maxxsize);
                static_cast<ito::IntMeta*>( m_params["sizey"].getMeta() )->setMax(maxysize);
				m_params["sizex"].setVal(curxsize);
				m_params["sizey"].setVal(curysize);

				m_params["x1"].setVal<int>(maxxsize-1);
				m_params["y1"].setVal<int>(maxysize-1);
				/*m_params["x1"].setMax(maxxsize-1);
				m_params["y1"].setMax(maxysize-1);*/
                static_cast<ito::IntMeta*>( m_params["x1"].getMeta() )->setMax(maxxsize-1);
                static_cast<ito::IntMeta*>( m_params["y1"].getMeta() )->setMax(maxysize-1);

                /*m_params["x0"].setMax(m_params["x1"].getVal<double>());
                m_params["y0"].setMax(m_params["y1"].getVal<double>());*/
                static_cast<ito::IntMeta*>( m_params["x0"].getMeta() )->setMax( m_params["x1"].getVal<int>() );
                static_cast<ito::IntMeta*>( m_params["y0"].getMeta() )->setMax( m_params["y1"].getVal<int>() );
				m_params["x0"].setVal<int>(0);
				m_params["y0"].setVal<int>(0);

                /*m_params["x1"].setMin(m_params["x0"].getVal<double>());
                m_params["y1"].setMin(m_params["x0"].getVal<double>());*/
                static_cast<ito::IntMeta*>( m_params["x1"].getMeta() )->setMin(maxxsize);
                static_cast<ito::IntMeta*>( m_params["y1"].getMeta() )->setMin(maxysize);

				retValue += PCOAllocateBuffer();
				if (iRetCode != 0)
				{
					retValue = this->PCOChkError(iRetCode);
				}
			}
            else if(!paramIt.key().compare("x0") ||
                   !paramIt.key().compare("x1") ||
                   !paramIt.key().compare("y0") ||
                   !paramIt.key().compare("y1"))
            {
                /*m_params["x0"].setMax(m_params["x1"].getVal<double>());
                m_params["y0"].setMax(m_params["y1"].getVal<double>());*/
                static_cast<ito::IntMeta*>( m_params["x0"].getMeta() )->setMax( m_params["x1"].getVal<int>() );
                static_cast<ito::IntMeta*>( m_params["y0"].getMeta() )->setMax( m_params["y1"].getVal<int>() );

                /*m_params["x1"].setMin(m_params["x0"].getVal<double>());
                m_params["y1"].setMin(m_params["x0"].getVal<double>());*/
                static_cast<ito::IntMeta*>( m_params["x1"].getMeta() )->setMin(m_params["x0"].getVal<int>());
                static_cast<ito::IntMeta*>( m_params["y1"].getMeta() )->setMin(m_params["x0"].getVal<int>());

                m_params["sizex"].setVal<int>(m_params["x1"].getVal<int>()-m_params["x0"].getVal<int>()+1);
                m_params["sizey"].setVal<int>(m_params["y1"].getVal<int>()-m_params["y0"].getVal<int>()+1);
            }
			else
			{
				iRetCode = setmode(this->m_hdriver, trigger_mode, 0, integration_time, m_verticalBinning, m_horizontalBinning, gain, 0, bitppix, shift);
				if (iRetCode != 0)
				{
					retValue = this->PCOChkError(iRetCode);
				}
			}
		}
		else
		{
			retValue = ito::RetVal(ito::retWarning, 0, tr("Parameter not found").toAscii().data());
		}
	}

end:

	retValue += checkData();

	if (running)
	{
		retValue += this->startDevice(0);
		setGrabberStarted(running);
	}

	if (!retValue.containsWarningOrError())
	{
		emit parametersChanged(m_params);
	}

	if (waitCond) 
    {
        waitCond->returnValue = retValue;
		waitCond->release();
    }

   return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal PCOPixelFly::init(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, ItomSharedSemaphore *waitCond)
{
	ItomSharedSemaphoreLocker locker(waitCond);
    
	ito::RetVal retValue(ito::retOk);
	int iRetCode = 0;
	int iBoardNumber = (*paramsOpt)[0].getVal<int>();
	int loadParamsFromXML = (*paramsOpt)[1].getVal<int>();
	int i = 0;
	int maxxsize = 0;
	int maxysize = 0;
	int curxsize = 0;
	int curysize = 0;
	int bitppix = 12;

	int trigger_mode = 0x11;
	int integration_time = 10000;
	unsigned int boardparam[5] = {0, 0, 0, 0, 0};
	
    double gain = 0;
    dword serial = 0;

    QFile paramFile = NULL;

	retValue += PCOLoadLibrary();

#if _WIN64
    if(map_buffer_ex == NULL)
    {
        retValue += ito::RetVal(ito::retError, 0, tr("For a 64bit implementation you need the 64bit driver of PCO PixelFly (Version 2.0.1)").toAscii().data());
    }
#endif

    if(!retValue.containsError())
    {
	
	    m_params["board_number"].setVal<int>(iBoardNumber);
        m_identifier = QString("board %1").arg(iBoardNumber);

	    if( ++InitList[iBoardNumber] > 1)	// It does not matter if the rest works or not. The close command will fix this anyway
	    {
		    retValue = ito::RetVal(ito::retError, 0, tr("Board already initialized. Try with another board number").toAscii().data());
	    }

	    Initnum++;  // so we have a new running instance of this grabber (or not)

	    if (!retValue.containsError())
	    {
		    iRetCode = initboard(iBoardNumber, &this->m_hdriver);  
		    if (!this->m_hdriver)
		    {
			    retValue = this->PCOChkError(iRetCode);
			    retValue += ito::RetVal(ito::retError, 0, tr("Unable to PCO-initialize board").toAscii().data());
		    }
		
	    }

	    if (!retValue.containsError())
	    {
    #if PCO_DRIVER_V2 == 1
            if(getboardval)
            {
                DWORD output;
                iRetCode = getboardval(this->m_hdriver,  PCC_VAL_BOARD_INFO, &output);
                serial = PCC_INFO_NR(output);
            }
    #endif
            if(serial == 0)
            {
		        //getboardpar returns len bytes to a array allocated outside the driver 
		        iRetCode = getboardpar(this->m_hdriver, &(boardparam[0]), 20);
		        //serial = PCC_BOARDNR(&(boardparam[0]));
    #if PCO_DRIVER_V2 == 1
                serial = boardparam[0] & 0x0F;
    #else
	            serial = PCC_BOARDNR(&(boardparam[0]));
    #endif
            }
	    }

        // Load parameterlist from XML-file
    
        if(loadParamsFromXML)
        {
            QMap<QString, ito::Param> paramListXML;	
            if (!retValue.containsError())
	        {
                retValue += ito::generateAutoSaveParamFile(this->getBasePlugin()->getFilename(), paramFile);
            }

            // Read parameter list from file to paramListXML
            if (!retValue.containsError())
	        {
                retValue += ito::loadXML2QLIST(&paramListXML, QString::number(iBoardNumber), paramFile);
            }

            // Merge parameter list from file to paramListXML with current mparams
            if (!retValue.containsError())
            {

            }
            paramListXML.clear();
        }

   	    if (!retValue.containsError())
	    {
		    // Camera-exposure is set in µsec, itom uses s
		    integration_time = (int)(m_params["integration_time"].getVal<double>()*1000000);
		    trigger_mode = m_params["trigger_mode"].getVal<int>();
		
		    iRetCode = setmode(this->m_hdriver, trigger_mode, 0, integration_time, m_horizontalBinning, m_verticalBinning, gain, 0, bitppix, 0);
		
		    if (iRetCode != 0)
		    {
			    retValue = this->PCOChkError(iRetCode);
		    }
	    }

	    if (!retValue.containsError())
	    {	
		    iRetCode = getsizes(this->m_hdriver, &maxxsize, &maxysize, &curxsize, &curysize, &bitppix);
		    if (iRetCode != 0)
		    {
			    retValue = this->PCOChkError(iRetCode);
		    }
		    else
		    {
			    m_params["bpp"].setVal<int>(bitppix);
				
				maxxsize = (int)(maxxsize /(m_verticalBinning+1));
			    maxysize = (int)(maxysize /(m_horizontalBinning+1));

			    /*m_params["sizex"].setMax(maxxsize);
			    m_params["sizey"].setMax(maxysize);*/
                static_cast<ito::IntMeta*>( m_params["sizex"].getMeta() )->setMax(maxxsize);
                static_cast<ito::IntMeta*>( m_params["sizey"].getMeta() )->setMax(maxysize);
			    m_params["sizex"].setVal<int>(curxsize);
			    m_params["sizey"].setVal<int>(curysize);
			    /*m_params["x0"].setMax(maxxsize-1);
			    m_params["y0"].setMax(maxysize-1);*/
                static_cast<ito::IntMeta*>( m_params["x0"].getMeta() )->setMax(maxxsize-1);
                static_cast<ito::IntMeta*>( m_params["y0"].getMeta() )->setMax(maxysize-1);
			    m_params["x0"].setVal<int>(0);
			    m_params["y0"].setVal<int>(0);
		    }
	    }

	    if (!retValue.containsError())
	    {
		    retValue += PCOAllocateBuffer();
	    }
	    //if (!retValue.containsError())
	    //{	
	    //	//create LUT for converting 12Bit to 8Bit data
	    //	this->m_pBWlut = create_bwlut(12, 0, 255); 
	    //	//initialize LUT full range,normal
	    //	convert_set(this->m_pBWlut, 0, 4095, 0);
	    //	Sleep(5);
	    //	this->m_isgrabbing = false;
	    //}

        if (!retValue.containsError())
	    {
		    retValue += checkData();
	    }
    
        if (retValue.containsError())
	    {
            if(!Initnum)
            {
                if(g_libcam)
                {
		            FreeLibrary(g_libcam);
                    g_libcam = NULL;
                }
                if(g_libpcocnv)
                {
		            FreeLibrary(g_libpcocnv);
                    g_libpcocnv= NULL;
                }
            }
            else
            {
                std::cerr << "DLLs not unloaded due to further running grabber instances\n" << std::endl;
            }
	    }
        else
        {
            m_saveParamsOnClose = true;
        }
    }

    if (waitCond)
    {
        waitCond->returnValue = retValue;
        waitCond->release();
    }
        
    setInitialized(true); //init method has been finished (independent on retval)
	return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal PCOPixelFly::close(ItomSharedSemaphore *waitCond)
{
	ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);
	
	if (m_timerID > 0)
	{ 
		killTimer(m_timerID);
		m_timerID=0;
	}

	int i = 0;
	int iRetCode = 0;

	if (this->m_hdriver == NULL)
	{
		retValue += ito::RetVal(ito::retWarning, 0, tr("Boardhandle deleted before closing procedure").toAscii().data());
		goto endclose;
	}

	setGrabberStarted(1);
	retValue += this->stopDevice(0);
	Sleep(50);	

	retValue += PCOFreeAllocatedBuffer();

	//delete_bwlut(this->m_pBWlut);

	iRetCode = closeboard(&this->m_hdriver);
	
	if (iRetCode != 0)
	{
		retValue += this->PCOChkError(iRetCode);
	}

endclose:
    int nr = m_params["board_number"].getVal<int>();
    InitList[nr] = 0;
    Initnum--; // so we closed a further instance of this grabber

    if(!Initnum)
	{
        if (g_libcam)
	    {
		    FreeLibrary(g_libcam);
	    }
        g_libcam = NULL;
        if (g_libpcocnv)
	    {
		    FreeLibrary(g_libpcocnv);
	    }
	    g_libpcocnv = NULL;
    }
    else
    {
        retValue += ito::RetVal(ito::retWarning, 0, tr("DLLs not unloaded due to still living instances of PCO-Cams").toAscii().data());
    }

    if (waitCond)
    {
        waitCond->returnValue = retValue;
        waitCond->release();
    }
	
	return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal PCOPixelFly::startDevice(ItomSharedSemaphore *waitCond)
{
	ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);
	
	int num = 0; // In case we wand num back
	int iRetCode = 0;
	unsigned int a[5] = {0, 0, 0, 0, 0};
    DWORD status_v2;
    int result;

	if (num == 0)	// Okay, there is maximal 1 Cam per Board
	{
#if PCO_DRIVER_V2 == 1
            if(getboardval)
            {
                iRetCode = getboardval(this->m_hdriver, PCC_VAL_BOARD_STATUS, &status_v2);
                result = PCC_STATUS_CAM_RUN( status_v2 );
            }
            else
            {
                iRetCode = getboardpar(this->m_hdriver, a, 20);
                result = PCC_STATUS_CAM_RUN(  (*((dword *)a+1)) );
            }
#else
		    iRetCode = getboardpar(this->m_hdriver, a, 20);
            result = PCC_CAM_RUN(a);
#endif
		if (iRetCode != 0)
		{
			retValue += PCOChkError(iRetCode);
		}
		else
		{
			//this makro is out of pccamdef.h, where you can find other usefull makros
            if (!result)
			{
				iRetCode = start_camera(this->m_hdriver);
				if (iRetCode != 0)
				{
					retValue += this->PCOChkError(iRetCode);
				}
				if(grabberStartedCount() > 0)
				{
					retValue += ito::RetVal(ito::retWarning, 0, tr("Camera was not running though running flag was != 0").toAscii().data());
				}
			}
			if (!retValue.containsError())
			{
				incGrabberStarted();
			}
		}
	}
	else
	{
		retValue = ito::retError;
	}
    if (waitCond)
    {
        waitCond->returnValue = retValue;
        waitCond->release();
    }
    return retValue;
}
         
//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal PCOPixelFly::stopDevice(ItomSharedSemaphore *waitCond)
{
	ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);
	int iRetCode = 0;
	unsigned int a[5] = {0, 0, 0, 0, 0};
    DWORD status_v2;

	int num = 0; // In case we want num back
    int result;

    decGrabberStarted();

	if(grabberStartedCount()<1)
	{
		if (num == 0)
		{
#if PCO_DRIVER_V2 == 1
            if(getboardval)
            {
                iRetCode = getboardval(this->m_hdriver, PCC_VAL_BOARD_STATUS, &status_v2);
                result = PCC_STATUS_CAM_RUN( status_v2 );
            }
            else
            {
                iRetCode = getboardpar(this->m_hdriver, a, 20);
                result = PCC_STATUS_CAM_RUN(  (*((dword *)a+1)) );
            }
#else
		    iRetCode = getboardpar(this->m_hdriver, a, 20);
            result = PCC_CAM_RUN(a);
#endif
			if (iRetCode != 0)
			{
				retValue = this->PCOChkError(iRetCode);
			}
			else
			{
				//this makro is out of pccamdef.h, where you can find other usefull makros
                if(result)
				{
					iRetCode = stop_camera(this->m_hdriver);
					if (iRetCode != 0)
					{
						retValue = this->PCOChkError(iRetCode);
					}
				}
				else
				{
					retValue += ito::RetVal(ito::retWarning, 0, tr("Camera was already stopped!!!").toAscii().data());
				}
			}
		}
		else
		{
			retValue = ito::retError;
		}
	}
	if(grabberStartedCount() < 0)
	{
		setGrabberStarted(0);
		retValue += ito::RetVal(ito::retWarning, 0, tr("Cameraflag was < 0").toAscii().data());
	}

    if (waitCond)
    {
        waitCond->returnValue = retValue;
        waitCond->release();
    }
	return retValue;
}
         
//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal PCOPixelFly::acquire(const int trigger, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);
	int iRetCode;
	int triggermode = m_params["trigger_mode"].getVal<int>();

	if (grabberStartedCount() <= 0)
	{
		retValue = ito::RetVal(ito::retError, 0, tr("Tried to acquire without starting device").toAscii().data());
	}
	else
	{
		this->m_isgrabbing = true;
		
		retValue += this->PCOResetEvents();
		retValue += this->PCORemoveFromList();
		retValue += this->PCOAddToList();
		
		if (triggermode&0x01)
		{
			if ((iRetCode = trigger_camera(this->m_hdriver)) != 0)
			{
				retValue += this->PCOChkError(iRetCode);
			}
		}
	
	}


    if (waitCond)
    {
        waitCond->returnValue = retValue;
        waitCond->release();  
    }
	return retValue;
}


//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal PCOPixelFly::retrieveData(ito::DataObject *externalDataObject)
{
    ito::RetVal retValue(ito::retOk);

	int iRetCode = 0;
	int iPicTimeOut = 2;

	long lcopysize = 0;
	long lsrcstrpos = 0;
	int y  = 0;
	int maxxsize = (int)m_params["sizex"].getMax();
	int maxysize = (int)m_params["sizey"].getMax();
	int curxsize = m_params["sizex"].getVal<int>();
	int curysize = m_params["sizey"].getVal<int>();
	int x0 = m_params["x0"].getVal<int>();
	int y0 = m_params["y0"].getVal<int>();

    bool hasListeners = false;
    bool copyExternal = false;

    if(m_autoGrabbingListeners.size() > 0)
    {
        hasListeners = true;
    }

    if(externalDataObject != NULL)
    {
        copyExternal = true;
    }

	if (this->m_isgrabbing == false)
	{
		retValue += ito::RetVal(ito::retWarning, 0, tr("Tried to get picture without triggering exposure").toAscii().data());
	}
	else
	{
		//here we wait until the Event is set to signaled state 
		//or the timeout runs out
		iRetCode = WaitForSingleObject(this->m_event[this->m_nextbuf], iPicTimeOut*1000);
		switch(iRetCode)
		{
			case WAIT_OBJECT_0:
				iRetCode = 0;
			break;

			case WAIT_TIMEOUT:
			case WAIT_FAILED:
			default:
				retValue += ito::RetVal(ito::retError, 0, tr("Failed during waiting for picture or dropped to timeout").toAscii().data());
			break;
		}

		this->m_waited[this->m_nextbuf] = 1;
	
		if (retValue != ito::retError)
		{// Now we shoud have a picture in the camera buffer

			retValue += this->PCORemoveThisFromListIfError(this->m_bufnumber[this->m_nextbuf]);

			switch (m_params["bpp"].getVal<int>())
			{
				case 8:
					//retValue += CopyBuf2Obj<uint8>(dObj, m_params, this->m_pAdr[this->m_nextbuf]);
					if (curxsize == maxxsize)
					{
						lsrcstrpos = y0 * maxxsize;
                        if(copyExternal) retValue += externalDataObject->copyFromData2D<ito::uint8>((ito::uint8*)this->m_pAdr[this->m_nextbuf]+lsrcstrpos, maxxsize, curysize);
						if(!copyExternal || hasListeners) retValue += m_data.copyFromData2D<ito::uint8>((ito::uint8*)this->m_pAdr[this->m_nextbuf]+lsrcstrpos, maxxsize, curysize);
					}
					else
					{
                        if(copyExternal) retValue += externalDataObject->copyFromData2D<ito::uint8>((ito::uint8*)this->m_pAdr[this->m_nextbuf], maxxsize, maxysize, x0, y0, curxsize, curysize);
						if(!copyExternal || hasListeners) retValue += m_data.copyFromData2D<ito::uint8>((ito::uint8*)this->m_pAdr[this->m_nextbuf], maxxsize, maxysize, x0, y0, curxsize, curysize);
					}
					break;
				case 16:
				case 12:
					if (curxsize == maxxsize)
					{
						if(copyExternal) retValue += externalDataObject->copyFromData2D<ito::uint16>((ito::uint16*) (this->m_pAdr[this->m_nextbuf])+lsrcstrpos, maxxsize, curysize);
						if(!copyExternal || hasListeners) retValue += m_data.copyFromData2D<ito::uint16>((ito::uint16*) (this->m_pAdr[this->m_nextbuf])+lsrcstrpos, maxxsize, curysize);
					}
					else
					{
                        if(copyExternal) retValue += externalDataObject->copyFromData2D<ito::uint16>((ito::uint16*) (this->m_pAdr[this->m_nextbuf]), maxxsize, maxysize, x0, y0, curxsize, curysize);
						if(!copyExternal || hasListeners) retValue += m_data.copyFromData2D<ito::uint16>((ito::uint16*) (this->m_pAdr[this->m_nextbuf]), maxxsize, maxysize, x0, y0, curxsize, curysize);
					}
					break;
				default:
					retValue += ito::RetVal(ito::retError, 0, tr("F Wrong picture Type").toAscii().data());
					break;
			}
		
			retValue += this->PCOResetEvents();
			this->m_waited[this->m_nextbuf] = 0;

			this->m_nextbuf++;
			if (this->m_nextbuf >= BUFFERNUMBER)
			{
				this->m_nextbuf = 0;
			}
		}
		this->m_isgrabbing = false;
	}

    return retValue;
}


//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal PCOPixelFly::getVal(void *vpdObj, ItomSharedSemaphore *waitCond)
{
	ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);
    ito::DataObject *dObj = reinterpret_cast<ito::DataObject *>(vpdObj);

	retValue += retrieveData();

	if(!retValue.containsError())
	{
		sendDataToListeners(0); //don't wait for live data, since user should get the data as fast as possible.

		if(dObj)
		{
			(*dObj) = this->m_data;
		}
	}

    if (waitCond) 
    {
        waitCond->returnValue = retValue;
		waitCond->release();
    }

    return retValue;
}
//----------------------------------------------------------------------------------------------------------------------------------
/*!
    \detail This method copies the recently grabbed camera frame to the given DataObject. Therefore this camera size must fit to the data structure of the 
    DataObject.

    \note This method is similar to VideoCapture::retrieve() of openCV

    \param [in,out] vpdObj is the pointer to a given dataObject (this pointer should be cast to ito::DataObject*) where the acquired data is deep copied to.
    \param [in] waitCond is the semaphore (default: NULL), which is released if this method has been terminated
    \return retOk if everything is ok, retError is camera has not been started or no data has been acquired by the method acquire.
    \sa DataObject, acquire
*/
ito::RetVal PCOPixelFly::copyVal(void *vpdObj, ItomSharedSemaphore *waitCond)
{
	ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);
    ito::DataObject *dObj = reinterpret_cast<ito::DataObject *>(vpdObj);

    if(!dObj)
	{
        retValue += ito::RetVal(ito::retError, 0, tr("Empty object handle retrieved from caller").toAscii().data());
    }
    else
    {
        retValue += checkData(dObj);  
    }

    if(!retValue.containsError())
	{
        retValue += retrieveData(dObj);  
    }

    if(!retValue.containsError())
	{
        sendDataToListeners(0); //don't wait for live data, since user should get the data as fast as possible.
    }

    if (waitCond) 
    {
        waitCond->returnValue = retValue;
		waitCond->release();
    }

    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
//! slot invoked if gain parameters in docking toolbox have been manually changed
/*!
    \param [in] gain
    \param [in] offset
*/
void PCOPixelFly::GainPropertiesChanged(double gain)
{
    if(checkNumericParamRange(m_params["gain"], gain))
    {
        setParam( QSharedPointer<ito::ParamBase>(new ito::ParamBase("gain", m_params["gain"].getType(), gain)));
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
//! slot invoked if  offset parameters in docking toolbox have been manually changed
/*!
    \param [in] gain
    \param [in] offset
*/
void PCOPixelFly::OffsetPropertiesChanged(double offset)
{
    setParam( QSharedPointer<ito::ParamBase>(new ito::ParamBase("offset", m_params["offset"].getType(), offset)));
}

//----------------------------------------------------------------------------------------------------------------------------------
//! slot invoked if integrationtime parameters in docking toolbox have been manually changed
/*!
    \param [in] gain
    \param [in] offset
*/
void PCOPixelFly::IntegrationPropertiesChanged(double integrationtime)
{
    if(checkNumericParamRange(m_params["integration_time"], integrationtime))
    {
        setParam( QSharedPointer<ito::ParamBase>(new ito::ParamBase("integration_time", m_params["integration_time"].getType(), integrationtime)));
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
// PIXELFLY GRABBER
