/* ********************************************************************
    Plugin "PcoPixelFly" for itom software
    URL: http://www.uni-stuttgart.de/ito
    Copyright (C) 2018, Institut fuer Technische Optik (ITO),
    Universitaet Stuttgart, Germany

    This file is part of a plugin for the measurement software itom.

    This itom-plugin is free software; you can redistribute it and/or modify it
    under the terms of the GNU Library General Public Licence as published by
    the Free Software Foundation; either version 2 of the Licence, or (at
    your option) any later version.

    itom and its plugins are distributed in the hope that it will be useful, but
    WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU Library
    General Public Licence for more details.

    You should have received a copy of the GNU Library General Public License
    along with itom. If not, see <http://www.gnu.org/licenses/>.
*********************************************************************** */

#define ITOM_IMPORT_API
#define ITOM_IMPORT_PLOTAPI

#include "PCOPixelFly.h"
#include "pluginVersion.h"
#include "gitVersion.h"
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

//int PCOPixelFlyInterface::m_instCounter = 5;  // initialization starts with five due to normal boards are 0..4

static char InitList[5] = {0, 0, 0, 0, 0};  /*!<A map with successfull initialized board (max = 5) */
static char Initnum = 0;    /*!< Number of successfull initialized PCO-Pixelfly-Board */

static     HINSTANCE g_libcam = NULL;    /*!< Handle to the pccam.dll in windows\system32 */
static  HINSTANCE g_libpcocnv = NULL;    /*!< Handle to the pcocnv.dll in .\plugins\PCO */

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal PCOPixelFlyInterface::getAddInInst(ito::AddInBase **addInInst)
{
    NEW_PLUGININSTANCE(PCOPixelFly)
    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal PCOPixelFlyInterface::closeThisInst(ito::AddInBase **addInInst)
{
    REMOVE_PLUGININSTANCE(PCOPixelFly)
    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
PCOPixelFlyInterface::PCOPixelFlyInterface()
{
    m_type = ito::typeDataIO | ito::typeGrabber;
    setObjectName("PCOPixelFly");

    m_description = QObject::tr("PCO Pixelfly cameras");

/*    char docstring[] = \
"This plugin connects the grabber family Pixelfly from PCO to itom. It has mainly been tested with the camera 'pixelfly qe', \
that is connected to the computer by the PCO PCI interface board 540. \n\
\n\
Please install first the necessary drivers for the camera and grabber board from www.pco.de. This plugin supports two families of \
drivers. The driver with major version 1 only supports Windows, 32bit operating systems, while the new driver version 2 also operates \
on 64bit Windows systems.";
    m_detaildescription = QObject::tr(docstring);*/
    m_detaildescription = QObject::tr("This plugin connects the grabber family Pixelfly from PCO to itom. It has mainly been tested with the camera 'pixelfly qe', \
that is connected to the computer by the PCO PCI interface board 540. \n\
\n\
Please install first the necessary drivers for the camera and grabber board from www.pco.de. This plugin supports two families of \
drivers. The driver with major version 1 only supports Windows, 32bit operating systems, while the new driver version 2 also operates \
on 64bit Windows systems.");

    m_author = "W. Lyda, M. Gronle, ITO, University Stuttgart";
    m_version = (PLUGIN_VERSION_MAJOR << 16) + (PLUGIN_VERSION_MINOR << 8) + PLUGIN_VERSION_PATCH;
    m_minItomVer = MINVERSION;
    m_maxItomVer = MAXVERSION;
    m_license = QObject::tr("LGPL / the contained camera SDK belongs to PCO - Computer Optics GmbH");
    m_aboutThis = QObject::tr(GITVERSION);

    ito::Param paramVal = ito::Param("boardNumber", ito::ParamBase::Int, 0, 3, 0, NULL);
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


//----------------------------------------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------------------------------------
const ito::RetVal PCOPixelFly::showConfDialog(void)
{
    return apiShowConfigurationDialog(this, new DialogPCOPixelFly(this));
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
    ito::Param paramVal("name", ito::ParamBase::String | ito::ParamBase::Readonly | ito::ParamBase::NoAutosave, "PCOPixelFly", NULL);
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("integration_time", ito::ParamBase::Double, 0.000010, 0.065535, 0.01, tr("Integrationtime of CCD programmed in s").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param("gain", ito::ParamBase::Double, 0.0, 1.0, 0.0, tr("Standard light mode (0, default), low light mode (1)").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param("x0", ito::ParamBase::Int, 0, 1391, 0, tr("Startvalue for ROI").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("y0", ito::ParamBase::Int, 0, 1023, 0, tr("Stoppvalue for ROI").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("x1", ito::ParamBase::Int, 0, 1391, 1391, tr("Stopvalue for ROI").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("y1", ito::ParamBase::Int, 0, 1023, 1023, tr("Stopvalue for ROI").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);

    int roi[] = {0, 0, 1392, 1024};
    paramVal = ito::Param("roi", ito::ParamBase::IntArray, 4, roi, tr("ROI (x,y,width,height)").toLatin1().data());
    ito::RectMeta *rm = new ito::RectMeta(ito::RangeMeta(0, 1392-1), ito::RangeMeta(0, 1024-1));
    paramVal.setMeta(rm, true);
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param("sizex", ito::ParamBase::Int | ito::ParamBase::Readonly, 1, 1392, 1392, tr("ROI-Size in x").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("sizey", ito::ParamBase::Int | ito::ParamBase::Readonly, 1, 1024, 1024, tr("ROI-Size in y").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param("bpp", ito::ParamBase::Int, 8, 12, 12, tr("bit depth in bits per pixel").toLatin1().data());
    paramVal.setMeta(new ito::IntMeta(8,12,4), true);
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param("binning", ito::ParamBase::Int, 101, 202, 101, tr("Activate 2x2 binning").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("trigger_mode", ito::ParamBase::Int | ito::ParamBase::Readonly, 0x10, 0x41, 0x11, tr("Set Triggermode, currently not implemented").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("shift_bits", ito::ParamBase::Int, 0, 5, 0, tr("In 8 bit, select a number of bits (0..5) that the 12bit acquired values should be shifted before transport with 8 bit precision. (only in 8bit bpp mode)").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("board_number", ito::ParamBase::Int | ito::ParamBase::Readonly | ito::ParamBase::NoAutosave, 0, 4, 0, tr("Number of this board").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("driver_version", ito::ParamBase::String | ito::ParamBase::Readonly | ito::ParamBase::NoAutosave, NULL, tr("driver version").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);

    memset((void*) &(this->m_bufnumber[0]), 0, BUFFERNUMBER*sizeof(int));
    memset((void*) &(this->m_event[0]), 0, BUFFERNUMBER*sizeof(HANDLE));
    memset((void*) &(this->m_waited[0]), 0, BUFFERNUMBER*sizeof(bool));
    memset((void*) &(this->m_pAdr[0]), 0, BUFFERNUMBER*sizeof(void*));

    if (hasGuiSupport())
    {
        //now create dock widget for this plugin
        DockWidgetPCOPixelFly *myDockWidget = new DockWidgetPCOPixelFly(this);
        Qt::DockWidgetAreas areas = Qt::AllDockWidgetAreas;
        QDockWidget::DockWidgetFeatures features = QDockWidget::DockWidgetClosable | QDockWidget::DockWidgetFloatable | QDockWidget::DockWidgetMovable;
        createDockWidget(QString(m_params["name"].getVal<char *>()), features, areas, myDockWidget);
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
PCOPixelFly::~PCOPixelFly()
{
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal PCOPixelFly::PCOLoadLibrary(void)
{
    ito::RetVal retValue = ito::retOk;

    QString version;
    libraryVersionNumber("pccam.dll", version);

    m_params["driver_version"].setVal<char*>(version.toLatin1().data());

    QStringList version2 = version.split(".");
    if (version2.size() > 0)
    {
        bool ok = false;
        int major = version2[0].toInt(&ok);
        if (ok && major > 0)
        {
            m_libraryMajor = major;
        }
    }

    if (!g_libcam && !Initnum)   // so no grabber is initialized yet, dll shoud not be loaded
    {

#if UNICODE
        g_libcam = LoadLibrary(L"pccam.dll");
#else
        g_libcam = LoadLibrary("pccam.dll");
#endif
    }

    if (!g_libcam)
    {
        retValue += ito::RetVal(ito::retError, 0, tr("Could not load library \"pccam.dll\"").toLatin1().data());
        return retValue;
    }

    initboard = (int(*)(int,HANDLE*))
               GetProcAddress(g_libcam,"INITBOARD");
    if (initboard == NULL)
    {
        retValue += ito::RetVal(ito::retError, 0, tr("Cannot get function INITBOARD").toLatin1().data());
    }
    closeboard = (int(*)(HANDLE*))
               GetProcAddress(g_libcam,"CLOSEBOARD");
    if (closeboard == NULL)
    {
        retValue += ito::RetVal(ito::retError, 0, tr("Cannot get function CLOSEBOARD").toLatin1().data());
    }
    getboardpar = (int(*)(HANDLE,unsigned int*,int))
               GetProcAddress(g_libcam,"GETBOARDPAR");
    if (getboardpar == NULL)
    {
        retValue += ito::RetVal(ito::retError, 0, tr("Cannot get function GETBOARDPAR").toLatin1().data());
    }

    getboardval = NULL;
    if (m_libraryMajor >= 2)
    {
        getboardval = (int(*)(HANDLE,int,void*))
               GetProcAddress(g_libcam,"GETBOARDVAL");
        if (getboardpar == NULL)
        {
            retValue += ito::RetVal(ito::retError, 0, tr("Cannot get function GETBOARDVAL").toLatin1().data());
        }
    }

    getsizes = (int(*)(HANDLE,int*,int*,int*,int*,int*))
               GetProcAddress(g_libcam,"GETSIZES");
    if (getsizes == NULL)
    {
        retValue += ito::RetVal(ito::retError, 0, tr("Cannot get function GETSIZES").toLatin1().data());
    }

    setmode = (int(*)(HANDLE,int,int,int,int,int,int,int,int,int))
               GetProcAddress(g_libcam,"SETMODE");
    if (setmode == NULL)
    {
        retValue += ito::RetVal(ito::retError, 0, tr("Cannot get function SETMODE").toLatin1().data());
    }
    start_camera = (int(*)(HANDLE))
               GetProcAddress(g_libcam,"START_CAMERA");
    if (start_camera == NULL)
    {
        retValue += ito::RetVal(ito::retError, 0, tr("Cannot get function START_CAMERA").toLatin1().data());
    }

    stop_camera = (int(*)(HANDLE))
               GetProcAddress(g_libcam,"STOP_CAMERA");
    if (stop_camera == NULL)
    {
        retValue += ito::RetVal(ito::retError, 0, tr("Cannot get function STOP_CAMERA").toLatin1().data());
    }

    trigger_camera = (int(*)(HANDLE))
               GetProcAddress(g_libcam,"TRIGGER_CAMERA");
    if (trigger_camera == NULL)
    {
        retValue += ito::RetVal(ito::retError, 0, tr("Cannot get function TRIGGER_CAMERA").toLatin1().data());
    }

    allocate_buffer = (int(*)(HANDLE,int*,int*))
               GetProcAddress(g_libcam,"ALLOCATE_BUFFER");
    if (allocate_buffer == NULL)
    {
        retValue += ito::RetVal(ito::retError, 0, tr("Cannot get function ALLOCATE_BUFFER").toLatin1().data());
    }

    free_buffer = (int(*)(HANDLE,int))
               GetProcAddress(g_libcam,"FREE_BUFFER");
    if (free_buffer == NULL)
    {
        retValue += ito::RetVal(ito::retError, 0, tr("Cannot get function FREE_BUFFER").toLatin1().data());
    }

    getbuffer_status = (int(*)(HANDLE,int,int,int*,int))
               GetProcAddress(g_libcam,"GETBUFFER_STATUS");
    if (getbuffer_status == NULL)
    {
        retValue += ito::RetVal(ito::retError, 0, tr("Cannot get function GETBUFFER_STATUS").toLatin1().data());
    }

    add_buffer_to_list = (int(*)(HANDLE,int,int,int,int))
               GetProcAddress(g_libcam,"ADD_BUFFER_TO_LIST");
    if (add_buffer_to_list == NULL)
    {
        retValue += ito::RetVal(ito::retError, 0, tr("Cannot get function ADD_BUFFER_TO_LIST").toLatin1().data());
    }

    remove_buffer_from_list = (int(*)(HANDLE,int))
               GetProcAddress(g_libcam,"REMOVE_BUFFER_FROM_LIST");
    if (remove_buffer_from_list == NULL)
    {
        retValue += ito::RetVal(ito::retError, 0, tr("Cannot get function REMOVE_BUFFER_FROM_LIST").toLatin1().data());
    }

    setbuffer_event = (int(*)(HANDLE,int,HANDLE*))
               GetProcAddress(g_libcam,"SETBUFFER_EVENT");
    if (setbuffer_event == NULL)
    {
        retValue += ito::RetVal(ito::retError, 0, tr("Cannot get function SET_BUFFER_EVENT").toLatin1().data());
    }

#if _WIN64
    map_buffer_ex = (int(*)(HANDLE,int,int,int,void**))
               GetProcAddress(g_libcam,"MAP_BUFFER_EX");
    if (map_buffer_ex == NULL)
    {
        retValue += ito::RetVal(ito::retError, 0, tr("Cannot get function MAP_BUFFER_EX").toLatin1().data());
    }
    map_buffer = NULL;
#else
    map_buffer = (int(*)(HANDLE,int,int,int,DWORD*))
               GetProcAddress(g_libcam,"MAP_BUFFER");
    if (map_buffer == NULL)
    {
        retValue += ito::RetVal(ito::retError, 0, tr("Cannot get function MAP_BUFFER").toLatin1().data());
    }
    map_buffer_ex = NULL;
#endif

    unmap_buffer = (int(*)(HANDLE,int))
               GetProcAddress(g_libcam,"UNMAP_BUFFER");
    if (unmap_buffer == NULL)
    {
        retValue += ito::RetVal(ito::retError, 0, tr("Cannot get function UNMAP_BUFFER").toLatin1().data());
    }

    if ((retValue != ito::retOk))
    {
        if (!Initnum)
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

//    if (!g_libpcocnv && !Initnum)   // so no grabber is initialized yet, dll shoud not be loaded
//    {
//#if UNICODE
//        if ((g_libpcocnv = LoadLibrary(L".\\plugins\\PCOPixelFly\\PCO\\pcocnv.dll")) == NULL)
//#else
//        if ((g_libpcocnv = LoadLibrary(".\\plugins\\PCOPixelFly\\PCO\\pcocnv.dll")) == NULL)
//#endif
//        {
//            retValue += ito::RetVal(ito::retError, 0, tr("LoadLibrary(\"pcocnv.dll\")").toLatin1().data());
//        }
//    }
//
//    create_bwlut = (void *(*)(int,int,int))
//               GetProcAddress(g_libpcocnv,"CREATE_BWLUT");
//    if (create_bwlut == NULL)
//    {
//        retValue += ito::RetVal(ito::retError, 0, tr("Cannot get function CREATE_BWLUT").toLatin1().data());
//    }
//    delete_bwlut = (int(*)(void *))
//               GetProcAddress(g_libpcocnv,"DELETE_BWLUT");
//    if (delete_bwlut == NULL)
//    {
//        retValue += ito::RetVal(ito::retError, 0, tr("Cannot get function DELETE_BWLUT").toLatin1().data());
//    }
//
//    convert_set = (int(*)(void *,int,int,int))
//              GetProcAddress(g_libpcocnv,"CONVERT_SET");
//    if (convert_set == NULL)
//    {
//        retValue += ito::RetVal(ito::retError, 0, tr("Cannot get function CONVERT_SET").toLatin1().data());
//    }
//
//    conv_buf_12to8 = (int(*)(int,int,int,unsigned short*,unsigned char*,void*))
//                 GetProcAddress(g_libpcocnv,"CONV_BUF_12TO8");
//    if (conv_buf_12to8 == NULL)
//    {
//        retValue += ito::RetVal(ito::retError, 0, tr("Cannot get function CONV_BUF_12TO8").toLatin1().data());
//    }

    if (retValue != ito::retOk)
    {
        if (Initnum > 0)
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

//---------------------------------------------------------------------------------------------------------------------
ito::RetVal PCOPixelFly::libraryVersionNumber(const QByteArray &fileName, QString &version)
{
    //source: http://stackoverflow.com/questions/940707/how-do-i-programatically-get-the-version-of-a-dll-or-exe-file
    version = "0.0.0.0";
#ifdef linux

    return ito::RetVal(ito::retError,0,tr("the library version can only be fetched on windows systems").toLatin1().data());
#else
    DWORD               dwSize              = 0;
    BYTE                *pbVersionInfo      = NULL;
    VS_FIXEDFILEINFO    *pFileInfo          = NULL;
    UINT                puLenFileInfo       = 0;

    #if UNICODE
        wchar_t *pszFilePath = new wchar_t[ fileName.size() + 2];
        int size = QString(fileName).toWCharArray(pszFilePath);
        pszFilePath[size] = '\0';
    #else
        char *pszFilePath = qstrdup(fileName.data());
    #endif

    // get the version info for the file requested
    dwSize = GetFileVersionInfoSize(pszFilePath, NULL);
    if (dwSize == 0)
    {
        delete[] pszFilePath;
        return ito::RetVal::format(ito::retError,0,"Error in GetFileVersionInfoSize: %d", GetLastError());
    }

    pbVersionInfo = new BYTE[ dwSize ];

    if (!GetFileVersionInfo(pszFilePath, 0, dwSize, pbVersionInfo))
    {
        delete[] pszFilePath;
        delete[] pbVersionInfo;
        return ito::RetVal::format(ito::retError,0,"Error in GetFileVersionInfo: %d", GetLastError());
    }

    delete[] pszFilePath;

    if (!VerQueryValue(pbVersionInfo, TEXT("\\"), (LPVOID*) &pFileInfo, &puLenFileInfo))
    {
        delete[] pbVersionInfo;
        return ito::RetVal::format(ito::retError,0,"Error in VerQueryValue: %d", GetLastError());
    }

    DWORD maj = (pFileInfo->dwFileVersionMS >> 16) & 0xff;
    DWORD min = (pFileInfo->dwFileVersionMS >> 0) & 0xff;
    DWORD build = (pFileInfo->dwFileVersionLS >>  16) & 0xff;
    DWORD revision = (pFileInfo->dwFileVersionLS >>  0) & 0xff;
    version = QString("%1.%2.%3.%4").arg(maj).arg(min).arg(build).arg(revision);
    //// pFileInfo->dwFileVersionMS is usually zero. However, you should check
    //// this if your version numbers seem to be wrong

    //printf("File Version: %d.%d.%d.%d\n",
    //    (pFileInfo->dwFileVersionLS >> 24) & 0xff,
    //    (pFileInfo->dwFileVersionLS >> 16) & 0xff,
    //    (pFileInfo->dwFileVersionLS >>  8) & 0xff,
    //    (pFileInfo->dwFileVersionLS >>  0) & 0xff
    //   );

    //// pFileInfo->dwProductVersionMS is usually zero. However, you should check
    //// this if your version numbers seem to be wrong

    //printf("Product Version: %d.%d.%d.%d\n",
    //    (pFileInfo->dwProductVersionLS >> 24) & 0xff,
    //    (pFileInfo->dwProductVersionLS >> 16) & 0xff,
    //    (pFileInfo->dwProductVersionLS >>  8) & 0xff,
    //    (pFileInfo->dwProductVersionLS >>  0) & 0xff
    //   );

    return ito::retOk;
#endif
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal PCOPixelFly::PCOChkError(int errornumber)
{
    int i;

    if (errornumber == 0)
    {
        return ito::retOk;
    }

    struct error
    {
        int value;
        const char *text;
    }

    errors[] =
    {    /* All Errormessages are taken from the PCO-Manual. */
        {    0,      QObject::tr("no Error").toLatin1().data()},
        {    -1,     QObject::tr("initialization failed, no camera connected").toLatin1().data()},
        {    -2,     QObject::tr("timeout in any function").toLatin1().data()},
        {    -3,     QObject::tr("function call with wrong parameter").toLatin1().data()},
        {    -4,     QObject::tr("cannot locate PCI card or card driver").toLatin1().data()},
        {    -5,     QObject::tr("wrong operating system").toLatin1().data()},
        {    -6,     QObject::tr("no or wrong driver installed").toLatin1().data()},
        {    -7,     QObject::tr("IO function failed").toLatin1().data()},
        {    -8,     QObject::tr("reserved").toLatin1().data()},
        {    -9,     QObject::tr("invalid camera mode").toLatin1().data()},
        {    -10,    QObject::tr("reserved").toLatin1().data()},
        {    -11,    QObject::tr("device is hold by another process").toLatin1().data()},
        {    -12,    QObject::tr("error in reading or writing data to board").toLatin1().data()},
        {    -13,    QObject::tr("wrong driver function").toLatin1().data()},
        {    -14,    QObject::tr("reserved").toLatin1().data()},
        {    -101,   QObject::tr("timeout in any driver function").toLatin1().data()},
        {    -102,   QObject::tr("board is hold by an other process").toLatin1().data()},
        {    -103,   QObject::tr("wrong boardtype").toLatin1().data()},
        {    -104,   QObject::tr("cannot match processhandle to a board").toLatin1().data()},
        {    -105,   QObject::tr("failed to init PCI").toLatin1().data()},
        {    -106,   QObject::tr("no board found").toLatin1().data()},
        {    -107,   QObject::tr("read configuratuion registers failed").toLatin1().data()},
        {    -108,   QObject::tr("board has wrong configuration").toLatin1().data()},
        {    -109,   QObject::tr("memory allocation error").toLatin1().data()},
        {    -110,   QObject::tr("camera is busy").toLatin1().data()},
        {    -111,   QObject::tr("board is not idle").toLatin1().data()},
        {    -112,   QObject::tr("wrong parameter in function cal").toLatin1().data()},
        {    -113,   QObject::tr("head is disconnected").toLatin1().data()},
        {    -114,   QObject::tr("head verification failed").toLatin1().data()},
        {    -115,   QObject::tr("board cannot work with attached head").toLatin1().data()},
        {    -116,   QObject::tr("board initialisation FPGA failed").toLatin1().data()},
        {    -117,   QObject::tr("board initialisation NVRAM failed").toLatin1().data()},
        {    -120,   QObject::tr("not enough IO-buffer space for return values").toLatin1().data()},
        {    -121,   QObject::tr("not enough IO-buffer space for return values").toLatin1().data()},
        {    -122,   QObject::tr("Head power is switched off").toLatin1().data()},
        {    -130,   QObject::tr("picture buffer not prepared for transfer").toLatin1().data()},
        {    -131,   QObject::tr("picture buffer in use").toLatin1().data()},
        {    -132,   QObject::tr("picture buffer hold by another process").toLatin1().data()},
        {    -133,   QObject::tr("picture buffer not found").toLatin1().data()},
        {    -134,   QObject::tr("picture buffer cannot be freed").toLatin1().data()},
        {    -135,   QObject::tr("cannot allocate more picture buffer").toLatin1().data()},
        {    -136,   QObject::tr("no memory left for picture buffer").toLatin1().data()},
        {    -137,   QObject::tr("memory reserve failed").toLatin1().data()},
        {    -138,   QObject::tr("memory commit failed").toLatin1().data()},
        {    -139,   QObject::tr("allocate internal memory LUT failed").toLatin1().data()},
        {    -140,   QObject::tr("allocate internal memory PAGETAB failed").toLatin1().data()},
        {    -148,   QObject::tr("event not available").toLatin1().data()},
        {    -149,   QObject::tr("delete event failed").toLatin1().data()},
        {    -156,   QObject::tr("enable interrupts failed").toLatin1().data()},
        {    -157,   QObject::tr("disable interrupts failed").toLatin1().data()},
        {    -158,   QObject::tr("no interrupt connected to the board").toLatin1().data()},
        {    -164,   QObject::tr("timeout in DMA").toLatin1().data()},
        {    -165,   QObject::tr("no dma buffer found").toLatin1().data()},
        {    -166,   QObject::tr("locking of pages failed").toLatin1().data()},
        {    -167,   QObject::tr("unlocking of pages failed").toLatin1().data()},
        {    -168,   QObject::tr("DMA buffersize to smal").toLatin1().data()},
        {    -169,   QObject::tr("PCI-Bus error in DMA").toLatin1().data()},
        {    -170,   QObject::tr("DMA is runnig, command not allowed").toLatin1().data()},
        {    -228,   QObject::tr("get processor failed").toLatin1().data()},
        {    -229,   QObject::tr("reserved").toLatin1().data()},
        {    -230,   QObject::tr("wrong processor found").toLatin1().data()},
        {    -231,   QObject::tr("wrong processor size").toLatin1().data()},
        {    -232,   QObject::tr("wrong processor device").toLatin1().data()},
        {    -233,   QObject::tr("read flash failed").toLatin1().data()},
        {    -224,   QObject::tr("not grabbing").toLatin1().data()}
    };

    for (i = 0; i < sizeof(errors) / sizeof(errors[0]); i++)
    {
        if (errors[i].value == errornumber)
        {
            return ito::RetVal(ito::retError, errornumber, errors[i].text);
        }
    }

    return ito::RetVal::format(ito::retError, 0, tr("unknown error code of PCO PixelFly (%i)").toLatin1().data(), errornumber);
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
	int curxsize = 0;
	int curysize = 0;
    int lsize = 0;
    int i = 0;
    DWORD linadr = 0;
    DWORD handleflag[100] = {0};
    bool valid = false;

	retValue += PCOChkError(getsizes(m_hdriver, &maxxsize, &maxysize, &curxsize, &curysize, &bitppix));

	if (!retValue.containsError())
	{

		/*maxxsize = m_params["sizex"].getVal<int>();
		maxysize = m_params["sizey"].getVal<int>();
		bitppix = m_params["bpp"].getVal<int>();*/

		for (i = 0; i < BUFFERNUMBER; i++)
		{
			this->m_bufnumber[i] = -1;
			lsize = (int)(curxsize * curysize * ceil(bitppix / 8.0));
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
				map_buffer_ex(this->m_hdriver, this->m_bufnumber[i], lsize, 0, &(m_pAdr[i]));
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
	}

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
            Sleep(50);    // Here is something terribly wrong and I do not not what
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
    ito::RetVal retValue;
    QString key;
    bool hasIndex = false;
    int index;
    QString suffix;
    QMap<QString,ito::Param>::iterator it;

    //parse the given parameter-name (if you support indexed or suffix-based parameters)
    retValue += apiParseParamName(val->getName(), key, hasIndex, index, suffix);

    if (retValue == ito::retOk)
    {
        //gets the parameter key from m_params map (read-only is allowed, since we only want to get the value).
        retValue += apiGetParamFromMapByKey(m_params, key, it, false);
    }

    if (!retValue.containsError())
    {
        if (hasIndex)
        {
            *val = apiGetParam(*it, hasIndex, index, retValue);
        }
        else
        {
            *val = *it;
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
    QString key;
    bool hasIndex;
    int index;
    QString suffix;
    QMap<QString, ito::Param>::iterator it;

    int running = 0;

    //parse the given parameter-name (if you support indexed or suffix-based parameters)
    retValue += apiParseParamName(val->getName(), key, hasIndex, index, suffix);

    if (!retValue.containsError())
    {
        //gets the parameter key from m_params map (read-only is not allowed and leads to ito::retError).
        retValue += apiGetParamFromMapByKey(m_params, key, it, true);
    }

    if (!retValue.containsError())
    {
        //if you program for itom 1.4.0 or higher (Interface version >= 1.3.1) you should use this
        //API method instead of the one above: The difference is, that incoming parameters that are
        //compatible but do not have the same type than the corresponding m_params value are cast
        //to the type of the internal parameter and incoming double values are rounded to the
        //next value (depending on a possible step size, if different than 0.0)
        retValue += apiValidateAndCastParam(*it, *val, false, true, true);
    }

    if (!retValue.containsError())
    {
        if (key == "roi")
        {
            if (hasIndex && (index < 0 || index >= 4))
            {
                retValue += ito::RetVal(ito::retError, 0, "roi[x] only allowed for index in [0,3]");
            }
            else if (val->getLen() != 4)
            {
                retValue += ito::RetVal(ito::retError, 0, "roi must have four values");
            }
        }
    }

    if (!retValue.containsError())
    {
        if (grabberStartedCount() > 0)
        {
            running = grabberStartedCount();
            setGrabberStarted(1);
            retValue += stopDevice(0);
        }

        int bitppix_old = m_params["bpp"].getVal<int>();
        int maxxsize = 0;
        int maxysize = 0;
        int curxsize = 0;
        int curysize = 0;

        retValue += it->copyValueFrom(&(*val));

        Sleep(5);

        int integration_time = (int)(m_params["integration_time"].getVal<double>()*1000000);
        int trigger_mode = m_params["trigger_mode"].getVal<int>();

        int gain = (int)(m_params["gain"].getVal<double>()+0.5); // Toggle Gain On / Off
        m_params["gain"].setVal<int>(gain);

        int bitppix = (int) m_params["bpp"].getVal<int>();
        int shift = (int) m_params["shift_bits"].getVal<int>();

        if (key == "binning" || key == "bpp")
        {
            if (key == "binning")
            {
                int newbinX = val->getVal<int>()/100;
                int newbinY = val->getVal<int>()-newbinX *100;

                int maxbinX = (int)it->getMax()/100;
                int maxbinY = (int)it->getMax()- maxbinX * 100;

                int minbinX = (int)it->getMin()/100;
                int minbinY = (int)it->getMin()- minbinX * 100;

                if (newbinX > maxbinX)
                {
                    retValue += ito::RetVal(ito::retError, 0, tr("New value in X is larger than maximal value, input ignored").toLatin1().data());
                }
                else if (newbinY > maxbinY)
                {
                    retValue += ito::RetVal(ito::retError, 0, tr("New value in Y is larger than maximal value, input ignored").toLatin1().data());
                }
                else if (newbinX < minbinX)
                {
                    retValue += ito::RetVal(ito::retError, 0, tr("New value in X is smaller than parameter range, input ignored").toLatin1().data());
                }
                else if (newbinY < minbinY)
                {
                    retValue += ito::RetVal(ito::retError, 0, tr("New value in Y is smaller than parameter range, input ignored").toLatin1().data());
                }
                else
                {
                    m_horizontalBinning =  newbinX - 1;
                    m_verticalBinning =  newbinY - 1;
                    it.value().setVal<int>(newbinX * 100 + newbinY);
                }
            }

            if ((bitppix != 8) && (bitppix != 12) && (bitppix != 16))
            {
                int iRetCode = getsizes(this->m_hdriver, &maxxsize, &maxysize, &curxsize, &curysize, &bitppix);
                if (iRetCode != 0)
                {
                    retValue = this->PCOChkError(iRetCode);
                    m_params["bpp"].setVal<int>(bitppix_old);
                }
                else
                {
                    m_params["bpp"].setVal<int>(bitppix);
                }

                retValue += ito::RetVal(ito::retError, 0, tr("Tried to set invalid bits per pixel").toLatin1().data());
            }

            if (!retValue.containsError())
            {
                if (bitppix > 8)
                {
                    shift = 0;
                    m_params["shift_bits"].setVal<int>(0);
                }

                retValue += PCOFreeAllocatedBuffer();

                retValue += this->PCOChkError(setmode(this->m_hdriver, trigger_mode, 0, integration_time, m_horizontalBinning, m_verticalBinning, gain, 0, bitppix, shift));
                retValue += this->PCOChkError(getsizes(this->m_hdriver, &maxxsize, &maxysize, &curxsize, &curysize, &bitppix));

                m_params["bpp"].setVal<int>(bitppix);

                maxxsize = (int)(maxxsize /(m_horizontalBinning+1));
                maxysize = (int)(maxysize /(m_verticalBinning+1));

                static_cast<ito::IntMeta*>(m_params["sizex"].getMeta())->setMax(maxxsize);
                static_cast<ito::IntMeta*>(m_params["sizey"].getMeta())->setMax(maxysize);
                m_params["sizex"].setVal(curxsize);
                m_params["sizey"].setVal(curysize);

                m_params["x1"].setVal<int>(maxxsize-1);
                m_params["y1"].setVal<int>(maxysize-1);
                static_cast<ito::IntMeta*>(m_params["x1"].getMeta())->setMax(maxxsize-1);
                static_cast<ito::IntMeta*>(m_params["y1"].getMeta())->setMax(maxysize-1);

                static_cast<ito::IntMeta*>(m_params["x0"].getMeta())->setMax(m_params["x1"].getVal<int>());
                static_cast<ito::IntMeta*>(m_params["y0"].getMeta())->setMax(m_params["y1"].getVal<int>());
                m_params["x0"].setVal<int>(0);
                m_params["y0"].setVal<int>(0);

                static_cast<ito::IntMeta*>(m_params["x1"].getMeta())->setMin(maxxsize);
                static_cast<ito::IntMeta*>(m_params["y1"].getMeta())->setMin(maxysize);

                int roi[] = {0, 0, curxsize, curysize};
                m_params["roi"].setVal<int*>(roi, 4);
                m_params["roi"].setMeta(new ito::RectMeta(ito::RangeMeta(0, maxxsize-1, 1), ito::RangeMeta(0, maxysize-1, 1)), true);

                retValue += PCOAllocateBuffer();
            }
        }
        else if (key == "x0" || key == "x1" || key == "y0" || key == "y1")
        {
            static_cast<ito::IntMeta*>(m_params["x0"].getMeta())->setMax(m_params["x1"].getVal<int>());
            static_cast<ito::IntMeta*>(m_params["y0"].getMeta())->setMax(m_params["y1"].getVal<int>());

            static_cast<ito::IntMeta*>(m_params["x1"].getMeta())->setMin(m_params["x0"].getVal<int>());
            static_cast<ito::IntMeta*>(m_params["y1"].getMeta())->setMin(m_params["x0"].getVal<int>());

            m_params["sizex"].setVal<int>(m_params["x1"].getVal<int>()-m_params["x0"].getVal<int>()+1);
            m_params["sizey"].setVal<int>(m_params["y1"].getVal<int>()-m_params["y0"].getVal<int>()+1);

            int *roi = m_params["roi"].getVal<int*>();
            roi[0] = m_params["x0"].getVal<int>();
            roi[1] = m_params["y0"].getVal<int>();
            roi[2] = m_params["sizex"].getVal<int>();
            roi[3] = m_params["sizey"].getVal<int>();
        }
        else if (key == "roi")
        {
            const int *roi = m_params["roi"].getVal<int*>();

            m_params["x0"].setVal<int>(roi[0]);
            m_params["y0"].setVal<int>(roi[1]);
            m_params["x1"].setVal<int>(roi[0] + roi[2] - 1);
            m_params["y1"].setVal<int>(roi[1] + roi[3] - 1);

            static_cast<ito::IntMeta*>(m_params["x0"].getMeta())->setMax(m_params["x1"].getVal<int>());
            static_cast<ito::IntMeta*>(m_params["y0"].getMeta())->setMax(m_params["y1"].getVal<int>());

            static_cast<ito::IntMeta*>(m_params["x1"].getMeta())->setMin(m_params["x0"].getVal<int>());
            static_cast<ito::IntMeta*>(m_params["y1"].getMeta())->setMin(m_params["x0"].getVal<int>());

            m_params["sizex"].setVal<int>(roi[2]);
            m_params["sizey"].setVal<int>(roi[3]);
        }
        else
        {
            retValue = this->PCOChkError(setmode(this->m_hdriver, trigger_mode, 0, integration_time, m_horizontalBinning, m_verticalBinning, gain, 0, bitppix, shift));
        }
    }

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

    int gain = 0;
    dword serial = 0;

    QFile paramFile;

    retValue += PCOLoadLibrary();

#if _WIN64
    if (map_buffer_ex == NULL)
    {
        retValue += ito::RetVal(ito::retError, 0, tr("For a 64bit implementation you need the 64bit driver of PCO PixelFly (Version 2.0.1)").toLatin1().data());
    }
#endif

    if (!retValue.containsError())
    {

        m_params["board_number"].setVal<int>(iBoardNumber);
        m_identifier = QString("board %1").arg(iBoardNumber);

        if (++InitList[iBoardNumber] > 1)    // It does not matter if the rest works or not. The close command will fix this anyway
        {
            retValue = ito::RetVal(ito::retError, 0, tr("Board already initialized. Try with another board number").toLatin1().data());
        }

        if (!retValue.containsError())
        {
            Initnum++;  // so we have a new running instance of this grabber (or not)

            iRetCode = initboard(iBoardNumber, &this->m_hdriver);
            if (!this->m_hdriver)
            {
                retValue = PCOChkError(iRetCode);
                retValue += ito::RetVal(ito::retError, 0, tr("Unable to PCO-initialize board").toLatin1().data());
            }

        }

        if (!retValue.containsError())
        {
    #if PCO_DRIVER_V2 == 1
            if (getboardval)
            {
                DWORD output;
                iRetCode = getboardval(this->m_hdriver,  PCC_VAL_BOARD_INFO, &output);
                if (iRetCode == 0)
                {
                    serial = PCC_INFO_NR(output);
                    iRetCode = getboardval(m_hdriver, PCC_VAL_BOARD_STATUS, &output);
                    if (iRetCode == 0)
                    {
                        if (output & 0x08000000)
                        {
                            retValue += ito::RetVal(ito::retError, 0, "camera head is disconnected from board. Please check the connection");
                        }
                    }
                    else
                    {
                        retValue += PCOChkError(iRetCode);
                    }
                }
                else
                {
                    retValue += PCOChkError(iRetCode);
                }
            }
    #endif
            if (serial == 0)
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

        if (loadParamsFromXML)
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
            // Camera-exposure is set in mu/sec, itom uses s
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

                static_cast<ito::IntMeta*>(m_params["sizex"].getMeta())->setMax(maxxsize);
                static_cast<ito::IntMeta*>(m_params["sizey"].getMeta())->setMax(maxysize);
                m_params["sizex"].setVal<int>(curxsize);
                m_params["sizey"].setVal<int>(curysize);
                static_cast<ito::IntMeta*>(m_params["x0"].getMeta())->setMax(maxxsize-1);
                static_cast<ito::IntMeta*>(m_params["y0"].getMeta())->setMax(maxysize-1);
                m_params["x0"].setVal<int>(0);
                m_params["y0"].setVal<int>(0);

                int roi[] = {0, 0, maxxsize, maxysize};
                m_params["roi"].setVal<int*>(roi, 4);
                m_params["roi"].setMeta(new ito::RectMeta(ito::RangeMeta(0, maxxsize-1, 1), ito::RangeMeta(0, maxysize-1, 1)), true);
            }
        }

        if (!retValue.containsError())
        {
            retValue += PCOAllocateBuffer();
        }
        //if (!retValue.containsError())
        //{
        //    //create LUT for converting 12Bit to 8Bit data
        //    this->m_pBWlut = create_bwlut(12, 0, 255);
        //    //initialize LUT full range,normal
        //    convert_set(this->m_pBWlut, 0, 4095, 0);
        //    Sleep(5);
        //    this->m_isgrabbing = false;
        //}

        if (!retValue.containsError())
        {
            retValue += checkData();
        }

        if (retValue.containsError())
        {
            if (!Initnum)
            {
                if (g_libcam)
                {
                    FreeLibrary(g_libcam);
                    g_libcam = NULL;
                }
                if (g_libpcocnv)
                {
                    FreeLibrary(g_libpcocnv);
                    g_libpcocnv= NULL;
                }
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
        retValue += ito::RetVal(ito::retWarning, 0, tr("Boardhandle deleted before closing procedure").toLatin1().data());
        goto endclose;
    }

	if (grabberStartedCount() >= 1)
	{
		setGrabberStarted(1);
		retValue += this->stopDevice(0);
		Sleep(50);
	}

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

    if (!Initnum)
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
        retValue += ito::RetVal(ito::retWarning, 0, tr("DLLs not unloaded due to still living instances of PCO-Cams").toLatin1().data());
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

    int num = 0; // In case we want num back
    int iRetCode = -7;
    int iter = 6;
    unsigned int a[5] = {0, 0, 0, 0, 0};
    DWORD status_v2;
    int result;

	if (grabberStartedCount() == 0)
	{
    //check if camera is already started
#if PCO_DRIVER_V2 == 1
		if (getboardval)
		{
			while (--iter > 0 && iRetCode == -7) //-7: IO error, try this command up to 5 times, since a secondary call sometimes works
			{
				iRetCode = getboardval(m_hdriver, PCC_VAL_BOARD_STATUS, &status_v2);
			}
			result = PCC_STATUS_CAM_RUN(status_v2);
		}
		else
		{
			while (--iter > 0 && iRetCode == -7) //-7: IO error, try this command up to 5 times, since a secondary call sometimes works
			{
				iRetCode = getboardpar(m_hdriver, a, 20);
			}
			result = PCC_STATUS_CAM_RUN( (*((dword *)a+1)));
		}
#else
		while (--iter > 0 && iRetCode == -7) //-7: IO error, try this command up to 5 times, since a secondary call sometimes works
		{
			iRetCode = getboardpar(m_hdriver, a, 20);
		}
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
				iter = 6;
				iRetCode = -7;
				while (--iter > 0 && iRetCode == -7) //-7: IO error, try this command up to 5 times, since a secondary call sometimes works then
				{
					iRetCode = start_camera(m_hdriver);
				}
				if (iRetCode != 0)
				{
					retValue += PCOChkError(iRetCode);
				}
			}
		}
	}

    if (!retValue.containsError())
    {
        incGrabberStarted();
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
    int iRetCode = -7;
    unsigned int a[5] = {0, 0, 0, 0, 0};
    DWORD status_v2;
    int iter = 6;

    int result;

    if (grabberStartedCount() == 1)
    {
#if PCO_DRIVER_V2 == 1
        if (getboardval)
        {
            while (--iter > 0 && iRetCode == -7) //-7: IO error, try this command up to 5 times, since a secondary call sometimes works then
            {
                iRetCode = getboardval(m_hdriver, PCC_VAL_BOARD_STATUS, &status_v2);
            }
            result = PCC_STATUS_CAM_RUN(status_v2);
        }
        else
        {
            while (--iter > 0 && iRetCode == -7) //-7: IO error, try this command up to 5 times, since a secondary call sometimes works then
            {
                iRetCode = getboardpar(m_hdriver, a, 20);
            }
            result = PCC_STATUS_CAM_RUN( (*((dword *)a+1)));
        }
#else
        while (--iter > 0 && iRetCode == -7) //-7: IO error, try this command up to 5 times, since a secondary call sometimes works then
        {
            iRetCode = getboardpar(m_hdriver, a, 20);
        }
        result = PCC_CAM_RUN(a);
#endif
        if (iRetCode != 0)
        {
            retValue = this->PCOChkError(iRetCode);
        }
        else
        {
            //this makro is out of pccamdef.h, where you can find other useful makros
            if (result) //camera is running, stop it
            {
                iter = 6;
                iRetCode = -7;
                while (--iter > 0 && iRetCode == -7) //-7: IO error, try this command up to 5 times, since a secondary call sometimes works then
                {
                    iRetCode = stop_camera(m_hdriver);
                }
                if (iRetCode != 0)
                {
                    retValue = PCOChkError(iRetCode);
                }
            }
            else
            {
                retValue += ito::RetVal(ito::retWarning, 0, tr("Camera was already stopped.").toLatin1().data());
            }
        }

		if (!retValue.containsError())
		{
			decGrabberStarted();
		}
    }
    else if (grabberStartedCount() <= 0)
    {
        setGrabberStarted(0);
        retValue += ito::RetVal(ito::retWarning, 0, tr("Camera was already stopped.").toLatin1().data());
    }
	else
	{
		decGrabberStarted();
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
    int iRetCode = -7;
    int triggermode = m_params["trigger_mode"].getVal<int>();

    if (grabberStartedCount() <= 0)
    {
        retValue = ito::RetVal(ito::retError, 0, tr("Tried to acquire without starting device").toLatin1().data());
    }
    else
    {
        this->m_isgrabbing = true;

        retValue += PCOResetEvents();
        retValue += PCORemoveFromList();
        retValue += PCOAddToList();

        if (triggermode&0x01)
        {
            int iter = 6;
            while (--iter > 0 && iRetCode == -7) //-7: IO error, try this command up to 5 times, since a secondary call sometimes works then
            {
                iRetCode = trigger_camera(m_hdriver);
            }
            if (iRetCode != 0)
            {
                retValue += PCOChkError(iRetCode);
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

    if (m_autoGrabbingListeners.size() > 0)
    {
        hasListeners = true;
    }

    if (externalDataObject != NULL)
    {
        copyExternal = true;
    }

    if (this->m_isgrabbing == false)
    {
        retValue += ito::RetVal(ito::retWarning, 0, tr("Tried to get picture without triggering exposure").toLatin1().data());
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
                retValue += ito::RetVal(ito::retError, 0, tr("Failed during waiting for picture or dropped to timeout").toLatin1().data());
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
                        if (copyExternal) retValue += externalDataObject->copyFromData2D<ito::uint8>((ito::uint8*)this->m_pAdr[this->m_nextbuf]+lsrcstrpos, maxxsize, curysize);
                        if (!copyExternal || hasListeners) retValue += m_data.copyFromData2D<ito::uint8>((ito::uint8*)this->m_pAdr[this->m_nextbuf]+lsrcstrpos, maxxsize, curysize);
                    }
                    else
                    {
                        if (copyExternal) retValue += externalDataObject->copyFromData2D<ito::uint8>((ito::uint8*)this->m_pAdr[this->m_nextbuf], maxxsize, maxysize, x0, y0, curxsize, curysize);
                        if (!copyExternal || hasListeners) retValue += m_data.copyFromData2D<ito::uint8>((ito::uint8*)this->m_pAdr[this->m_nextbuf], maxxsize, maxysize, x0, y0, curxsize, curysize);
                    }
                    break;
                case 16:
                case 12:
                    if (curxsize == maxxsize)
                    {
						lsrcstrpos = y0 * maxxsize;
                        if (copyExternal) retValue += externalDataObject->copyFromData2D<ito::uint16>((ito::uint16*) (this->m_pAdr[this->m_nextbuf])+lsrcstrpos, maxxsize, curysize);
                        if (!copyExternal || hasListeners) retValue += m_data.copyFromData2D<ito::uint16>((ito::uint16*) (this->m_pAdr[this->m_nextbuf])+lsrcstrpos, maxxsize, curysize);
                    }
                    else
                    {
                        if (copyExternal) retValue += externalDataObject->copyFromData2D<ito::uint16>((ito::uint16*) (this->m_pAdr[this->m_nextbuf]), maxxsize, maxysize, x0, y0, curxsize, curysize);
                        if (!copyExternal || hasListeners) retValue += m_data.copyFromData2D<ito::uint16>((ito::uint16*) (this->m_pAdr[this->m_nextbuf]), maxxsize, maxysize, x0, y0, curxsize, curysize);
                    }
                    break;
                default:
                    retValue += ito::RetVal(ito::retError, 0, tr("F Wrong picture Type").toLatin1().data());
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

    if (!retValue.containsError())
    {
        sendDataToListeners(0); //don't wait for live data, since user should get the data as fast as possible.

        if (dObj)
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

    if (!dObj)
    {
        retValue += ito::RetVal(ito::retError, 0, tr("Empty object handle retrieved from caller").toLatin1().data());
    }
    else
    {
        retValue += checkData(dObj);
    }

    if (!retValue.containsError())
    {
        retValue += retrieveData(dObj);
    }

    if (!retValue.containsError())
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
void PCOPixelFly::dockWidgetVisibilityChanged(bool visible)
{
    if (getDockWidget())
    {
        QWidget *widget = getDockWidget()->widget();
        if (visible)
        {
            connect(this, SIGNAL(parametersChanged(QMap<QString, ito::Param>)), widget, SLOT(parametersChanged(QMap<QString, ito::Param>)));
            emit parametersChanged(m_params);
        }
        else
        {
            disconnect(this, SIGNAL(parametersChanged(QMap<QString, ito::Param>)), widget, SLOT(parametersChanged(QMap<QString, ito::Param>)));
        }
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
// PIXELFLY GRABBER
