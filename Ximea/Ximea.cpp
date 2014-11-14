/* ********************************************************************
    Plugin "Ximea" for itom software
    URL: http://www.twip-os.com
    Copyright (C) 2013, twip optical solutions GmbH
	Copyright (C) 2013, Institut für Technische Optik, Universität Stuttgart

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

#include "Ximea.h"
#include "XimeaFuncsImport.h"
#include "pluginVersion.h"
#include "dockWidgetXimea.h"
#include "dialogXimea.h"

#include "common/sharedFunctionsQt.h"

#include "common/helperCommon.h"

#if linux
    #include <dlfcn.h>
    #include <unistd.h>
#endif
#include <QFile>
#include <qstring.h>
#include <qstringlist.h>
#include <QtCore/QtPlugin>
#include <qmetaobject.h>

Q_DECLARE_METATYPE(ito::DataObject)

//int XimeaInterface::m_instCounter = 5;  // initialization starts with five due to normal boards are 0..4

static char InitList[5] = {0, 0, 0, 0, 0};  /*!<A map with successfull initialized boards (max = 5) */
static char Initnum = 0;    /*!< Number of successfull initialized cameras */



//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal XimeaInterface::getAddInInst(ito::AddInBase **addInInst)
{
    NEW_PLUGININSTANCE(Ximea)
    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal XimeaInterface::closeThisInst(ito::AddInBase **addInInst)
{
    REMOVE_PLUGININSTANCE(Ximea)
    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
XimeaInterface::XimeaInterface()
{
    m_type = ito::typeDataIO | ito::typeGrabber;
    setObjectName("Ximea");

    m_description = QObject::tr("Ximea xiQ-Camera");
    m_detaildescription = QObject::tr("Developed for Windows only. Tested with Ximea xIQ-cameras for black and white.");
    m_author = "C. Kohler, twip optical solutions GmbH, Stuttgart";
    m_version = (PLUGIN_VERSION_MAJOR << 16) + (PLUGIN_VERSION_MINOR << 8) + PLUGIN_VERSION_PATCH;
    m_minItomVer = MINVERSION;
    m_maxItomVer = MAXVERSION;
    m_license = QObject::tr("LGPL / do not copy Ximea-DLLs");
    m_aboutThis = QObject::tr("N.A.");     
    
    ito::Param paramVal = ito::Param("camera Number", ito::ParamBase::Int | ito::ParamBase::In, 0, 254, 0, "The index of the addressed camera starting with 0");
    m_initParamsOpt.append(paramVal);

    paramVal = ito::Param("restoreLast", ito::ParamBase::Int | ito::ParamBase::In, 0, 1, 0, "Toogle if the driver should try to connect to the last initilized camera");
    m_initParamsOpt.append(paramVal);

    paramVal = ito::Param("bandwidthLimit", ito::ParamBase::Int | ito::ParamBase::In, 0, 100000, 0, "bandwidth limit in Mb/sec. If 0 the maximum bandwidth of the USB3 controller is used [default]. The allowed value range depends on the device and will be checked at startup.");
    m_initParamsOpt.append(paramVal);

    return;
}

//----------------------------------------------------------------------------------------------------------------------------------
XimeaInterface::~XimeaInterface()
{
    m_initParamsMand.clear();
    m_initParamsOpt.clear();
}

//----------------------------------------------------------------------------------------------------------------------------------
Q_EXPORT_PLUGIN2(Ximeainterface, XimeaInterface)

//----------------------------------------------------------------------------------------------------------------------------------
Ximea::Ximea() : 
	AddInGrabber(),  
	m_saveParamsOnClose(false), 
	m_handle(NULL), 
	m_isgrabbing(0), 
	m_acqRetVal(ito::retOk), 
	m_pvShadingSettings(NULL),
    ximeaLib(NULL)
{
    //qRegisterMetaType<ito::DataObject>("ito::DataObject");
    //qRegisterMetaType<QMap<QString, ito::Param> >("QMap<QString, ito::Param>");

    //register exec functions
    QVector<ito::Param> pMand = QVector<ito::Param>()
                               << ito::Param("darkImage", ito::ParamBase::DObjPtr | ito::ParamBase::In, NULL, tr("Dark Image, if null, empty image will be generated").toLatin1().data())
                               << ito::Param("whiteImage", ito::ParamBase::DObjPtr | ito::ParamBase::In, NULL, tr("White Image, if null, empty image will be generated").toLatin1().data())
                               << ito::Param("x0", ito::ParamBase::Int | ito::ParamBase::In, 0, 1280, 0, tr("Position of ROI in x").toLatin1().data())
                               << ito::Param("y0", ito::ParamBase::Int | ito::ParamBase::In, 0, 1024, 0, tr("Position of ROI in y").toLatin1().data());
    QVector<ito::Param> pOpt = QVector<ito::Param>();

    QVector<ito::Param> pOut = QVector<ito::Param>();
    registerExecFunc("initializeShading", pMand, pOpt, pOut, tr("Initialize pixel shading correction. At the moment you can only use one set of data which will be rescaled each time"));

    pMand = QVector<ito::Param>()
            << ito::Param("illumination", ito::ParamBase::Int | ito::ParamBase::In, 0, 9, 0, tr("Current intensity value").toLatin1().data());
    pOpt = QVector<ito::Param>();
    pOut = QVector<ito::Param>();
    registerExecFunc("updateShading", pMand, pOpt, pOut, tr("Change value of the shading correction"));

    pMand = QVector<ito::Param>()
            << ito::Param("integration_time", ito::ParamBase::Double, 0.0, 0.00, 0.000, tr("Integrationtime of CCD programmed in s").toLatin1().data())
            << ito::Param("shadingCorrectionFaktor", ito::ParamBase::DoubleArray | ito::ParamBase::In, NULL, tr("Corresponding values for shading correction").toLatin1().data());
    pOpt = QVector<ito::Param>();
    pOut = QVector<ito::Param>();
    registerExecFunc("shadingCorrectionValues", pMand, pOpt, pOut, tr("Change value of the shading correction"));
    /*
    pMand = QVector<ito::Param>();
    pOpt = QVector<ito::Param>() << ito::Param("darkImage", ito::ParamBase::DObjPtr | ito::ParamBase::In, NULL, tr("Dark Image, if null, empty image will be generated").toLatin1().data())
                                 << ito::Param("whiteImage", ito::ParamBase::DObjPtr | ito::ParamBase::In, NULL, tr("White Image, if null, empty image will be generated").toLatin1().data());
    registerExecFunc("updateShading", pMand, pOpt, pOut, tr("Initialize pixel shading correction"));
    registerExecFunc("calculateShading", pMand, pOpt, pOut, tr("Initialize pixel shading correction"));
    pOpt.clear();
    */
    //end register exec functions

   ito::Param paramVal("name", ito::ParamBase::String | ito::ParamBase::Readonly | ito::ParamBase::NoAutosave, "Ximea", NULL);
   m_params.insert(paramVal.getName(), paramVal);
   paramVal = ito::Param("integration_time", ito::ParamBase::Double, 0.00000, 0.000, 0.0000, tr("Exposure time of chip (in seconds).").toLatin1().data());
   m_params.insert(paramVal.getName(), paramVal);
   paramVal = ito::Param("gain", ito::ParamBase::Double, 0.0, 1.0, 0.0, tr("gain in dB").toLatin1().data());
   m_params.insert(paramVal.getName(), paramVal);
   paramVal = ito::Param("offset", ito::ParamBase::Double, 0.0, 1.0, 0.0, tr("Currently not used.").toLatin1().data());
   m_params.insert(paramVal.getName(), paramVal);

   paramVal = ito::Param("gamma", ito::ParamBase::Double, 0.3, 1.0, 1.0, tr("Luminosity gamma value (0.3 highest correction, 1 no correction).").toLatin1().data());
   m_params.insert(paramVal.getName(), paramVal);
   paramVal = ito::Param("sharpness", ito::ParamBase::Double, -4.0, 4.0, 0.0, tr("Sharpness strenght (-4 less sharp, +4 more sharp).").toLatin1().data());
   m_params.insert(paramVal.getName(), paramVal);

   paramVal = ito::Param("hdr_enable", ito::ParamBase::Int, 0, 1, 1, tr("Enable HDR mode.").toLatin1().data());
   m_params.insert(paramVal.getName(), paramVal);
   paramVal = ito::Param("hdr_knee1", ito::ParamBase::Int, 0, 100, 40, tr("First kneepoint (% of sensor saturation).").toLatin1().data());
   m_params.insert(paramVal.getName(), paramVal);
   paramVal = ito::Param("hdr_knee2", ito::ParamBase::Int, 0, 100, 60, tr("Second kneepoint (% of sensor saturation).").toLatin1().data());
   m_params.insert(paramVal.getName(), paramVal);
   paramVal = ito::Param("hdr_it1", ito::ParamBase::Int, 0, 100, 50, tr("Exposure time of first slope(in % of Exposure time).").toLatin1().data());
   m_params.insert(paramVal.getName(), paramVal);
   paramVal = ito::Param("hdr_it2", ito::ParamBase::Int, 0, 100, 75, tr("Exposure time of second slope(in % of Exposure time).").toLatin1().data());
   m_params.insert(paramVal.getName(), paramVal);


#if defined(ITOM_ADDININTERFACE_VERSION) && ITOM_ADDININTERFACE_VERSION > 0x010300
   int roi[] = {0, 0, 0, 0};
   paramVal = ito::Param("roi", ito::ParamBase::IntArray, 4, roi, tr("ROI (x, y, width, height) [this replaces the values x0, x1, y0, y1].").toLatin1().data());
   ito::RectMeta *rm = new ito::RectMeta(ito::RangeMeta(0, 2048), ito::RangeMeta(0, 2048));
   paramVal.setMeta(rm, true);
   m_params.insert(paramVal.getName(), paramVal);
#endif
   
	paramVal = ito::Param("x0", ito::ParamBase::Int, 0, 2047, 0, tr("Startvalue for ROI.").toLatin1().data());
	m_params.insert(paramVal.getName(), paramVal);
	paramVal = ito::Param("y0", ito::ParamBase::Int, 0, 2047, 0, tr("Startvalue for ROI.").toLatin1().data());
	m_params.insert(paramVal.getName(), paramVal);
	paramVal = ito::Param("x1", ito::ParamBase::Int, 0, 2047, 2047, tr("Stopvalue for ROI.").toLatin1().data());
	m_params.insert(paramVal.getName(), paramVal);
	paramVal = ito::Param("y1", ito::ParamBase::Int, 0, 2047, 2047, tr("Stopvalue for ROI.").toLatin1().data());
	m_params.insert(paramVal.getName(), paramVal);
	paramVal = ito::Param("sizex", ito::ParamBase::Int | ito::ParamBase::Readonly, 1, 2048, 1280, tr("ROI-Size in x (cols).").toLatin1().data());
	m_params.insert(paramVal.getName(), paramVal);
	paramVal = ito::Param("sizey", ito::ParamBase::Int | ito::ParamBase::Readonly, 1, 2048, 1024, tr("ROI-Size in y (rows).").toLatin1().data());
	m_params.insert(paramVal.getName(), paramVal);

	paramVal = ito::Param("bpp", ito::ParamBase::Int, 8, 12, 14, tr("Bit depth of the output data from camera in bpp (can differ from sensor bit depth).").toLatin1().data());
	m_params.insert(paramVal.getName(), paramVal);

	paramVal = ito::Param("binning", ito::ParamBase::Int, 101, 404, 101, tr("1x1 (101), 2x2 (202) or 4x4 (404) binning if available. See param binning type for setting the way binning is executed.").toLatin1().data());
	m_params.insert(paramVal.getName(), paramVal);
	paramVal = ito::Param("binning_type", ito::ParamBase::Int, 0, 1, 0, tr("type of binning if binning is enabled. 0: pixels are interpolated, 1: pixels are skipped (faster).").toLatin1().data());
	m_params.insert(paramVal.getName(), paramVal);
	paramVal = ito::Param("trigger_mode", ito::ParamBase::Int, 0, 4, 0, tr("Set Triggermode, 0: free run, 1: ext. rising edge, 2: ext. falling edge, 3: software.").toLatin1().data());
	m_params.insert(paramVal.getName(), paramVal);
	paramVal = ito::Param("trigger_mode2", ito::ParamBase::Int, 0, 3, 1, tr("Set Triggermode2, 0: Exposure Frame Start, 1: Exposure Frame duration, 2: Frame Burst Start, 3: Frame Burst duration.").toLatin1().data());
	m_params.insert(paramVal.getName(), paramVal);
	paramVal = ito::Param("timing_mode", ito::ParamBase::Int, 0, 1, 1, tr("Acquisition timing: 0: free run, 1: by frame rate.").toLatin1().data());
	m_params.insert(paramVal.getName(), paramVal);
	paramVal = ito::Param("framerate", ito::ParamBase::Double, 0.0, 1000.0, 60.0, tr("Set framerate (in fps). Must be supported by sensor.").toLatin1().data());
	m_params.insert(paramVal.getName(), paramVal);
	paramVal = ito::Param("camNumber", ito::ParamBase::Int | ito::ParamBase::Readonly | ito::ParamBase::NoAutosave, 0, 4, 0, tr("Number / ximea-internal IDX of this camera.").toLatin1().data());
	m_params.insert(paramVal.getName(), paramVal);
	paramVal = ito::Param("badPixel", ito::ParamBase::Int, 0, 1, 1, tr("Enable bad pixel correction.").toLatin1().data());
	m_params.insert(paramVal.getName(), paramVal);
	paramVal = ito::Param("serialNumber", ito::ParamBase::Int |ito::ParamBase::Readonly, 0, 1, 1, tr("Serial Number of device.").toLatin1().data());
	m_params.insert(paramVal.getName(), paramVal);
	paramVal = ito::Param("sensor_type", ito::ParamBase::String |ito::ParamBase::Readonly, "unknown", tr("Sensor type of the attached camera").toLatin1().data());
	m_params.insert(paramVal.getName(), paramVal);
    //now create dock widget for this plugin
    DockWidgetXimea *XI = new DockWidgetXimea(m_params, getID());

    connect(this, SIGNAL(parametersChanged(QMap<QString, ito::Param>)), XI, SLOT(parametersChanged(QMap<QString, ito::Param>)));
    connect(XI, SIGNAL(OffsetPropertiesChanged(double)), this, SLOT(OffsetPropertiesChanged(double)));
    connect(XI, SIGNAL(GainPropertiesChanged(double)), this, SLOT(GainPropertiesChanged(double)));
    connect(XI, SIGNAL(IntegrationPropertiesChanged(double)), this, SLOT(IntegrationPropertiesChanged(double)));

    Qt::DockWidgetAreas areas = Qt::AllDockWidgetAreas;
    QDockWidget::DockWidgetFeatures features = QDockWidget::DockWidgetClosable | QDockWidget::DockWidgetFloatable | QDockWidget::DockWidgetMovable;
    createDockWidget(QString(m_params["name"].getVal<char *>()), features, areas, XI);
}

//----------------------------------------------------------------------------------------------------------------------------------
Ximea::~Ximea()
{
    m_params.clear();
// Already freed in Ximea::close
//    if(!Initnum && ximeaLib)
//    {
//#if linux
//        dlclose(ximeaLib);
//#else
//        FreeLibrary(ximeaLib);
//#endif
//        ximeaLib = NULL;
//    }
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal Ximea::getErrStr(const int error, const QString &command)
{
    switch (error)
    {
        case 0:
            return ito::RetVal(ito::retOk, error, "");
        break;
        //errors from m3Api.h
        case 1:
            return ito::RetVal(ito::retError, error, tr("Ximea (m3api): Invalid handle during sending (during sending '%1')").arg(command).arg(command).toLatin1().data());
        break;
        case 2:
            return ito::RetVal(ito::retError, error, tr("Ximea (m3api): Register read error (during sending '%1')").arg(command).arg(command).toLatin1().data());
        break;
        case 3:
            return ito::RetVal(ito::retError, error, tr("Ximea (m3api): Register write error (during sending '%1')").arg(command).toLatin1().data());
        break;
        case 4:
            return ito::RetVal(ito::retError, error, tr("Ximea (m3api): Freeing resources error (during sending '%1')").arg(command).toLatin1().data());
        break;
        case 5:
            return ito::RetVal(ito::retError, error, tr("Ximea (m3api): Freeing channel error (during sending '%1')").arg(command).toLatin1().data());
        break;
        case 6:
            return ito::RetVal(ito::retError, error, tr("Ximea (m3api): Freeing bandwith error (during sending '%1')").arg(command).toLatin1().data());
        break;
        case 7:
            return ito::RetVal(ito::retError, error, tr("Ximea (m3api): Read block error (during sending '%1')").arg(command).toLatin1().data());
        break;
        case 8:
            return ito::RetVal(ito::retError, error, tr("Ximea (m3api): Write block error (during sending '%1')").arg(command).toLatin1().data());
        break;
        case 9:
            return ito::RetVal(ito::retError, error, tr("Ximea (m3api): No image (during sending '%1')").arg(command).toLatin1().data());
        break;
        case 10:
            return ito::RetVal(ito::retError, error, tr("Ximea (m3api): Timeout (during sending '%1')").arg(command).toLatin1().data());
        break;
        case 11:
            return ito::RetVal(ito::retError, error, tr("Ximea (m3api): Invalid arguments supplied (during sending '%1')").arg(command).toLatin1().data());
        break;
        case 12:
            return ito::RetVal(ito::retError, error, tr("Ximea (m3api): Not supported (during sending '%1')").arg(command).toLatin1().data());
        break;
        case 13:
            return ito::RetVal(ito::retError, error, tr("Ximea (m3api): Attach buffers error (during sending '%1')").arg(command).toLatin1().data());
        break;
        case 14:
            return ito::RetVal(ito::retError, error, tr("Ximea (m3api): Overlapped result (during sending '%1')").arg(command).toLatin1().data());
        break;
        case 15:
            return ito::RetVal(ito::retError, error, tr("Ximea (m3api): Memory allocation error (during sending '%1')").arg(command).toLatin1().data());
        break;
        case 16:
            return ito::RetVal(ito::retError, error, tr("Ximea (m3api): DLL context is NULL (during sending '%1')").arg(command).toLatin1().data());
        break;
        case 17:
            return ito::RetVal(ito::retError, error, tr("Ximea (m3api): DLL context is non zero (during sending '%1')").arg(command).toLatin1().data());
        break;
        case 18:
            return ito::RetVal(ito::retError, error, tr("Ximea (m3api): DLL context exists (during sending '%1')").arg(command).toLatin1().data());
        break;
        case 19:
            return ito::RetVal(ito::retError, error, tr("Ximea (m3api): Too many devices connected (during sending '%1')").arg(command).toLatin1().data());
        break;
        case 20:
            return ito::RetVal(ito::retError, error, tr("Ximea (m3api): Camera context error (during sending '%1')").arg(command).toLatin1().data());
        break;
        case 21:
            return ito::RetVal(ito::retError, error, tr("Ximea (m3api): Unknown hardware (during sending '%1')").arg(command).toLatin1().data());
        break;
        case 22:
            return ito::RetVal(ito::retError, error, tr("Ximea (m3api): Invalid TM file (during sending '%1')").arg(command).toLatin1().data());
        break;
        case 23:
            return ito::RetVal(ito::retError, error, tr("Ximea (m3api): Invalid TM tag (during sending '%1')").arg(command).toLatin1().data());
        break;
        case 24:
            return ito::RetVal(ito::retError, error, tr("Ximea (m3api): Incomplete TM (during sending '%1')").arg(command).toLatin1().data());
        break;
        case 25:
            return ito::RetVal(ito::retError, error, tr("Ximea (m3api): Bus reset error (during sending '%1')").arg(command).toLatin1().data());
        break;
        case 26:
            return ito::RetVal(ito::retError, error, tr("Ximea (m3api): Not implemented (during sending '%1')").arg(command).toLatin1().data());
        break;
        case 27:
            return ito::RetVal(ito::retError, error, tr("Ximea (m3api): Shading too bright (during sending '%1')").arg(command).toLatin1().data());
        break;
        case 28:
            return ito::RetVal(ito::retError, error, tr("Ximea (m3api): Shading too dark (during sending '%1')").arg(command).toLatin1().data());
        break;
        case 29:
            return ito::RetVal(ito::retError, error, tr("Ximea (m3api): Gain is too low (during sending '%1')").arg(command).toLatin1().data());
        break;
        case 30:
            return ito::RetVal(ito::retError, error, tr("Ximea (m3api): Invalid bad pixel list (during sending '%1')").arg(command).toLatin1().data());
        break;
        case 31:
            return ito::RetVal(ito::retError, error, tr("Ximea (m3api): Bad pixel list realloc error (during sending '%1')").arg(command).toLatin1().data());
        break;
        case 32:
            return ito::RetVal(ito::retError, error, tr("Ximea (m3api): Invalid pixel list (during sending '%1')").arg(command).toLatin1().data());
        break;
        case 33:
            return ito::RetVal(ito::retError, error, tr("Ximea (m3api): Invalid Flash File System (during sending '%1')").arg(command).toLatin1().data());
        break;
        case 34:
            return ito::RetVal(ito::retError, error, tr("Ximea (m3api): Invalid profile (during sending '%1')").arg(command).toLatin1().data());
        break;
        case 35:
            return ito::RetVal(ito::retError, error, tr("Ximea (m3api): Invalid bad pixel list (during sending '%1')").arg(command).toLatin1().data());
        break;
        case 36:
            return ito::RetVal(ito::retError, error, tr("Ximea (m3api): Invalid buffer (during sending '%1')").arg(command).toLatin1().data());
        break;
        case 38:
            return ito::RetVal(ito::retError, error, tr("Ximea (m3api): Invalid data (during sending '%1')").arg(command).toLatin1().data());
        break;
        case 39:
            return ito::RetVal(ito::retError, error, tr("Ximea (m3api): Timing generator is busy (during sending '%1')").arg(command).toLatin1().data());
        break;
        case 40:
            return ito::RetVal(ito::retError, error, tr("Ximea (m3api): Wrong operation open/write/read/close (during sending '%1')").arg(command).toLatin1().data());
        break;
        case 41:
            return ito::RetVal(ito::retError, error, tr("Ximea (m3api): Acquisition already started (during sending '%1')").arg(command).toLatin1().data());
        break;
        case 42:
            return ito::RetVal(ito::retError, error, tr("Ximea (m3api): Old version of device driver installed to the system (during sending '%1')").arg(command).toLatin1().data());
        break;
        case 43:
            return ito::RetVal(ito::retError, error, tr("Ximea (m3api): To get error code please call GetLastError function (during sending '%1')").arg(command).toLatin1().data());
        break;
        case 44:
            return ito::RetVal(ito::retError, error, tr("Ximea (m3api): Data can't be processed (during sending '%1')").arg(command).toLatin1().data());
        break;
        case 45:
            return ito::RetVal(ito::retError, error, tr("Ximea (m3api): Error occured and acquisition has been stoped or didn't start (during sending '%1')").arg(command).toLatin1().data());
        break;
        case 46:
            return ito::RetVal(ito::retError, error, tr("Ximea (m3api): Acquisition has been stoped with error (during sending '%1')").arg(command).toLatin1().data());
        break;
        case 47:
            return ito::RetVal(ito::retError, error, tr("Ximea (m3api): Input ICC profile missed or corrupted (during sending '%1')").arg(command).toLatin1().data());
        break;
        case 48:
            return ito::RetVal(ito::retError, error, tr("Ximea (m3api): Output ICC profile missed or corrupted (during sending '%1')").arg(command).toLatin1().data());
        break;
        case 49:
            return ito::RetVal(ito::retError, error, tr("Ximea (m3api): Device not ready to operate (during sending '%1')").arg(command).toLatin1().data());
        break;
        case 50:
            return ito::RetVal(ito::retError, error, tr("Ximea (m3api): Shading too contrast (during sending '%1')").arg(command).toLatin1().data());
        break;
        case 51:
            return ito::RetVal(ito::retError, error, tr("Ximea (m3api): Module already initialized (during sending '%1')").arg(command).toLatin1().data());
        break;
        case 52:
            return ito::RetVal(ito::retError, error, tr("Ximea (m3api): Application doesn't enough privileges(one or more applications opened) (during sending '%1')").arg(command).toLatin1().data());
        break;
        case 53:
            return ito::RetVal(ito::retError, error, tr("Ximea (m3api): installed driver incompatible with current software (during sending '%1')").arg(command).toLatin1().data());
        break;
        case 54:
            return ito::RetVal(ito::retError, error, tr("Ximea (m3api): TM file was not loaded successfully from resources (during sending '%1')").arg(command).toLatin1().data());
        break;
        case 55:
            return ito::RetVal(ito::retError, error, tr("Ximea (m3api): Device has been reseted, abnormal initial state (during sending '%1')").arg(command).toLatin1().data());
        break;
        case 56:
            return ito::RetVal(ito::retError, error, tr("Ximea (m3api): No Devices found (during sending '%1')").arg(command).toLatin1().data());
        break;
        case 57:
            return ito::RetVal(ito::retError, error, tr("Ximea (m3api): Resource (device) or function  locked by mutex (during sending '%1')").arg(command).toLatin1().data());
        break;

        //errors from xiApi.h
        case 100:
            return ito::RetVal(ito::retError, error, tr("Ximea (m3api): unknown parameter (during sending '%1')").arg(command).toLatin1().data());
        break;
        case 101:
            return ito::RetVal(ito::retError, error, tr("Ximea (m3api): wrong parameter value (during sending '%1')").arg(command).toLatin1().data());
        break;
        case 103:
            return ito::RetVal(ito::retError, error, tr("Ximea (m3api): wrong parameter type (during sending '%1')").arg(command).toLatin1().data());
        break;
        case 104:
            return ito::RetVal(ito::retError, error, tr("Ximea (m3api): wrong parameter size (during sending '%1')").arg(command).toLatin1().data());
        break;
        case 105:
            return ito::RetVal(ito::retError, error, tr("Ximea (m3api): input buffer too small (during sending '%1')").arg(command).toLatin1().data());
        break;
        case 106:
            return ito::RetVal(ito::retError, error, tr("Ximea (m3api): parameter info not supported (during sending '%1')").arg(command).toLatin1().data());
        break;
        case 107:
            return ito::RetVal(ito::retError, error, tr("Ximea (m3api): parameter info not supported (during sending '%1')").arg(command).toLatin1().data());
        break;
        case 108:
            return ito::RetVal(ito::retError, error, tr("Ximea (m3api): data format not supported (during sending '%1')").arg(command).toLatin1().data());
        break;
        case 109:
            return ito::RetVal(ito::retError, error, tr("Ximea (m3api): read only parameter (during sending '%1')").arg(command).toLatin1().data());
        break;
        case 110:
            return ito::RetVal(ito::retError, error, tr("Ximea (m3api): no devices found (during sending '%1')").arg(command).toLatin1().data());
        break;
        case 111:
            return ito::RetVal(ito::retError, error, tr("Ximea (m3api): this camera does not support currently available bandwidth (during sending '%1')").arg(command).toLatin1().data());
        break;
        default:
            return ito::RetVal(ito::retError, error, tr("Ximea (m3api): unknown error code (during sending '%1')").arg(command).toLatin1().data());
        break;
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal Ximea::LoadLib(void)
{
    ito::RetVal retValue = ito::retOk;

    if (!ximeaLib)
    {
#if linux
        ximeaLib = dlopen("./lib/libm3api.so", RTLD_LAZY);
        if (!ximeaLib)
        {
            char *error = dlerror();
            return ito::RetVal(ito::retError, 0, error);
        }
#else
#if _WIN64
#if UNICODE
        ximeaLib = LoadLibrary(L"m3apiX64.dll"); //L"./lib/m3apiX64.dll");
#else
        ximeaLib = LoadLibrary("m3apiX64.dll"); //"./lib/m3apiX64.dll");
#endif
        //ximeaLib = LoadLibrary("./plugins/Ximea/m3apiX64.dll");
        if (!ximeaLib)
        {
            return ito::RetVal(ito::retError, 0, tr("LoadLibrary(\"m3apiX64.dll\")").toLatin1().data());
        }
#else
#if UNICODE
        ximeaLib = LoadLibrary(L"m3api.dll");
#else
        ximeaLib = LoadLibrary("m3api.dll");
#endif
        if (!ximeaLib)
        {
            return ito::RetVal(ito::retError, 0, tr("LoadLibrary(\"./plugins/Ximea/m3api.dll\")").toLatin1().data());
        }
#endif
#endif

#if linux
        if ((pxiGetNumberDevices = (XI_RETURN(*)(PDWORD)) dlsym(ximeaLib, "xiGetNumberDevices")) == NULL)
            retValue += ito::RetVal(ito::retError, 0, tr("Cannot get function xiGetNumberDevices").toLatin1().data());

        if ((pxiOpenDevice = (XI_RETURN(*)(DWORD,PHANDLE)) dlsym(ximeaLib, "xiOpenDevice")) == NULL)
            retValue += ito::RetVal(ito::retError, 0, tr("Cannot get function xiOpenDevice").toLatin1().data());

        if ((pxiCloseDevice = (XI_RETURN(*)(HANDLE)) dlsym(ximeaLib, "xiCloseDevice")) == NULL)
            retValue += ito::RetVal(ito::retError, 0, tr("Cannot get function xiCloseDevice").toLatin1().data());

        if ((pxiStartAcquisition = (XI_RETURN(*)(HANDLE)) dlsym(ximeaLib, "xiStartAcquisition")) == NULL)
            retValue += ito::RetVal(ito::retError, 0, tr("Cannot get function xiStartAcquisition").toLatin1().data());

        if ((pxiStopAcquisition = (XI_RETURN(*)(HANDLE)) dlsym(ximeaLib, "xiStopAcquisition")) == NULL)
            retValue += ito::RetVal(ito::retError, 0, tr("Cannot get function xiStopAcquisition").toLatin1().data());

        if ((pxiGetImage = (XI_RETURN(*)(HANDLE,DWORD,LPXI_IMG)) dlsym(ximeaLib, "xiGetImage")) == NULL)
            retValue += ito::RetVal(ito::retError, 0, tr("Cannot get function xiGetImage").toLatin1().data());

        if ((pxiSetParam = (XI_RETURN(*)(HANDLE,const char*,void*,DWORD,XI_PRM_TYPE)) dlsym(ximeaLib, "xiSetParam")) == NULL)
            retValue += ito::RetVal(ito::retError, 0, tr("Cannot get function xiSetParam").toLatin1().data());

        if ((pxiGetParam = (XI_RETURN(*)(HANDLE,const char*,void*,DWORD*,XI_PRM_TYPE*)) dlsym(ximeaLib, "xiGetParam")) == NULL)
            retValue += ito::RetVal(ito::retError, 0, tr("Cannot get function xiGetParam").toLatin1().data());

        if ((pUpdateFrameShading = (MM40_RETURN(*)(HANDLE,HANDLE,LPMMSHADING)) dlsym(ximeaLib, "mmUpdateFrameShading")) == NULL)
            retValue += ito::RetVal(ito::retError, 0, tr("Cannot get function mmUpdateFrameShading").toLatin1().data());

        if ((pCalculateShading = (MM40_RETURN(*)(HANDLE, LPMMSHADING, DWORD, DWORD, LPWORD, LPWORD)) dlsym(ximeaLib, "mmCalculateShading")) == NULL)
            retValue += ito::RetVal(ito::retError, 0, tr("Cannot get function mmCalculateShading").toLatin1().data());

        /*if ((pCalculateShadingRaw = (MM40_RETURN(*)(LPMMSHADING, DWORD, DWORD, LPWORD, LPWORD)) dlsym(ximeaLib, "mmCalculateShadingRaw")) == NULL)
            retValue += ito::RetVal(ito::retError, 0, tr("Cannot get function mmCalculateShading").toLatin1().data());*/
        

        if ((pInitializeShading = (MM40_RETURN(*)(HANDLE, LPMMSHADING,  DWORD , DWORD , WORD , WORD )) dlsym(ximeaLib, "mmInitializeShading")) == NULL)
            retValue += ito::RetVal(ito::retError, 0, tr("Cannot get function mmInitializeShading").toLatin1().data());

        /*if ((pSetShadingRaw = (MM40_RETURN(*)(LPMMSHADING)) dlsym(ximeaLib, "mmSetShadingRaw")) == NULL)
            retValue += ito::RetVal(ito::retError, 0, tr("Cannot get function mmSetShadingRaw").toLatin1().data());*/

        if ((pProcessFrame = (MM40_RETURN(*)(HANDLE)) dlsym(ximeaLib, "mmProcessFrame")) == NULL)
            retValue += ito::RetVal(ito::retError, 0, tr("Cannot get function mmProcessFrame").toLatin1().data());
#else
        if ((pxiGetNumberDevices = (XI_RETURN(*)(PDWORD)) GetProcAddress(ximeaLib, "xiGetNumberDevices")) == NULL)
            retValue += ito::RetVal(ito::retError, 0, tr("Cannot get function xiGetNumberDevices").toLatin1().data());

        if ((pxiOpenDevice = (XI_RETURN(*)(DWORD,PHANDLE)) GetProcAddress(ximeaLib, "xiOpenDevice")) == NULL)
            retValue += ito::RetVal(ito::retError, 0, tr("Cannot get function xiOpenDevice").toLatin1().data());

        if ((pxiCloseDevice = (XI_RETURN(*)(HANDLE)) GetProcAddress(ximeaLib, "xiCloseDevice")) == NULL)
            retValue += ito::RetVal(ito::retError, 0, tr("Cannot get function xiCloseDevice").toLatin1().data());

        if ((pxiStartAcquisition = (XI_RETURN(*)(HANDLE)) GetProcAddress(ximeaLib, "xiStartAcquisition")) == NULL)
            retValue += ito::RetVal(ito::retError, 0, tr("Cannot get function xiStartAcquisition").toLatin1().data());

        if ((pxiStopAcquisition = (XI_RETURN(*)(HANDLE)) GetProcAddress(ximeaLib, "xiStopAcquisition")) == NULL)
            retValue += ito::RetVal(ito::retError, 0, tr("Cannot get function xiStopAcquisition").toLatin1().data());

        if ((pxiGetImage = (XI_RETURN(*)(HANDLE,DWORD,LPXI_IMG)) GetProcAddress(ximeaLib, "xiGetImage")) == NULL)
            retValue += ito::RetVal(ito::retError, 0, tr("Cannot get function xiGetImage").toLatin1().data());

        if ((pxiSetParam = (XI_RETURN(*)(HANDLE,const char*,void*,DWORD,XI_PRM_TYPE)) GetProcAddress(ximeaLib, "xiSetParam")) == NULL)
            retValue += ito::RetVal(ito::retError, 0, tr("Cannot get function xiSetParam").toLatin1().data());

        if ((pxiGetParam = (XI_RETURN(*)(HANDLE,const char*,void*,DWORD*,XI_PRM_TYPE*)) GetProcAddress(ximeaLib, "xiGetParam")) == NULL)
            retValue += ito::RetVal(ito::retError, 0, tr("Cannot get function xiGetParam").toLatin1().data());

        if ((pUpdateFrameShading = (MM40_RETURN(*)(HANDLE,HANDLE,LPMMSHADING)) GetProcAddress(ximeaLib, "mmUpdateFrameShading")) == NULL)
            retValue += ito::RetVal(ito::retError, 0, tr("Cannot get function mmUpdateFrameShading").toLatin1().data());

        if ((pCalculateShading = (MM40_RETURN(*)(HANDLE, LPMMSHADING, DWORD, DWORD, LPWORD, LPWORD)) GetProcAddress(ximeaLib, "mmCalculateShading")) == NULL)
            retValue += ito::RetVal(ito::retError, 0, tr("Cannot get function mmCalculateShading").toLatin1().data());


        /*if ((pCalculateShadingRaw = (MM40_RETURN(*)(LPMMSHADING, DWORD, DWORD, LPWORD, LPWORD)) GetProcAddress(ximeaLib, "mmCalculateShadingRaw")) == NULL)
            retValue += ito::RetVal(ito::retError, 0, tr("Cannot get function mmCalculateShadingRaw").toLatin1().data());*/

        if ((pInitializeShading = (MM40_RETURN(*)(HANDLE, LPMMSHADING,  DWORD , DWORD , WORD , WORD )) GetProcAddress(ximeaLib, "mmInitializeShading")) == NULL)
            retValue += ito::RetVal(ito::retError, 0, tr("Cannot get function mmInitializeShading").toLatin1().data());

        /*if ((pSetShadingRaw = (MM40_RETURN(*)(LPMMSHADING)) GetProcAddress(ximeaLib, "mmSetShadingRaw")) == NULL)
            retValue += ito::RetVal(ito::retError, 0, tr("Cannot get function mmSetShadingRaw").toLatin1().data());*/

        if ((pProcessFrame = (MM40_RETURN(*)(HANDLE)) GetProcAddress(ximeaLib, "mmProcessFrame")) == NULL)
            retValue += ito::RetVal(ito::retError, 0, tr("Cannot get function mmProcessFrame").toLatin1().data());
        
#endif
    }

    if ((retValue != ito::retOk))
    {
        if(!Initnum)
        {
#if linux
            dlclose(ximeaLib);
#else
            FreeLibrary(ximeaLib);
#endif
            ximeaLib = NULL;
        }
        else
        {
            std::cerr << "DLLs not unloaded due to further running grabber instances\n" << std::endl;
        }
        return retValue;
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
ito::RetVal Ximea::getParam(QSharedPointer<ito::Param> val, ItomSharedSemaphore *waitCond)
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

    if(retValue == ito::retOk)
    {
        //gets the parameter key from m_params map (read-only is allowed, since we only want to get the value).
        retValue += apiGetParamFromMapByKey(m_params, key, it, false);
    }

    if(!retValue.containsError())
    {
        *val = it.value();
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
ito::RetVal Ximea::setParam(QSharedPointer<ito::ParamBase> val, ItomSharedSemaphore *waitCond)
{
	ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);
    QString key;
    bool hasIndex;
    int index;
    QString suffix;
    QMap<QString, ito::Param>::iterator it;

	XI_PRM_TYPE intType = xiTypeInteger;
	DWORD intSize = sizeof(int);

    int running = 0;
    
    //int trigger_mode = XI_TRG_OUT;    //in new api trg_out does not exist anymore, so we just use free run
    XI_RETURN ret;

    //parse the given parameter-name (if you support indexed or suffix-based parameters)
    retValue += apiParseParamName( val->getName(), key, hasIndex, index, suffix );

    if(!retValue.containsError())
    {
        //gets the parameter key from m_params map (read-only is not allowed and leads to ito::retError).
        retValue += apiGetParamFromMapByKey(m_params, key, it, true);
    }

    if(!retValue.containsError())
    {
#if defined(ITOM_ADDININTERFACE_VERSION) && ITOM_ADDININTERFACE_VERSION < 0x010300
        //old style api, round the incoming double value to the allowed step size.
        //in a new itom api, this is automatically done by a new api function.
        if (val->getType() == ito::ParamBase::Double || val->getType() == ito::ParamBase::Int)
        {
            double value = val->getVal<double>();
            if (it->getType() == ito::ParamBase::Double)
            {
                ito::DoubleMeta *meta = (ito::DoubleMeta*)it->getMeta();
                if (meta)
                {
                    double step = meta->getStepSize();
                    if (step != 0.0)
                    {
                        int multiple = qRound((value - meta->getMin()) / step);
                        value = meta->getMin() + multiple * step;
                        value = qBound(meta->getMin(), value, meta->getMax());
                        val->setVal<double>(value);
                    }
                }
            }
        }
        retValue += apiValidateParam(*it, *val, false, true);
#else
        retValue += apiValidateAndCastParam(*it, *val, false, true, true);
#endif
    }

    if(!retValue.containsError())
    {
		if (grabberStartedCount())
        {
            running = grabberStartedCount();
            setGrabberStarted(1);
            retValue += this->stopDevice(0);
        }
        else //todo: why this?
        {
            setGrabberStarted(1);
            this->stopDevice(0);
        }

		
        //Sleep(5); //todo: why this?

        if (QString::compare(key, "bpp", Qt::CaseInsensitive) == 0)
        {
			int bitppix = val->getVal<int>();
			int bpp = -1;
			retValue += getErrStr(pxiSetParam(m_handle, XI_PRM_IMAGE_DATA_FORMAT, &bpp, sizeof(int), xiTypeInteger), "XI_PRM_IMAGE_DATA_FORMAT");	
			if (bitppix == 8)
			{
				int bpp = XI_MONO8;
                retValue += getErrStr(pxiSetParam(m_handle, XI_PRM_IMAGE_DATA_FORMAT, &bpp, sizeof(int), xiTypeInteger), "XI_PRM_IMAGE_DATA_FORMAT");						
				if (ret = pxiGetParam(m_handle, XI_PRM_OUTPUT_DATA_BIT_DEPTH, &bitppix, &intSize, &intType))
				{
					retValue += getErrStr(ret, "XI_PRM_OUTPUT_DATA_BIT_DEPTH");
				}
				else
				{
					val->setVal<int>(bitppix);
				}
		
			}
			else if (bitppix == 10)
			{
                int bpp = XI_MONO16;
                retValue += getErrStr(pxiSetParam(m_handle, XI_PRM_IMAGE_DATA_FORMAT, &bpp, sizeof(int), xiTypeInteger), "XI_PRM_IMAGE_DATA_FORMAT");
				if (ret = pxiGetParam(m_handle, XI_PRM_OUTPUT_DATA_BIT_DEPTH, &bitppix, &intSize, &intType))
				{
					retValue += getErrStr(ret, "XI_PRM_OUTPUT_DATA_BIT_DEPTH");
				}
				else
				{
					val->setVal<int>(bitppix);
				}
			}
			else if (bitppix == 12)
			{
				int bpp = XI_MONO16;
                retValue += getErrStr(pxiSetParam(m_handle, XI_PRM_IMAGE_DATA_FORMAT, &bpp, sizeof(int), xiTypeInteger), "XI_PRM_IMAGE_DATA_FORMAT");
				if (ret = pxiGetParam(m_handle, XI_PRM_OUTPUT_DATA_BIT_DEPTH, &bitppix, &intSize, &intType))
				{
					retValue += getErrStr(ret, "");
				}
				else
				{
					val->setVal<int>(bitppix);
				}
			}
			else if (bitppix == 14)
			{
				int bpp = XI_MONO16;
				retValue += getErrStr(pxiSetParam(m_handle, XI_PRM_IMAGE_DATA_FORMAT, &bpp, sizeof(int), xiTypeInteger), "XI_PRM_IMAGE_DATA_FORMAT");
				if (ret = pxiGetParam(m_handle, XI_PRM_OUTPUT_DATA_BIT_DEPTH, &(*val), &intSize, &intType))
				{
					retValue += getErrStr(ret, "XI_PRM_OUTPUT_DATA_BIT_DEPTH");
				}
				else
				{
					val->setVal<int>(bitppix);
				}
			}
			else
			{
				retValue = ito::RetVal(ito::retError, 0, tr("bpp value not supported").toLatin1().data());
			}

			if (!retValue.containsError())
			{
				it->copyValueFrom(&(*val)); //copy value from user to m_params, represented by iterator it
				retValue += synchronizeCameraSettings(sExposure | sRoi);
			}
			
        }
        else if (QString::compare(key, "binning", Qt::CaseInsensitive) == 0)
        {
			int bin = val->getVal<int>(); //requested binning value from user
            int curxsize;
            int curysize;
            if(bin != 101 && bin != 202 && bin != 404)
            {
                retValue = ito::RetVal(ito::retError, 0, tr("Binning value must be 101 (1x1), 202 (2x2) or 404 (4x4) (depending on supported binning values)").toLatin1().data());
            }
			else
			{
				bin = (bin % 100); //101 -> 1, 202 -> 2, 404 -> 4
				retValue += getErrStr(pxiSetParam(m_handle, XI_PRM_DOWNSAMPLING, &bin, sizeof(int), xiTypeInteger), "XI_PRM_DOWNSAMPLING");

				if (!retValue.containsError())
				{
					it->copyValueFrom(&(*val)); //copy value from user to m_params, represented by iterator it

					int maxxsize, maxysize;

					retValue += getErrStr(pxiGetParam(m_handle, XI_PRM_WIDTH XI_PRM_INFO_MAX, &maxxsize, &intSize, &intType), "XI_PRM_WIDTH XI_PRM_INFO_MAX");
					retValue += getErrStr(pxiGetParam(m_handle, XI_PRM_HEIGHT XI_PRM_INFO_MAX, &maxysize, &intSize, &intType), "XI_PRM_HEIGHT XI_PRM_INFO_MAX");
					retValue += getErrStr(pxiGetParam(m_handle, XI_PRM_WIDTH, &curxsize, &intSize, &intType), "XI_PRM_WIDTH");
					retValue += getErrStr(pxiGetParam(m_handle, XI_PRM_HEIGHT, &curysize, &intSize, &intType), "XI_PRM_HEIGHT");

					maxxsize = (int)(maxxsize / (bin));
					maxysize = (int)(maxysize / (bin));

					static_cast<ito::IntMeta*>( m_params["sizex"].getMeta() )->setMax(maxxsize);
					static_cast<ito::IntMeta*>( m_params["sizey"].getMeta() )->setMax(maxysize);
					m_params["sizex"].setVal(curxsize);
					m_params["sizey"].setVal(curysize);

					m_params["x0"].setMeta( new ito::IntMeta(0,maxxsize-1), true );
					m_params["y0"].setMeta( new ito::IntMeta(0,maxysize-1), true );
					m_params["x0"].setVal<int>(0);
					m_params["y0"].setVal<int>(0);

					m_params["x1"].setMeta( new ito::IntMeta(1,maxxsize-1), true );
					m_params["y1"].setMeta( new ito::IntMeta(1,maxysize-1), true );
					m_params["x1"].setVal<int>(curxsize-1);
					m_params["y1"].setVal<int>(curysize-1);
				}
			}

			if (!retValue.containsError())
			{
				retValue += synchronizeCameraSettings(sBinning | sRoi | sExposure);
			}
        }
		else if (QString::compare(key, "binning_type", Qt::CaseInsensitive) == 0)
		{
			int type = val->getVal<int>();
			retValue += getErrStr(pxiSetParam(m_handle, XI_PRM_DOWNSAMPLING_TYPE, &type, sizeof(int), intType), "XI_PRM_DOWNSAMPLING_TYPE");
		}
        else if (QString::compare(key, "badPixel", Qt::CaseInsensitive) == 0 )
        {
            int enable = val->getVal<int>() > 0 ? 1 : 0;
            int maxVal = 0;
            int curVal = 0;

            retValue += getErrStr(pxiSetParam(m_handle, XI_PRM_BPC, &enable, sizeof(int), intType), "XI_PRM_BPC");
            if (ret = pxiGetParam(m_handle, XI_PRM_BPC, &curVal, &intSize, &intType))
            {
                retValue += getErrStr(ret, "XI_PRM_BPC");
            }
            else
            {
                m_params["badPixel"].setVal(curVal);
            }
        }
        else if (QString::compare(key, "hdr_enable", Qt::CaseInsensitive) == 0)
        {
            int enable = val->getVal<int>() > 0 ? 1 : 0;
            int intTime1 = (int)m_params["hdr_it1"].getVal<int>();
            int intTime2 = (int)m_params["hdr_it2"].getVal<int>();
			int knee1 = (int)m_params["hdr_knee1"].getVal<int>();
            int knee2 = (int)m_params["hdr_knee2"].getVal<int>();
#ifdef USE_OLD_API
            if(enable)
            {
                integration_time += integration_time / 4;
                //if ((ret = pxiSetParam(m_handle, XI_PRM_HDR , &enable, sizeof(int), intType)))
                retValue += getErrStr(pxiSetParam(m_handle, "hdr" , &enable, sizeof(int), intType), "XI_PRM_INFO");
                retValue += getErrStr(pxiGetParam(m_handle, "hdr" XI_PRM_INFO , &enable, &intSize, &intType), "XI_PRM_INFO"); 

                //if ((ret = pxiSetParam(m_handle, XI_PRM_HDR_RATIO , &knee1, sizeof(int), intType)))
                //if ((ret = pxiSetParam(m_handle, "hdr_ratio" , &knee1, sizeof(int), intType)))
                //{
                //    retValue += getErrStr(ret, "XI_PRM_HDR_RATIO");
                //}
                retValue += getErrStr(pxiSetParam(m_handle, XI_PRM_EXPOSURE, &integration_time, sizeof(int), intType), "XI_PRM_EXPOSURE");
            }
            else
            {
                //if ((ret = pxiSetParam(m_handle, XI_PRM_HDR , &enable, sizeof(int), intType)))
                retValue += getErrStr(pxiSetParam(m_handle, "hdr" , &enable, sizeof(int), intType), "XI_PRM_HDR");
                retValue += getErrStr(pxiSetParam(m_handle, XI_PRM_EXPOSURE, &integration_time, sizeof(int), intType), "XI_PRM_EXPOSURE");
            }

#else  
            if(enable)
            {
                retValue += getErrStr(pxiSetParam(m_handle, XI_PRM_KNEEPOINT1 , &knee1, sizeof(int), intType), "XI_PRM_KNEEPOINT1");
                retValue += getErrStr(pxiSetParam(m_handle, XI_PRM_KNEEPOINT2 , &knee2, sizeof(int), intType), "XI_PRM_KNEEPOINT2");
                retValue += getErrStr(pxiSetParam(m_handle, XI_PRM_HDR_T1 , &intTime1, sizeof(int), intType), "XI_PRM_HDR_T1");
                retValue += getErrStr(pxiSetParam(m_handle, XI_PRM_HDR_T2, &intTime2, sizeof(int), intType), "XI_PRM_HDR_T2");

            }
            retValue += getErrStr(pxiSetParam(m_handle, XI_PRM_HDR, &enable, sizeof(int), intType), "XI_PRM_HDR");

			if (!retValue.containsError())
			{
				it->copyValueFrom(&(*val)); //copy value from user to m_params, represented by iterator it
			}
#endif
        }
        else if (QString::compare(key, "integration_time", Qt::CaseInsensitive) == 0)
        {

			int integration_time = secToMusec(val->getVal<double>());
            retValue += getErrStr(pxiSetParam(m_handle, XI_PRM_EXPOSURE, &integration_time, sizeof(int), intType), "XI_PRM_EXPOSURE");				
			if ((ret = pxiGetParam(m_handle, XI_PRM_EXPOSURE, &integration_time, &intSize, &intType)))
				retValue += getErrStr(ret, "XI_PRM_EXPOSURE");
			else
				m_params["integration_time"].setVal<double>(musecToSec(integration_time));

			if (!retValue.containsError())
			{
				retValue += synchronizeCameraSettings(sExposure | sFrameRate | sGain);
			}

        }
        else if (QString::compare(key, "sharpness", Qt::CaseInsensitive) == 0)
        {
			float sharpness = (float)val->getVal<double>();
            retValue += getErrStr(pxiSetParam(m_handle, XI_PRM_SHARPNESS, &sharpness, sizeof(float), xiTypeFloat), "XI_PRM_SHARPNESS");
			if (!retValue.containsError())
			{
				it->copyValueFrom(&(*val)); //copy value from user to m_params, represented by iterator it
				retValue += synchronizeCameraSettings(sSharpness);
			}
        }
        else if (QString::compare(key, "gamma", Qt::CaseInsensitive) == 0)
        {
			float gamma = (float)val->getVal<double>();
            retValue += getErrStr(pxiSetParam(m_handle, XI_PRM_GAMMAY, &gamma, sizeof(float), xiTypeFloat), "XI_PRM_GAMMAY");
			if (!retValue.containsError())
			{
				it->copyValueFrom(&(*val)); //copy value from user to m_params, represented by iterator it
				retValue += synchronizeCameraSettings(sGamma);
			}
        }    
        else if (QString::compare(key, "trigger_mode", Qt::CaseInsensitive) == 0)
        {
			int trigger_mode = val->getVal<int>();
            retValue += getErrStr(pxiSetParam(m_handle, XI_PRM_TRG_SOURCE, &trigger_mode, sizeof(int), intType), "XI_PRM_TRG_SOURCE");
			if (!retValue.containsError())
			{
				it->copyValueFrom(&(*val)); //copy value from user to m_params, represented by iterator it
				retValue += synchronizeCameraSettings(sTriggerMode | sTriggerMode2 | sFrameRate | sExposure);
			}
        }
#ifndef USE_OLD_API
        else if (QString::compare(key, "trigger_mode2", Qt::CaseInsensitive) == 0)
        {
			int trigger_mode2 = val->getVal<int>();
            retValue += getErrStr(pxiSetParam(m_handle, XI_PRM_TRG_SELECTOR, &trigger_mode2, sizeof(int), intType), "XI_PRM_TRG_SELECTOR");
			if (!retValue.containsError())
			{
				it->copyValueFrom(&(*val)); //copy value from user to m_params, represented by iterator it
				retValue += synchronizeCameraSettings(sTriggerMode | sTriggerMode2 | sFrameRate | sExposure);
			}
        }

        else if (QString::compare(key, "timing_mode", Qt::CaseInsensitive) == 0)
        {
#ifndef USE_OLD_API
			int timing_mode = val->getVal<int>();
            retValue += getErrStr(pxiSetParam(m_handle, XI_PRM_ACQ_TIMING_MODE, &timing_mode, sizeof(int), intType), "XI_PRM_ACQ_TIMING_MODE");
			if (!retValue.containsError())
			{
				it->copyValueFrom(&(*val)); //copy value from user to m_params, represented by iterator it
			}
#else
			retValue += ito::retVal(ito::retError, 0, "timing_mode can only be changed with old Ximea API");
#endif
        }
#endif
        else if (QString::compare(key, "framerate", Qt::CaseInsensitive) == 0)
        {
			float frameRate = val->getVal<double>();
            retValue += getErrStr(pxiSetParam(m_handle, XI_PRM_FRAMERATE, &frameRate, sizeof(float), xiTypeFloat), "XI_PRM_FRAMERATE");
			if (!retValue.containsError())
			{
				it->copyValueFrom(&(*val)); //copy value from user to m_params, represented by iterator it
				retValue += synchronizeCameraSettings(sExposure | sFrameRate | sExposure);
			}
        }
        else if (QString::compare(key, "gain", Qt::CaseInsensitive) == 0)
        {

			float gain = val->getVal<double>();
            retValue += getErrStr(pxiSetParam(m_handle, XI_PRM_GAIN, &gain, sizeof(float), xiTypeFloat), "XI_PRM_GAIN");
			if (!retValue.containsError())
			{
				it->copyValueFrom(&(*val)); //copy value from user to m_params, represented by iterator it
				retValue += synchronizeCameraSettings(sGain | sFrameRate | sExposure);
			}
        }
        else if (QString::compare(key, "x1", Qt::CaseInsensitive) == 0)
        {
            int size = val->getVal<int>() - m_params["x0"].getVal<int>() + 1;
            if(size % 2 != 0)
            {
                retValue += ito::RetVal(ito::retWarning, 0, tr("Size must be multiple of 2").toLatin1().data());
            }
			else if ((ret = pxiSetParam(m_handle, XI_PRM_WIDTH, &size, sizeof(int), intType)))
            {
                retValue += getErrStr(ret, "XI_PRM_WIDTH");
            }

			if (!retValue.containsError())
			{
				it->copyValueFrom(&(*val)); //copy value from user to m_params, represented by iterator it
				ito::IntMeta* meta = static_cast<ito::IntMeta*>(m_params["x0"].getMeta());
				m_params["sizex"].setVal<int>(size);
				//get max value of offset again in order to consider possible step sizes... depending on the current size
				retValue += getErrStr(pxiGetParam(m_handle, XI_PRM_OFFSET_X XI_PRM_INFO_INCREMENT, &size, &intSize, &intType), "XI_PRM_OFFSET_X XI_PRM_INFO_INCREMENT");
				meta->setMax(it->getVal<int>() - size + 1);
				retValue += synchronizeCameraSettings(sExposure | sRoi | sFrameRate);
			}
        }
        else if (QString::compare(key, "y1", Qt::CaseInsensitive) == 0)
        {
			int size = val->getVal<int>() - m_params["y0"].getVal<int>() + 1;

            if(size % 2 != 0)
            {
                retValue += ito::RetVal(ito::retWarning, 0, tr("Size must be multiple of 2").toLatin1().data());
            }
			else if ((ret = pxiSetParam(m_handle, XI_PRM_HEIGHT, &size, sizeof(int), intType)))
            {
                retValue += getErrStr(ret, "XI_PRM_HEIGHT");
            }

			if (!retValue.containsError())
			{
				it->copyValueFrom(&(*val)); //copy value from user to m_params, represented by iterator it
				ito::IntMeta* meta = static_cast<ito::IntMeta*>(m_params["y0"].getMeta());
				m_params["sizey"].setVal<int>(size);
				//get max value of offset again in order to consider possible step sizes... depending on the current size
				retValue += getErrStr(pxiGetParam(m_handle, XI_PRM_OFFSET_Y XI_PRM_INFO_INCREMENT, &size, &intSize, &intType), "XI_PRM_OFFSET_Y XI_PRM_INFO_INCREMENT");
				meta->setMax(it->getVal<int>() - size + 1);
				retValue += synchronizeCameraSettings(sExposure | sRoi | sFrameRate);
			}
        }
        else if (QString::compare(key, "x0", Qt::CaseInsensitive) == 0)
        {
			int x0old = m_params["x0"].getVal<int>();
            int offset = val->getVal<int>();
            int maxsize = m_params["sizex"].getMax();
            int size = m_params["x1"].getVal<int>() - offset + 1;

            if (size % 2 != 0)
            {
                retValue += ito::RetVal(ito::retWarning, 0, tr("Size must be multiple of 2").toLatin1().data());
            }
			else if (x0old < offset)
            {
                //m_params["sizex"].setVal<int>(size);
                retValue += getErrStr(pxiSetParam(m_handle, XI_PRM_WIDTH, &size, sizeof(int), intType), "XI_PRM_WIDTH");
                if (!retValue.containsError())
				{
					retValue += getErrStr(pxiSetParam(m_handle, XI_PRM_OFFSET_X, &offset, sizeof(int), intType), "XI_PRM_OFFSET_X");
				}
            }
            else
            {
                retValue += getErrStr(pxiSetParam(m_handle, XI_PRM_OFFSET_X, &offset, sizeof(int), intType), "XI_PRM_OFFSET_X");
				if (!retValue.containsError())
				{
					retValue += getErrStr(pxiSetParam(m_handle, XI_PRM_WIDTH, &size, sizeof(int), intType), "XI_PRM_WIDTH");
				}
            }

			if (!retValue.containsError())
			{
				it->copyValueFrom(&(*val)); //copy value from user to m_params, represented by iterator it
				ito::IntMeta* meta = static_cast<ito::IntMeta*>(m_params["x1"].getMeta());
				m_params["sizex"].setVal<int>(size);
				//get min value of height again in order to consider possible step sizes... depending on the current size
				retValue += getErrStr(pxiGetParam(m_handle, XI_PRM_WIDTH XI_PRM_INFO_MIN, &size, &intSize, &intType), "XI_PRM_WIDTH XI_PRM_INFO_MIN");
				meta->setMin(val->getVal<int>() + size - 1);
				retValue += synchronizeCameraSettings(sExposure | sRoi | sFrameRate);
			}
        }
        else if (QString::compare(key, "y0", Qt::CaseInsensitive) == 0)
        {
			int y0old = m_params["y0"].getVal<int>();
            int offset = val->getVal<int>();
            int maxsize = m_params["sizey"].getMax();
            int size = m_params["y1"].getVal<int>() - offset + 1;

            if (size % 2 != 0)
            {
                retValue += ito::RetVal(ito::retWarning, 0, tr("Size must be multiple of 2").toLatin1().data());
            }
			else if (y0old < offset)
            {
                retValue += getErrStr(pxiSetParam(m_handle, XI_PRM_HEIGHT, &size, sizeof(int), intType), "XI_PRM_HEIGHT");
				if (!retValue.containsError())
				{
					retValue += getErrStr(pxiSetParam(m_handle, XI_PRM_OFFSET_Y, &offset, sizeof(int), intType), "XI_PRM_OFFSET_Y");
				}
            }
            else
            {
                retValue += getErrStr(pxiSetParam(m_handle, XI_PRM_OFFSET_Y, &offset, sizeof(int), intType), "XI_PRM_OFFSET_Y");
				if (!retValue.containsError())
				{
					retValue += getErrStr(pxiSetParam(m_handle, XI_PRM_HEIGHT, &size, sizeof(int), intType), "XI_PRM_HEIGHT");
				}
            }

			if (!retValue.containsError())
			{
				it->copyValueFrom(&(*val)); //copy value from user to m_params, represented by iterator it
				ito::IntMeta* meta = static_cast<ito::IntMeta*>(m_params["y1"].getMeta());
				m_params["sizey"].setVal<int>(size);
				//get min value of height again in order to consider possible step sizes... depending on the current size
				retValue += getErrStr(pxiGetParam(m_handle, XI_PRM_HEIGHT XI_PRM_INFO_MIN, &size, &intSize, &intType), "XI_PRM_HEIGHT XI_PRM_INFO_MIN");
				meta->setMin(val->getVal<int>() + size - 1);
				retValue += synchronizeCameraSettings(sExposure | sRoi | sFrameRate);
			}
        }
		else if(QString::compare(key, "roi", Qt::CaseInsensitive) == 0)
		{
			if (!hasIndex)
			{
				if (val->getLen() !=4)
				{
					retValue += ito::RetVal(ito::retError, 0, "roi must have 4 values");
				}
				else
				{
					int *roi_set = val->getVal<int*>();
					int offset_x = roi_set[0];
					int offset_y = roi_set[1];
					int width = roi_set[2];
					int height = roi_set[3];
					
					ito::RectMeta *rm = static_cast<ito::RectMeta*>(m_params["roi"].getMeta());
					

					if (offset_x + width >= rm->getWidthRangeMeta().getMax())
					{
						retValue += getErrStr(pxiSetParam(m_handle, XI_PRM_WIDTH, &width, intSize, intType), "XI_PRM_WIDTH");
						retValue += getErrStr(pxiSetParam(m_handle, XI_PRM_OFFSET_X, &offset_x, intSize, intType), "XI_PRM_OFFSET_X");
					}
					else
					{
						retValue += getErrStr(pxiSetParam(m_handle, XI_PRM_OFFSET_X, &offset_x, intSize, intType), "XI_PRM_OFFSET_X");
						retValue += getErrStr(pxiSetParam(m_handle, XI_PRM_WIDTH, &width, intSize, intType), "XI_PRM_WIDTH");						
					}

					if (offset_y + height >= rm->getHeightRangeMeta().getMax())
					{
						retValue += getErrStr(pxiSetParam(m_handle, XI_PRM_HEIGHT, &height, intSize, intType), "XI_PRM_HEIGHT");
						retValue += getErrStr(pxiSetParam(m_handle, XI_PRM_OFFSET_Y, &offset_y, intSize, intType), "XI_PRM_OFFSET_Y");
					}
 					else
					{
						retValue += getErrStr(pxiSetParam(m_handle, XI_PRM_OFFSET_Y, &offset_y, intSize, intType), "XI_PRM_OFFSET_Y");
						retValue += getErrStr(pxiSetParam(m_handle, XI_PRM_HEIGHT, &height, intSize, intType), "XI_PRM_HEIGHT");						
					}

					if (!retValue.containsError())
					{
						int *roi = m_params["roi"].getVal<int*>();
						roi[0] = offset_x;
						roi[1] = offset_y;
						roi[2] = width;
						roi[3] = height;
						m_params["x0"].setVal<int>(offset_x);
						m_params["x1"].setVal<int>(width);
						m_params["y0"].setVal<int>(offset_y);
						m_params["y1"].setVal<int>(height);

						m_params["sizex"].setVal<int>(width);
						m_params["sizey"].setVal<int>(height);
						retValue += synchronizeCameraSettings(sExposure | sRoi | sFrameRate);
					}			
				}
			}

		}

		else
		{
			it->copyValueFrom(&(*val));
		}
    }

	if (running)
    {
        retValue += this->startDevice(0);
        setGrabberStarted(running);
    }

    if(!retValue.containsError())
    {
        emit parametersChanged(m_params); //send changed parameters to any connected dialogs or dock-widgets
		retValue += checkData();
    }

    if (waitCond)
    {
        waitCond->returnValue = retValue;
        waitCond->release();
    }

    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal Ximea::setXimeaParam(const char *paramName, int newValue)
{
    int min = std::numeric_limits<int>::max();
    int max = 0;
    int inc = 1;
    ito::RetVal retval;
    DWORD pSize = sizeof(int);
    XI_PRM_TYPE pType = xiTypeInteger;


    QByteArray name;

    //get parameter ranges
    name = QByteArray(paramName) + XI_PRM_INFO_MIN;
    retval +=  getErrStr(pxiGetParam(m_handle, name.data(), &min, &pSize, &pType), (QString(name.data()), " XI_PRM_INFO_MIN"));

    name = QByteArray(paramName) + XI_PRM_INFO_MAX;
    retval +=  getErrStr(pxiGetParam(m_handle, name.data(), &max, &pSize, &pType), (QString(name.data()), "XI_PRM_INFO_MAX"));

#ifndef USE_OLD_API
    name = QByteArray(paramName) + XI_PRM_INFO_INCREMENT;
    retval +=  getErrStr(pxiGetParam(m_handle, name.data(), &inc, &pSize, &pType), (QString(name.data()), "XI_PRM_INFO_INCREMENT"));
#endif

    if (!retval.containsError())
    {
        //check incoming parameter
        if (newValue < min || newValue > max)
        {
            retval += ito::RetVal::format(ito::retError,0, "xiApi-Parameter '%s' is out of allowed range [%i,%i]", paramName, min, max);
        }
#ifndef USE_OLD_API
        else if ( (newValue - min) % inc != 0)
        {
            retval += ito::RetVal::format(ito::retError,0, "xiApi-Parameter '%s' must have an increment of %i (minimum value %i)", paramName, inc, min);
        }
#endif

        if (!retval.containsError())
        {
            retval += getErrStr(pxiSetParam(m_handle, paramName, &newValue, sizeof(int), xiTypeInteger), QString(paramName));
        }
    }

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal Ximea::init(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);
    int bandwidthLimit = paramsOpt->at(2).getVal<int>(); //0 auto bandwidth calculation
	int iCamNumber = (*paramsOpt)[0].getVal<int>();
    XI_RETURN ret;

#ifndef USE_OLD_API
    int timing_mode = 0;
#endif
    QFile paramFile;

    // Load parameterlist from XML-file
    int loadPrev = (*paramsOpt)[1].getVal<int>();

    DWORD pSize = sizeof(int);
	DWORD intSize = sizeof(int);
	DWORD floatSize = sizeof(float);
    XI_PRM_TYPE pType = xiTypeInteger;
	XI_PRM_TYPE intType = xiTypeInteger;
    XI_PRM_TYPE strType = xiTypeString;
	XI_PRM_TYPE floatType = xiTypeFloat;
	QMap<QString, ito::Param>::iterator it;
    retValue += LoadLib();

    if (!retValue.containsError())
    {
	
	    m_params["camNumber"].getVal<int>(iCamNumber);
        Initnum++;  // so we have a new running instance of this grabber (or not)

        if( ++InitList[iCamNumber] > 1)    // It does not matter if the rest works or not. The close command will fix this anyway
        {
            retValue = ito::RetVal(ito::retError, 0, tr("Camera already initialized. Try with another camera number").toLatin1().data());
        }
        else
        {
            ret = pxiOpenDevice(iCamNumber, &m_handle);
            if (!m_handle || ret != XI_OK)
            {
                m_handle = NULL;
                retValue += getErrStr(ret, "pxiOpenDevice");
                retValue += ito::RetVal(ito::retError, 0, tr("Unable open camera").toLatin1().data());
            }
            else
            {
                char strBuf[1024];
                int serialNumber;
                DWORD strBufSize = 1024 * sizeof(char);
			    m_params["camNumber"].setVal<int>(iCamNumber);
                retValue += getErrStr(pxiGetParam(m_handle, XI_PRM_DEVICE_NAME, &strBuf, &strBufSize, &strType), "XI_PRM_DEVICE_NAME");
			    m_params["sensor_type"].setVal<char*>(strBuf);
                retValue += getErrStr(pxiGetParam(m_handle, XI_PRM_DEVICE_SN, &serialNumber, &pSize, &pType), "XI_PRM_DEVICE_SN");
			    m_params["serialNumber"].setVal<int>(serialNumber);
                if (!retValue.containsError())
                {
                    QString serialNumberHex = QString::number(serialNumber, 16);
                    m_identifier = QString("%1 (SN:%2)").arg(strBuf).arg(serialNumberHex);
                }
            }

        
    
            if (!retValue.containsError())
            {
                //get available bandwidth in Mb/sec
                //int availableBandwidth;
                //retValue += getErrStr(pxiGetParam(m_handle, XI_PRM_AVAILABLE_BANDWIDTH, &availableBandwidth, &pSize, &pType), "XI_PRM_AVAILABLE_BANDWIDTH");
                //std::cout << "available bandwidth: " << availableBandwidth << std::endl;
    #ifndef USE_OLD_API
                if (bandwidthLimit > 0) //manually set bandwidthLimit
                {
                    retValue += setXimeaParam(XI_PRM_AUTO_BANDWIDTH_CALCULATION, XI_OFF);
                    retValue += setXimeaParam(XI_PRM_LIMIT_BANDWIDTH, bandwidthLimit);
                }
    #endif
            }

            // Load parameterlist from XML-file
            if(loadPrev && !retValue.containsError())
            {
                QMap<QString, ito::Param> paramListXML;
                if (!retValue.containsError())
                {
                    retValue += ito::generateAutoSaveParamFile(this->getBasePlugin()->getFilename(), paramFile);
                }

                // Read parameter list from file to paramListXML
                if (!retValue.containsError())
                {
                    retValue += ito::loadXML2QLIST(&paramListXML, QString::number(iCamNumber), paramFile);
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
			    int integration_time, integration_max, integration_min, integration_step;  		
			    retValue += getErrStr(pxiGetParam(m_handle, XI_PRM_EXPOSURE XI_PRM_INFO_MIN, &integration_min, &intSize, &intType), "XI_PRM_EXPOSURE XI_PRM_INFO_MIN");
			    retValue += getErrStr(pxiGetParam(m_handle, XI_PRM_EXPOSURE XI_PRM_INFO_MAX, &integration_max, &intSize, &intType), "XI_PRM_EXPOSURE XI_PRM_INFO_MAX");
			    retValue += getErrStr(pxiGetParam(m_handle, XI_PRM_EXPOSURE XI_PRM_INFO_INCREMENT, &integration_step, &intSize, &intType), "XI_PRM_EXPOSURE XI_PRM_INFO_INCREMENT");
			    retValue += getErrStr(pxiGetParam(m_handle, XI_PRM_EXPOSURE, &integration_time, &intSize, &intType), "XI_PRM_EXPOSURE");
			    if (integration_step == 0)
			    {
				    integration_step = integration_max - integration_min;
			    }	

			    m_params["integration_time"].setVal<double>(musecToSec(integration_time));
			    m_params["integration_time"].setMeta(new ito::DoubleMeta(musecToSec(integration_min + integration_step), musecToSec(integration_max - integration_step), musecToSec(integration_step)), true);

			    // set binning
			    int binning, binning_min, binning_max, binning_type;
			    retValue += getErrStr(pxiGetParam(m_handle, XI_PRM_DOWNSAMPLING, &binning, &intSize, &intType), "XI_PRM_DOWNSAMPLING");
			    retValue += getErrStr(pxiGetParam(m_handle, XI_PRM_DOWNSAMPLING XI_PRM_INFO_MIN, &binning_min, &intSize, &intType), "XI_PRM_DOWNSAMPLING XI_PRM_INFO_MIN");
			    retValue += getErrStr(pxiGetParam(m_handle, XI_PRM_DOWNSAMPLING XI_PRM_INFO_MAX, &binning_max, &intSize, &intType), "XI_PRM_DOWNSAMPLING XI_PRM_INFO_MAX");
			    m_params["binning"].setVal<int>(binning * 101); //1 -> 101, 2 -> 202, 4 -> 404
			    m_params["binning"].setMeta(new ito::IntMeta(binning_min * 101, binning_max * 101), true);

			    // set binning type
			    retValue += getErrStr(pxiGetParam(m_handle, XI_PRM_DOWNSAMPLING_TYPE, &binning_type, &intSize, &intType), "XI_PRM_DOWNSAMPLING_TYPE");
			    m_params["binning_type"].setVal<int>(binning_type);

			    //sets framerate value interval
			    float framerate, framerate_min, framerate_max, framerate_inc;
			    retValue += getErrStr(pxiGetParam(m_handle, XI_PRM_FRAMERATE, &framerate, &floatSize, &floatType), "XI_PRM_FRAMERATE");
			    retValue += getErrStr(pxiGetParam(m_handle, XI_PRM_FRAMERATE XI_PRM_INFO_MIN, &framerate_min, &floatSize, &floatType), "XI_PRM_FRAMERATE XI_PRM_INFO_MIN");
			    retValue += getErrStr(pxiGetParam(m_handle, XI_PRM_FRAMERATE XI_PRM_INFO_MAX, &framerate_max, &floatSize, &floatType), "XI_PRM_FRAMERATE XI_PRM_INFO_MAX");
			    retValue += getErrStr(pxiGetParam(m_handle, XI_PRM_FRAMERATE XI_PRM_INFO_INCREMENT, &framerate_inc, &floatSize, &floatType), "XI_PRM_FRAMERATE XI_PRM_INFO_INCREMENT");
			    m_params["framerate"].setVal<double>(framerate);
			    m_params["framerate"].setMeta(new ito::DoubleMeta(framerate_min, framerate_max, framerate_inc), true);

			    //need new API
			    //sets offset of black_level
			    int offset, offset_min, offset_max, offset_inc;
			    retValue += getErrStr(pxiGetParam(m_handle, XI_PRM_IMAGE_BLACK_LEVEL, &offset, &intSize, &intType), "XI_PRM_IMAGE_BLACK_LEVEL");
			    retValue += getErrStr(pxiGetParam(m_handle, XI_PRM_IMAGE_BLACK_LEVEL XI_PRM_INFO_MIN, &offset_min, &intSize, &intType), "XI_PRM_IMAGE_BLACK_LEVEL XI_PRM_INFO_MIN");
			    retValue += getErrStr(pxiGetParam(m_handle, XI_PRM_IMAGE_BLACK_LEVEL XI_PRM_INFO_MAX, &offset_max, &intSize, &intType), "XI_PRM_IMAGE_BLACK_LEVEL XI_PRM_INFO_MAX");
			    retValue += getErrStr(pxiGetParam(m_handle, XI_PRM_IMAGE_BLACK_LEVEL XI_PRM_INFO_INCREMENT, &offset_inc, &intSize, &intType), "XI_PRM_IMAGE_BLACK_LEVEL XI_PRM_INFO_INCREMENT");
			    m_params["offset"].setVal<double>(offset);
			    m_params["offset"].setMeta(new ito::DoubleMeta(offset_min, offset_max, offset_inc), true);
			    
			
			    //sets gamma value interval
			    float gamma, gamma_min, gamma_max, gamma_inc;
			    retValue += getErrStr(pxiGetParam(m_handle, XI_PRM_GAMMAY, &gamma, &floatSize, &floatType), "XI_PRM_GAMMAY");
			    retValue += getErrStr(pxiGetParam(m_handle, XI_PRM_GAMMAY XI_PRM_INFO_MIN, &gamma_min, &floatSize, &floatType), "XI_PRM_GAMMAY XI_PRM_INFO_MIN");
			    retValue += getErrStr(pxiGetParam(m_handle, XI_PRM_GAMMAY XI_PRM_INFO_MAX, &gamma_max, &floatSize, &floatType), "XI_PRM_GAMMAY XI_PRM_INFO_MAX");
			    retValue += getErrStr(pxiGetParam(m_handle, XI_PRM_GAMMAY XI_PRM_INFO_INCREMENT, &gamma_inc, &floatSize, &floatType), "XI_PRM_GAMMAY XI_PRM_INFO_INCREMENT");
			    m_params["gamma"].setVal<double>(gamma);
			    m_params["gamma"].setMeta(new ito::DoubleMeta(gamma_min, gamma_max, gamma_inc), true);

			    //sets sharpness value interval
			    float sharpness, sharpness_min, sharpness_max, sharpness_inc;
			    retValue += getErrStr(pxiGetParam(m_handle, XI_PRM_SHARPNESS, &sharpness, &floatSize, &floatType), "XI_PRM_SHARPNESS");
			    retValue += getErrStr(pxiGetParam(m_handle, XI_PRM_SHARPNESS XI_PRM_INFO_MIN, &sharpness_min, &floatSize, &floatType), "XI_PRM_SHARPNESS XI_PRM_INFO_MIN");
			    retValue += getErrStr(pxiGetParam(m_handle, XI_PRM_SHARPNESS XI_PRM_INFO_MAX, &sharpness_max, &floatSize, &floatType), "XI_PRM_SHARPNESS XI_PRM_INFO_MAX");
			    retValue += getErrStr(pxiGetParam(m_handle, XI_PRM_SHARPNESS XI_PRM_INFO_INCREMENT, &sharpness_inc, &floatSize, &floatType), "XI_PRM_SHARPNESS XI_PRM_INFO_INCREMENT");
			    m_params["sharpness"].setVal<double>(sharpness);
			    m_params["sharpness"].setMeta(new ito::DoubleMeta(sharpness_min, sharpness_max, sharpness_inc), true);

			    //sets gain value interval
		        float gain, gain_min, gain_max, gain_inc;
			    retValue += getErrStr(pxiGetParam(m_handle, XI_PRM_GAIN, &gain, &floatSize, &floatType), "XI_PRM_GAIN");
			    retValue += getErrStr(pxiGetParam(m_handle, XI_PRM_GAIN XI_PRM_INFO_MIN, &gain_min, &floatSize, &floatType), "XI_PRM_GAIN XI_PRM_INFO_MIN");
			    retValue += getErrStr(pxiGetParam(m_handle, XI_PRM_GAIN XI_PRM_INFO_MAX, &gain_max, &floatSize, &floatType), "XI_PRM_GAIN XI_PRM_INFO_MAX");
			    retValue += getErrStr(pxiGetParam(m_handle, XI_PRM_GAIN XI_PRM_INFO_INCREMENT, &gain_inc, &floatSize, &floatType), "XI_PRM_GAIN XI_PRM_INFO_INCREMENT");
			    m_params["gain"].setVal<double>(gain);
			    m_params["gain"].setMeta(new ito::DoubleMeta(gain_min, gain_max, gain_inc), true);

			    //Sets trigger mode
			    int trigger_mode; 
			    retValue += getErrStr(pxiGetParam(m_handle, XI_PRM_TRG_SOURCE, &trigger_mode, &intSize, &intType), "XI_PRM_TRG_SOURCE");
			    m_params["trigger_mode"].getVal<int>(trigger_mode);

			    //sets trigger mode 2
			    int trigger_mode2;
			    retValue += getErrStr(pxiGetParam(m_handle, XI_PRM_TRG_SELECTOR, &trigger_mode2, &intSize, &intType), "XI_PRM_TRG_SELECTOR");

			    if(trigger_mode2 = XI_TRG_SEL_FRAME_START)
			    {
				    m_params["trigger_mode2"].setVal<int>(0);
			    }
			    else if(trigger_mode2 = XI_TRG_SEL_EXPOSURE_ACTIVE)
			    {
				    m_params["trigger_mode2"].setVal<int>(1);
			    }
			    else if(trigger_mode2 = XI_TRG_SEL_FRAME_BURST_START)
			    {
				    m_params["trigger_mode2"].setVal<int>(2);
			    }
			    else if(trigger_mode2 = XI_TRG_SEL_FRAME_BURST_ACTIVE)
			    {
				    m_params["trigger_mode2"].setVal<int>(3);
			    }

    #ifndef USE_OLD_API
			    int timing_mode = 0;
			    retValue += getErrStr(pxiGetParam(m_handle, XI_PRM_ACQ_TIMING_MODE, &timing_mode, &intSize, &intType), "XI_PRM_ACQ_TIMING_MODE");
			    m_params["timing_mode"].getVal<int>(timing_mode);
    #endif
			    int output_bit_depth = 0;
			    int bitppix = XI_MONO8;

			    retValue += getErrStr(pxiSetParam(m_handle, XI_PRM_IMAGE_DATA_FORMAT, &bitppix, sizeof(int), xiTypeInteger), "XI_PRM_IMAGE_DATA_FORMAT");
			    retValue += getErrStr(pxiGetParam(m_handle, XI_PRM_OUTPUT_DATA_BIT_DEPTH, &output_bit_depth, &pSize, &pType), "XI_PRM_OUTPUT_DATA_BIT_DEPTH");
			    static_cast<ito::IntMeta*>(m_params["bpp"].getMeta())->setMin(output_bit_depth);

			    bitppix = XI_MONO16;
			    retValue += getErrStr(pxiSetParam(m_handle, XI_PRM_IMAGE_DATA_FORMAT, &bitppix, sizeof(int), xiTypeInteger), "XI_PRM_IMAGE_DATA_FORMAT");
			    retValue += getErrStr(pxiGetParam(m_handle, XI_PRM_OUTPUT_DATA_BIT_DEPTH, &output_bit_depth, &pSize, &pType), "XI_PRM_OUTPUT_DATA_BIT_DEPTH");
			    static_cast<ito::IntMeta*>(m_params["bpp"].getMeta())->setMax(output_bit_depth);
			    m_params["bpp"].setVal<int>(output_bit_depth);

			    // bad pixel correction
			    int badpix;
			    retValue += getErrStr(pxiGetParam(m_handle, XI_PRM_BPC, &badpix, &intSize, &intType), "XI_PRM_BPC");
			    m_params["badPixel"].setVal(badpix);
            }

		    //sets ROI values
            if (!retValue.containsError())
            {
			    int offset_x, offsetMin_x, offsetMax_x, offsetInc_x;
			    int size_x, sizeMin_x, sizeMax_x, sizeInc_x;

			    //obtain current offsetX and width values
			    retValue += getErrStr(pxiGetParam(m_handle, XI_PRM_WIDTH, &size_x, &intSize, &intType), "XI_PRM_WIDTH");
			    retValue += getErrStr(pxiGetParam(m_handle, XI_PRM_WIDTH XI_PRM_INFO_MIN, &sizeMin_x, &intSize, &intType), "XI_PRM_WIDTH XI_PRM_INFO_MIN");
			    retValue += getErrStr(pxiGetParam(m_handle, XI_PRM_WIDTH XI_PRM_INFO_MAX, &sizeMax_x, &intSize, &intType), "XI_PRM_WIDTH XI_PRM_INFO_MAX");
			    retValue += getErrStr(pxiGetParam(m_handle, XI_PRM_WIDTH XI_PRM_INFO_INCREMENT, &sizeInc_x, &intSize, &intType), "XI_PRM_WIDTH XI_PRM_INFO_INCREMENT");
			    if (sizeInc_x == 0)
			    {
				    //sizeInc_x = sizeMax_x - sizeMin_x;
				    sizeInc_x = 1;
			    }

			    retValue += getErrStr(pxiGetParam(m_handle, XI_PRM_OFFSET_X, &offset_x, &intSize, &intType), "XI_PRM_OFFSET_X");
			    retValue += getErrStr(pxiGetParam(m_handle, XI_PRM_OFFSET_X XI_PRM_INFO_MIN, &offsetMin_x, &intSize, &intType), "XI_PRM_OFFSET_X XI_PRM_INFO_MIN");
			    retValue += getErrStr(pxiGetParam(m_handle, XI_PRM_OFFSET_X XI_PRM_INFO_MAX, &offsetMax_x, &intSize, &intType), "XI_PRM_OFFSET_X XI_PRM_INFO_MAX");
			    retValue += getErrStr(pxiGetParam(m_handle, XI_PRM_OFFSET_X XI_PRM_INFO_INCREMENT, &offsetInc_x, &intSize, &intType), "XI_PRM_OFFSET_X XI_PRM_INFO_INCREMENT");
			    if (offsetInc_x == 0)
			    {
				    //offsetInc_x = offsetMax_x - offsetMin_x;
					offsetInc_x = 1;
			    }

			    m_params["x0"].setVal<int>(offset_x);
			    m_params["x0"].setMeta(new ito::IntMeta(offsetMin_x, sizeMax_x - sizeMin_x, offsetInc_x), true);
			    m_params["x1"].setVal<int>(offset_x + size_x - 1);
			    m_params["x1"].setMeta(new ito::IntMeta(offset_x + sizeMin_x - 1, sizeMax_x - 1, sizeInc_x), true);
			    m_params["sizex"].setVal<int>(size_x);
			    m_params["sizex"].setMeta(new ito::IntMeta(sizeMin_x, sizeMax_x, sizeInc_x), true);

			    //obtain current offsetY and height values
			    int offset_y, offsetMin_y, offsetMax_y, offsetInc_y;
			    int size_y, sizeMin_y, sizeMax_y, sizeInc_y;
			    retValue += getErrStr(pxiGetParam(m_handle, XI_PRM_HEIGHT, &size_y, &intSize, &intType), "XI_PRM_HEIGHT");
			    retValue += getErrStr(pxiGetParam(m_handle, XI_PRM_HEIGHT XI_PRM_INFO_MIN, &sizeMin_y, &intSize, &intType), "XI_PRM_HEIGHT XI_PRM_INFO_MIN");
			    retValue += getErrStr(pxiGetParam(m_handle, XI_PRM_HEIGHT XI_PRM_INFO_MAX, &sizeMax_y, &intSize, &intType), "XI_PRM_HEIGHT XI_PRM_INFO_MAX");
			    retValue += getErrStr(pxiGetParam(m_handle, XI_PRM_HEIGHT XI_PRM_INFO_INCREMENT, &sizeInc_y, &intSize, &intType), "XI_PRM_HEIGHT XI_PRM_INFO_INCREMENT");
			    if (sizeInc_y == 0)
			    {
				    //sizeInc_y = sizeMax_y - sizeMin_y;
					sizeInc_y =1;
			    }

			    retValue += getErrStr(pxiGetParam(m_handle, XI_PRM_OFFSET_Y, &offset_y, &intSize, &intType), "XI_PRM_OFFSET_Y");
			    retValue += getErrStr(pxiGetParam(m_handle, XI_PRM_OFFSET_Y XI_PRM_INFO_MIN, &offsetMin_y, &intSize, &intType), "XI_PRM_OFFSET_Y XI_PRM_INFO_MIN");
			    retValue += getErrStr(pxiGetParam(m_handle, XI_PRM_OFFSET_Y XI_PRM_INFO_MAX, &offsetMax_y, &intSize, &intType), "XI_PRM_OFFSET_Y XI_PRM_INFO_MAX");
			    retValue += getErrStr(pxiGetParam(m_handle, XI_PRM_OFFSET_Y XI_PRM_INFO_INCREMENT, &offsetInc_y, &intSize, &intType), "XI_PRM_OFFSET_Y XI_PRM_INFO_INCREMENT");
			    if (offsetInc_y == 0)
			    {
				    //offsetInc_y = offsetMax_y - offsetMin_y;
					offsetInc_y = 1;
			    }

			    m_params["y0"].setVal<int>(offset_y);
			    m_params["y0"].setMeta(new ito::IntMeta(offsetMin_y, sizeMax_y - sizeMin_y, offsetInc_y), true);
			    m_params["y1"].setVal<int>(offset_y + size_y - 1);
			    m_params["y1"].setMeta(new ito::IntMeta(offset_y + sizeMin_y - 1, sizeMax_y - 1, sizeInc_y), true);
			    m_params["sizey"].setVal<int>(size_y);
			    m_params["sizey"].setMeta(new ito::IntMeta(sizeMin_y, sizeMax_y, sizeInc_y), true); 


#if defined(ITOM_ADDININTERFACE_VERSION) && ITOM_ADDININTERFACE_VERSION > 0x010300
		    it = m_params.find("roi");
            int *roi = it->getVal<int*>();
		    roi[0] = offset_x;
		    roi[1] = offset_y;
		    roi[2] = size_x;
		    roi[3] = size_y;
		    ito::RangeMeta widthMeta(offsetMin_x, sizeMax_x + offset_x -1, offsetInc_x, sizeMin_x, sizeMax_x + offset_x, sizeInc_x);
		    ito::RangeMeta heightMeta(offsetMin_y, sizeMax_y + offset_y -1, offsetInc_y, sizeMin_y, sizeMax_y + offset_x, sizeInc_y);	
			it->setMeta(new ito::RectMeta(widthMeta, heightMeta), true);
			
#endif
		    }


            if (!retValue.containsError())
            {
                int val = XI_BP_SAFE;
                retValue += getErrStr(pxiSetParam(m_handle, XI_PRM_BUFFER_POLICY, &val, sizeof(int), xiTypeInteger), "XI_PRM_BUFFER_POLICY");
				retValue += synchronizeCameraSettings();
            }

            if (!retValue.containsError())
            {
                retValue += checkData();
            }
        }

        if (retValue.containsError())
        {
            if(Initnum <= 1) //this instance already incremented Initnum in any cases, it will be decremented in case of error in the close method
            {
                if (ximeaLib)
                {
    #if linux
                    dlclose(ximeaLib);
    #else
                    FreeLibrary(ximeaLib);
    #endif
                    ximeaLib = NULL;
                }
            }
            else
            {
                //std::cerr << "DLLs not unloaded due to further running grabber instances\n" << std::endl;
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
ito::RetVal Ximea::synchronizeCameraSettings(int what /*= sAll */)
{
	UINT uintVal;
	UINT uintVal3[3];
	DOUBLE dVal;
	DOUBLE dVal3[3];
	ito::RetVal retValue, rettemp;
	QMap<QString, ito::Param>::iterator it;

	DWORD pSize = sizeof(int);
	DWORD intSize = sizeof(int);
	DWORD floatSize = sizeof(float);
    XI_PRM_TYPE pType = xiTypeInteger;
	XI_PRM_TYPE intType = xiTypeInteger;
    XI_PRM_TYPE strType = xiTypeString;
	XI_PRM_TYPE floatType = xiTypeFloat;

	if (what & sExposure)
	{
		it = m_params.find("integration_time");

		// Camera-exposure is set in µsec, itom uses s
		int integration_time, integration_max, integration_min, integration_step;  		
		retValue += getErrStr(pxiGetParam(m_handle, XI_PRM_EXPOSURE XI_PRM_INFO_MIN, &integration_min, &intSize, &intType), "XI_PRM_EXPOSURE XI_PRM_INFO_MIN");
		retValue += getErrStr(pxiGetParam(m_handle, XI_PRM_EXPOSURE XI_PRM_INFO_MAX, &integration_max, &intSize, &intType), "XI_PRM_EXPOSURE XI_PRM_INFO_MAX");
		retValue += getErrStr(pxiGetParam(m_handle, XI_PRM_EXPOSURE XI_PRM_INFO_INCREMENT, &integration_step, &intSize, &intType), "XI_PRM_EXPOSURE XI_PRM_INFO_INCREMENT");
		retValue += getErrStr(pxiGetParam(m_handle, XI_PRM_EXPOSURE, &integration_time, &intSize, &intType), "XI_PRM_EXPOSURE");
		if (integration_step == 0)
		{
			integration_step = integration_max - integration_min;
		}	
		it->setVal<double>(musecToSec(integration_time));
		it->setMeta(new ito::DoubleMeta(musecToSec(integration_min + integration_step), musecToSec(integration_max - integration_step), musecToSec(integration_step)), true);
	}
	if (what & sBinning)
	{
		it = m_params.find("binning");

		// set binning
		int binning, binning_min, binning_max, binning_type;
		retValue += getErrStr(pxiGetParam(m_handle, XI_PRM_DOWNSAMPLING, &binning, &intSize, &intType), "XI_PRM_DOWNSAMPLING");
		retValue += getErrStr(pxiGetParam(m_handle, XI_PRM_DOWNSAMPLING XI_PRM_INFO_MIN, &binning_min, &intSize, &intType), "XI_PRM_DOWNSAMPLING XI_PRM_INFO_MIN");
		retValue += getErrStr(pxiGetParam(m_handle, XI_PRM_DOWNSAMPLING XI_PRM_INFO_MAX, &binning_max, &intSize, &intType), "XI_PRM_DOWNSAMPLING XI_PRM_INFO_MAX");
		it->setVal<int>(binning * 101);
		it->setMeta(new ito::IntMeta(binning_min * 101, binning_max * 101), true);
	}
	if (what & sFrameRate)
	{
		it = m_params.find("framerate");

		//sets framerate value interval
		float framerate, framerate_min, framerate_max, framerate_inc;
		retValue += getErrStr(pxiGetParam(m_handle, XI_PRM_FRAMERATE, &framerate, &floatSize, &floatType), "XI_PRM_FRAMERATE");
		retValue += getErrStr(pxiGetParam(m_handle, XI_PRM_FRAMERATE XI_PRM_INFO_MIN, &framerate_min, &floatSize, &floatType), "XI_PRM_FRAMERATE XI_PRM_INFO_MIN");
		retValue += getErrStr(pxiGetParam(m_handle, XI_PRM_FRAMERATE XI_PRM_INFO_MAX, &framerate_max, &floatSize, &floatType), "XI_PRM_FRAMERATE XI_PRM_INFO_MAX");
		retValue += getErrStr(pxiGetParam(m_handle, XI_PRM_FRAMERATE XI_PRM_INFO_INCREMENT, &framerate_inc, &floatSize, &floatType), "XI_PRM_FRAMERATE XI_PRM_INFO_INCREMENT");
		it->setVal<double>(framerate);
		it->setMeta(new ito::DoubleMeta(framerate_min, framerate_max, framerate_inc), true);
	}
	if (what & sRoi)
	{
		it = m_params.find("roi");
		int offset_x, offsetMin_x, offsetMax_x, offsetInc_x;
		int size_x, sizeMin_x, sizeMax_x, sizeInc_x;

		//obtain current offsetX and width values
		retValue += getErrStr(pxiGetParam(m_handle, XI_PRM_WIDTH, &size_x, &intSize, &intType), "XI_PRM_WIDTH");
		retValue += getErrStr(pxiGetParam(m_handle, XI_PRM_WIDTH XI_PRM_INFO_MIN, &sizeMin_x, &intSize, &intType), "XI_PRM_WIDTH XI_PRM_INFO_MIN");
		retValue += getErrStr(pxiGetParam(m_handle, XI_PRM_WIDTH XI_PRM_INFO_MAX, &sizeMax_x, &intSize, &intType), "XI_PRM_WIDTH XI_PRM_INFO_MAX");
		retValue += getErrStr(pxiGetParam(m_handle, XI_PRM_WIDTH XI_PRM_INFO_INCREMENT, &sizeInc_x, &intSize, &intType), "XI_PRM_WIDTH XI_PRM_INFO_INCREMENT");
		if (sizeInc_x == 0)
		{
			//sizeInc_x = sizeMax_x - sizeMin_x;
			sizeInc_x = 1;
		}

		retValue += getErrStr(pxiGetParam(m_handle, XI_PRM_OFFSET_X, &offset_x, &intSize, &intType), "XI_PRM_OFFSET_X");
		retValue += getErrStr(pxiGetParam(m_handle, XI_PRM_OFFSET_X XI_PRM_INFO_MIN, &offsetMin_x, &intSize, &intType), "XI_PRM_OFFSET_X XI_PRM_INFO_MIN");
		retValue += getErrStr(pxiGetParam(m_handle, XI_PRM_OFFSET_X XI_PRM_INFO_MAX, &offsetMax_x, &intSize, &intType), "XI_PRM_OFFSET_X XI_PRM_INFO_MAX");
		retValue += getErrStr(pxiGetParam(m_handle, XI_PRM_OFFSET_X XI_PRM_INFO_INCREMENT, &offsetInc_x, &intSize, &intType), "XI_PRM_OFFSET_X XI_PRM_INFO_INCREMENT");
		if (offsetInc_x == 0)
		{
			//offsetInc_x = offsetMax_x - offsetMin_x;
			offsetInc_x = 1;
		}

		m_params["x0"].setVal<int>(offset_x);
		m_params["x0"].setMeta(new ito::IntMeta(offsetMin_x, sizeMax_x - sizeMin_x, offsetInc_x), true);
		m_params["x1"].setVal<int>(offset_x + size_x - 1);
		m_params["x1"].setMeta(new ito::IntMeta(offset_x + sizeMin_x - 1, sizeMax_x - 1, sizeInc_x), true);
		m_params["sizex"].setVal<int>(size_x);
		m_params["sizex"].setMeta(new ito::IntMeta(sizeMin_x, sizeMax_x, sizeInc_x), true);

		//obtain current offsetY and height values
		int offset_y, offsetMin_y, offsetMax_y, offsetInc_y;
		int size_y, sizeMin_y, sizeMax_y, sizeInc_y;
		retValue += getErrStr(pxiGetParam(m_handle, XI_PRM_HEIGHT, &size_y, &intSize, &intType), "XI_PRM_HEIGHT");
		retValue += getErrStr(pxiGetParam(m_handle, XI_PRM_HEIGHT XI_PRM_INFO_MIN, &sizeMin_y, &intSize, &intType), "XI_PRM_HEIGHT XI_PRM_INFO_MIN");
		retValue += getErrStr(pxiGetParam(m_handle, XI_PRM_HEIGHT XI_PRM_INFO_MAX, &sizeMax_y, &intSize, &intType), "XI_PRM_HEIGHT XI_PRM_INFO_MAX");
		retValue += getErrStr(pxiGetParam(m_handle, XI_PRM_HEIGHT XI_PRM_INFO_INCREMENT, &sizeInc_y, &intSize, &intType), "XI_PRM_HEIGHT XI_PRM_INFO_INCREMENT");
		if (sizeInc_y == 0)
		{
			//sizeInc_y = sizeMax_y - sizeMin_y;
			sizeInc_y =1;
		}

		retValue += getErrStr(pxiGetParam(m_handle, XI_PRM_OFFSET_Y, &offset_y, &intSize, &intType), "XI_PRM_OFFSET_Y");
		retValue += getErrStr(pxiGetParam(m_handle, XI_PRM_OFFSET_Y XI_PRM_INFO_MIN, &offsetMin_y, &intSize, &intType), "XI_PRM_OFFSET_Y XI_PRM_INFO_MIN");
		retValue += getErrStr(pxiGetParam(m_handle, XI_PRM_OFFSET_Y XI_PRM_INFO_MAX, &offsetMax_y, &intSize, &intType), "XI_PRM_OFFSET_Y XI_PRM_INFO_MAX");
		retValue += getErrStr(pxiGetParam(m_handle, XI_PRM_OFFSET_Y XI_PRM_INFO_INCREMENT, &offsetInc_y, &intSize, &intType), "XI_PRM_OFFSET_Y XI_PRM_INFO_INCREMENT");
		if (offsetInc_y == 0)
		{
			//offsetInc_y = offsetMax_y - offsetMin_y;
			offsetInc_y = 1;
		}

		m_params["y0"].setVal<int>(offset_y);
		m_params["y0"].setMeta(new ito::IntMeta(offsetMin_y, sizeMax_y - sizeMin_y, offsetInc_y), true);
		m_params["y1"].setVal<int>(offset_y + size_y - 1);
		m_params["y1"].setMeta(new ito::IntMeta(offset_y + sizeMin_y - 1, sizeMax_y - 1, sizeInc_y), true);
		m_params["sizey"].setVal<int>(size_y);
		m_params["sizey"].setMeta(new ito::IntMeta(sizeMin_y, sizeMax_y, sizeInc_y), true); 


#if defined(ITOM_ADDININTERFACE_VERSION) && ITOM_ADDININTERFACE_VERSION > 0x010300
        int *roi = it->getVal<int*>();
		roi[0] = offset_x;
		roi[1] = offset_y;
		roi[2] = size_x;
		roi[3] = size_y;
		ito::RangeMeta widthMeta(offsetMin_x, sizeMax_x + offset_x -1, offsetInc_x, sizeMin_x, sizeMax_x + offset_x, sizeInc_x);
		ito::RangeMeta heightMeta(offsetMin_y, sizeMax_y + offset_y -1, offsetInc_y, sizeMin_y, sizeMax_y + offset_x, sizeInc_y);	
		it->setMeta(new ito::RectMeta(widthMeta, heightMeta), true);
#endif
	}
	if (what & sGain)
	{
		it = m_params.find("gain");
		//sets gain value interval
		float gain, gain_min, gain_max, gain_inc;
		retValue += getErrStr(pxiGetParam(m_handle, XI_PRM_GAIN, &gain, &floatSize, &floatType), "XI_PRM_GAIN");
		retValue += getErrStr(pxiGetParam(m_handle, XI_PRM_GAIN XI_PRM_INFO_MIN, &gain_min, &floatSize, &floatType), "XI_PRM_GAIN XI_PRM_INFO_MIN");
		retValue += getErrStr(pxiGetParam(m_handle, XI_PRM_GAIN XI_PRM_INFO_MAX, &gain_max, &floatSize, &floatType), "XI_PRM_GAIN XI_PRM_INFO_MAX");
		retValue += getErrStr(pxiGetParam(m_handle, XI_PRM_GAIN XI_PRM_INFO_INCREMENT, &gain_inc, &floatSize, &floatType), "XI_PRM_GAIN XI_PRM_INFO_INCREMENT");
		it->setVal<double>(gain);
		it->setMeta(new ito::DoubleMeta(gain_min, gain_max, gain_inc), true);
	}
	if (what & sOffset)
	{
		it = m_params.find("offset");

		//sets offset of black_level
		int offset, offset_min, offset_max, offset_inc;
		retValue += getErrStr(pxiGetParam(m_handle, XI_PRM_IMAGE_BLACK_LEVEL, &offset, &intSize, &intType), "XI_PRM_IMAGE_BLACK_LEVEL");
		retValue += getErrStr(pxiGetParam(m_handle, XI_PRM_IMAGE_BLACK_LEVEL XI_PRM_INFO_MIN, &offset_min, &intSize, &intType), "XI_PRM_IMAGE_BLACK_LEVEL XI_PRM_INFO_MIN");
		retValue += getErrStr(pxiGetParam(m_handle, XI_PRM_IMAGE_BLACK_LEVEL XI_PRM_INFO_MAX, &offset_max, &intSize, &intType), "XI_PRM_IMAGE_BLACK_LEVEL XI_PRM_INFO_MAX");
		retValue += getErrStr(pxiGetParam(m_handle, XI_PRM_IMAGE_BLACK_LEVEL XI_PRM_INFO_INCREMENT, &offset_inc, &intSize, &intType), "XI_PRM_IMAGE_BLACK_LEVEL XI_PRM_INFO_INCREMENT");
		it->setVal<double>(offset);
		it->setMeta(new ito::DoubleMeta(offset_min, offset_max, offset_inc), true);
	}
	if (what & sGamma)
	{
		it = m_params.find("gamma");

		//sets gamma value interval
		float gamma, gamma_min, gamma_max, gamma_inc;
		retValue += getErrStr(pxiGetParam(m_handle, XI_PRM_GAMMAY, &gamma, &floatSize, &floatType), "XI_PRM_GAMMAY");
		retValue += getErrStr(pxiGetParam(m_handle, XI_PRM_GAMMAY XI_PRM_INFO_MIN, &gamma_min, &floatSize, &floatType), "XI_PRM_GAMMAY XI_PRM_INFO_MIN");
		retValue += getErrStr(pxiGetParam(m_handle, XI_PRM_GAMMAY XI_PRM_INFO_MAX, &gamma_max, &floatSize, &floatType), "XI_PRM_GAMMAY XI_PRM_INFO_MAX");
		retValue += getErrStr(pxiGetParam(m_handle, XI_PRM_GAMMAY XI_PRM_INFO_INCREMENT, &gamma_inc, &floatSize, &floatType), "XI_PRM_GAMMAY XI_PRM_INFO_INCREMENT");
		it->setVal<double>(gamma);
		it->setMeta(new ito::DoubleMeta(gamma_min, gamma_max, gamma_inc), true);
	}
	if (what & sSharpness)
	{
		it = m_params.find("sharpness");

		//sets sharpness value interval
		float sharpness, sharpness_min, sharpness_max, sharpness_inc;
		retValue += getErrStr(pxiGetParam(m_handle, XI_PRM_SHARPNESS, &sharpness, &floatSize, &floatType), "XI_PRM_SHARPNESS");
		retValue += getErrStr(pxiGetParam(m_handle, XI_PRM_SHARPNESS XI_PRM_INFO_MIN, &sharpness_min, &floatSize, &floatType), "XI_PRM_SHARPNESS XI_PRM_INFO_MIN");
		retValue += getErrStr(pxiGetParam(m_handle, XI_PRM_SHARPNESS XI_PRM_INFO_MAX, &sharpness_max, &floatSize, &floatType), "XI_PRM_SHARPNESS XI_PRM_INFO_MAX");
		retValue += getErrStr(pxiGetParam(m_handle, XI_PRM_SHARPNESS XI_PRM_INFO_INCREMENT, &sharpness_inc, &floatSize, &floatType), "XI_PRM_SHARPNESS XI_PRM_INFO_INCREMENT");
		it->setVal<double>(sharpness);
		it->setMeta(new ito::DoubleMeta(sharpness_min, sharpness_max, sharpness_inc), true);
	}
	if (what & sTriggerMode)
	{
		it = m_params.find("triggermode");
		//Sets trigger mode
		int trigger_mode; 
		retValue += getErrStr(pxiGetParam(m_handle, XI_PRM_TRG_SOURCE, &trigger_mode, &intSize, &intType), "XI_PRM_TRG_SOURCE");
		it->getVal<int>(trigger_mode);
	}
	if (what & sTriggerMode2)
	{
		it = m_params.find("triggermode2");
		//sets trigger mode 2
		int trigger_mode2;
		retValue += getErrStr(pxiGetParam(m_handle, XI_PRM_TRG_SELECTOR, &trigger_mode2, &intSize, &intType), "XI_PRM_TRG_SELECTOR");

		if(trigger_mode2 = XI_TRG_SEL_FRAME_START)
		{
			it->setVal<int>(0);
		}
		else if(trigger_mode2 = XI_TRG_SEL_EXPOSURE_ACTIVE)
		{
			it->setVal<int>(1);
		}
		else if(trigger_mode2 = XI_TRG_SEL_FRAME_BURST_START)
		{
			it->setVal<int>(2);
		}
		else if(trigger_mode2 = XI_TRG_SEL_FRAME_BURST_ACTIVE)
		{
			it->setVal<int>(3);
		}
		
	}
	if (what & sBpp)
	{
		it = m_params.find("bpp");
		int output_bit_depth = 0;
		retValue += getErrStr(pxiGetParam(m_handle, XI_PRM_OUTPUT_DATA_BIT_DEPTH, &output_bit_depth, &pSize, &pType), "XI_PRM_OUTPUT_DATA_BIT_DEPTH");
		it->setVal<int>(output_bit_depth);
	}
	
	return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal Ximea::close(ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);
    XI_RETURN ret;

    if (this->m_handle == NULL)
    {
        retValue += ito::RetVal(ito::retWarning, 0, tr("Camera handle deleted before closing procedure").toLatin1().data());
        goto endclose;
    }

    setGrabberStarted(1);
    retValue += this->stopDevice(0);
    Sleep(50);

    if ((ret = pxiCloseDevice(m_handle)))
    {
        retValue += getErrStr(ret, "pxiCloseDevice");
    }

endclose:
    int nr = m_params["camNumber"].getVal<int>();
    InitList[nr] = 0;
    Initnum--; // so we closed a further instance of this grabber

    if(!Initnum)
    {
        if (ximeaLib)
        {
#if linux
            dlclose(ximeaLib);
#else
            FreeLibrary(ximeaLib);
#endif
            ximeaLib = NULL;
        }
    }
    else
    {
        retValue += ito::RetVal(ito::retWarning, 0, tr("DLLs not unloaded due to still living instances of Ximea-Cams").toLatin1().data());
    }

    if (waitCond)
    {
        waitCond->returnValue = retValue;
        waitCond->release();
    }

    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal Ximea::startDevice(ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);

    if(grabberStartedCount() < 1)
    {
        setGrabberStarted(0);
        XI_RETURN ret;
        ret = pxiStartAcquisition(m_handle);
        if (ret)
            retValue += getErrStr(ret, "pxiStartAcquisition");
    }

    if (!retValue.containsError())
    {
        incGrabberStarted();
    }

    m_isgrabbing = Ximea::grabberRunning;

    if (waitCond)
    {
        waitCond->returnValue = retValue;
        waitCond->release();
    }
    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal Ximea::stopDevice(ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);

    decGrabberStarted();

    if(grabberStartedCount() < 1)
    {
        XI_RETURN ret;
        if ((ret = pxiStopAcquisition(m_handle)))
            retValue += getErrStr(ret, "pxiStopAcquisition");
        m_isgrabbing = Ximea::grabberStopped;
    }
    if(grabberStartedCount() < 0)
    {
        retValue += ito::RetVal(ito::retWarning, 0, tr("Cameraflag was < 0").toLatin1().data());
        setGrabberStarted(0);
    }

    if (waitCond)
    {
        waitCond->returnValue = retValue;
        waitCond->release();
    }
    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal Ximea::acquire(const int trigger, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);
    int triggermode = m_params["trigger_mode"].getVal<int>();

    if (grabberStartedCount() <= 0)
    {
        retValue = ito::RetVal(ito::retError, 0, tr("Tried to acquire without starting device").toLatin1().data());

        m_isgrabbing &= ~Ximea::grabberGrabbed;
        m_isgrabbing |= Ximea::grabberGrabbed;
    }
    else
    {
        XI_RETURN ret;
        if(m_isgrabbing & Ximea::grabberGrabbed)
        {
            retValue = ito::RetVal(ito::retWarning, 0, tr("Tried to acquire multiple times without calling getVal. This acquire was ignored.").toLatin1().data());
        }
        else
        {
            if (triggermode == XI_TRG_SOFTWARE)
            {
                if ((ret = pxiSetParam(m_handle, XI_PRM_TRG_SOFTWARE, 0, sizeof(int), xiTypeInteger))) //TODO: isn't it necessary to set the value to XI_TRG_SOFTWARE here?
                {
                    retValue += getErrStr(ret, "XI_PRM_TRG_SOFTWARE"); 
                    m_acqRetVal += retValue;
                }
            }
        }

        if (waitCond)
        {
            waitCond->returnValue = retValue;
            waitCond->release();
        }

        int iPicTimeOut = 2000; //timeout in ms
        XI_IMG img;
        img.size = sizeof(XI_IMG);
        img.bp = m_data.rowPtr(0, 0);
        int curxsize = m_params["sizex"].getVal<int>();
        int curysize = m_params["sizey"].getVal<int>();


        img.bp_size = curxsize * curysize * (m_data.getType() == ito::tUInt16 ? 2 : 1);
        if ((ret = pxiGetImage(m_handle, iPicTimeOut, &img)))
        {
            retValue += getErrStr(ret, "pxiGetImage");
			if (ret == 105)
			{
				

			}
            m_acqRetVal += retValue;
            m_isgrabbing |= Ximea::grabberGrabError;
        }
        if(m_shading.active)
        {
            ito::uint16* ptrSub = m_shading.sub;
            ito::uint16* ptrMul = m_shading.mul;
            ito::uint16* ptrDst = (ito::uint16*)m_data.rowPtr(0, m_shading.y0);
            ptrDst += m_shading.x0;
            ito::int32 stepY = img.width - m_shading.xsize;
            for(int y = 0; y < m_shading.ysize; y++)
            {
                
                for(int x = 0; x < m_shading.xsize; x++)
                {
                    if(*ptrSub > *ptrDst)
                    {
                        *ptrDst = 0;
                    }
                    else
                    {
                        *ptrDst -= *ptrSub;
                        //*ptrDst *= *ptrMul;
                    }
                    ptrDst++;
                    ptrMul++;
                    ptrSub++;
                }            
                ptrDst += stepY;
            }
            
        
        }

        m_isgrabbing |= Ximea::grabberGrabbed;
        return retValue;
    }

    if (waitCond)
    {
        waitCond->returnValue = retValue;
        waitCond->release();
    }

    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal Ximea::retrieveData(ito::DataObject *externalDataObject)
{
    ito::RetVal retValue(ito::retOk);

    bool copyExternal = externalDataObject != NULL;

    if (!(this->m_isgrabbing & Ximea::grabberRunning))
    {
        retValue += ito::RetVal(ito::retWarning, 0, tr("Tried to get picture without starting device").toLatin1().data());
    }
    else if (!(this->m_isgrabbing & Ximea::grabberGrabbed))
    {
        retValue += ito::RetVal(ito::retWarning, 0, tr("Tried to get picture without triggering exposure").toLatin1().data());
    }
    else if (this->m_isgrabbing & Ximea::grabberGrabError)
    {
        retValue += m_acqRetVal;
        m_acqRetVal = ito::retOk;      
    }
    else if(copyExternal)
    {
        //here we wait until the Event is set to signaled state
        //or the timeout runs out

        cv::Mat* internalMat = (cv::Mat*)(m_data.get_mdata()[0]);
        cv::Mat* externalMat = (cv::Mat*)(externalDataObject->get_mdata()[externalDataObject->seekMat(0)]);

        if (externalMat->isContinuous())
        {
            memcpy(externalMat->ptr(0), internalMat->ptr(0), internalMat->cols * internalMat->rows * externalMat->elemSize());
        }
        else
        {
            for (int y = 0; y < internalMat->rows; y++)
            {
                memcpy(externalMat->ptr(y), internalMat->ptr(y), internalMat->cols * externalMat->elemSize());
            }
        }

    }

    this->m_isgrabbing &= ~ Ximea::grabberGrabbed;
    this->m_isgrabbing &= ~ Ximea::grabberGrabError;

    return retValue;
}


//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal Ximea::getVal(void *vpdObj, ItomSharedSemaphore *waitCond)
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
ito::RetVal Ximea::copyVal(void *vpdObj, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);
    ito::DataObject *dObj = reinterpret_cast<ito::DataObject *>(vpdObj);

    if(!dObj)
    {
        retValue += ito::RetVal(ito::retError, 0, tr("Empty object handle retrieved from caller").toLatin1().data());
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
void Ximea::updateParameters(QMap<QString, ito::Param> params)
{
    int bitppix_old = 0;
    int binning_old = 0;
    int bitppix_new = 0;
    int binning_new = 0;
    double offset_new = 0.0;
    double value = 0.0;

    char name[40]={0};

    bitppix_old = m_params["bpp"].getVal<int>();
    binning_old = m_params["binning"].getVal<int>();

    foreach(const ito::Param &param1, params)
    {
        memset(name,0,sizeof(name));
        sprintf(name,"%s", param1.getName());
        if(!strlen(name))
            continue;
        QMap<QString, ito::Param>::iterator paramIt = m_params.find(name);
        if (paramIt != m_params.end() && paramIt->isNumeric())
        {
            value = param1.getVal<double>();
            if((value <= paramIt->getMax()) || (value >= paramIt->getMin()))
            {
                paramIt.value().setVal<double>(value);
            }
        }
    }

    bitppix_new = m_params["bpp"].getVal<int>();
    binning_new = m_params["binning"].getVal<int>();
    offset_new = m_params["offset"].getVal<double>();

    if (bitppix_new != bitppix_old)
    {
        setParam(QSharedPointer<ito::ParamBase>(new ito::ParamBase("bpp", ito::ParamBase::Int, bitppix_new)), NULL);
    }
    else if (binning_new != binning_old)
    {
        setParam(QSharedPointer<ito::ParamBase>(new ito::ParamBase("binning", ito::ParamBase::Int, binning_new)), NULL);
    }
    else
    {
        setParam(QSharedPointer<ito::ParamBase>(new ito::ParamBase("offset", ito::ParamBase::Double, offset_new)), NULL);
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------------------------------------
//! slot invoked if gain parameters in docking toolbox have been manually changed
/*!
    \param [in] gain
    \param [in] offset
*/
void Ximea::GainPropertiesChanged(double gain)
{
    if( gain <= m_params["gain"].getMax() &&
        gain >= m_params["gain"].getMin())
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
void Ximea::OffsetPropertiesChanged(double offset)
{
    if( offset <= m_params["offset"].getMax() &&
        offset >= m_params["offset"].getMin())
    {
        setParam( QSharedPointer<ito::ParamBase>(new ito::ParamBase("offset", m_params["offset"].getType(), offset)));
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
//ito::RetVal Ximea::adjustROIMeta(bool horizontalNotVertical)
//{
//	if (horizontalNotVertical)
//	{
//	}
//	else
//	{
//		//obtain current offsetY and height values
//		//in order to get the full offset and height range, we first need to resize the height to the minimum value
//		//then get the offset ranges and finally reset the height to the previous value.
//		ret = pxiGetParam(m_handle, XI_PRM_HEIGHT, &size, &intSize, &intType);
//		ret = pxiGetParam(m_handle, XI_PRM_HEIGHT XI_PRM_INFO_MIN, &sizeMin, &intSize, &intType);
//		ret = pxiGetParam(m_handle, XI_PRM_HEIGHT XI_PRM_INFO_MAX, &sizeMax, &intSize, &intType);
//		ret = pxiGetParam(m_handle, XI_PRM_HEIGHT XI_PRM_INFO_INCREMENT, &sizeInc, &intSize, &intType);
//		if (sizeInc == 0)
//		{
//			sizeInc = sizeMax - sizeMin;
//		}
//
//		ret = pxiSetParam(m_handle, XI_PRM_HEIGHT, &sizeMin, intSize, intType); //temporary reset
//		ret = pxiGetParam(m_handle, XI_PRM_OFFSET_Y, &offset, &intSize, &intType);
//		ret = pxiGetParam(m_handle, XI_PRM_OFFSET_Y XI_PRM_INFO_MIN, &offsetMin, &intSize, &intType);
//		ret = pxiGetParam(m_handle, XI_PRM_OFFSET_Y XI_PRM_INFO_MAX, &offsetMax, &intSize, &intType);
//		ret = pxiGetParam(m_handle, XI_PRM_OFFSET_Y XI_PRM_INFO_INCREMENT, &offsetInc, &intSize, &intType);
//		if (offsetInc == 0)
//		{
//			offsetInc = offsetMax - offsetMin;
//		}
//		ret = pxiSetParam(m_handle, XI_PRM_HEIGHT, &size, intSize, intType); //reset to previous value
//
//		m_params["y0"].setVal<int>(offset);
//		m_params["y0"].setMeta(new ito::IntMeta(offsetMin, offsetMax, offsetInc), true);
//		m_params["y1"].setVal<int>(offset + size - 1);
//		m_params["y1"].setMeta(new ito::IntMeta(offset + sizeMin - 1, sizeMax - 1, sizeInc), true);
//		m_params["sizey"].setVal<int>(size);
//		m_params["sizey"].setMeta(new ito::IntMeta(sizeMin, sizeMax, sizeInc), true); 
//	}
//}
//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal Ximea::execFunc(const QString funcName, QSharedPointer<QVector<ito::ParamBase> > paramsMand, QSharedPointer<QVector<ito::ParamBase> > paramsOpt, QSharedPointer<QVector<ito::ParamBase> > /*paramsOut*/, ItomSharedSemaphore *waitCond)
{
    ito::RetVal retValue = ito::retOk;
    ito::ParamBase *param1 = NULL;
    ito::ParamBase *param2 = NULL;
    ito::ParamBase *param3 = NULL;
    ito::ParamBase *param4 = NULL;
    /*
    if(m_pvShadingSettings == NULL)
    {
        m_pvShadingSettings = (void*) new MMSHADING();
        ((LPMMSHADING)m_pvShadingSettings)->shCX = 0;
        ((LPMMSHADING)m_pvShadingSettings)->shCX = 0;
        ((LPMMSHADING)m_pvShadingSettings)->lpMul = NULL;
        ((LPMMSHADING)m_pvShadingSettings)->lpSub = NULL;
    }

    LPMMSHADING shading = (LPMMSHADING)m_pvShadingSettings;
    */
    
    int illm = 0;

    if (funcName == "updateShading")
    {    
        param1 = ito::getParamByName(&(*paramsMand), "illumination", &retValue);

        if (!retValue.containsError())
        {
            illm = param1->getVal<int>();
            updateShadingCorrection(illm);
        }
    }
    else if (funcName == "shadingCorrectionValues")
    {    
        param1 = ito::getParamByName(&(*paramsMand), "integration_time", &retValue);
        param2 = ito::getParamByName(&(*paramsMand), "shadingCorrectionFaktor", &retValue);
        int intTime = 0;
        if (!retValue.containsError())
        {
            intTime = (int)(param1->getVal<double>() * 1000);
            if(param2->getLen() < 20)
            {
                retValue += ito::RetVal(ito::retError, 1, tr("Fill shading correction factor failed").toLatin1().data());
            }

            
        }

        if (!retValue.containsError())
        {
            QVector<QPointF> newVals(10);
            double* dptr = param2->getVal<double*>();
            newVals[0] = QPointF(0.0, 1.0);
            for(int i = 1; i < 10; i ++)
            {
                newVals[i].setX(dptr[i*2]);
                newVals[i].setY(dptr[i*2+1]);
            }
            m_shading.m_correction.insert(intTime, newVals);
        }

        
    }
    else if (funcName == "initializeShading")
    {

        param1 = ito::getParamByName(&(*paramsMand), "darkImage", &retValue);
        param2 = ito::getParamByName(&(*paramsMand), "whiteImage", &retValue);              

        param3 = ito::getParamByName(&(*paramsMand), "x0", &retValue);
        param4 = ito::getParamByName(&(*paramsMand), "y0", &retValue);

        int xsize = m_params["sizex"].getVal<int>();
        int ysize = m_params["sizey"].getVal<int>();

        if (!retValue.containsError())
        {
            int x0 = param3->getVal<int>();
            int y0 = param4->getVal<int>();

            ito::DataObject* darkObj = (ito::DataObject*)(param1->getVal<void*>());
            ito::DataObject* whiteObj = (ito::DataObject*)(param2->getVal<void*>());
            if(darkObj == NULL || whiteObj == NULL )
            {
                m_shading.valid = false;
                m_shading.active = false;
            }
            else if(darkObj->getType() != ito::tUInt16 || darkObj->getDims() != 2 || (darkObj->getSize(0) + y0) > ysize || (darkObj->getSize(1) + x0) > xsize)
            {
                m_shading.valid = false;
                m_shading.active = false;
            }
            else if(whiteObj->getType() != ito::tUInt16 || whiteObj->getDims() != 2 || (whiteObj->getSize(0) + y0)> ysize || (whiteObj->getSize(1) + x0) > xsize)
            {
                m_shading.valid = false;
                m_shading.active = false;
            }
            else if(whiteObj->getSize(0) != darkObj->getSize(0) || whiteObj->getSize(1) != darkObj->getSize(1))
            {
                m_shading.valid = false;
                m_shading.active = false;
            }
            else
            {
                m_shading.valid = true;
                m_shading.active = true;
                if(m_shading.mul != NULL) delete m_shading.mul;
                if(m_shading.sub != NULL) delete m_shading.sub;
                if(m_shading.subBase != NULL) delete m_shading.subBase;
                if(m_shading.mulBase != NULL) delete m_shading.mulBase;

                m_shading.mul = new ito::uint16[whiteObj->getSize(0)*whiteObj->getSize(1)];
                m_shading.sub = new ito::uint16[whiteObj->getSize(0)*whiteObj->getSize(1)];
                m_shading.mulBase = new ito::uint16[whiteObj->getSize(0)*whiteObj->getSize(1)];
                m_shading.subBase = new ito::uint16[whiteObj->getSize(0)*whiteObj->getSize(1)];
                m_shading.x0 = x0;
                m_shading.y0 = y0;
                m_shading.xsize = whiteObj->getSize(1);
                m_shading.ysize = whiteObj->getSize(0);
                for(int y = 0; y < m_shading.ysize; y++)
                {
                    ito::uint16* darkPtr = (ito::uint16*)(darkObj->rowPtr(0, y));
                    ito::uint16* whitePtr = (ito::uint16*)(whiteObj->rowPtr(0, y));
                    for(int x = 0; x < m_shading.xsize; x++)
                    {
                        
                        m_shading.subBase[y*m_shading.xsize + x] = whitePtr[x];
                        m_shading.mulBase[y*m_shading.xsize + x] = darkPtr[x];
                    }
                }
                updateShadingCorrection(0);
            }
        }
        /*
        if (!retValue.containsError())
        {
            int ret = pInitializeShading(m_handle, shading, this->m_params["sizex"].getVal<int>(), this->m_params["sizey"].getVal<int>(), param1->getVal<int>(), param2->getVal<int>());
            if(ret != MM40_OK)
            {
                retValue += ito::RetVal(ito::retError, ret, tr("mmInitializeShading failed").toLatin1().data());
            }
            else
            {
            
                //ret = pSetShadingRaw(shading);
                //if(ret != MM40_OK)
                //{
                //    retValue += ito::RetVal(ito::retError, ret, tr("mmSetShadingRaw failed").toLatin1().data());
                //}
                
            }
        }
        */
    }
/*
    else if (funcName == "updateShading")
    {    
        param1 = ito::getParamByName(&(*paramsOpt), "darkImage", &retValue);
        param2 = ito::getParamByName(&(*paramsOpt), "whiteImage", &retValue);       

        if (!retValue.containsError())
        {
            int ret = pUpdateFrameShading(m_handle, NULL, shading);
            if(ret != MM40_OK)
            {
                retValue += ito::RetVal(ito::retError, ret, tr("pUpdateFrameShading failed").toLatin1().data());
            }
            
        }
    }
    else if (funcName == "calculateShading")
    {    
        param1 = ito::getParamByName(&(*paramsOpt), "darkImage", &retValue);
        param2 = ito::getParamByName(&(*paramsOpt), "whiteImage", &retValue);              

        cv::Mat_<WORD> darkMat = cv::Mat_<WORD>::zeros(cv::Size(this->m_params["sizex"].getVal<int>(), this->m_params["sizey"].getVal<int>()));
        cv::Mat_<WORD> whiteMat = cv::Mat_<WORD>::ones(cv::Size(this->m_params["sizex"].getVal<int>(), this->m_params["sizey"].getVal<int>()));

        ito::DataObject* darkObj = (ito::DataObject*)(param1->getVal<void*>());
        ito::DataObject* whiteObj = (ito::DataObject*)(param2->getVal<void*>());

        if(darkObj && darkObj->getType() == ito::tUInt16 && darkObj->getDims() == 2 && darkObj->getSize(0) == darkMat.rows && darkObj->getSize(1) == darkMat.cols)
        {
            for(int y = 0; y < darkMat.rows; y++)
            {
                memcpy(darkMat.ptr(y), darkObj->rowPtr(0,y), 2 * darkMat.cols);
            }
        }
        if(whiteObj && whiteObj->getType() == ito::tUInt16 && whiteObj->getDims() == 2 && whiteObj->getSize(0) == whiteMat.rows && whiteObj->getSize(1) == whiteMat.cols)
        {
            for(int y = 0; y < darkMat.rows; y++)
            {
                memcpy(whiteMat.ptr(y), whiteObj->rowPtr(0,y), 2 * whiteMat.cols);
            }
        }

        LPWORD pBlack = darkMat.ptr<WORD>();
        LPWORD pWhite = whiteMat.ptr<WORD>();

        if (!retValue.containsError())
        {
            int ret = pCalculateShading(m_handle, shading, this->m_params["sizex"].getVal<int>(), this->m_params["sizey"].getVal<int>(), pBlack, pWhite); 
            //int ret = pCalculateShadingRaw(shading, this->m_params["sizex"].getVal<int>(), this->m_params["sizey"].getVal<int>(), pBlack, pWhite); 
            if(ret != MM40_OK)
            {
                retValue += ito::RetVal(ito::retError, ret, tr("pCalculateShading failed").toLatin1().data());
            }
            else
            {
            }
            
        }
    }
*/
    else
    {
        retValue += ito::RetVal(ito::retError, 0, tr("function name '%1' does not exist").arg(funcName.toLatin1().data()).toLatin1().data());
    }

    if (waitCond)
    {
        waitCond->returnValue = retValue;
        waitCond->release();
        waitCond->deleteSemaphore();
    }

    return retValue;
}
//----------------------------------------------------------------------------------------------------------------------------------
void Ximea::updateShadingCorrection(int value)
{

    QPointF correction(0.0, 1.0);

    if(!m_shading.valid)
        return;
    value = value < 0 ? 0 : value > 9 ? 9 : value;
    int intTime = (int)(m_params["integration_time"].getVal<double>() * 1000);
    if(m_shading.m_correction.contains(intTime) && m_shading.m_correction[intTime].size() > value)
        correction = m_shading.m_correction[intTime][value];

    float x = correction.x();
    float y = correction.y();
    for(int px = 0; px < m_shading.ysize * m_shading.xsize; px++)
    {
        m_shading.sub[px] = m_shading.subBase[px] * x;
        m_shading.mul[px] = 1.0;
    }

}
//----------------------------------------------------------------------------------------------------------------------------------
void Ximea::activateShadingCorrection(bool enable)
{
    if(!m_shading.valid)
    {
        m_shading.active = false;
        return;
    }
    m_shading.active = enable;
    return;
}

//----------------------------------------------------------------------------------------------------------------------------------
const ito::RetVal Ximea::showConfDialog(void)
{
    //ito::RetVal retValue(ito::retOk);

    //dialogXimea *confDialog = new dialogXimea(this);
	return apiShowConfigurationDialog(this, new dialogXimea(this));
    
	/*
	
	
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
	*/
    
}

