/* ********************************************************************
    Plugin "FireGrabber" for itom software
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

#include "FireGrabber.h"

//#include "stdafx.h"

#ifdef WIN32
    #include <stdio.h>
    #include <conio.h>
#endif

#include <qstring.h>
#include <qstringlist.h>
#include <QtCore/QtPlugin>

#include <qmetaobject.h>
#include <qdockwidget.h>
#include <qpushbutton.h>
#include "dockWidgetFireGrabber.h"

#include "pluginVersion.h"
#include "gitVersion.h"

Q_DECLARE_METATYPE(ito::DataObject)

int FireGrabber::m_numberOfInstances = 0;  // initialization
    
//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal FireGrabberInterface::getAddInInst(ito::AddInBase **addInInst)
{
    NEW_PLUGININSTANCE(FireGrabber)
    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal FireGrabberInterface::closeThisInst(ito::AddInBase **addInInst)
{
   REMOVE_PLUGININSTANCE(FireGrabber)
   return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
FireGrabberInterface::FireGrabberInterface()
{
    m_type = ito::typeDataIO | ito::typeGrabber;
    setObjectName("FireGrabber"); 

    //for the docstring, please don't set any spaces at the beginning of the line.
/*    char* docstring = \
"Camera plugin that uses the FireGrab driver from the AVT FirePackage in order to communicate with corresponding cameras. The cameras are connected to the computer via \
firewire. \n\
\n\
This plugin can only be loaded and used once the AVT FirePackage driver has been correctly installed on your computer. For more information about AVT FirePackage and their \
license browse to http://www.alliedvisiontec.com. This plugin was mainly tested with the cameras AVT Malin, Guppy and Pike. Not all parameters are supported by this plugin.";
*/
#ifndef WIN32
    m_description = QObject::tr("FireForLinux (Firewire)");
#else
    m_description = QObject::tr("Fire Package Capture (Firewire)");
#endif
//    m_detaildescription = QObject::tr(docstring);
    m_detaildescription = QObject::tr(
"Camera plugin that uses the FireGrab driver from the AVT FirePackage in order to communicate with corresponding cameras. The cameras are connected to the computer via \
firewire. \n\
\n\
This plugin can only be loaded and used once the AVT FirePackage driver has been correctly installed on your computer. For more information about AVT FirePackage and their \
license browse to http://www.alliedvisiontec.com. This plugin was mainly tested with the cameras AVT Malin, Guppy and Pike. Not all parameters are supported by this plugin.");
#ifndef WIN32
    m_author = "G. Baer, M. Gronle, ITO, University Stuttgart";
#else
    m_author = "A. Bielke, M. Gronle, ITO, University Stuttgart";
#endif
    m_version = (PLUGIN_VERSION_MAJOR << 16) + (PLUGIN_VERSION_MINOR << 8) + PLUGIN_VERSION_PATCH;
    m_minItomVer = MINVERSION;
    m_maxItomVer = MAXVERSION;

#ifndef WIN32
    m_license = QObject::tr("LGPL; you need an installed fireForLinux driver, which requires further licenses if you are not using any AVT camera (see fireForLinux documentation).");
#else
    m_license = QObject::tr("LGPL; you need an installed AVT FirePackage driver, which requires further licenses if you are not using any AVT camera (see AVT FirePackage documentation).");
#endif
    m_aboutThis = QObject::tr(GITVERSION);
    
    m_autoLoadPolicy = ito::autoLoadNever;
    m_autoSavePolicy = ito::autoSaveNever;

    // TODO: break if camera is already connected ( no multiple connections allowed.
    // TODO: implement other selection possibilities.. (see windows implementation.
    // TODO: check if code runs for multiple cameras at same time. (not done yet)

#ifdef WIN32
    m_initParamsOpt.append(ito::Param("cameraID", ito::ParamBase::Int | ito::ParamBase::In, 0, std::numeric_limits<int>::max(), 0, tr("specific number of the camera, don't use with cameraNumber (0 = unused) - this parameter does not exist in linux version.").toLatin1().data()));
    m_initParamsOpt.append(ito::Param("vendorID", ito::ParamBase::Int | ito::ParamBase::In, 0, std::numeric_limits<int>::max(), 0, tr("number of the vendor (e.g. Allied: vendorID=673537), don't use with cameraNumber (0 = unused) - this parameter does not exist in linux version.").toLatin1().data()));
#endif
    m_initParamsOpt.append(ito::Param("cameraNumber", ito::ParamBase::Int | ito::ParamBase::In, 0, 25, 0, tr("number of order of plugging the cameras (0 = parameter not used)").toLatin1().data()));

    m_initParamsOpt.append(ito::Param("timebase", ito::ParamBase::Int | ito::ParamBase::In, 1, 1000, 20, tr("timebase [in _s] as step width of integration time. This mainly influences the allowed range of the integration time (only AVT cameras). Values: 1, 2, 5, 10, 20 [default], 50, 100, 200, 500, 1000").replace("_", QLatin1String("\u00B5")).toLatin1().data()));
}

//----------------------------------------------------------------------------------------------------------------------------------
FireGrabberInterface::~FireGrabberInterface()
{
    m_initParamsMand.clear();
    m_initParamsOpt.clear();
}

//----------------------------------------------------------------------------------------------------------------------------------


//----------------------------------------------------------------------------------------------------------------------------------
const ito::RetVal FireGrabber::showConfDialog(void)
{
	return apiShowConfigurationDialog(this, new DialogFireGrabber(this));
}

//----------------------------------------------------------------------------------------------------------------------------------
FireGrabber::FireGrabber() : 
    AddInGrabber()
{
   ito::Param paramVal("name", ito::ParamBase::String | ito::ParamBase::Readonly, "FireGrabber", NULL);
   m_params.insert(paramVal.getName(), paramVal);

   paramVal = ito::Param("vendorID", ito::ParamBase::Int | ito::ParamBase::Readonly, 0, std::numeric_limits<int>::max(), 0, tr("vendor ID of the camera").toLatin1().data());
   m_params.insert(paramVal.getName(), paramVal);
   paramVal = ito::Param("cameraID", ito::ParamBase::Int | ito::ParamBase::Readonly, 0, std::numeric_limits<int>::max(), 0, tr("camera ID of the camera").toLatin1().data());
   m_params.insert(paramVal.getName(), paramVal);

   paramVal = ito::Param("vendorName", ito::ParamBase::String | ito::ParamBase::Readonly, NULL, tr("vendor name of the camera [if connected]").toLatin1().data());
   m_params.insert(paramVal.getName(), paramVal);

   paramVal = ito::Param("modelName", ito::ParamBase::String | ito::ParamBase::Readonly, NULL, tr("model name of the camera [if connected]").toLatin1().data());
   m_params.insert(paramVal.getName(), paramVal);

   paramVal = ito::Param("integration_time", ito::ParamBase::Double, 0.001, 5000.0, 0.01, tr("Integrationtime of CCD programmed in s").toLatin1().data());
   m_params.insert(paramVal.getName(), paramVal);
   paramVal = ito::Param("frame_time", ito::ParamBase::Double | ito::ParamBase::Readonly, 0.005, 150.0, 33.333333, tr("Transmission time per frame in s").toLatin1().data());
   m_params.insert(paramVal.getName(), paramVal);
   
   paramVal = ito::Param("bpp", ito::ParamBase::Int, 8, 24, 8, tr("bit depth of camera").toLatin1().data());
   m_params.insert(paramVal.getName(), paramVal);

   paramVal = ito::Param("timebase", ito::ParamBase::Int | ito::ParamBase::Readonly, 1, 1000, 20, tr("timebase (step width of integration_time) in mus").replace("mu", QLatin1String("\u00B5")).toLatin1().data());
   m_params.insert(paramVal.getName(), paramVal);

   paramVal = ito::Param("brightness", ito::ParamBase::Double, 0.0, 1.0, 0.0, tr("Brightness value (if supported)").toLatin1().data());
   m_params.insert(paramVal.getName(), paramVal);
   paramVal = ito::Param("sharpness", ito::ParamBase::Double, 0.0, 1.0, 0.0, tr("Sharpness value (if supported)").toLatin1().data());
   m_params.insert(paramVal.getName(), paramVal);
   paramVal = ito::Param("gamma", ito::ParamBase::Int, 0, 1, 0, tr("Gamma correction (0: off, 1: on, default: off)").toLatin1().data());
   m_params.insert(paramVal.getName(), paramVal);
   paramVal = ito::Param("gain", ito::ParamBase::Double, 0.0, 1.0, 0.0, tr("Virtual gain").toLatin1().data());
   m_params.insert(paramVal.getName(), paramVal);
   paramVal = ito::Param("offset", ito::ParamBase::Double | ito::ParamBase::Readonly, 0.0, 0.0, 0.0, tr("Offset not used here.").toLatin1().data());
   m_params.insert(paramVal.getName(), paramVal);

   int roi[] = { 0, 0, 2048, 2048 };
   paramVal = ito::Param("roi", ito::ParamBase::IntArray, 4, roi, tr("ROI (x,y,width,height) [this replaces the values x0,x1,y0,y1]").toLatin1().data());
   ito::RectMeta *rm = new ito::RectMeta(ito::RangeMeta(roi[0], roi[2] - 1), ito::RangeMeta(roi[1], roi[3] - 1)); //RangeMeta includes the last value, therefore -1
   paramVal.setMeta(rm, true);
   m_params.insert(paramVal.getName(), paramVal);

   paramVal = ito::Param("x0", ito::ParamBase::Int, 0, 2047, 0, tr("Startvalue for ROI").toLatin1().data());
   m_params.insert(paramVal.getName(), paramVal);
   paramVal = ito::Param("y0", ito::ParamBase::Int, 0, 2047, 0, tr("Startvalue for ROI").toLatin1().data());
   m_params.insert(paramVal.getName(), paramVal);

   paramVal = ito::Param("x1", ito::ParamBase::Int, 0, 2047, 2047, tr("Stopvalue for ROI").toLatin1().data());
   m_params.insert(paramVal.getName(), paramVal);
   paramVal = ito::Param("y1", ito::ParamBase::Int, 0, 2047, 2047, tr("Stopvalue for ROI").toLatin1().data());
   m_params.insert(paramVal.getName(), paramVal);

   paramVal = ito::Param("sizex", ito::ParamBase::Int | ito::ParamBase::Readonly, 1, 2048, 2048, tr("Pixelsize in x (cols)").toLatin1().data());
   m_params.insert(paramVal.getName(), paramVal);
   paramVal = ito::Param("sizey", ito::ParamBase::Int | ito::ParamBase::Readonly, 1, 2048, 2048, tr("Pixelsize in y (rows)").toLatin1().data());
   m_params.insert(paramVal.getName(), paramVal);
   
   //paramVal = ito::Param("fps", ito::ParamBase::Int | ito::ParamBase::Readonly, 0, 100, 0, tr("Read frames per second").toLatin1().data());
   //m_params.insert(paramVal.getName(), paramVal);

   if (hasGuiSupport())
   {
       //now create dock widget for this plugin
       DockWidgetFireGrabber *dw = new DockWidgetFireGrabber(this);
       Qt::DockWidgetAreas areas = Qt::AllDockWidgetAreas;
       QDockWidget::DockWidgetFeatures features = QDockWidget::DockWidgetClosable | QDockWidget::DockWidgetFloatable | QDockWidget::DockWidgetMovable;
       createDockWidget(QString(m_params["name"].getVal<char *>()), features, areas, dw);
   }
}

//----------------------------------------------------------------------------------------------------------------------------------
FireGrabber::~FireGrabber()
{
   m_params.clear();
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal FireGrabber::AlliedChkError(int errornumber)
{
    ito::RetVal retValue = ito::retOk;
    int i;

    struct error
    {
        int value;
        const char* text;
    } 

#ifdef WIN32 // for windos firepackage
    static const errors[] =
    {    /* All Errormassages are taken from the Fire Grab-Manual. */     
        { FCE_NOERROR          /*   0  */,   "No Error"},
        { HALER_NOCARD            /* 1  */,  "Card is not present"},
        { HALER_NONTDEVICE        /* 2  */,  "No logical Device"},
        { HALER_NOMEM             /* 3  */,  "Not enough memory"},
        { HALER_MODE              /* 4  */,  "Not allowed in this mode"},
        { HALER_TIMEOUT           /* 5  */,  "Timeout"},
        { HALER_ALREADYSTARTED    /* 6  */,  "Something is started"},
        { HALER_NOTSTARTED        /* 7  */,  "Not started"},
        { HALER_BUSY              /* 8  */,  "Busy at the moment"},
        { HALER_NORESOURCES       /* 9  */,  "No resources available"},
        { HALER_NODATA           /* 10  */,  "No data available"},
        { HALER_NOACK            /* 11  */,  "Didn't get acknowledge"},
        { HALER_NOIRQ            /* 12  */,  "Interruptinstallerror"},
        { HALER_NOBUSRESET       /* 13  */,  "Error waiting for busreset"},
        { HALER_NOLICENSE        /* 14  */,  "No license"},
        { HALER_RCODEOTHER       /* 15  */,  "RCode not RCODE_COMPLETE"},
        { HALER_PENDING          /* 16  */,  "Something still pending"},
        { HALER_INPARMS          /* 17  */,  "Input parameter range"},
        { HALER_CHIPVERSION      /* 18  */,  "Unrecognized chipversion"},
        { HALER_HARDWARE         /* 19  */,  "Hardware error"},
        { HALER_NOTIMPLEMENTED   /* 20  */,  "Not implemented"},
        { HALER_CANCELLED        /* 21  */,  "Cancelled"},
        { HALER_NOTLOCKED        /* 22  */,  "Memory is not locked"},
        { HALER_GENERATIONCNT    /* 23  */,  "Bus reset in between"},
        { HALER_NOISOMANAGER     /* 24  */,  "No IsoManager present"},
        { HALER_NOBUSMANAGER     /* 25  */,  "No BusManager present"},
        { HALER_UNEXPECTED       /* 26  */,  "Unexpected value"},
        { HALER_REMOVED          /* 27  */,  "Target was removed"},
        { HALER_NOBUSRESOURCES   /* 28  */,  "No ISO resources available"},
        { HALER_DMAHALTED        /* 29  */,  "DMA halted"},
        { FCE_ALREADYOPENED      /* 1001*/,  "Something already opened"},
        { FCE_NOTOPENED          /* 1002*/,  "Need open before"},
        { FCE_NODETAILS          /* 1003*/,  "No details"},
        { FCE_DRVNOTINSTALLED    /* 1004*/,  "Driver not installed"},
        { FCE_MISSINGBUFFERS     /* 1005*/,  "Don't have buffers"},
        { FCE_INPARMS            /* 1006*/,  "Parameter error"},
        { FCE_CREATEDEVICE       /* 1007*/,  "Error creating WinDevice"},
        { FCE_WINERROR           /* 1008*/,  "Internal Windows error"},
        { FCE_IOCTL              /* 1009*/,  "Error DevIoCtl"},
        { FCE_DRVRETURNLENGTH    /* 1010*/,  "Wrong length return data"},
        { FCE_INVALIDHANDLE      /* 1011*/,  "Wrong handle"},
        { FCE_NOTIMPLEMENTED     /* 1012*/,  "Function not implemented"},
        { FCE_DRVRUNNING         /* 1013*/,  "Driver runs already"},
        { FCE_STARTERROR         /* 1014*/,  "Couldn't start"},
        { FCE_INSTALLERROR       /* 1015*/,  "Installation error"},
        { FCE_DRVVERSION         /* 1016*/,  "Driver has wrong version"},
        { FCE_NODEADDRESS        /* 1017*/,  "Wrong nodeaddress"},
        { FCE_PARTIAL            /* 1018*/,  "Partial info. copied"},
        { FCE_NOMEM              /* 1019*/,  "No memory"},
        { FCE_NOTAVAILABLE       /* 1020*/,  "Requested function not available"},
        { FCE_NOTCONNECTED       /* 1021*/,  "Not connected to target"},
        { FCE_ADJUSTED           /* 1022*/,  "A parameter had to be adjusted"},
    };

#else // for fire4linux
    static const errors[] =
    {
          { DC1394_SUCCESS                    /* =  0, */, "No Error"},
          { DC1394_FAILURE                    /* = -1, */, "Fail"},
          { DC1394_NOT_A_CAMERA               /* = -2, */, "Not a Camera"},
          { DC1394_FUNCTION_NOT_SUPPORTED     /* = -3, */, "Function not supported"},
          { DC1394_CAMERA_NOT_INITIALIZED     /* = -4, */, "Camera not initialized"},
          { DC1394_MEMORY_ALLOCATION_FAILURE  /* = -5, */, "Mem alloc fail"},
          { DC1394_TAGGED_REGISTER_NOT_FOUND  /* = -6, */, "Tagged register not found"},
          { DC1394_NO_ISO_CHANNEL             /* = -7, */, "No ISO channel"},
          { DC1394_NO_BANDWIDTH               /* = -8, */, "No bandwidth"},
          { DC1394_IOCTL_FAILURE              /* = -9, */, "IOCTL fail"},
          { DC1394_CAPTURE_IS_NOT_SET         /* = -10, */, "Capture not set"},
          { DC1394_CAPTURE_IS_RUNNING         /* = -11, */, "Capture running"},
          { DC1394_RAW1394_FAILURE            /* = -12, */, "Raw1394 fail"},
          { DC1394_FORMAT7_ERROR_FLAG_1       /* = -13, */, "Format7 error flag 1"},
          { DC1394_FORMAT7_ERROR_FLAG_2       /* = -14, */, "Format7 error flag 2"},
          { DC1394_INVALID_ARGUMENT_VALUE     /* = -15, */, "Invalid argval"},
          { DC1394_REQ_VALUE_OUTSIDE_RANGE    /* = -16, */, "Value outside range"},
          { DC1394_INVALID_FEATURE            /* = -17, */, "Invalid feature"},
          { DC1394_INVALID_VIDEO_FORMAT       /* = -18, */, "Invalid video format"},
          { DC1394_INVALID_VIDEO_MODE         /* = -19, */, "Invalid video mode"},
          { DC1394_INVALID_FRAMERATE          /* = -20, */, "Invalid framerate"},
          { DC1394_INVALID_TRIGGER_MODE       /* = -21, */, "Invalid trigger mode"},
          { DC1394_INVALID_TRIGGER_SOURCE     /* = -22, */, "Invalid trigger source"},
          { DC1394_INVALID_ISO_SPEED          /* = -23, */, "Invalid ISO speed"},
          { DC1394_INVALID_IIDC_VERSION       /* = -24, */, "Invalid IIDC version"},
          { DC1394_INVALID_COLOR_CODING       /* = -25, */, "Invalid color coding"},
          { DC1394_INVALID_COLOR_FILTER       /* = -26, */, "Invalid color filter"},
          { DC1394_INVALID_CAPTURE_POLICY     /* = -27, */, "Invalid capture policy"},
          { DC1394_INVALID_ERROR_CODE         /* = -28, */, "Invalid error code"},
          { DC1394_INVALID_BAYER_METHOD       /* = -29, */, "Invalid bayering"},
          { DC1394_INVALID_VIDEO1394_DEVICE   /* = -30, */, "Invalid video1394 device"},
          { DC1394_INVALID_OPERATION_MODE     /* = -31, */, "Invalid operation mode"},
          { DC1394_INVALID_TRIGGER_POLARITY   /* = -32, */, "Invalid trigger polarity"},
          { DC1394_INVALID_FEATURE_MODE       /* = -33, */, "Invalid feature mode"},
          { DC1394_INVALID_LOG_TYPE           /* = -34, */, "Invalid log type"},
          { DC1394_INVALID_BYTE_ORDER         /* = -35, */, "Invalid byte order"},
          { DC1394_INVALID_STEREO_METHOD      /* = -36, */, "Invalid stereo method"},
          { DC1394_BASLER_NO_MORE_SFF_CHUNKS  /* = -37, */, "Basler no more SFF chucks"},
          { DC1394_BASLER_CORRUPTED_SFF_CHUNK /* = -38, */, "Basler corrupted SFF chuck"},
          { DC1394_BASLER_UNKNOWN_SFF_CHUNK   /* = -39  */, "Basler unknown SFF chuck"},
    };
#endif
    if (errornumber > 0)
    {
        bool found = false;

        for (i = 0; i < sizeof(errors) / sizeof(errors[0]); i++)
        {
            if (errors[i].value == errornumber)
            {
                retValue += ito::RetVal::format(ito::retError, 0, tr(&errors[i].text[0]).toLatin1().data());
                found = true;
            }
        }

        if (!found)
        {
            retValue += ito::RetVal(ito::retError, 0, tr("Unknown Error Code").toLatin1().data());
        }
    }
        
    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
// Funktion to set and update data imformations
//----------------------------------------------------------------------------------------------------------------------------------
/*!
    \details This method copies the complete tparam of the corresponding parameter to val

    \param [in,out] val  is a input of type::tparam containing name, value and further informations
    \param [in] waitCond is the semaphore (default: NULL), which is released if this method has been terminated
    \return retOk in case that everything is ok, else retError
    \sa ito::tParam, ItomSharedSemaphore
*/
ito::RetVal FireGrabber::getParam(QSharedPointer<ito::Param> val, ItomSharedSemaphore *waitCond)
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
        //put your switch-case.. for getting the right value here

        //finally, save the desired value in the argument val (this is a shared pointer!)
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
ito::RetVal FireGrabber::setParam(QSharedPointer<ito::ParamBase> val, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);
    QString key;
    bool hasIndex;
    int index;
    QString suffix;
    ParamMapIterator it;

#ifdef WIN32
    UINT32 Result = 0;
#else
    uint32_t min, max, value;
    dc1394error_t Result = dc1394error_t(0);
#endif
    // parse the given parameter-name (if you support indexed or suffix-based parameters)
    retValue += apiParseParamName( val->getName(), key, hasIndex, index, suffix );

    if (!retValue.containsError())
    {
        // gets the parameter key from m_params map (read-only is not allowed and leads to ito::retError).
        retValue += apiGetParamFromMapByKey(m_params, key, it, true);
    }

    if (!retValue.containsError())
    {
        // here the new parameter is checked whether it's type corresponds or can be cast into the
        // value in m_params and whether the new type fits to the requirements of any possible
        // meta structure.
        retValue += apiValidateAndCastParam(*it, *val, false, true, true);
    }

    if (!retValue.containsError())
    {
        if ((!key.compare("x1") || !key.compare("y1") || !key.compare("x0") || !key.compare("y0"))) //adjust region of intrest
        {
            if (m_isgrabbing == true)
            {
                retValue += ito::RetVal(ito::retError, 0, tr("camera is grabbing, stop camera first").toLatin1().data());
            }
            else
            {
                int x0temp = key.compare("x0") == 0 ? val->getVal<int>() : m_params["x0"].getVal<int>();
                int x1temp = key.compare("x1") == 0 ? val->getVal<int>() : m_params["x1"].getVal<int>();
                int y0temp = key.compare("y0") == 0 ? val->getVal<int>() : m_params["y0"].getVal<int>();
                int y1temp = key.compare("y1") == 0 ? val->getVal<int>() : m_params["y1"].getVal<int>();
                retValue += adjustROI(x0temp, x1temp, y0temp, y1temp);
            }
        }
		else if (key == "roi")
		{
			int x0temp = m_params["x0"].getVal<int>();
			int x1temp = m_params["x1"].getVal<int>();
			int y0temp = m_params["y0"].getVal<int>();
			int y1temp = m_params["y1"].getVal<int>();

			if (!hasIndex)
			{
				const int* roi = val->getVal<const int*>();
				x0temp = roi[0];
				y0temp = roi[1];
				x1temp = roi[0] + roi[2] - 1;
				y1temp = roi[1] + roi[3] - 1;
			}
			else
			{
				switch (index)
				{
				case 0:
					x0temp = val->getVal<int>();
					break;
				case 1:
					y0temp = val->getVal<int>();
					break;
				case 2:
					x1temp = x0temp + val->getVal<int>() - 1;
					break;
				case 3:
					y1temp = y0temp + val->getVal<int>() - 1;
					break;
				}
			}

			retValue += adjustROI(x0temp, x1temp, y0temp, y1temp);
		}
        else if (!key.compare("integration_time")) // set integration time
        {
#ifdef WIN32
            unsigned long dblVal = (unsigned long)(exposureSecToShutter(val->getVal<double>())); //convert to camera specific value
            Result = Camera.SetParameter(FGP_SHUTTER, dblVal);
            retValue += AlliedChkError(Result);
#else
            uint32_t dblVal = (exposureSecToShutter(val->getVal<double>())); //convert to camera specific value
            retValue += AlliedChkError(dc1394_feature_set_value(camera,DC1394_FEATURE_SHUTTER, dblVal));
            retValue += AlliedChkError(Result);
#endif
        }
        else if (!key.compare("bpp")) // the bits per pixel (only 8 and 16bit monochromatic implemented yet.
        {
#ifdef WIN32
            Result = Camera.GetParameter(FGP_XSIZE, &m_xSize);
            Result = Camera.GetParameter(FGP_YSIZE, &m_ySize);
            if (val->getVal<int>() == 8)
            {
                Result = Camera.SetParameter(FGP_IMAGEFORMAT, MAKEIMAGEFORMAT(RES_SCALABLE, CM_Y8, 0));
            }
            else if (val->getVal<int>() == 16)
            {
                Result = Camera.SetParameter(FGP_IMAGEFORMAT, MAKEIMAGEFORMAT(RES_SCALABLE, CM_Y16, 0));
            }
            else
            {
                retValue += ito::RetVal(ito::retError, 0, tr("only 8 and 16 bit are implemented").toLatin1().data());
            }

            retValue += AlliedChkError(Result);

            if (!retValue.containsError())
            {
                Result = Camera.SetParameter(FGP_XSIZE, m_xSize);
                Result = Camera.SetParameter(FGP_YSIZE, m_ySize);
                retValue += AlliedChkError(Result);
            }
#else
            dc1394_get_image_size_from_video_mode(camera, video_mode, &m_xSize, &m_ySize);
            if (val->getVal<int>() == 8)
            {
                dc1394_format7_set_color_coding(camera, DC1394_VIDEO_MODE_FORMAT7_0, DC1394_COLOR_CODING_MONO8);

               // frame->color_coding = DC1394_COLOR_CODING_MONO8;
                coding = DC1394_COLOR_CODING_MONO8;

            }
            else if (val->getVal<int>() == 16)
            {
               // frame->color_coding = DC1394_COLOR_CODING_MONO16;
                dc1394_format7_set_color_coding(camera, DC1394_VIDEO_MODE_FORMAT7_0, DC1394_COLOR_CODING_MONO16);

                coding = DC1394_COLOR_CODING_MONO16;
            }
            else
            {
                retValue += ito::RetVal(ito::retError, 0, tr("only 8 and 16 bit are implemented").toLatin1().data());
            }

            retValue += AlliedChkError(Result);
#endif
        }
        else if (!key.compare("gain")) // set gain
        {
#ifdef WIN32
            FGPINFO valInfo = m_camProperties["gain"];
            unsigned long valnew = valInfo.MinValue + (valInfo.MaxValue - valInfo.MinValue) * val->getVal<double>();
            retValue += AlliedChkError(Camera.SetParameter(FGP_GAIN, valnew));
#else
            Result = dc1394_feature_get_boundaries(camera, DC1394_FEATURE_GAIN, &min, &max);
            retValue += AlliedChkError(Result);
            Result =  dc1394_feature_get_value(camera, DC1394_FEATURE_GAIN, &value);
            retValue += AlliedChkError(Result);
            unsigned long valnew = min + (max - min) * val->getVal<double>();
            retValue += AlliedChkError(dc1394_feature_set_value(camera, DC1394_FEATURE_GAIN,valnew));
#endif
        }
        else if (!key.compare("brightness")) //set brigtness
        {
#ifdef WIN32
            FGPINFO valInfo = m_camProperties["brightness"];
            unsigned long valnew = valInfo.MinValue + (valInfo.MaxValue - valInfo.MinValue) * val->getVal<double>();
            retValue += AlliedChkError(Camera.SetParameter(FGP_BRIGHTNESS, valnew));
#else
            Result = dc1394_feature_get_boundaries(camera, DC1394_FEATURE_BRIGHTNESS, &min, &max);
            retValue += AlliedChkError(Result);
            Result =  dc1394_feature_get_value(camera, DC1394_FEATURE_BRIGHTNESS, &value);
            retValue += AlliedChkError(Result);
            unsigned long valnew = min + (max - min) * val->getVal<double>();
            retValue += AlliedChkError(dc1394_feature_set_value(camera, DC1394_FEATURE_BRIGHTNESS,valnew));
#endif
        }
        else if (!key.compare("sharpness")) //set sharpness
        {
#ifdef WIN32
            FGPINFO valInfo = m_camProperties["sharpness"];
            unsigned long valnew = valInfo.MinValue + (valInfo.MaxValue - valInfo.MinValue) * val->getVal<double>();
            retValue += AlliedChkError(Camera.SetParameter(FGP_SHARPNESS, valnew));
#else
            Result = dc1394_feature_get_boundaries(camera, DC1394_FEATURE_SHARPNESS, &min, &max);
            retValue += AlliedChkError(Result);
            Result =  dc1394_feature_get_value(camera, DC1394_FEATURE_SHARPNESS, &value);
            retValue += AlliedChkError(Result);
            unsigned long valnew = min + (max - min) * val->getVal<double>();
            retValue += AlliedChkError(dc1394_feature_set_value(camera, DC1394_FEATURE_SHARPNESS,valnew));
#endif
        }
        else if (!key.compare("gamma")) //set gamma
        {
#ifdef WIN32
            retValue += AlliedChkError(Camera.SetParameter(FGP_GAMMA, it->getVal<int>()));
#else
            Result = dc1394_feature_get_boundaries(camera, DC1394_FEATURE_GAMMA, &min, &max);
            retValue += AlliedChkError(Result);
            Result =  dc1394_feature_get_value(camera, DC1394_FEATURE_GAMMA, &value);
            retValue += AlliedChkError(Result);
            unsigned long valnew = min + (max - min) * val->getVal<double>();
            retValue += AlliedChkError(dc1394_feature_set_value(camera, DC1394_FEATURE_GAMMA,valnew));   
#endif
        }

        if (!retValue.containsError())
        {
            retValue += it->copyValueFrom( &(*val) );
        }
#ifndef WIN32
        usleep(50000); //here the internal camera firmware is slow and we do not get a ready signal out of it so better wait or it will chrash
#endif
}

    emit parametersChanged(m_params); //send changed parameters to any connected dialogs or dock-widgets

    retValue += checkData();

    if (waitCond)
    {
        waitCond->returnValue = retValue;
        waitCond->release();
    }
    return retValue;
}


#ifndef WIN32
//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal FireGrabber::adjustROI(int x0, int x1, int y0, int y1)
{
    ito::RetVal retval;

    uint32_t  xSizeMax, ySizeMax, xPosMax, yPosMax, hUnitImage, vUnitImage, hUnitPos, vUnitPos;
    uint32_t  xSizeIs, ySizeIs, xPosIs, yPosIs;

    int32_t sizex = 1 + x1 - x0;
    int32_t sizey = 1 + y1 - y0;

    //get information    
    retval += AlliedChkError(dc1394_format7_get_max_image_size(camera,video_mode,&xSizeMax,&ySizeMax));
    retval += AlliedChkError(dc1394_format7_get_unit_size(camera,video_mode,&hUnitImage,&vUnitImage));
    retval += AlliedChkError(dc1394_format7_get_image_size(camera,video_mode,&xSizeIs,&ySizeIs));
    retval += AlliedChkError(dc1394_format7_get_image_position(camera,video_mode,&xPosIs,&yPosIs));
    retval += AlliedChkError( dc1394_format7_get_unit_position(camera,video_mode,&hUnitPos,&vUnitPos));
    xPosMax=xSizeMax;
    yPosMax=ySizeMax;

    if (!retval.containsError())
    {
        //check values
        if (video_mode < DC1394_VIDEO_MODE_FORMAT7_MIN)
        {
            retval += ito::RetVal(ito::retError,0,"The image format of this camera is not scalable");
        }

        if (x0 < 0 || x0 > (int)xPosMax || x0 % hUnitImage != 0)
        {
            retval += ito::RetVal::format(ito::retError,0,"x0 needs to be in range [%i,%i] (stepsize: %i)", 0, xPosMax, hUnitImage);
        }
        if (y0 < 0 || y0 > (int)yPosMax || y0 % vUnitImage != 0 )
        {
            retval += ito::RetVal::format(ito::retError,0,"y0 needs to be in range [%i,%i] (stepsize: %i)", 0, yPosMax, vUnitImage);
        }
        if (sizex < hUnitPos || sizex > (int)xSizeMax -x0 || sizex % hUnitPos != 0)
        {
            retval += ito::RetVal::format(ito::retError,0,"sizex needs to be in range [%i,%i] (stepsize: %i)", 0, xSizeMax, hUnitPos);
        }
        if (sizey < vUnitPos || sizey > (int)ySizeMax -y0 || sizey % vUnitPos != 0)
        {
            retval += ito::RetVal::format(ito::retError,0,"sizey needs to be in range [%i,%i] (stepsize: %i)", 0, ySizeMax, vUnitPos);
        }
    }

    if (!retval.containsError())
    {
        if (x0 != xPosIs || sizex != xSizeIs || y0 != yPosIs || sizey != ySizeIs) // something has changed
        {
            //try to change the values
            if (grabberStartedCount() > 0)
            {
                retval += ito::RetVal(ito::retError,0,"ROI values can only be changed if the camera has been stopped");
            }
            else
            {
                uint32_t packet_size;
                retval += AlliedChkError(dc1394_format7_get_packet_size(camera, video_mode, &packet_size));

                //retval += AlliedChkError(dc1394_format7_set_image_position(camera,video_mode,x0,y0));
                //retval += AlliedChkError(dc1394_format7_set_image_size(camera,video_mode,sizex,sizey));

                retval += AlliedChkError(dc1394_format7_set_roi(camera, video_mode, coding, packet_size, (uint32_t)x0, (uint32_t)y0, sizex, sizey));
                usleep(50000); //here the internal camera firmware is slow and we do not get a ready signal out of it so better wait or it will chrash

                retval += AlliedChkError(dc1394_format7_get_image_size(camera,video_mode,&xSizeIs,&ySizeIs));
                retval += AlliedChkError(dc1394_format7_get_image_position(camera,video_mode,&xPosIs,&yPosIs));                

                m_ySize = ySizeIs;
                m_xSize = xSizeIs;

                m_params["x0"].setVal<int>( xPosIs );
                m_params["y0"].setVal<int>( yPosIs );
                m_params["x1"].setVal<int>( xPosIs + xSizeIs - 1 );
                m_params["y1"].setVal<int>( yPosIs + ySizeIs - 1 );
				int* roi = m_params["roi"].getVal<int*>();
				roi[0] = xPosIs;
				roi[1] = yPosIs;
				roi[2] = xSizeIs;
				roi[3] = ySizeIs;
                m_params["sizex"].setVal<int>(xSizeIs);
                m_params["sizey"].setVal<int>(ySizeIs);


                double fps = packet_size * 8000.0 / static_cast<double>(xSizeIs * ySizeIs);
                double ftime = 1.0 / fps;
                m_params["frame_time"].setVal<double>(ftime);
            }
        }
    }
    return retval;
}
#else
ito::RetVal FireGrabber::adjustROI(int x0, int x1, int y0, int y1)
{
    ito::RetVal retval;

    FGPINFO xSizeInfo, ySizeInfo, xPosInfo, yPosInfo;
    UINT32 format;
    UINT32 resizeResult;


    int sizex = 1 + x1 - x0;
    int sizey = 1 + y1 - y0;

    //get information
    retval += AlliedChkError(Camera.GetParameterInfo(FGP_XSIZE, &xSizeInfo));
    retval += AlliedChkError(Camera.GetParameterInfo(FGP_YSIZE, &ySizeInfo));
    retval += AlliedChkError(Camera.GetParameterInfo(FGP_XPOSITION, &xPosInfo));
    retval += AlliedChkError(Camera.GetParameterInfo(FGP_YPOSITION, &yPosInfo));

    retval += AlliedChkError(Camera.GetParameter(FGP_IMAGEFORMAT,&format));

    if (!retval.containsError())
    {
        //check values
        if (IMGRES(format) != RES_SCALABLE)
        {
            retval += ito::RetVal(ito::retError,0,"The image format of this camera is not scalable");
        }

        if (x0 < (int)xPosInfo.MinValue || x0 > (int)xPosInfo.MaxValue || (x0 - (int)xPosInfo.MinValue) % xPosInfo.Unit != 0)
        {
            retval += ito::RetVal::format(ito::retError,0,"x0 needs to be in range [%i,%i] (stepsize: %i)", xPosInfo.MinValue, xPosInfo.MaxValue, xPosInfo.Unit);
        }
        if (y0 < (int)yPosInfo.MinValue || y0 > (int)yPosInfo.MaxValue || (y0 - (int)yPosInfo.MinValue) % yPosInfo.Unit != 0)
        {
            retval += ito::RetVal::format(ito::retError,0,"y0 needs to be in range [%i,%i] (stepsize: %i)", yPosInfo.MinValue, yPosInfo.MaxValue, yPosInfo.Unit);
        }
        if (sizex < (int)xSizeInfo.MinValue || sizex > (int)xSizeInfo.MaxValue || sizex % xSizeInfo.Unit != 0)
        {
            retval += ito::RetVal::format(ito::retError,0,"sizex needs to be in range [%i,%i] (stepsize: %i)", xSizeInfo.MinValue, xSizeInfo.MaxValue, xSizeInfo.Unit);
        }
        if (sizey < (int)ySizeInfo.MinValue || sizey > (int)ySizeInfo.MaxValue || sizey % ySizeInfo.Unit != 0)
        {
            retval += ito::RetVal::format(ito::retError,0,"sizey needs to be in range [%i,%i] (stepsize: %i)", ySizeInfo.MinValue, ySizeInfo.MaxValue, ySizeInfo.Unit);
        }   
    }

    if (!retval.containsError())
    {
        if (x0 != xPosInfo.IsValue || sizex != xSizeInfo.IsValue || y0 != yPosInfo.IsValue || sizey != ySizeInfo.IsValue)
        {
            //try to change the values
            if (grabberStartedCount() > 0)
            {
                retval += ito::RetVal(ito::retError,0,"ROI values can only be changed if the camera has been stopped");
            }
            else
            {
                resizeResult = Camera.SetParameter(FGP_RESIZE, 1); //may return 4 (HALER_MODE) (not allowed in this mode, since this is only necessary if camera is running)

				retval += AlliedChkError(Camera.GetParameterInfo(FGP_XPOSITION, &xPosInfo));
				retval += AlliedChkError(Camera.GetParameterInfo(FGP_YPOSITION, &yPosInfo));

				if (xPosInfo.IsValue >= x0)
				{
					retval += AlliedChkError(Camera.SetParameter(FGP_XPOSITION, x0));
					retval += AlliedChkError(Camera.SetParameter(FGP_XSIZE, sizex));
				}
				else
				{
					retval += AlliedChkError(Camera.SetParameter(FGP_XSIZE, sizex));
					retval += AlliedChkError(Camera.SetParameter(FGP_XPOSITION, x0));
				}

				if (yPosInfo.IsValue >= y0)
				{
					retval += AlliedChkError(Camera.SetParameter(FGP_YPOSITION, y0));
					retval += AlliedChkError(Camera.SetParameter(FGP_YSIZE, sizey));
				}
				else
				{
					retval += AlliedChkError(Camera.SetParameter(FGP_YSIZE, sizey));
					retval += AlliedChkError(Camera.SetParameter(FGP_YPOSITION, y0));
				}

                if (resizeResult != HALER_MODE)
                {
                    Camera.SetParameter(FGP_RESIZE, 0);
                }
                
                retval += AlliedChkError(Camera.GetParameterInfo(FGP_XSIZE, &xSizeInfo));
                retval += AlliedChkError(Camera.GetParameterInfo(FGP_YSIZE, &ySizeInfo));
                retval += AlliedChkError(Camera.GetParameterInfo(FGP_XPOSITION, &xPosInfo));
                retval += AlliedChkError(Camera.GetParameterInfo(FGP_YPOSITION, &yPosInfo));

                m_ySize = ySizeInfo.IsValue;
                m_xSize = xSizeInfo.IsValue;

                m_params["x0"].setVal<int>( xPosInfo.IsValue );
                m_params["y0"].setVal<int>( yPosInfo.IsValue );
                m_params["x1"].setVal<int>( xPosInfo.IsValue + xSizeInfo.IsValue - 1 );
                m_params["y1"].setVal<int>( yPosInfo.IsValue + ySizeInfo.IsValue - 1 );
				int* roi = m_params["roi"].getVal<int*>();
				roi[0] = xPosInfo.IsValue;
				roi[1] = yPosInfo.IsValue;
				roi[2] = xSizeInfo.IsValue;
				roi[3] = ySizeInfo.IsValue;
                m_params["sizex"].setVal<int>(xSizeInfo.IsValue);
                m_params["sizey"].setVal<int>(ySizeInfo.IsValue);

                FGPINFO packsize;
                retval += AlliedChkError(Camera.GetParameterInfo(FGP_PACKETSIZE, &packsize));
                double fps = packsize.IsValue * 8000.0 / static_cast<double>(xSizeInfo.IsValue * ySizeInfo.IsValue);
                double ftime = 1.0 / fps;
                m_params["frame_time"].setVal<double>(ftime);
            }
        }
    }
    return retval;
}
#endif

#ifndef WIN32
//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal FireGrabber::init(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);

    camera=NULL;
    video_mode = (dc1394video_mode_t)0;
    list=NULL;
    frame=NULL;
    m_isgrabbing = false;
    m_acquireReady = false;

    // get the optional parameters "cameraID" and "vendorID"
    int camID = 0;
    int venID = 0;
    int plugNR = 0;
    int nodeNR = 0;
    int tempID = 0;
    //Allied: vendorID = 673537
    //Allied Pike: cameraID = 269109919 (specific!!!)
    //Allied Marlin: cameraID = 235356461 (specific!!!)    

    plugNR = paramsOpt->at(0).getVal<int>();
    int timebase = paramsOpt->at(1).getVal<int>();

    // TODO: implement other selection methods
    //camID = paramsOpt->value(0).getVal<int>();
    //venID = paramsOpt->value(1).getVal<int>();
    //plugNR = paramsOpt->value(2).getVal<int>();

    int connected = false;

    if ((camID != 0 || venID != 0) && plugNR != 0)
    {
        camID = 0;
        venID = 0;
        retValue += ito::RetVal(ito::retWarning, 0, tr("too much input parameters, plugin number is used").toLatin1().data());
    }

    if (timebase != 1 && timebase != 2 && timebase != 5 && timebase != 10 && \
        timebase != 20 && timebase != 50 && timebase != 100 && timebase != 200 && \
        timebase != 500 && timebase != 1000)
    {
        retValue += ito::RetVal(ito::retError, 0, tr("timebase must be 1, 2, 5, 10, 20, 50, 100, 200, 500 or 1000 (timebase is only considered for AVT cameras)").toLatin1().data());
    }

    FireGrabber::m_numberOfInstances++;

    // get camera list
    dc1394error_t Result=(dc1394error_t)0;

    d = dc1394_new ();
    if (!d)
        retValue += ito::RetVal(ito::retError, 0, tr("could not load dc1394 shared library").toLatin1().data());

    //Returns the list of cameras available on the computer. If present, multiple cards will be probed
    Result=dc1394_camera_enumerate (d, &list);
    retValue += AlliedChkError(Result);
    if (list->num == 0) {
        retValue += ito::RetVal(ito::retWarning, 0, tr("no camera found").toLatin1().data());
        return retValue;
    }

    if (plugNR != 0)
    {
        //Create a new camera based on a GUID (Global Unique IDentifier)
        camera = dc1394_camera_new (d, list->ids[plugNR - 1].guid);
        if (!camera) {
            retValue += ito::RetVal(ito::retError, 0, tr("failed to init camera").toLatin1().data());
            return retValue;
        }else{
            connected = true;
        }
        nodeNR = plugNR - 1;
        // Frees the memory allocated in dc1394_enumerate_cameras for the camera list
        // dc1394_camera_free_list (list);
    }
    else
    {
        // TODO: Add all the camera init things with id / node / whatever like in the windows version here
        retValue += ito::RetVal(ito::retError, 0, tr("Not Implemented. Please select camera over plug number").toLatin1().data());
        return retValue;
    }

    if (!retValue.containsError() && connected)
    {
        // set video modes / shutter / gain and other params
        Result=dc1394_video_get_supported_modes(camera,&video_modes);
        retValue += AlliedChkError(Result);

        for (i=video_modes.num-1;i>=0;i--) {
            // if we find the full res scalable mode we use it (format7)
            if (video_modes.modes[i]==DC1394_VIDEO_MODE_FORMAT7_0) {
                //Returns the color coding from the video mode. Works with scalable image formats too.
                dc1394_get_color_coding_from_video_mode(camera,video_modes.modes[i], &coding);
                if (coding==DC1394_COLOR_CODING_MONO16) {
                    video_mode=video_modes.modes[i];
                    m_params["bpp"].setVal<int>( 16 );
                    break;
                }
                else if (coding==DC1394_COLOR_CODING_MONO8) {
                    video_mode=video_modes.modes[i];
                    m_params["bpp"].setVal<int>( 8 );
                    break;
                }
            }

            /*else{ // else we use the highest res mode
                for (i=video_modes.num-1;i>=0;i--) {
                    //Tells whether the video mode is scalable or not.
                    if (!dc1394_is_video_mode_scalable(video_modes.modes[i])) {
                        //Returns the color coding from the video mode. Works with scalable image formats too.
                        dc1394_get_color_coding_from_video_mode(camera,video_modes.modes[i], &coding);
                        if (coding==DC1394_COLOR_CODING_MONO8) {
                            video_mode=video_modes.modes[i];
                            break;
                        }
                    }
                }
            }*/
        }

        if (i < 0) {
            retValue += ito::RetVal(ito::retWarning, 0, tr("Could not get a valid MONO8 mode" ).toLatin1().data());
            dc1394_video_set_transmission(camera, DC1394_OFF);
            dc1394_capture_stop(camera);
            dc1394_camera_free(camera);
        }

        //Returns the color coding from the video mode. Works with scalable image formats too.
        Result=dc1394_get_color_coding_from_video_mode(camera, video_mode,&coding);

        // get highest framerate
        //Result=dc1394_video_get_supported_framerates(camera,video_mode,&framerates);
        //retValue += AlliedChkError(Result);
        //framerate=framerates.framerates[framerates.num-1];

        //Sets the current ISO speed. Speeds over 400Mbps require 1394B.
        Result=dc1394_video_set_iso_speed(camera, DC1394_ISO_SPEED_400);
        retValue += AlliedChkError(Result);

        //Sets the current vide mode.
        Result=dc1394_video_set_mode(camera, video_mode);
        retValue += AlliedChkError(Result);

        uint32_t packet_size;
        retValue += AlliedChkError(dc1394_format7_get_packet_size(camera, video_mode, &packet_size));
        Result = dc1394_format7_get_max_image_size(camera, video_mode, &m_xSize, &m_ySize);
        retValue += AlliedChkError(Result);
        // set the ROI to full camera size
        retValue += AlliedChkError(dc1394_format7_set_roi(camera, video_mode, coding, packet_size, 0,0, m_xSize, m_ySize));
        retValue += AlliedChkError(Result);

        //Sets the current framerate. This is meaningful only if the video mode is not scalable.
        //Result=dc1394_video_set_framerate(camera, framerate);
        //retValue += AlliedChkError(Result);

        //Setup the capture, using a ring buffer of a certain size (num_dma_buffers) and certain options (flags)
        //Result=dc1394_capture_setup(camera,1, DC1394_CAPTURE_FLAGS_DEFAULT);
        retValue += AlliedChkError(Result);


        //check available parameters
        dc1394feature_info_t *feature;

        double val;

        char *vendorName = camera->vendor;
        char *modelName = camera->model;
        //Result = DC1394_SUCCESS;

        if (!retValue.containsError())
        {
            m_params["vendorName"].setVal<char*>(vendorName);
            m_params["modelName"].setVal<char*>(modelName);
            m_identifier = QString("%1 (%2)").arg(modelName).arg(vendorName);

            retValue += initAVTCameras(vendorName, modelName, timebase);
        }
        else
        {
            m_identifier = QString::number(tempID);
        }

        //brightness
        dc1394bool_t present;
        dc1394bool_t readable;
        uint32_t min, max, value;

        Result = dc1394_feature_is_present(camera, DC1394_FEATURE_BRIGHTNESS, &present);
        retValue += AlliedChkError(Result);
        Result = dc1394_feature_is_readable(camera, DC1394_FEATURE_BRIGHTNESS, &readable);
        retValue += AlliedChkError(Result);

        if (Result == DC1394_SUCCESS & present == DC1394_TRUE & readable == DC1394_TRUE)
        {
            Result = dc1394_feature_set_mode(camera, DC1394_FEATURE_BRIGHTNESS, DC1394_FEATURE_MODE_MANUAL);
            retValue += AlliedChkError(Result);
            Result = dc1394_feature_get_boundaries(camera, DC1394_FEATURE_BRIGHTNESS, &min, &max);
            retValue += AlliedChkError(Result);
            Result =  dc1394_feature_get_value(camera, DC1394_FEATURE_BRIGHTNESS, &value);
            retValue += AlliedChkError(Result);

            val = (value - min) / (max - min);
            m_params["brightness"].setVal<double>(val);
        }
        else
        {
            m_params.remove("brightness");
        }

        //sharpness
        Result = dc1394_feature_is_present(camera, DC1394_FEATURE_SHARPNESS, &present);
        retValue += AlliedChkError(Result);
        Result = dc1394_feature_is_readable(camera, DC1394_FEATURE_SHARPNESS, &readable);
        retValue += AlliedChkError(Result);

        if (Result == DC1394_SUCCESS & present == DC1394_TRUE & readable == DC1394_TRUE)
        {
            Result = dc1394_feature_set_mode(camera, DC1394_FEATURE_SHARPNESS, DC1394_FEATURE_MODE_MANUAL);
            retValue += AlliedChkError(Result);
            Result = dc1394_feature_get_boundaries(camera, DC1394_FEATURE_SHARPNESS, &min, &max);
            retValue += AlliedChkError(Result);
            Result =  dc1394_feature_get_value(camera, DC1394_FEATURE_SHARPNESS, &value);
            retValue += AlliedChkError(Result);

            val = (value - min) / (max - min);
            m_params["sharpness"].setVal<double>(val);
        }
        else
        {
            m_params.remove("sharpness");
        }

        //gamma
        Result = dc1394_feature_is_present(camera, DC1394_FEATURE_GAMMA, &present);
        retValue += AlliedChkError(Result);
        Result = dc1394_feature_is_readable(camera, DC1394_FEATURE_GAMMA, &readable);
        retValue += AlliedChkError(Result);

        if (Result == DC1394_SUCCESS & present == DC1394_TRUE & readable == DC1394_TRUE)
        {
            Result = dc1394_feature_set_mode(camera, DC1394_FEATURE_GAMMA, DC1394_FEATURE_MODE_MANUAL);
            retValue += AlliedChkError(Result);
            Result =  dc1394_feature_get_value(camera, DC1394_FEATURE_GAMMA, &value);
            m_params["gamma"].setVal<double>(value);
        }
        else
        {
            m_params.remove("gamma");
        }

        //gain
        Result = dc1394_feature_is_present(camera, DC1394_FEATURE_GAIN, &present);
        retValue += AlliedChkError(Result);
        Result = dc1394_feature_is_readable(camera, DC1394_FEATURE_GAIN, &readable);
        retValue += AlliedChkError(Result);

        if (Result == DC1394_SUCCESS & present == DC1394_TRUE & readable == DC1394_TRUE)
        {
            Result = dc1394_feature_set_mode(camera, DC1394_FEATURE_GAIN, DC1394_FEATURE_MODE_MANUAL);
            retValue += AlliedChkError(Result);
            Result = dc1394_feature_get_boundaries(camera, DC1394_FEATURE_GAIN, &min, &max);
            retValue += AlliedChkError(Result);
            Result =  dc1394_feature_get_value(camera, DC1394_FEATURE_GAIN, &value);
            retValue += AlliedChkError(Result);

            val = (value - min) / (max - min);
            m_params["gain"].setVal<double>(val);
        }
        else
        {
            m_params["gain"].setMeta( new ito::DoubleMeta(0.0,0.0), true );
            m_params["gain"].setFlags(ito::ParamBase::Readonly);
        }

        //offset
        m_params["offset"].setMeta( new ito::DoubleMeta(0.0,0.0), true );
        m_params["offset"].setFlags(ito::ParamBase::Readonly);


        //shutter
        Result = dc1394_feature_is_present(camera, DC1394_FEATURE_SHUTTER, &present);
        retValue += AlliedChkError(Result);
        Result = dc1394_feature_is_readable(camera, DC1394_FEATURE_SHUTTER, &readable);
        retValue += AlliedChkError(Result);

        if (Result == DC1394_SUCCESS & present == DC1394_TRUE & readable == DC1394_TRUE)
        {
            Result = dc1394_feature_set_mode(camera, DC1394_FEATURE_SHUTTER, DC1394_FEATURE_MODE_MANUAL);
            retValue += AlliedChkError(Result);
            Result = dc1394_feature_get_boundaries(camera, DC1394_FEATURE_SHUTTER, &min, &max);
            retValue += AlliedChkError(Result);
            Result =  dc1394_feature_get_value(camera, DC1394_FEATURE_SHUTTER, &value);
            retValue += AlliedChkError(Result);

            double minIntegrationTime = shutterToExposureSec(min);
            double maxIntegrationTime = shutterToExposureSec(max);
            double integrationTime = shutterToExposureSec(value);
            m_params["integration_time"].setMeta( new ito::DoubleMeta(minIntegrationTime,maxIntegrationTime), true);
            m_params["integration_time"].setVal<double>(integrationTime);

            retValue += AlliedChkError(dc1394_feature_set_mode(camera, DC1394_FEATURE_SHUTTER, DC1394_FEATURE_MODE_MANUAL));

        }
        else
        {
            m_params["integration_time"].setMeta( new ito::DoubleMeta(0.0,0.0), true);
            m_params["integration_time"].setFlags(ito::ParamBase::Readonly);
        }

        Result = dc1394_format7_get_image_size(camera, video_mode, &m_xSize, &m_ySize);
        // Result = dc1394_get_image_size_from_video_mode(camera, video_mode, &m_xSize, &m_ySize);

        m_params["sizex"].setVal<double>(m_xSize);
        m_params["sizey"].setVal<double>(m_ySize);
        m_params["x1"].setVal<double>(m_xSize - 1);
        m_params["y1"].setVal<double>(m_ySize - 1);
		int roi[] = { 0, 0, m_xSize, m_ySize };
		m_params["roi"].setVal<int*>(roi, 4);

        // Obtain data geometry
		uint32_t  xSizeMax, ySizeMax, xPosMax, yPosMax, hUnitImage, vUnitImage, hUnitPos, vUnitPos;

		//get information    
		Result = dc1394_format7_get_max_image_size(camera,video_mode,&xSizeMax,&ySizeMax);
		Result = dc1394_format7_get_unit_size(camera,video_mode,&hUnitImage,&vUnitImage);
		Result = dc1394_format7_get_unit_position(camera,video_mode,&hUnitPos,&vUnitPos);
		xPosMax=xSizeMax;
		yPosMax=ySizeMax;

		m_params["x0"].setMeta(new ito::IntMeta(0, xPosMax, hUnitPos), true);
		m_params["y0"].setMeta(new ito::IntMeta(0, yPosMax, vUnitPos), true);
		m_params["x1"].setMeta(new ito::IntMeta(0, xSizeMax - 1, hUnitPos), true);
		m_params["y1"].setMeta(new ito::IntMeta(0, ySizeMax - 1, vUnitPos), true);
		m_params["sizex"].setMeta(new ito::IntMeta(0, xSizeMax, hUnitImage), true);
		m_params["sizey"].setMeta(new ito::IntMeta(0, ySizeMax, vUnitImage), true);

		ito::RectMeta roiMeta(\
			ito::RangeMeta(0, xSizeMax - 1, hUnitPos, 0, xSizeMax, hUnitImage), \
			ito::RangeMeta(0, ySizeMax - 1, vUnitPos, 0, ySizeMax, vUnitImage));
		m_params["roi"].setMeta(&roiMeta, false);

        Result = dc1394_feature_is_present(camera, DC1394_FEATURE_FRAME_RATE, &present);
        retValue += AlliedChkError(Result);
        Result = dc1394_feature_is_readable(camera, DC1394_FEATURE_FRAME_RATE, &readable);
        retValue += AlliedChkError(Result);

        if (Result == DC1394_SUCCESS & present == DC1394_TRUE & readable == DC1394_TRUE)
        {
            Result = dc1394_feature_set_mode(camera, DC1394_FEATURE_FRAME_RATE, DC1394_FEATURE_MODE_MANUAL);
            retValue += AlliedChkError(Result);
            Result =  dc1394_feature_get_value(camera, DC1394_FEATURE_FRAME_RATE, &value);
            retValue += AlliedChkError(Result);

            uint32_t packet_size;
            retValue += AlliedChkError(dc1394_format7_get_packet_size(camera, video_mode, &packet_size));
            double fps = packet_size * 8000.0 / static_cast<double>(m_xSize * m_ySize);
            double ftime = 1.0 / fps;
            m_params["frame_time"].setVal<double>(ftime);
        }
    }

    if (retValue.containsError())
    {
        retValue += ito::RetVal(ito::retError,0, tr("no camera has been connected").toLatin1().data());
    }
    else
    {
        retValue += checkData(); 

        emit parametersChanged(m_params);
    }
    
    m_isgrabbing = false;

    


    if (waitCond)
    {
        waitCond->returnValue = retValue;
        waitCond->release();
    }

    setInitialized(true); //init method has been finished (independent on retval)

    return retValue;
}

#else
//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal FireGrabber::init(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);

    // get the optional parameters "cameraID" and "vendorID"
    int nodeNR = 0;
    int tempID = 0;
    //Allied: vendorID = 673537
    //Allied Pike: cameraID = 269109919 (specific!!!)
    //Allied Marlin: cameraID = 235356461 (specific!!!)
    int camID = paramsOpt->at(0).getVal<int>();
    int venID = paramsOpt->at(1).getVal<int>();
    int plugNR = paramsOpt->at(2).getVal<int>();
    int timebase = paramsOpt->at(3).getVal<int>();


    int connected = false;

    if ((camID != 0 || venID != 0) && plugNR != 0)
    {
        camID = 0;
        venID = 0;
        retValue += ito::RetVal(ito::retWarning, 0, tr("too much input parameters, plugin number is used").toLatin1().data());
    }

    if (timebase != 1 && timebase != 2 && timebase != 5 && timebase != 10 && \
        timebase != 20 && timebase != 50 && timebase != 100 && timebase != 200 && \
        timebase != 500 && timebase != 1000)
    {
        retValue += ito::RetVal(ito::retError, 0, tr("timebase must be 1, 2, 5, 10, 20, 50, 100, 200, 500 or 1000 (timebase is only considered for AVT cameras)").toLatin1().data());
    }

    FireGrabber::m_numberOfInstances++;

    ResultType Result = 0;

    UINT32 NodeCnt;
    FGNODEINFO NodeInfo[25];

    if (!retValue.containsError())
    {

        if (FireGrabber::m_numberOfInstances == 1) // we are the first user, open DLL
        {
            retValue += AlliedChkError(FGInitModule(NULL));
        }

        Result = FGGetNodeList(NodeInfo, 25, &NodeCnt);  // Get list of connected nodes

        if (plugNR != 0)
        {

            Result = Camera.Connect(&NodeInfo[plugNR - 1].Guid);
            if (Result == 0)
            {
                connected = true;
            }
            nodeNR = plugNR - 1;
        }
        else
        {
            // determine if camID and venID define the demanded camera clearly
            if (NodeCnt >= 2)
            {
                for (unsigned long i = 0; i <= NodeCnt - 2; i++)
                {
                    for (unsigned long j = i; j <= NodeCnt - 2; j++)
                    {
                        if (camID != 0 && venID == 0 && (NodeInfo[i].Guid.Low == NodeInfo[j + 1].Guid.Low))
                        {
                            retValue += ito::RetVal(ito::retWarning, 0, tr("same cameraID for more than one camera, connected the first plugged and free camera").toLatin1().data());
                        }
                        if (camID == 0 && venID != 0 && (NodeInfo[i].Guid.High == NodeInfo[j + 1].Guid.High))
                        {
                            retValue += ito::RetVal(ito::retWarning, 0, tr("same vendorID for more than one camera, connected the first plugged and free camera").toLatin1().data());
                        }
                    }
                }
            }

            // find the right port of demanded camera and connect it
            for (unsigned long i = 0; i <= NodeCnt - 1; i++)
            {
                if (camID == NodeInfo[i].Guid.Low && venID == NodeInfo[i].Guid.High)    //camID and venID given
                {
                    Result = Camera.Connect(&NodeInfo[i].Guid);        // Connect with node i
                    if (Result == 0)
                    {
                        connected = true;
                    }
                    nodeNR = i;
                    break;
                }
                else if (camID == NodeInfo[i].Guid.Low && NodeInfo[i].Busy == 0 && venID == 0)    //only camID given
                {
                    Result = Camera.Connect(&NodeInfo[i].Guid);
                    if (Result == 0)
                    {
                        connected = true;
                    }
                    nodeNR = i;
                    break;
                }
                else if (camID == 0 && venID == NodeInfo[i].Guid.High && NodeInfo[i].Busy == 0)    //only venID given
                {
                    Result = Camera.Connect(&NodeInfo[i].Guid);
                    if (Result == 0)
                    {
                        connected = true;
                    }
                    nodeNR = i;
                    break;
                }
                else if (camID == 0 && venID == 0 && NodeInfo[i].Busy == 0) //no camID and venID given, connect the first plugged and free camera to its node
                {
                    Result = Camera.Connect(&NodeInfo[i].Guid);   // Connect with node i
                    if (Result == 0)
                    {
                        connected = true;
                    }
                    nodeNR = i;
                    break;
                }
            }
        }
    }

    if (!retValue.containsError() && Result == 0 && connected)
    {
        // Set camera to scalable mode and max size

        //check available parameters
        FGPINFO info, xInfo, yInfo, xPosInfo, yPosInfo;
        double val;

        char *vendorName = new char[512];
        vendorName[511] = '\0'; //zero-terminate it for security reason
        char *modelName = new char[512];
        modelName[511] = '\0'; //zero-terminate it for security reason

        tempID = NodeInfo[nodeNR].Guid.High;
        m_params["vendorID"].setVal(tempID);
        tempID = NodeInfo[nodeNR].Guid.Low;
        m_params["cameraID"].setVal(tempID);
        Result = Camera.GetDeviceName(vendorName, 511, modelName); //vendorName and modelName are zero-terminated then
        if (Result == 0)
        {
            m_params["vendorName"].setVal<char*>(vendorName);
            m_params["modelName"].setVal<char*>(modelName);
            m_identifier = QString("%1 (%2)").arg(modelName).arg(vendorName);
            retValue += initAVTCameras(vendorName, modelName, timebase);
        }
        else
        {
            m_identifier = QString::number(tempID);
        }

        delete[] vendorName; vendorName = NULL;
        delete[] modelName; modelName = NULL;


        Result = Camera.GetParameterInfo(FGP_BRIGHTNESS, &info);
        if (Result == FCE_NOTAVAILABLE)
        {
            m_params.remove("brightness");
        }
        else
        {
            val = (info.IsValue - info.MinValue) / (info.MaxValue - info.MinValue);
            m_params["brightness"].setVal<double>(val);
            m_camProperties["brightness"] = info;
        }

        Result = Camera.GetParameterInfo(FGP_SHARPNESS, &info);
        if (Result == FCE_NOTAVAILABLE)
        {
            m_params.remove("sharpness");
        }
        else
        {
            val = (info.IsValue - info.MinValue) / (info.MaxValue - info.MinValue);
            m_params["sharpness"].setVal<double>(val);
            m_camProperties["sharpness"] = info;
        }

        Result = Camera.GetParameterInfo(FGP_GAMMA, &info);
        if (Result == FCE_NOTAVAILABLE)
        {
            m_params.remove("gamma");
        }
        else
        {
            m_params["gamma"].setVal<int>(info.IsValue);
            m_camProperties["gamma"] = info;
        }

        Result = Camera.GetParameterInfo(FGP_GAIN, &info);
        if (Result == FCE_NOTAVAILABLE)
        {
            m_params["gain"].setMeta( new ito::DoubleMeta(0.0,0.0), true );
            m_params["gain"].setFlags(ito::ParamBase::Readonly);
        }
        else
        {
            val = (info.IsValue - info.MinValue) / (info.MaxValue - info.MinValue);
            m_params["gain"].setVal<double>(val);
            m_camProperties["gain"] = info;
        }

        //offset
        m_params["offset"].setMeta( new ito::DoubleMeta(0.0,0.0), true );
        m_params["offset"].setFlags(ito::ParamBase::Readonly);


        Result = Camera.GetParameterInfo(FGP_SHUTTER, &info);
        if (Result == FCE_NOTAVAILABLE)
        {
            m_params["integration_time"].setMeta( new ito::DoubleMeta(0.0,0.0), true);
            m_params["integration_time"].setFlags(ito::ParamBase::Readonly);
        }
        else
        {
            double minIntegrationTime = shutterToExposureSec(info.MinValue);
            double maxIntegrationTime = shutterToExposureSec(info.MaxValue);
            double integrationTime = shutterToExposureSec(info.IsValue);
            m_params["integration_time"].setMeta( new ito::DoubleMeta(minIntegrationTime,maxIntegrationTime), true);
            m_params["integration_time"].setVal<double>(integrationTime);
            m_camProperties["shutter"] = info;
        }

        Result = Camera.SetParameter(FGP_IMAGEFORMAT, MAKEIMAGEFORMAT(RES_SCALABLE, CM_Y8, 0));

        Result = Camera.GetParameterInfo(FGP_XSIZE, &xInfo);
        Result = Camera.GetParameterInfo(FGP_YSIZE, &yInfo);
		Result = Camera.GetParameterInfo(FGP_XPOSITION, &xPosInfo);
		Result = Camera.GetParameterInfo(FGP_YPOSITION, &yPosInfo);

        Result = Camera.SetParameter(FGP_XPOSITION, 0);
        Result = Camera.SetParameter(FGP_YPOSITION, 0);
        Result = Camera.SetParameter(FGP_XSIZE, xInfo.MaxValue);
        Result = Camera.SetParameter(FGP_YSIZE, yInfo.MaxValue);

        m_params["sizex"].setVal<double>(xInfo.MaxValue);
        m_params["sizey"].setVal<double>(yInfo.MaxValue);
        m_params["x1"].setVal<double>(xInfo.MaxValue - 1);
        m_params["y1"].setVal<double>(yInfo.MaxValue - 1);
		int roi[] = { 0, 0, xInfo.MaxValue, yInfo.MaxValue };
		m_params["roi"].setVal<int*>(roi, 4);

        Result = Camera.GetParameterInfo(FGP_XSIZE, &xInfo);
        Result = Camera.GetParameterInfo(FGP_YSIZE, &yInfo);

		m_params["x0"].setMeta(new ito::IntMeta(xPosInfo.MinValue, xPosInfo.MaxValue, xPosInfo.Unit), true);
		m_params["y0"].setMeta(new ito::IntMeta(yPosInfo.MinValue, yPosInfo.MaxValue, yPosInfo.Unit), true);
		m_params["x1"].setMeta(new ito::IntMeta(xPosInfo.MinValue, xInfo.MaxValue - 1, xPosInfo.Unit), true);
		m_params["y1"].setMeta(new ito::IntMeta(yPosInfo.MinValue, yInfo.MaxValue - 1, yPosInfo.Unit), true);
		m_params["sizex"].setMeta(new ito::IntMeta(xInfo.MinValue, xInfo.MaxValue, xInfo.Unit), true);
		m_params["sizey"].setMeta(new ito::IntMeta(yInfo.MinValue, yInfo.MaxValue, yInfo.Unit), true);

		ito::RectMeta roiMeta(\
			ito::RangeMeta(xPosInfo.MinValue, xInfo.MaxValue - 1, xPosInfo.Unit, xInfo.MinValue, xInfo.MaxValue, xInfo.Unit), \
			ito::RangeMeta(yPosInfo.MinValue, yInfo.MaxValue - 1, yPosInfo.Unit, yInfo.MinValue, yInfo.MaxValue, yInfo.Unit));
		m_params["roi"].setMeta(&roiMeta, false);

        FGPINFO packsize;
        retValue += AlliedChkError(Camera.GetParameterInfo(FGP_PACKETSIZE, &packsize));
        double fps = packsize.IsValue * 8000.0 / static_cast<double>(xInfo.IsValue * yInfo.IsValue);
        double ftime = 1.0 / fps;
        m_params["frame_time"].setVal<double>(ftime);
    }

    if (Result != 0)
    {
        retValue = AlliedChkError(Result);
    }
    else if (connected == false)
    {
        retValue += ito::RetVal(ito::retError,0, tr("no camera has been connected").toLatin1().data());
    }

    m_isgrabbing = false;

    retValue += checkData();

    if (!retValue.containsError())
    {
        emit parametersChanged(m_params);
    }

    if (waitCond)
    {
        waitCond->returnValue = retValue;
        waitCond->release();
    }

    setInitialized(true); //init method has been finished (independent on retval)

    return retValue;
}
#endif

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal FireGrabber::initAVTCameras(const char *vendorName, const char *modelName, int desiredTimebase)
{
    ito::RetVal retValue;

    if (strcmp(vendorName, "AVT") == 0)
    {
        m_exposureParams.AVTCam = true;
        
        //try to read timebase register
#ifdef WIN32
        UINT32 regValue;
        ResultType Result = Camera.ReadRegister(0xF1000208, &regValue);
        ResultType ResultOK = FCE_NOERROR;
#else
        uint32_t regValue;
        ResultType Result = dc1394_avt_get_timebase(camera, &regValue);
        ResultType ResultOK = DC1394_SUCCESS;
#endif
        if (Result == ResultOK)
        {
            if (regValue & 0x80000000) //timebase present
            {
                int timebases[] = { 1, 2, 5, 10, 20, 50, 100, 200, 500, 1000 };
                
                //try to set the desired typebase
                int newid = 4; //20 mus per default
                for (int idx = 0; idx < sizeof(timebases) / sizeof(int); ++idx)
                {
                    if (timebases[idx] == desiredTimebase)
                    {
                        newid = idx;
                        break;
                    }
                }

#ifdef WIN32
                Result = Camera.WriteRegister(0xF1000208, (regValue & 0xfffffff0) + newid);
#else
                dc1394_avt_set_timebase(camera, (regValue & 0xfffffff0) + newid);
#endif
                

                //read register
#ifdef WIN32
                Camera.ReadRegister(0xF1000208, &regValue);
#else
                dc1394_avt_get_timebase(camera, &regValue);
#endif
                int id = regValue & 0x0000000f;

                m_exposureParams.timebaseMs = timebases[id] / 1000.0;
                m_params["timebase"].setVal<int>(timebases[id]);
            }
            else
            {
                retValue += ito::RetVal(ito::retWarning, 0, tr("timebase register of camera is not available. Timebase is set to 20 %1s per default").arg(QChar(0x00, 0xB5)).toLatin1().data());  // \mu s
                m_exposureParams.timebaseMs = 20.0 / 1000.0;
            }
        }
        else
        {
            retValue += ito::RetVal(ito::retWarning, 0, tr("timebase register of camera could not be read. Timebase is set to 20 %1s per default").arg(QChar(0x00, 0xB5)).toLatin1().data());  // \mu s
            m_exposureParams.timebaseMs = 20.0 / 1000.0;
        }

        QMap<QString, int> offsets;

        offsets["Marlin F033B"] = 12;
        offsets["Marlin F033C"] = 12;
        offsets["Marlin F046B"] = 12;
        offsets["Marlin F046C"] = 12;
        offsets["Marlin F080B"] = 30;
        offsets["Marlin F080C"] = 30;
        offsets["Marlin F080B-30fps"] = 30;
        offsets["Marlin F080C-30fps"] = 30;
        offsets["Marlin F145B2"] = 18;
        offsets["Marlin F145C2"] = 18;
        offsets["Marlin F146B"] = 26;
        offsets["Marlin F146C"] = 26;
        offsets["Marlin F201B"] = 39;
        offsets["Marlin F201C"] = 39;
        offsets["Marlin F131B"] = 1;
        offsets["Marlin F131C"] = 1;

        offsets["Pike F-032"] = 17;
        offsets["Pike F-100"] = 42;
        offsets["Pike F-145"] = 38;
        offsets["Pike F-145-15fps"] = 70;
        offsets["Pike F-210"] = 42;
        offsets["Pike F-421"] = 69;
        offsets["Pike F-505"] = 26;
        offsets["Pike F-1100"] = 128;
        offsets["Pike F-1600"] = 635;

        offsets["GUPPY F-033"] = 109;
        offsets["GUPPY F-036"] = -21;
        offsets["GUPPY F-038"] = 42;
        offsets["GUPPY F-038 NIR"] = 42;
        offsets["GUPPY F-044"] = 42;
        offsets["GUPPY F-044 NIR"] = 42;
        offsets["GUPPY F-046"] = 22;
        offsets["GUPPY F-080"] = 34;
        offsets["GUPPY F-146"] = 20;
        offsets["GUPPY F-503"] = -42;

        if (offsets.contains(modelName))
        {
            m_exposureParams.offsetMs = offsets[modelName] / 1000.0;
        }
        else
        {
            retValue += ito::RetVal(ito::retWarning, 0, tr("no exposure offset is available for this camera model. Therefore the offset is set to 0 and your exposure time might be few microseconds smaller than the real value.").toLatin1().data());
            m_exposureParams.offsetMs = 0.0;
        }
    }
    else
    {
        m_exposureParams.AVTCam = false;
        m_exposureParams.offsetMs = 0.0;
        m_exposureParams.timebaseMs = 20.0;
        retValue += ito::RetVal(ito::retWarning, 0, tr("Camera model is not known. Therefore the integration time represents the shutter value, not the real exposure time in seconds.").toLatin1().data());
    }

    return retValue;
}

#ifndef WIN32
//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal FireGrabber::close(ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);
    if (camera!=0){
        dc1394_video_set_transmission(camera, DC1394_OFF);
        dc1394_capture_stop(camera);
        dc1394_camera_free(camera);
    }
    camera=0;
    FireGrabber::m_numberOfInstances--;

    if (FireGrabber::m_numberOfInstances < 1) //we are the last user, close DLL
    {
        dc1394_free (d);
    }
    if (waitCond)
    {
        waitCond->returnValue = retValue;
        waitCond->release();
    }

    return retValue;
}

#else
//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal FireGrabber::close(ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);
    if (grabberStartedCount() > 0)
    {
        Camera.CloseCapture();
    }

    Camera.Disconnect();                        // Disconnect camera

    FireGrabber::m_numberOfInstances--;

    if (FireGrabber::m_numberOfInstances < 1) //we are the last user, close DLL
    {
        FGExitModule();                            // Exit module
    }
    if (waitCond)
    {
        waitCond->returnValue = retValue;
        waitCond->release();
    }

    return retValue;
}

#endif

//----------------------------------------------------------------------------------------------------------------------------------
double FireGrabber::shutterToExposureSec(int shutter)
{
    if (m_exposureParams.AVTCam)
    {
        return (m_exposureParams.timebaseMs * shutter + m_exposureParams.offsetMs) / 1000.0;
    }
    else
    {
        return static_cast<double>(shutter);
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
int FireGrabber::exposureSecToShutter(double exposure)
{
    if (m_exposureParams.AVTCam)
    {
        return ((exposure * 1000) - m_exposureParams.offsetMs) / m_exposureParams.timebaseMs;
    }
    else
    {
        return static_cast<int>(exposure);
    }
}

#ifndef WIN32
//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal FireGrabber::startDevice(ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);
    dc1394error_t Result = dc1394error_t(0);
    incGrabberStarted();

    if (grabberStartedCount() == 1) //the first instance
    {
        //  retValue += AlliedChkError(Camera.SetParameter(FGP_DMAMODE, DMA_LIMP)); //this is important
        //  retValue += AlliedChkError(Camera.SetParameter(FGP_BURSTCOUNT, BC_INFINITE));

        if (!retValue.containsError())
        {            
            retValue += AlliedChkError(dc1394_capture_setup(camera,1, DC1394_CAPTURE_FLAGS_DEFAULT));
            retValue += AlliedChkError(dc1394_video_set_transmission(camera, DC1394_ON));
            retValue += AlliedChkError(dc1394_capture_dequeue(camera, DC1394_CAPTURE_POLICY_WAIT, &frame));

            //  retValue += AlliedChkError(Camera.OpenCapture());
            if (!retValue.containsError())
            {
                if (m_params["bpp"].getVal<int>() == 8)
                {
                    coding = DC1394_COLOR_CODING_MONO8;

                    retValue += AlliedChkError(dc1394_format7_set_color_coding(camera, DC1394_VIDEO_MODE_FORMAT7_0, coding));
                }
                else if (m_params["bpp"].getVal<int>() == 16)
                {
                    coding = DC1394_COLOR_CODING_MONO16;
                    retValue += AlliedChkError(dc1394_format7_set_color_coding(camera, DC1394_VIDEO_MODE_FORMAT7_0, coding));
                }

               // if (!retValue.containsError())
               // {
               //     m_isgrabbing = true;
               // }

                //Starts/stops the isochronous data transmission. In other words, use this to control the image flow.
                //retValue += AlliedChkError(dc1394_video_set_transmission(camera, DC1394_ON));
                //retValue += ito::RetVal(ito::retError,0, tr("Could not start camera iso transmission").toLatin1().data());
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

#else
//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal FireGrabber::startDevice(ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);

    UINT32 Result = 0;
    incGrabberStarted();

    if (grabberStartedCount() == 1) //the first instance
    {
        retValue += AlliedChkError(Camera.SetParameter(FGP_DMAMODE, DMA_LIMP)); //this is important
        retValue += AlliedChkError(Camera.SetParameter(FGP_BURSTCOUNT, BC_INFINITE));

        if (!retValue.containsError())
        {

            retValue += AlliedChkError(Camera.OpenCapture());
            if (!retValue.containsError())
            {
                retValue += AlliedChkError(Camera.StartDevice());
                retValue += AlliedChkError(Camera.DiscardFrames());
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
#endif
         
//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal FireGrabber::stopDevice(ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);

    decGrabberStarted();
    
#ifndef WIN32
    if (grabberStartedCount() == 0 && camera!=0)
    {
        // Stop image device
        retValue += AlliedChkError(dc1394_video_set_transmission(camera,DC1394_OFF));
        retValue += AlliedChkError(dc1394_capture_stop(camera));
    }
#else
    if (grabberStartedCount() == 0)
    {
        // Stop image device
        retValue += AlliedChkError(Camera.StopDevice());
        retValue += AlliedChkError(Camera.CloseCapture());
    }
#endif

    else if (grabberStartedCount() < 0)
    {
        retValue += ito::RetVal(ito::retWarning, 1001, tr("stopDevice not executed since camera has not been started.").toLatin1().data());
        setGrabberStarted(0);
    }

    m_isgrabbing = false;

    if (waitCond)
    {
        waitCond->returnValue = retValue;
        waitCond->release();
    }

    //Result = Camera.StopDevice();    // Stop the device
    return ito::retOk;
}
         
//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal FireGrabber::acquire(const int trigger, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);
#ifndef WIN32
    if (grabberStartedCount() > 0){
        retValue += AlliedChkError(dc1394_capture_enqueue(camera,frame));

        m_acquireReady = true;
    }
    else
    {
        retValue += ito::RetVal(ito::retError, 1001, tr("Camera has to be started before acquiring frame.").toLatin1().data());
    }
#else
    retValue += AlliedChkError(Camera.DiscardFrames());
    retValue += AlliedChkError(Camera.PutFrame(NULL));
#endif

    if (waitCond)
    {
        waitCond->returnValue = retValue;
        waitCond->release();  
    }

    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
#ifndef WIN32
ito::RetVal FireGrabber::retrieveData(ito::DataObject *externalDataObject)
{
    ito::RetVal retValue(ito::retOk);
    dc1394error_t Result=(dc1394error_t)0;

    bool copyExternal = false;
    bool hasLiveList = false;

    if (externalDataObject)
    {
        copyExternal = true;
    }

    if (m_autoGrabbingListeners.size() > 0)
    {
        hasLiveList = true;
    }
    if (m_acquireReady == true){
        //obtain frame from camera
        retValue += AlliedChkError(dc1394_capture_dequeue(camera, DC1394_CAPTURE_POLICY_WAIT, &frame));

        if (Result != 0)
        {
            retValue += AlliedChkError(Result);
        }
        retValue += AlliedChkError(dc1394_get_image_size_from_video_mode(camera, video_mode, &m_xSize, &m_ySize));

        if (!retValue.containsError())
        {
            //copy frame.pData to m_data or externalDataObject

            if (m_params["bpp"].getVal<int>() == 8 )
            {
                if (copyExternal) retValue += externalDataObject->copyFromData2D<ito::uint8>((ito::uint8*)frame->image, m_xSize, m_ySize);
                if (!copyExternal || hasLiveList) retValue += m_data.copyFromData2D<ito::uint8>((ito::uint8*)frame->image, m_xSize, m_ySize);
            }
            else if (m_params["bpp"].getVal<int>() == 16 )
            {
                if (copyExternal) retValue += externalDataObject->copyFromData2D<ito::uint16>((ito::uint16*)frame->image, m_xSize, m_ySize);
                if (!copyExternal || hasLiveList) retValue += m_data.copyFromData2D<ito::uint16>((ito::uint16*)frame->image, m_xSize, m_ySize);

                ito::uint16 *rowPtr = NULL;
                ito::uint8 *rowPtr8 = NULL;
                ito::uint8 *framePtr = (ito::uint8*)frame->image;

                size_t frameIdx;
                cv::Mat *mat;

                if (copyExternal)
                {
                    //with respect to the byte-order of the camera-channel (big-endian) it must be swapped to little-endian for itom
                    mat = externalDataObject->getCvPlaneMat(0);
                    frameIdx = 0;

                    for (size_t m = 0; m < m_ySize ; m++)
                    {
                        rowPtr = mat->ptr<ito::uint16>(m);
                        rowPtr8 = (ito::uint8*)rowPtr;
                        for (size_t n = 0; n < m_xSize ; n++)
                        {
                            rowPtr8[n * 2 + 1] = framePtr[frameIdx];
                            rowPtr8[n * 2] = framePtr[frameIdx + 1];
                            frameIdx += 2;

                            //rowPtr8[n*2+1] = framePtr[frameIdx++];
                            //rowPtr8[n*2] = framePtr[frameIdx++];
                        }
                    }
                }

                if (!copyExternal || hasLiveList)
                {
                    //with respect to the byte-order of the camera-channel (big-endian) it must be swapped to little-endian for itom
                    mat = m_data.get_mdata()[0];
                    frameIdx = 0;
                    for (size_t m = 0; m < m_ySize ; m++)
                    {
                        rowPtr = mat->ptr<ito::uint16>(m);
                        rowPtr8 = (ito::uint8*)rowPtr;
                        for (size_t n = 0; n < m_xSize ; n++)
                        {
                            rowPtr8[n * 2 + 1] = framePtr[frameIdx];
                            rowPtr8[n * 2] = framePtr[frameIdx + 1];
                            frameIdx += 2;

                            //rowPtr8[n*2+1] = framePtr[frameIdx++];
                            //rowPtr8[n*2] = framePtr[frameIdx++];
                        }
                    }
                }
            }
        }
        m_acquireReady = false;
    }
    else
    {
        retValue += ito::RetVal(ito::retError, 1001, tr("No frame to catch. Frame needs to be aquired before retrieving.").toLatin1().data());
    }
    //returns the frame back to standby queue of camera. PutFrame(NULL) in acquire then puts it to DMA queue, where camera can put new images in.

    return retValue;
}

#else
//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal FireGrabber::retrieveData(ito::DataObject *externalDataObject)
{
    ito::RetVal retValue(ito::retOk);
    UINT32 Result = 0;    
    FGFRAME frame;

    bool RetCode = false;

    int curxsize = m_params["sizex"].getVal<int>();
    int curysize = m_params["sizey"].getVal<int>();
    int x0 = m_params["x0"].getVal<int>();
    int y0 = m_params["y0"].getVal<int>();
    int maxxsize = (int)m_params["sizex"].getMax();
    int maxysize = (int)m_params["sizey"].getMax();
    bool resizeRequired = (x0 > 0 || y0 > 0);

    int type = 0;

    bool copyExternal = false;
    bool hasLiveList = false;

    if (externalDataObject)
    {
        copyExternal = true;
    }

    if (m_autoGrabbingListeners.size() > 0)
    {
        hasLiveList = true;
    }

    //obtain frame from camera
    Result = Camera.GetFrame(&frame);

    if (Result != 0)
    {
        retValue += AlliedChkError(Result);
    }

    if (frame.Flags & FGF_INVALID)
    {
        retValue += ito::RetVal(ito::retError, 0, tr("Framebuffer of FireWire-DLL invalid during retrieveImage").toLatin1().data());
    }

    retValue += AlliedChkError(Camera.GetParameter(FGP_XSIZE, &m_xSize));
    retValue += AlliedChkError(Camera.GetParameter(FGP_YSIZE, &m_ySize));
    if (!retValue.containsError())
    {
        //copy frame.pData to m_data or externalDataObject

        if (m_params["bpp"].getVal<int>() == 8 )
        {
            if (copyExternal) retValue += externalDataObject->copyFromData2D<ito::uint8>((ito::uint8*)frame.pData, m_xSize, m_ySize);
            if (!copyExternal || hasLiveList) retValue += m_data.copyFromData2D<ito::uint8>((ito::uint8*)frame.pData, m_xSize, m_ySize);
        }
        else if (m_params["bpp"].getVal<int>() == 16)
        {
            if (copyExternal) retValue += externalDataObject->copyFromData2D<ito::uint16>((ito::uint16*)frame.pData, m_xSize, m_ySize);
            if (!copyExternal || hasLiveList) retValue += m_data.copyFromData2D<ito::uint16>((ito::uint16*)frame.pData, m_xSize, m_ySize);

            ito::uint16 *rowPtr = NULL;
            ito::uint8 *rowPtr8 = NULL;
            ito::uint8 *framePtr = (ito::uint8*)frame.pData;

            size_t frameIdx;
            cv::Mat *mat;

            if (copyExternal)
            {
                //with respect to the byte-order of the camera-channel (big-endian) it must be swapped to little-endian for itom
                mat = externalDataObject->getCvPlaneMat(0);
                frameIdx = 0;
            
                for (size_t m = 0; m < m_ySize ; m++)
                {
                    rowPtr = mat->ptr<ito::uint16>(m);
                    rowPtr8 = (ito::uint8*)rowPtr;
                    for (size_t n = 0; n < m_xSize ; n++)
                    {
                        rowPtr8[n * 2 + 1] = framePtr[frameIdx];
                        rowPtr8[n * 2] = framePtr[frameIdx + 1];
                        frameIdx += 2;

                        //rowPtr8[n*2+1] = framePtr[frameIdx++];
                        //rowPtr8[n*2] = framePtr[frameIdx++];
                    }
                }
            }
            
            if (!copyExternal || hasLiveList)
            {
                //with respect to the byte-order of the camera-channel (big-endian) it must be swapped to little-endian for itom
                mat = m_data.get_mdata()[0];
                frameIdx = 0;
                for (size_t m = 0; m < m_ySize ; m++)
                {
                    rowPtr = mat->ptr<ito::uint16>(m);
                    rowPtr8 = (ito::uint8*)rowPtr;
                    for (size_t n = 0; n < m_xSize ; n++)
                    {
                        rowPtr8[n * 2 + 1] = framePtr[frameIdx];
                        rowPtr8[n * 2] = framePtr[frameIdx + 1];
                        frameIdx += 2;

                        //rowPtr8[n*2+1] = framePtr[frameIdx++];
                        //rowPtr8[n*2] = framePtr[frameIdx++];
                    }
                }
            }
        }
    }

    //returns the frame back to standby queue of camera. PutFrame(NULL) in acquire then puts it to DMA queue, where camera can put new images in.

    retValue += AlliedChkError(Camera.PutFrame(&frame));
    return retValue;
}

#endif

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal FireGrabber::getVal(void *vpdObj, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);
    ito::DataObject *dObj = reinterpret_cast<ito::DataObject *>(vpdObj);

    retValue += retrieveData();

    if (!retValue.containsError())
    {
        if (dObj == NULL)
        {
            retValue += ito::RetVal(ito::retError, 1004, tr("data object of getVal is NULL or cast failed").toLatin1().data());
        }
        else
        {
            retValue += sendDataToListeners(0);
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
//! Returns the grabbed camera frame as a deep copy.
/*!
    This method copies the recently grabbed camera frame to the given DataObject. Therefore this camera size must fit to the data structure of the 
    DataObject.

    \note This method is similar to VideoCapture::retrieve() of openCV

    \param [in,out] vpdObj is the pointer to a given dataObject (this pointer should be cast to ito::DataObject*) where the acquired image is deep copied to.
    \param [in] waitCond is the semaphore (default: NULL), which is released if this method has been terminated
    \return retOk if everything is ok, retError is camera has not been started or no image has been acquired by the method acquire.
    \sa DataObject, acquire
*/
ito::RetVal FireGrabber::copyVal(void *vpdObj, ItomSharedSemaphore *waitCond)
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
        sendDataToListeners(0); //don't wait for live image, since user should get the image as fast as possible.
    }

    if (waitCond) 
    {
        waitCond->returnValue = retValue;
        waitCond->release();
    }

    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
//! slot invoked if gain or offset parameters in docking toolbox have been manually changed
/*!
    \param [in] gain
    \param [in] offset
*/
void FireGrabber::GainOffsetPropertiesChanged(double gain, double offset)
{
    setParam( QSharedPointer<ito::ParamBase>(new ito::ParamBase("gain", ito::ParamBase::Double, gain)), NULL);
    setParam( QSharedPointer<ito::ParamBase>(new ito::ParamBase("offset", ito::ParamBase::Double, offset)), NULL);
}

//----------------------------------------------------------------------------------------------------------------------------------
//! slot invoked if gain or offset parameters in docking toolbox have been manually changed
/*!
    \param [in] gain
    \param [in] offset
*/
void FireGrabber::IntegrationPropertiesChanged(double integrationtime)
{
    setParam( QSharedPointer<ito::ParamBase>(new ito::ParamBase("integration_time", ito::ParamBase::Double, integrationtime)), NULL);
}


//----------------------------------------------------------------------------------------------------------------------------------
//! slot called if the dock widget of the plugin becomes (in)visible
/*!
Overwrite this method if the plugin has a dock widget. If so, you can connect the parametersChanged signal of the plugin
with the dock widget once its becomes visible such that no resources are used if the dock widget is not visible. Right after
a re-connection emit parametersChanged(m_params) in order to send the current status of all plugin parameters to the dock widget.
*/
void FireGrabber::dockWidgetVisibilityChanged(bool visible)
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
// FIRE GRABBER

