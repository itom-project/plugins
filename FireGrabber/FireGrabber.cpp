#include "FireGrabber.h"

//#include "stdafx.h"
#include <stdio.h>
#include <conio.h>

#include <qstring.h>
#include <qstringlist.h>
#include <QtCore/QtPlugin>

#include <qmetaobject.h>
#include <qdockwidget.h>
#include <qpushbutton.h>
#include "dockWidgetFireGrabber.h"

#include "pluginVersion.h"

Q_DECLARE_METATYPE(ito::DataObject)

int FireGrabber::m_numberOfInstances = 0;  // initialization
	
//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal FireGrabberInterface::getAddInInst(ito::AddInBase **addInInst)
{
    FireGrabber* newInst = new FireGrabber();
    newInst->setBasePlugin(this);
    *addInInst = qobject_cast<ito::AddInBase*>(newInst);

    m_InstList.append(*addInInst);

    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal FireGrabberInterface::closeThisInst(ito::AddInBase **addInInst)
{
   if (*addInInst)
   {
      delete ((FireGrabber *)*addInInst);
      int idx = m_InstList.indexOf(*addInInst);
      m_InstList.removeAt(idx);
   }

   return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
FireGrabberInterface::FireGrabberInterface()
{
	m_type = ito::typeDataIO | ito::typeGrabber;
	setObjectName("FireGrabber"); 

    //for the docstring, please don't set any spaces at the beginning of the line.
    char* docstring = \
"Camera plugin that uses the FireGrab driver from the AVT FirePackage in order to communicate with corresponding cameras. The cameras are connected to the computer via \
firewire. \n\
\n\
This plugin can only be loaded and used once the AVT FirePackage driver has been correctly installed on your computer. For more information about AVT FirePackage and their \
license browse to http://www.alliedvisiontec.com. This plugin was mainly tested with the cameras AVT Malin, Guppy and Pike. Not all parameters are supported by this plugin.";

	m_description = QObject::tr("Fire Package Capture (Firewire)");
    m_detaildescription = QObject::tr(docstring);
    m_author = "A. Bielke, M. Gronle, ITO, University Stuttgart";
    m_version = (PLUGIN_VERSION_MAJOR << 16) + (PLUGIN_VERSION_MINOR << 8) + PLUGIN_VERSION_PATCH;
    m_minItomVer = MINVERSION;
    m_maxItomVer = MAXVERSION;
    m_license = QObject::tr("LGPL; you need an installed AVT FirePackage driver, which requires further licenses if you are not using any AVT camera (see AVT FirePackage documentation).");
    m_aboutThis = QObject::tr("N.A.");        
    
    
    m_autoLoadPolicy = ito::autoLoadNever;
    m_autoSavePolicy = ito::autoSaveNever;

    m_initParamsOpt.append(ito::Param("cameraID", ito::ParamBase::Int | ito::ParamBase::In, 0, std::numeric_limits<int>::max(), 0, tr("specific number of the camera, don't use with cameraNumber (0 = unused)").toAscii().data()));
    m_initParamsOpt.append(ito::Param("vendorID", ito::ParamBase::Int | ito::ParamBase::In, 0, std::numeric_limits<int>::max(), 0, tr("number of the vendor (e.g. Allied: vendorID=673537), don't use with cameraNumber (0 = unused)").toAscii().data()));
	m_initParamsOpt.append(ito::Param("cameraNumber", ito::ParamBase::Int | ito::ParamBase::In, 0, 25, 0, tr("number of order of plugging the cameras (0 = parameter not used)").toAscii().data()));
}

//----------------------------------------------------------------------------------------------------------------------------------
FireGrabberInterface::~FireGrabberInterface()
{
    m_initParamsMand.clear();
    m_initParamsOpt.clear();
}

//----------------------------------------------------------------------------------------------------------------------------------
Q_EXPORT_PLUGIN2(FireGrabberinterface, FireGrabberInterface)

//----------------------------------------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------------------------------------


const ito::RetVal FireGrabber::showConfDialog(void)
{
    dialogFireGrabber *confDialog = new dialogFireGrabber(this);

    connect(this, SIGNAL(parametersChanged(QMap<QString, ito::Param>)), confDialog, SLOT(valuesChanged(QMap<QString, ito::Param>)));
    QMetaObject::invokeMethod(this, "sendParameterRequest");

    if (confDialog->exec())
    {
        confDialog->sendVals();
    }

    disconnect(this, SIGNAL(parametersChanged(QMap<QString, ito::Param>)), confDialog, SLOT(valuesChanged(QMap<QString, ito::Param>)));
    delete confDialog;

    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
FireGrabber::FireGrabber() : 
	AddInGrabber()
{
   ito::Param paramVal("name", ito::ParamBase::String | ito::ParamBase::Readonly, "FireGrabber", NULL);
   m_params.insert(paramVal.getName(), paramVal);

   paramVal = ito::Param("vendorID", ito::ParamBase::Int | ito::ParamBase::Readonly, 0, std::numeric_limits<int>::max(), 0, tr("vendor ID of the camera").toAscii().data());
   m_params.insert(paramVal.getName(), paramVal);
   paramVal = ito::Param("cameraID", ito::ParamBase::Int | ito::ParamBase::Readonly, 0, std::numeric_limits<int>::max(), 0, tr("camera ID of the camera").toAscii().data());
   m_params.insert(paramVal.getName(), paramVal);

   paramVal = ito::Param("vendorName", ito::ParamBase::String | ito::ParamBase::Readonly, NULL, tr("vendor name of the camera [if connected]").toAscii().data());
   m_params.insert(paramVal.getName(), paramVal);

   paramVal = ito::Param("modelName", ito::ParamBase::String | ito::ParamBase::Readonly, NULL, tr("model name of the camera [if connected]").toAscii().data());
   m_params.insert(paramVal.getName(), paramVal);

   paramVal = ito::Param("integration_time", ito::ParamBase::Double, 0.001, 5000.0, 0.01, tr("Integrationtime of CCD programmed in s").toAscii().data());
   m_params.insert(paramVal.getName(), paramVal);
   paramVal = ito::Param("frame_time", ito::ParamBase::Double | ito::ParamBase::Readonly, 0.005, 150.0, 33.333333, tr("Transmission time per frame in s").toAscii().data());
   m_params.insert(paramVal.getName(), paramVal);
   
   paramVal = ito::Param("bpp", ito::ParamBase::Int, 8, 24, 8, tr("bit depth of camera").toAscii().data());
   m_params.insert(paramVal.getName(), paramVal);

   paramVal = ito::Param("brightness", ito::ParamBase::Double, 0.0, 1.0, 0.0, tr("Brightness value (if supported)").toAscii().data());
   m_params.insert(paramVal.getName(), paramVal);
   paramVal = ito::Param("sharpness", ito::ParamBase::Double, 0.0, 1.0, 0.0, tr("Sharpness value (if supported)").toAscii().data());
   m_params.insert(paramVal.getName(), paramVal);
   paramVal = ito::Param("gamma", ito::ParamBase::Int, 0, 1, 0, tr("Gamma correction (0: off, 1: on, default: off)").toAscii().data());
   m_params.insert(paramVal.getName(), paramVal);
   paramVal = ito::Param("gain", ito::ParamBase::Double, 0.0, 1.0, 0.0, tr("Virtual gain").toAscii().data());
   m_params.insert(paramVal.getName(), paramVal);
   paramVal = ito::Param("offset", ito::ParamBase::Double | ito::ParamBase::Readonly, 0.0, 0.0, 0.0, tr("Offset not used here.").toAscii().data());
   m_params.insert(paramVal.getName(), paramVal);

   paramVal = ito::Param("x0", ito::ParamBase::Int, 0, 2047, 0, tr("Startvalue for ROI").toAscii().data());
   m_params.insert(paramVal.getName(), paramVal);
   paramVal = ito::Param("y0", ito::ParamBase::Int, 0, 2047, 0, tr("Startvalue for ROI").toAscii().data());
   m_params.insert(paramVal.getName(), paramVal);

   paramVal = ito::Param("x1", ito::ParamBase::Int, 0, 2047, 2047, tr("Stopvalue for ROI").toAscii().data());
   m_params.insert(paramVal.getName(), paramVal);
   paramVal = ito::Param("y1", ito::ParamBase::Int, 0, 2047, 2047, tr("Stopvalue for ROI").toAscii().data());
   m_params.insert(paramVal.getName(), paramVal);

   paramVal = ito::Param("sizex", ito::ParamBase::Int | ito::ParamBase::Readonly, 1, 2048, 2048, tr("Pixelsize in x (cols)").toAscii().data());
   m_params.insert(paramVal.getName(), paramVal);
   paramVal = ito::Param("sizey", ito::ParamBase::Int | ito::ParamBase::Readonly, 1, 2048, 2048, tr("Pixelsize in y (rows)").toAscii().data());
   m_params.insert(paramVal.getName(), paramVal);
   
   //paramVal = ito::Param("fps", ito::ParamBase::Int | ito::ParamBase::Readonly, 0, 100, 0, tr("Read frames per second").toAscii().data());
   //m_params.insert(paramVal.getName(), paramVal);

    //now create dock widget for this plugin
    DockWidgetFireGrabber *dw = new DockWidgetFireGrabber();
    Qt::DockWidgetAreas areas = Qt::AllDockWidgetAreas;
    QDockWidget::DockWidgetFeatures features = QDockWidget::DockWidgetClosable | QDockWidget::DockWidgetFloatable | QDockWidget::DockWidgetMovable;
    createDockWidget(QString(m_params["name"].getVal<char *>()), features, areas, dw);
}


//----------------------------------------------------------------------------------------------------------------------------------
FireGrabber::~FireGrabber()
{
   m_params.clear();
}


ito::RetVal FireGrabber::AlliedChkError(int errornumber)
{
	ito::RetVal retValue = ito::retOk;
	int i;

	struct error
	{
		int value;
		const char* text;
	} 

	static const errors[] =
	{	/* All Errormassages are taken from the Fire Grab-Manual. */ 	
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
		{ FCE_ADJUSTED           /* 1022*/,  "A pararmeter had to be adjusted"},
	};

    if (errornumber > 0)
    {
        bool found = false;

	    for (i = 0; i < sizeof(errors) / sizeof(errors[0]); i++)
	    {
		    if (errors[i].value == errornumber)
		    {
			    retValue += ito::RetVal::format(ito::retError, 0, tr(&errors[i].text[0]).toAscii().data());
			    found = true;
		    }
	    }

        if (!found)
        {
            retValue += ito::RetVal(ito::retError, 0, tr("Unknown Error Code").toAscii().data());
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

    if(retValue == ito::retOk)
    {
        //gets the parameter key from m_params map (read-only is allowed, since we only want to get the value).
        retValue += apiGetParamFromMapByKey(m_params, key, it, false);
    }

    if(!retValue.containsError())
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
    QMap<QString, ito::Param>::iterator it;
    UINT32 Result = 0;

    //parse the given parameter-name (if you support indexed or suffix-based parameters)
    retValue += apiParseParamName( val->getName(), key, hasIndex, index, suffix );

    if(!retValue.containsError())
    {
        //gets the parameter key from m_params map (read-only is not allowed and leads to ito::retError).
        retValue += apiGetParamFromMapByKey(m_params, key, it, true);
    }

    if(!retValue.containsError())
    {
        //here the new parameter is checked whether it's type corresponds or can be cast into the
        // value in m_params and whether the new type fits to the requirements of any possible
        // meta structure.
        retValue += apiValidateParam(*it, *val, false, true);
    }

    if(!retValue.containsError())
    {
        if ((!key.compare("x1") || !key.compare("y1") || !key.compare("x0") || !key.compare("y0")))
	    {
            if (m_isgrabbing == true)
            {
		        retValue += ito::RetVal(ito::retError, 0, tr("camera is grabbing, stop camera first").toAscii().data());		
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
        else
        {
            if (!key.compare("integration_time"))
		    {
			    unsigned long dblVal = (unsigned long)(exposureSecToShutter(val->getVal<double>()));
			    Result = Camera.SetParameter(FGP_SHUTTER, dblVal);
                retValue += AlliedChkError(Result);
		    }
		    else if (!key.compare("bpp"))
		    {
			    Result = Camera.GetParameter(FGP_XSIZE, &m_xSize);
			    Result = Camera.GetParameter(FGP_YSIZE, &m_ySize);

			    if (it->getVal<int>() == 8)
			    {
				    Result = Camera.SetParameter(FGP_IMAGEFORMAT, MAKEIMAGEFORMAT(RES_SCALABLE, CM_Y8, 0));
			    }
			    else if (it->getVal<int>() == 16)
			    {
				    Result = Camera.SetParameter(FGP_IMAGEFORMAT, MAKEIMAGEFORMAT(RES_SCALABLE, CM_Y16, 0));
			    }
			    else
			    {
				    retValue += ito::RetVal(ito::retError, 0, tr("only 8 and 16 bit are implemented").toAscii().data());
			    }

                retValue += AlliedChkError(Result);

			    if (!retValue.containsError())
			    {
				    Result = Camera.SetParameter(FGP_XSIZE, m_xSize);
				    Result = Camera.SetParameter(FGP_YSIZE, m_ySize);
                    retValue += AlliedChkError(Result);
			    }
		    }
		    else if (!key.compare("gain"))
		    {
                FGPINFO valInfo = m_camProperties["gain"];
                unsigned long valnew = valInfo.MinValue + (valInfo.MaxValue - valInfo.MinValue) * val->getVal<double>();
                retValue += AlliedChkError(Camera.SetParameter(FGP_GAIN, valnew));
		    }
            else if (!key.compare("brightness"))
		    {
                FGPINFO valInfo = m_camProperties["brightness"];
                unsigned long valnew = valInfo.MinValue + (valInfo.MaxValue - valInfo.MinValue) * val->getVal<double>();
                retValue += AlliedChkError(Camera.SetParameter(FGP_BRIGHTNESS, valnew));
		    }
            else if (!key.compare("sharpness"))
		    {
                FGPINFO valInfo = m_camProperties["sharpness"];
                unsigned long valnew = valInfo.MinValue + (valInfo.MaxValue - valInfo.MinValue) * val->getVal<double>();
                retValue += AlliedChkError(Camera.SetParameter(FGP_SHARPNESS, valnew));
		    }
            else if (!key.compare("gamma"))
		    {
                retValue += AlliedChkError(Camera.SetParameter(FGP_GAMMA, it->getVal<int>()));
		    }
		    
            if (!retValue.containsError())
            {
                retValue += it->copyValueFrom( &(*val) );
            }
        }

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

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal FireGrabber::adjustROI(int x0, int x1, int y0, int y1)
{
    FGPINFO xSizeInfo, ySizeInfo, xPosInfo, yPosInfo;
    ito::RetVal retval;
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
        if(IMGRES(format) != RES_SCALABLE)
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
                retval += AlliedChkError(Camera.SetParameter(FGP_XPOSITION, x0));
                retval += AlliedChkError(Camera.SetParameter(FGP_XSIZE, sizex));
                retval += AlliedChkError(Camera.SetParameter(FGP_YPOSITION, y0));
                retval += AlliedChkError(Camera.SetParameter(FGP_YSIZE, sizey));

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

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal FireGrabber::init(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, ItomSharedSemaphore *waitCond)
{
	ItomSharedSemaphoreLocker locker(waitCond);
	ito::RetVal retValue(ito::retOk);
    UINT32 Result = 0;
    
	//Allied: vendorID = 673537
	//Allied Pike: cameraID = 269109919 (specific!!!)
	//Allied Marlin: cameraID = 235356461 (specific!!!)
	
	// get the optional parameters "cameraID" and "vendorID"
	int camID = NULL;
	int venID = NULL;
	int plugNR = NULL;
	int nodeNR = NULL;
	FGPINFO shutterinfo;
	int tempID = NULL;
    camID = paramsOpt->value(0).getVal<int>();
	venID = paramsOpt->value(1).getVal<int>();
	plugNR = paramsOpt->value(2).getVal<int>();
	int connected = false;
    UINT32 NodeCnt;
    FGNODEINFO NodeInfo[25];
	
	if ((camID != 0 || venID != 0) && plugNR != 0)
	{
		camID = 0;
		venID = 0;
		retValue += ito::RetVal(ito::retWarning, 0, tr("too much input parameters, plugin number is used").toAscii().data());
	}

	FireGrabber::m_numberOfInstances++;

	if (FireGrabber::m_numberOfInstances == 1) // we are the first user, open DLL
	{
		retValue += AlliedChkError(FGInitModule(NULL));
	}

	Result = FGGetNodeList(NodeInfo, 25, &NodeCnt);  // Get list of connected nodes
	
	if (plugNR != 0)
	{
		Result = Camera.Connect(&NodeInfo[plugNR - 1].Guid);
		if(Result == 0)
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
					    retValue += ito::RetVal(ito::retWarning, 0, tr("same cameraID for more than one camera, connected the first plugged and free camera").toAscii().data());
				    }
				    if (camID == 0 && venID != 0 && (NodeInfo[i].Guid.High == NodeInfo[j + 1].Guid.High))
				    {
					    retValue += ito::RetVal(ito::retWarning, 0, tr("same vendorID for more than one camera, connected the first plugged and free camera").toAscii().data());
				    }
			    }
		    }
	    }
	
	    // find the right port of demanded camera and connect it
	    for (unsigned long i = 0; i <= NodeCnt-1; i++)
	    {
		    if (camID == NodeInfo[i].Guid.Low && venID == NodeInfo[i].Guid.High)	//camID and venID given
		    {
				    Result = Camera.Connect(&NodeInfo[i].Guid);		// Connect with node i
					if(Result == 0)
					{
						connected = true;
					}
				    nodeNR = i;
				    break;
		    }
		    else if (camID == NodeInfo[i].Guid.Low && NodeInfo[i].Busy == 0 && venID == 0)	//only camID given
		    {
				    Result = Camera.Connect(&NodeInfo[i].Guid);
					if(Result == 0)
					{
						connected = true;
					}
				    nodeNR = i;
				    break;
		    }
		    else if (camID == 0 && venID == NodeInfo[i].Guid.High && NodeInfo[i].Busy == 0)	//only venID given
		    {
				    Result = Camera.Connect(&NodeInfo[i].Guid);
					if(Result == 0)
					{
						connected = true;
					}
				    nodeNR = i;
				    break;
		    }
		    else if (camID == 0 && venID == 0 && NodeInfo[i].Busy == 0) //no camID and venID given, connect the first plugged and free camera to its node
		    {
				    Result = Camera.Connect(&NodeInfo[i].Guid);   // Connect with node i
					if(Result == 0)
					{
						connected = true;
					}
				    nodeNR = i;
				    break;
		    }
	    }
	}

	// Set camera to scalable mode and max size
	if (Result == 0 && connected)
	{
        //check available parameters
        FGPINFO info, xInfo, yInfo;
        double val;

        tempID = NodeInfo[nodeNR].Guid.High;
		m_params["vendorID"].setVal(tempID);
		tempID = NodeInfo[nodeNR].Guid.Low;
		m_params["cameraID"].setVal(tempID);
        
        char *vendorName = new char[512];
        vendorName[511] = '\0'; //zero-terminate it for security reason
        char *modelName = new char[512];
        modelName[511] = '\0'; //zero-terminate it for security reason

        Result = Camera.GetDeviceName(vendorName, 511, modelName); //vendorName and modelName are zero-terminated then
        if (Result == 0)
        {
            m_params["vendorName"].setVal<char*>(vendorName);
            m_params["modelName"].setVal<char*>(modelName);
            m_identifier = QString("%1 (%2)").arg(modelName).arg(vendorName);

            if (strcmp(vendorName,"AVT") == 0)
            {
                m_exposureParams.AVTCam = true;
                UINT32 regValue;
                //try to read timebase register
                Result = Camera.ReadRegister(0xF1000208, &regValue);

                if (Result == FCE_NOERROR)
                {
                    if (regValue & 0x80000000) //timebase present
                    {
                        int id = regValue & 0x0000000f;
                        int timebases[] = {1,2,5,10,20,50,100,200,500,1000};
                        m_exposureParams.timebaseMs = timebases[id] / 1000.0;
                    }
                    else
                    {
                        retValue += ito::RetVal(ito::retWarning,0,"timebase register of camera is not available. Timebase is set to 20µs per default");
                        m_exposureParams.timebaseMs = 20.0 / 1000.0;
                    }
                }
                else
                {
                    retValue += ito::RetVal(ito::retWarning,0,"timebase register of camera could not be read. Timebase is set to 20µs per default");
                    m_exposureParams.timebaseMs = 20.0 / 1000.0;
                }

                QMap<QString,int> offsets;
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
                    retValue += ito::RetVal(ito::retWarning,0,"no exposure offset is available for this camera model. Therefore the offset is set to 0 and your exposure time might be few microseconds smaller than the real value.");
                    m_exposureParams.offsetMs = 0.0;
                }
            }
            else
            {
                m_exposureParams.AVTCam = false;
                m_exposureParams.offsetMs = 0.0;
                m_exposureParams.timebaseMs = 20.0;
                retValue += ito::RetVal(ito::retWarning,0,"Camera model is not known. Therefore the integration time represents the shutter value, not the real exposure time in seconds.");
            }
        }
        else
        {
            m_identifier = QString::number(tempID);
        }

        delete[] vendorName; vendorName = NULL;
        delete[] modelName; modelName = NULL;

        //brightness
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

        //sharpness
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

        //gamma
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

        //gain
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

        //shutter
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

		Result = Camera.SetParameter(FGP_XPOSITION, 0);
		Result = Camera.SetParameter(FGP_YPOSITION, 0);
		Result = Camera.SetParameter(FGP_XSIZE, xInfo.MaxValue);
		Result = Camera.SetParameter(FGP_YSIZE, yInfo.MaxValue);

		m_params["sizex"].setVal<double>(xInfo.MaxValue);
		m_params["sizey"].setVal<double>(yInfo.MaxValue);
		m_params["x1"].setVal<double>(xInfo.MaxValue - 1);
		m_params["y1"].setVal<double>(yInfo.MaxValue - 1);

		// Obtain data geometry
		Result = Camera.GetParameterInfo(FGP_XSIZE, &xInfo);
		Result = Camera.GetParameterInfo(FGP_YSIZE, &yInfo);
	
		static_cast<ito::IntMeta*>( m_params["sizex"].getMeta() )->setMax(xInfo.MaxValue);
		static_cast<ito::IntMeta*>( m_params["sizey"].getMeta() )->setMax(yInfo.MaxValue);
		static_cast<ito::IntMeta*>( m_params["sizex"].getMeta() )->setMin(xInfo.MinValue);
		static_cast<ito::IntMeta*>( m_params["sizey"].getMeta() )->setMin(yInfo.MinValue);
		static_cast<ito::IntMeta*>( m_params["x0"].getMeta() )->setMax(xInfo.MaxValue - 1);
		static_cast<ito::IntMeta*>( m_params["y0"].getMeta() )->setMax(yInfo.MaxValue - 1);
		static_cast<ito::IntMeta*>( m_params["x1"].getMeta() )->setMax(xInfo.MaxValue - 1);
		static_cast<ito::IntMeta*>( m_params["y1"].getMeta() )->setMax(yInfo.MaxValue - 1);

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
	else if(connected == false)
	{
		retValue += ito::RetVal(ito::retError,0, tr("no camera has been connected").toAscii().data());
	}
	
	m_isgrabbing = false;

	retValue += checkData(); 

    if (waitCond)
    {
        waitCond->returnValue = retValue;
        waitCond->release();
    }

    setInitialized(true); //init method has been finished (independent on retval)

    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal FireGrabber::close(ItomSharedSemaphore *waitCond)
{
	ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);
	
    if (grabberStartedCount() > 0)
    {
		Camera.CloseCapture();
    }

	Camera.Disconnect();						// Disconnect camera

	FireGrabber::m_numberOfInstances--;
	if (FireGrabber::m_numberOfInstances < 1) //we are the last user, close DLL
	{
		FGExitModule();							// Exit module
	}

    if (waitCond)
    {
        waitCond->returnValue = retValue;
        waitCond->release();
    }
	
	return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
double FireGrabber::shutterToExposureSec(int shutter)
{
    if(m_exposureParams.AVTCam)
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
    if(m_exposureParams.AVTCam)
    {
        return ((exposure * 1000) - m_exposureParams.offsetMs) / m_exposureParams.timebaseMs;
    }
    else
    {
        return static_cast<int>(exposure);
    }
}

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
		    // Start DMA logic
		    retValue += AlliedChkError(Camera.OpenCapture());
	
		    if (!retValue.containsError())
		    {
                // Start image device
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
         
//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal FireGrabber::stopDevice(ItomSharedSemaphore *waitCond)
{
	ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);

    decGrabberStarted();
    if (grabberStartedCount() == 0)
    {
        // Start image device
	    retValue += AlliedChkError(Camera.StopDevice());

		retValue += AlliedChkError(Camera.CloseCapture());
    }
    else if (grabberStartedCount() < 0)
    {
        retValue += ito::RetVal(ito::retWarning, 1001, tr("stopDevice not executed since camera has not been started.").toAscii().data());
        setGrabberStarted(0);
    }

	m_isgrabbing = false;

    if (waitCond)
    {
        waitCond->returnValue = retValue;
        waitCond->release();
    }

	//Result = Camera.StopDevice();	// Stop the device

	return ito::retOk;
}
         
//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal FireGrabber::acquire(const int trigger, ItomSharedSemaphore *waitCond)
{ItomSharedSemaphoreLocker locker(waitCond);

    ito::RetVal retValue(ito::retOk);

    retValue += AlliedChkError(Camera.DiscardFrames());
    retValue += AlliedChkError(Camera.PutFrame(NULL));

	m_isgrabbing = true;


    if (waitCond)
    {
        waitCond->returnValue = retValue;
        waitCond->release();  
    }
	return retValue;
}





//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal FireGrabber::retrieveData(ito::DataObject *externalDataObject)
{
    ito::RetVal retValue(ito::retOk);
    UINT32 Result = 0;
	
	bool RetCode = false;
    FGFRAME frame;

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
		retValue += ito::RetVal(ito::retError, 0, "Framebuffer of FireWire-DLL invalid during retrieveImage");
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
            ito::uint16 *rowPtr = NULL;
			ito::uint8 *rowPtr8 = NULL;
			ito::uint8 *framePtr = (ito::uint8*)frame.pData;
			size_t frameIdx;
            cv::Mat *mat;

            if (copyExternal)
            {
			    //with respect to the byte-order of the camera-channel (big-endian) it must be swapped to little-endian für itom
			    mat = (cv::Mat*)externalDataObject->get_mdata()[0];
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

					    /*rowPtr8[n*2+1] = framePtr[frameIdx++];
					    rowPtr8[n*2] = framePtr[frameIdx++];*/
				    }
			    }
            }
            
            if (!copyExternal || hasLiveList)
            {
                //with respect to the byte-order of the camera-channel (big-endian) it must be swapped to little-endian für itom
			    mat = (cv::Mat*)m_data.get_mdata()[0];
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

					    /*rowPtr8[n*2+1] = framePtr[frameIdx++];
					    rowPtr8[n*2] = framePtr[frameIdx++];*/
				    }
			    }
            }
		}
	}

    //returns the frame back to standby queue of camera. PutFrame(NULL) in acquire then puts it to DMA queue, where camera can put new images in.
    retValue += AlliedChkError(Camera.PutFrame(&frame));

	//retValue += AlliedChkError(Camera.StopDevice());	// Stop the device

    return retValue;
}
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
			retValue += ito::RetVal(ito::retError, 1004, tr("data object of getVal is NULL or cast failed").toAscii().data());
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
        retValue += ito::RetVal(ito::retError, 0, tr("Empty object handle retrieved from caller").toAscii().data());
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
void FireGrabber::dockWidgetVisibilityChanged(bool visible)
{
    if (getDockWidget())
    {
        DockWidgetFireGrabber *dw = qobject_cast<DockWidgetFireGrabber*>(getDockWidget()->widget());
        if (visible)
        {
            connect(this, SIGNAL(parametersChanged(QMap<QString, ito::Param>)), dw, SLOT(valuesChanged(QMap<QString, ito::Param>)));
            connect(dw, SIGNAL(GainOffsetPropertiesChanged(double,double)), this, SLOT(GainOffsetPropertiesChanged(double,double)));
            connect(dw, SIGNAL(IntegrationPropertiesChanged(double)), this, SLOT(IntegrationPropertiesChanged(double)));

            QMetaObject::invokeMethod(dw, "setIdentifier", Q_ARG(QString, m_identifier));

            emit parametersChanged(m_params);
        }
        else
        {
            disconnect(this, SIGNAL(parametersChanged(QMap<QString, ito::Param>)), dw, SLOT(valuesChanged(QMap<QString, ito::Param>)));
            disconnect(dw, SIGNAL(GainOffsetPropertiesChanged(double,double)), this, SLOT(GainOffsetPropertiesChanged(double,double)));
            disconnect(dw, SIGNAL(IntegrationPropertiesChanged(double)), this, SLOT(IntegrationPropertiesChanged(double)));
        }
    }
}
//----------------------------------------------------------------------------------------------------------------------------------
// FIRE GRABBER