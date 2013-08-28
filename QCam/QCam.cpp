#include "QCam.h"
#include "dockWidgetQCam.h"

#include <QFile>
#include <qstring.h>
#include <qstringlist.h>
#include <QtCore/QtPlugin>

#include "pluginVersion.h"

Q_DECLARE_METATYPE(ito::DataObject)

int QCam::instanceCounter = 0;


//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal QCamInterface::getAddInInst(ito::AddInBase **addInInst)
{
    QCam* newInst = new QCam();
    newInst->setBasePlugin(this);
    *addInInst = qobject_cast<ito::AddInBase*>(newInst);

    m_InstList.append(*addInInst);

    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal QCamInterface::closeThisInst(ito::AddInBase **addInInst)
{
   if (*addInInst)
   {
      delete ((QCam *)*addInInst);
      int idx = m_InstList.indexOf(*addInInst);
      m_InstList.removeAt(idx);
   }

   return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
QCamInterface::QCamInterface() : AddInInterfaceBase()
{
   m_type = ito::typeDataIO | ito::typeGrabber;
   setObjectName("QCam");

    m_description = QObject::tr("Firewire QCam cameras from QImaging");

    //for the docstring, please don't set any spaces at the beginning of the line.
    char* docstring = \
"...";
	m_detaildescription = QObject::tr(docstring);
	m_author            = "M. Holtom, College of Eng., Swansea University; M. Gronle, ITO, University Stuttgart";
    m_license           = QObject::tr("itom-plugin under LGPL / QCam driver under ???");
    m_version           = (PLUGIN_VERSION_MAJOR << 16) + (PLUGIN_VERSION_MINOR << 8) + PLUGIN_VERSION_PATCH;
    m_minItomVer        = MINVERSION;
    m_maxItomVer        = MAXVERSION;
    m_aboutThis         = tr("N.A."); 


}

//----------------------------------------------------------------------------------------------------------------------------------
QCamInterface::~QCamInterface()
{
    m_initParamsMand.clear();
    m_initParamsOpt.clear();
}

//----------------------------------------------------------------------------------------------------------------------------------
Q_EXPORT_PLUGIN2(QCaminterface, QCamInterface)


//----------------------------------------------------------------------------------------------------------------------------------
QCam::QCam() :  
	AddInGrabber(),
	m_frameCallbackRetVal(ito::retOk),
	m_frameCallbackFrameIdx(-1),
	m_waitingForAcquire(false)
{    
    ito::Param paramVal("name", ito::ParamBase::String | ito::ParamBase::Readonly | ito::ParamBase::NoAutosave, "QCam", NULL);
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param("integration_time", ito::ParamBase::Double, 0.0, 0.0, 0.0, tr("Integrationtime of CCD programmed in s").toAscii().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("gain", ito::ParamBase::Double, 0.0, 1.0, 0.0, tr("Gain").toAscii().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("offset", ito::ParamBase::Double, 0.0, 1.0, 0.0, tr("Offset").toAscii().data());
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param("x0", ito::ParamBase::Int, 0, 1391, 0, tr("first pixel of ROI in x-direction").toAscii().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("y0", ito::ParamBase::Int, 0, 1023, 0, tr("first pixel of ROI in y-direction").toAscii().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("x1", ito::ParamBase::Int, 0, 1391, 0, tr("last pixel of ROI in x-direction").toAscii().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("y1", ito::ParamBase::Int, 0, 1023, 0, tr("last pixel of ROI in y-direction").toAscii().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("sizex", ito::ParamBase::Int | ito::ParamBase::Readonly, 1, 1920, 1024, tr("width of ROI").toAscii().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("sizey", ito::ParamBase::Int | ito::ParamBase::Readonly, 1, 1920, 1024, tr("height of ROI").toAscii().data());
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param("bpp", ito::ParamBase::Int, 8, 16, 8, tr("bit depth per pixel").toAscii().data());
    m_params.insert(paramVal.getName(), paramVal);

    //now create dock widget for this plugin
    DockWidgetQCam *dockWidget = new DockWidgetQCam(m_params, getID());
    connect(this, SIGNAL(parametersChanged(QMap<QString, ito::Param>)), dockWidget, SLOT(valuesChanged(QMap<QString, ito::Param>)));
    connect(dockWidget, SIGNAL(changeParameters(QMap<QString, ito::ParamBase>)), this, SLOT(updateParameters(QMap<QString, ito::ParamBase>)));

    Qt::DockWidgetAreas areas = Qt::AllDockWidgetAreas;
    QDockWidget::DockWidgetFeatures features = QDockWidget::DockWidgetClosable | QDockWidget::DockWidgetFloatable | QDockWidget::DockWidgetMovable;
    createDockWidget(QString(m_params["name"].getVal<char *>()), features, areas, dockWidget);

}

//----------------------------------------------------------------------------------------------------------------------------------
QCam::~QCam()
{
   m_params.clear();
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal QCam::init(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, ItomSharedSemaphore *waitCond)
{
	ItomSharedSemaphoreLocker locker(waitCond);
    
	ito::RetVal retValue(ito::retOk);
	QCam_CamListItem  camList[10];
	unsigned long camListLen = 10;

	if (instanceCounter == 0)
	{
		retValue += errorCheck(QCam_LoadDriver());
	}
	instanceCounter++;

	if (!retValue.containsError())
	{
		//get list of cameras
		retValue += errorCheck(QCam_ListCameras(camList, &camListLen)); //camListLen is now the number of cameras available. It may be larger than your QCam_CamListItem array length!
		if (!retValue.containsError())
		{
			if (camListLen > 0)
			{
				int i = 0;
				while ((i < 10) && (camList[i].isOpen > 0))
				{
					++i;
				}
        
				if (i < 10)
				{
					retValue += errorCheck(QCam_OpenCamera(camList[i].cameraId, &m_camHandle));
				}
				else
				{
					retValue += ito::RetVal(ito::retError,0,"the first 10 connected cameras are already opened.");
				}
			}
			else
			{
				retValue += ito::RetVal(ito::retError,0,"no QCam cameras are connected to this computer");
			}
		}
	}

	if (!retValue.containsError())
	{
		retValue += errorCheck( QCam_SetStreaming( m_camHandle, true ));
	}

	if (!retValue.containsError())
	{
		unsigned long cameraType;
		QCam_GetInfo( m_camHandle, qinfCameraType,  &cameraType);
		m_identifier = QString("type: %1").arg(cameraType);

		//initialize the setting structure
		m_camSettings.size = sizeof(m_camSettings);

		//read the default settings the camera has
		QCam_ReadDefaultSettings(m_camHandle, &m_camSettings);

		QCam_Abort( m_camHandle);

		QCam_SetParam(&m_camSettings, qprmTriggerType, qcTriggerSoftware);

		//get size and bit depth of camera
		unsigned long size, height, width;
		unsigned long maxBitDepth;
		QCam_GetInfo( m_camHandle, qinfBitDepth, &maxBitDepth );

		if (maxBitDepth <= 8)
		{
			QCam_SetParam(&m_camSettings, qprmImageFormat, qfmtMono8);
		}
		else
		{
			QCam_SetParam(&m_camSettings, qprmImageFormat, qfmtMono16);
		}

		//adjust range of bpp-param
		ito::IntMeta *paramMeta = (ito::IntMeta*)(m_params["bpp"].getMeta());
		paramMeta->setMin(8);
		paramMeta->setMax(maxBitDepth);
		m_params["bpp"].setVal<int>(maxBitDepth);

		QCam_GetInfo( m_camHandle, qinfImageSize, &size );
		QCam_GetInfo( m_camHandle, qinfImageHeight, &height );
		QCam_GetInfo( m_camHandle, qinfImageWidth, &width );

		paramMeta = (ito::IntMeta*)(m_params["x0"].getMeta());
		paramMeta->setMin(0);
		paramMeta->setMax(width);
		m_params["x0"].setVal<int>(0);

		paramMeta = (ito::IntMeta*)(m_params["x1"].getMeta());
		paramMeta->setMin(0);
		paramMeta->setMax(width);
		m_params["x1"].setVal<int>(width-1);

		paramMeta = (ito::IntMeta*)(m_params["y0"].getMeta());
		paramMeta->setMin(0);
		paramMeta->setMax(height);
		m_params["y0"].setVal<int>(0);

		paramMeta = (ito::IntMeta*)(m_params["y1"].getMeta());
		paramMeta->setMin(0);
		paramMeta->setMax(height);
		m_params["y1"].setVal<int>(height-1);

		paramMeta = (ito::IntMeta*)(m_params["sizex"].getMeta());
		paramMeta->setMin(0);
		paramMeta->setMax(width);
		m_params["sizex"].setVal<int>(width);

		paramMeta = (ito::IntMeta*)(m_params["sizey"].getMeta());
		paramMeta->setMin(0);
		paramMeta->setMax(height);
		m_params["sizey"].setVal<int>(height);

		retValue += checkData(); //resize the internal dataObject m_data to the right size and type

		//finally send the new settings to the camera
		retValue += errorCheck( QCam_SendSettingsToCam( m_camHandle,&m_camSettings ) );
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
ito::RetVal QCam::close(ItomSharedSemaphore *waitCond)
{
	ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);
	
	if (m_timerID > 0)
	{ 
		killTimer(m_timerID);
		m_timerID=0;
	}
    
    if (m_camHandle > 0)
    {
		retValue += errorCheck( QCam_SetStreaming( m_camHandle, false ));
        retValue += errorCheck( QCam_CloseCamera(m_camHandle));
    }

	instanceCounter--;
	if (instanceCounter == 0)
	{
		QCam_ReleaseDriver();
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
    \details This method copies the complete tparam of the corresponding parameter to val

    \param [in,out] val  is a input of type::tparam containing name, value and further informations
    \param [in] waitCond is the semaphore (default: NULL), which is released if this method has been terminated
    \return retOk in case that everything is ok, else retError
    \sa ito::tParam, ItomSharedSemaphore
*/
ito::RetVal QCam::getParam(QSharedPointer<ito::Param> val, ItomSharedSemaphore *waitCond)
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
ito::RetVal QCam::setParam(QSharedPointer<ito::ParamBase> val, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);

    int ret = 0;
	int i = 0;
	int maxxsize = 1;
	int maxysize = 1;

	int runningANDstopped = 0;

	int vbin = 0;
	int hbin = 0;
	double gain = 0.0;
    double offset = 0.0;
	
	int trigger_mode = 0;
    int trigger_on = 0;
	
	QString key = val->getName();

//    if(key == "")	// Check if the key is valied
//    {
//        retValue += ito::RetVal(ito::retError, 0, tr("name of given parameter is empty.").toAscii().data());
//    }
//    else	// key valid so go on
//    {
//		QMap<QString, ito::Param>::iterator paramIt = m_params.find(key);	// try to find the parameter in the parameter list
//
//		if (paramIt != m_params.end()) // Okay the camera has this parameter so go on
//		{
//    
//            if(paramIt->getFlags() & ito::ParamBase::Readonly)
//            {
//                retValue += ito::RetVal(ito::retWarning, 0, tr("Parameter is read only, input ignored").toAscii().data());
//                goto end;
//            }
//			else if(val->isNumeric() && paramIt->isNumeric())
//			{
//				double curval = val->getVal<double>();
//				if( curval > paramIt->getMax())
//				{
//				    retValue += ito::RetVal(ito::retError, 0, tr("New value is larger than parameter range, input ignored").toAscii().data());
//                    goto end;
//				}
//				else if(curval < paramIt->getMin())
//				{
//				    retValue += ito::RetVal(ito::retError, 0, tr("New value is smaller than parameter range, input ignored").toAscii().data());
//                    goto end;
//				}
//				else 
//				{
//				    paramIt.value().setVal<double>(curval);
//				}
//			}
//			else if (paramIt->getType() == val->getType())
//			{
//				retValue += paramIt.value().copyValueFrom( &(*val) );
//			}
//			else
//			{
//				retValue += ito::RetVal(ito::retError, 0, tr("Parameter type conflict").toAscii().data());
//				goto end;
//			}
//		
//			Sleep(5);
//
//			trigger_mode = m_params["trigger_mode"].getVal<int>();
//            trigger_on = m_params["trigger_enable"].getVal<int>();
//			gain = (double)(m_params["gain"].getVal<double>()); 
//            offset = (double)(m_params["offset"].getVal<double>());	
//            int timeout_ms = (int)(m_params["timeout"].getVal<double>()*1000);	
//			//vbin = m_params["binning"].getVal<int>();
//			//hbin = m_params["binning"].getVal<int>();
//
//            if(!key.compare("gain"))
//            {
//                unsigned short  gainMin = 0, gainMax = 0;
//                m_pC1394gain->GetRange(&gainMin, &gainMax);
//			    m_pC1394gain->SetValue((unsigned short)((gainMax-gainMin)*gain)+gainMin, 0);
//            }
//            else if(!key.compare("offset"))
//            {
//                unsigned short offsMin, offsMax, offsoldVal, dummy;
//			    m_pC1394offset->GetRange(&offsMin, &offsMax);
//			    m_pC1394offset->GetValue(&offsoldVal, &dummy);
//			    unsigned short offsVal = (unsigned short)((offsMax-offsMin)*offset)+offsMin;
//			    ret = m_pC1394offset->SetValue(offsVal, 0);
//			    ret = m_ptheCamera->CaptureImage();
//			    if (ret)
//			    {
//				    m_pC1394offset->SetValue(offsoldVal, 0);
//			    }
//            }
//            else 
//            {
//			    if (grabberStartedCount())
//			    {
//				    runningANDstopped = grabberStartedCount();
//				    setGrabberStarted(1);
//				    retValue += this->stopDevice(0);
//			    }
//                if(!key.compare("trigger_mode"))
//                {
//                    if (m_ptheCamera->HasFeature(FEATURE_TRIGGER_MODE))
//			        {
//				        ret=m_pC1394trigger->SetMode((unsigned short)trigger_mode);
//			        }
//			        else
//                        retValue = ito::RetVal(ito::retError, 0, tr("Camera has no trigger feature").toAscii().data());
//                }
//                else if(!key.compare("trigger_enable"))
//                {
//			        if (m_ptheCamera->HasFeature(FEATURE_TRIGGER_MODE))
//			        {
//				        if(trigger_on > 0)
//				        {
//					        m_pC1394trigger->SetOnOff(true);
//					        if ( (ret = m_ptheCamera->StartImageAcquisitionEx( 6, timeout_ms, ACQ_START_VIDEO_STREAM)))  
//					        {
//						        if (ret == -14)
//                                    retValue = ito::RetVal(ito::retError, 0, tr("FireWire: StartDataCapture failed,\nmaybe video rate too high!").toAscii().data()); 
//						        else if (ret==-15)
//                                    retValue = ito::RetVal(ito::retError, 0, tr("FireWire: StartDataCapture failed,\ntime out!").toAscii().data());
//						        return -1;
//					        }
//                            m_ptheCamera->StopImageAcquisition();
//
//					        ret = m_ptheCamera->GetNode();
//				        }
//				        else
//				        {
//					        m_pC1394trigger->SetOnOff(false);
//					        if ( (ret = m_ptheCamera->StartImageAcquisition() ) )  
//					        {
//						        if (ret == -14)
//                                    retValue = ito::RetVal(ito::retError, 0, tr("FireWire: StartDataCapture failed,\nmaybe video rate too high!").toAscii().data()); 
//						        else if (ret==-15)
//                                    retValue = ito::RetVal(ito::retError, 0, tr("FireWire: StartDataCapture failed,\ntime out!").toAscii().data());
//						        return -1;
//					        }
//					        ret = m_ptheCamera->GetNode();
//                            m_ptheCamera->StopImageAcquisition();
//				        }
//			        }
//			        else
//				        retValue = ito::RetVal(ito::retError, 0, tr("FireWire: StartDataCapture failed,\ntime out!").toAscii().data());
//                }
//            }
//		}
//		else
//		{
//			retValue = ito::RetVal(ito::retWarning, 0, tr("Parameter not found").toAscii().data());
//		}
//	}
//
//end:

	retValue += checkData();

	if (runningANDstopped)
	{
		retValue += this->startDevice(0);
		setGrabberStarted(runningANDstopped);
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
const ito::RetVal QCam::showConfDialog(void)
{
	ito::RetVal retValue(ito::retOk);
	int bitppix_old = 12;
	int binning_old = 0;
	int bitppix_new = 12;
	int binning_new = 0;
	double offset_new = 0.0;

    DialogQCam *confDialog = new DialogQCam();

    if(grabberStartedCount() > 0)
        return ito::RetVal(ito::retWarning, 0, tr("Please run stopDevice() and shut down live data before configuration").toAscii().data());

	connect(this, SIGNAL(parametersChanged(QMap<QString, ito::Param>)), confDialog, SLOT(valuesChanged(QMap<QString, ito::Param>)));
	connect(confDialog, SIGNAL(changeParameters(QMap<QString, ito::ParamBase>)), this , SLOT(updateParameters(QMap<QString, ito::ParamBase>)));

	confDialog->setVals(&m_params);
    if (confDialog->exec())
    {
        confDialog->getVals(&m_params);
        
        foreach(const ito::ParamBase &param1, m_params)
	    {	
            retValue += setParam(QSharedPointer<ito::ParamBase>(new ito::ParamBase(param1)), NULL);
	    }
    }
    delete confDialog;

    return retValue;
}



//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal QCam::startDevice(ItomSharedSemaphore *waitCond)
{
	ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);

    if(grabberStartedCount() < 1)
    {
		QCam_Abort( m_camHandle);

        //start your camera
		unsigned long size;
		QCam_GetInfo(m_camHandle, qinfImageSize, &size);

		for (int i = 0; i < NUMBERBUFFERS; ++i)
		{
			m_frames[i].pBuffer = new byte[size];
			m_frames[i].bufferSize = size;
			memset( m_frames[i].pBuffer, 0, size * sizeof(byte) );
			QCam_QueueFrame(m_camHandle, &(m_frames[i]), qCamFrameCallback, qcCallbackDone, this, i);
		}
    }

    incGrabberStarted();

    if (waitCond)
    {
        waitCond->returnValue = retValue;
        waitCond->release();
    }
    return retValue;
}
         
//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal QCam::stopDevice(ItomSharedSemaphore *waitCond)
{
	ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);

    decGrabberStarted();

	if(grabberStartedCount() < 1)
	{
		//stop your camera
		QCam_Abort(m_camHandle);

		for (int i = 0; i < NUMBERBUFFERS; ++i)
		{
			delete[] m_frames[i].pBuffer;
			m_frames[i].pBuffer = NULL;
		}

	}
    
	if(grabberStartedCount() < 0)
	{
		retValue += ito::RetVal(ito::retWarning, 0, tr("camera has already stopped").toAscii().data());
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
ito::RetVal QCam::acquire(const int trigger, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);

    ito::RetVal retValue(ito::retOk);

	if (grabberStartedCount() <= 0)
	{
		retValue = ito::RetVal(ito::retError, 0, tr("Tried to acquire without starting device").toAscii().data());
	}
	else
	{
		//invalidate all callback-related member variables
		m_frameCallbackFrameIdx = -1;
		m_frameCallbackRetVal = ito::retOk;
		m_waitingForAcquire = true;
		QCam_Trigger(m_camHandle);
	}

    if (waitCond)
    {
        waitCond->returnValue = retValue;
        waitCond->release();  
    }
	return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
void QCam::frameCallback(unsigned long userData, QCam_Err errcode, unsigned long flags)
{
	qDebug() << "frameCallback" << userData;
	if (flags & qcCallbackDone)
	{
		if (userData < 0 || userData >= NUMBERBUFFERS)
		{
			m_frameCallbackRetVal = ito::RetVal(ito::retError,0,"callback-done failed. Corrupt data!");
		}
		else
		{
			if (errcode == qerrSuccess)
			{
				m_frameCallbackFrameIdx = userData;
			}
			else if (errcode == qerrBlackFill)
			{
				m_frameCallbackRetVal = ito::RetVal(ito::retWarning,0,"errors while data transmission. Pixels might be filled with black values");
				m_frameCallbackFrameIdx = userData;
			}
		}

		m_waitingForAcquire = false;
	}
	//else(...)
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal QCam::retrieveData(ito::DataObject *externalDataObject)
{
    ito::RetVal retValue(ito::retOk);

	if (m_waitingForAcquire)
	{
		while(m_waitingForAcquire)
		{
			QCoreApplication::processEvents();
		}
	}

	if (m_frameCallbackFrameIdx == -1)
	{
		retValue += ito::RetVal(ito::retError,0,"no grabbed image data available");
		retValue += requeueFrame();
		return retValue;
	}

	QCam_Frame *currentFrame = &(m_frames[m_frameCallbackFrameIdx]);

	int width = m_params["sizex"].getVal<int>(); 
	int height = m_params["sizey"].getVal<int>(); 
	int bpp = m_params["bpp"].getVal<int>();

	//check whether the data in currentFrame correspond to width and height
	if (currentFrame->width != width || currentFrame->height != height)
	{
		retValue += ito::RetVal(ito::retError,0,"size of image data does not fit to size parameters of plugin");
		retValue += requeueFrame();
		return retValue;
	}

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

	switch(bpp)
	{
	case 8:
		if (copyExternal)
		{
			retValue += externalDataObject->copyFromData2D<ito::uint8>((const ito::uint8*)(currentFrame->pBuffer), width, height);
		}
		if (!copyExternal || hasListeners)
		{
			retValue += m_data.copyFromData2D<ito::uint8>((const ito::uint8*)(currentFrame->pBuffer), width, height);
		}
		break;
	case 10:
	case 12:
	case 14:
	case 16:
		if (copyExternal)
		{
			retValue += externalDataObject->copyFromData2D<ito::uint16>((const ito::uint16*)(currentFrame->pBuffer), width, height);
		}
		if (!copyExternal || hasListeners)
		{
			retValue += m_data.copyFromData2D<ito::uint16>((const ito::uint16*)(currentFrame->pBuffer), width, height);
		}
		break;
	default:
		retValue += ito::RetVal(ito::retError,0,"unsupported bpp");
	}

	retValue += requeueFrame();

    return retValue;
}


//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal QCam::getVal(void *vpdObj, ItomSharedSemaphore *waitCond)
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
			(*dObj) = m_data;
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
ito::RetVal QCam::copyVal(void *vpdObj, ItomSharedSemaphore *waitCond)
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
ito::RetVal QCam::requeueFrame()
{
	if (m_frameCallbackFrameIdx >= 0)
	{
		QCam_QueueFrame(m_camHandle, &(m_frames[m_frameCallbackFrameIdx]), qCamFrameCallback, qcCallbackDone, this, m_frameCallbackFrameIdx);
	}

	m_frameCallbackFrameIdx = -1; 
	m_frameCallbackRetVal = ito::retOk;

	return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
// Moved to addInGrabber.cpp, equal for all grabbers / ADDA


//----------------------------------------------------------------------------------------------------------------------------------
void QCam::updateParameters(QMap<QString, ito::ParamBase> params)
{ 
	foreach(const ito::ParamBase &param1, params)
	{	
        setParam(QSharedPointer<ito::ParamBase>(new ito::ParamBase(param1)), NULL);
	}
}


//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal QCam::errorCheck(QCam_Err errcode)
{
	ito::RetVal retval;

	switch(errcode)
	{
	case qerrSuccess:
		break;
	case qerrNotSupported:    // Function is not supported for this device
		retval += ito::RetVal(ito::retError,1,"Function is not supported for this device");
		break;
    case qerrInvalidValue:
		retval += ito::RetVal(ito::retError,2,"A parameter used was invalid");
		break;
    case qerrBadSettings:
		retval += ito::RetVal(ito::retError,3,"The QCam_Settings structure is corrupted");
		break;
	case qerrNoUserDriver:
		retval += ito::RetVal(ito::retError,4,"No user driver");
		break;
    case qerrNoFirewireDriver:
		retval += ito::RetVal(ito::retError,5,"Firewire device driver is missing");
		break;
	case qerrDriverConnection:
		retval += ito::RetVal(ito::retError,6,"Driver connection error");
		break;
    case qerrDriverAlreadyLoaded:
		retval += ito::RetVal(ito::retError,7,"The driver has already been loaded");
		break;
    case qerrDriverNotLoaded: 
		retval += ito::RetVal(ito::retError,8,"The driver has not been loaded.");
		break;
    case qerrInvalidHandle:
		retval += ito::RetVal(ito::retError,9,"The QCam_Handle has been corrupted");
		break;
    case qerrUnknownCamera:
		retval += ito::RetVal(ito::retError,10,"Camera model is unknown to this version of QCam");
		break;
    case qerrInvalidCameraId:
		retval += ito::RetVal(ito::retError,11,"Camera id used in QCam_OpenCamera is invalid");
		break;
    case qerrNoMoreConnections:
		retval += ito::RetVal(ito::retError,12,"Deprecated");
		break;
	case qerrHardwareFault:
		retval += ito::RetVal(ito::retError,13,"Hardware fault");
		break;
	case qerrFirewireFault:
		retval += ito::RetVal(ito::retError,14,"Firewire fault");
		break;
	case qerrCameraFault:
		retval += ito::RetVal(ito::retError,15,"Camera fault");
		break;
	case qerrDriverFault:
		retval += ito::RetVal(ito::retError,16,"Driver fault");
		break;
	case qerrInvalidFrameIndex:
		retval += ito::RetVal(ito::retError,17,"Invalid frame index");
		break;
    case qerrBufferTooSmall:
		retval += ito::RetVal(ito::retError,18,"Frame buffer (pBuffer) is too small for image");
		break;
	case qerrOutOfMemory:
		retval += ito::RetVal(ito::retError,19,"Out of memory");
		break;
	case qerrOutOfSharedMemory:
		retval += ito::RetVal(ito::retError,20,"Out of shared memory");
		break;
    case qerrBusy:
		retval += ito::RetVal(ito::retError,21,"The function used cannot be processed at this time");
		break;
    case qerrQueueFull:
		retval += ito::RetVal(ito::retError,22,"The queue for frame and settings changes is full");
		break;
    case qerrCancelled:
		retval += ito::RetVal(ito::retError,23,"Cancelled");
		break;
    case qerrNotStreaming:
		retval += ito::RetVal(ito::retError,24,"The function used requires that streaming be on");
		break;
    case qerrLostSync:
		retval += ito::RetVal(ito::retError,25,"The host and the computer are out of sync, the frame returned is invalid");
		break;
    case qerrBlackFill:
		retval += ito::RetVal(ito::retError,26,"Data is missing in the frame returned");
		break;
    case qerrFirewireOverflow:
		retval += ito::RetVal(ito::retError,27,"The host has more data than it can process, restart streaming.");
		break;
    case qerrUnplugged: 
		retval += ito::RetVal(ito::retError,28,"The camera has been unplugged or turned off");
		break;
    case qerrAccessDenied:
		retval += ito::RetVal(ito::retError,29,"The camera is already open");
		break;
    case qerrStreamFault:
		retval += ito::RetVal(ito::retError,30,"Stream allocation failed, there may not be enough bandwidth");
		break;
    case qerrQCamUpdateNeeded:
		retval += ito::RetVal(ito::retError,31,"QCam needs to be updated");
		break;
    case qerrRoiTooSmall:
		retval += ito::RetVal(ito::retError,32,"The ROI used is too small");
		break;
    case qerr_last:
		retval += ito::RetVal(ito::retError,33,"last");
		break;
    case _qerr_force32:
		retval += ito::RetVal(ito::retError,-1,"force32");
		break;
	}

	return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal QCam::supportedFormats(bool &mono, bool &colorFilter, bool &colorBayer)
{
	ito::uint32 mask;
	unsigned long ccdType;
	ito::RetVal retval;
	QCam_Err qerr;

	if (m_camHandle)
	{
		QCam_GetInfo(m_camHandle, qinfCcdType, &ccdType);
		if (ccdType == qcCcdMonochrome)
		{
			// Check to see if a Color Filter Wheel is supported
			// before we say that it is.
			qerr = QCam_IsRangeTable (&m_camSettings, qprmColorWheel);

			if (qerr == qerrSuccess)
			{
				mono = true;
				colorFilter = true;
			}
			else
			{
				mono = true;
				colorFilter = false;
			}
		}
		else if (ccdType == qcCcdColorBayer)
		{
			mono = false;
			colorFilter = false;
			colorBayer = true;
		}
	}
	else
	{
		retval += ito::RetVal(ito::retError,0,"invalid camera handle");
	}
	return retval;
}



//----------------------------------------------------------------------------------------------------------------------------------
void QCAMAPI qCamFrameCallback(void * userPtr, unsigned long userData, QCam_Err errcode, unsigned long flags)
{
	((QCam*)userPtr)->frameCallback(userData, errcode, flags);
}