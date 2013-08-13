#include "QCam.h"
#include "dockWidgetQCam.h"

#include <QFile>
#include <qstring.h>
#include <qstringlist.h>
#include <QtCore/QtPlugin>

#include "pluginVersion.h"

Q_DECLARE_METATYPE(ito::DataObject)


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
	m_author            = "M. Gronle, ITO, University Stuttgart";
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
QCam::QCam() :  AddInGrabber()
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
    paramVal = ito::Param("sizex", ito::ParamBase::Int | ito::ParamBase::ReadOnly, 1, 1392, 1392, tr("width of ROI").toAscii().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("sizey", ito::ParamBase::Int | ito::ParamBase::ReadOnly, 1, 1024, 1024, tr("height of ROI").toAscii().data());
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param("bpp", ito::ParamBase::Int, 8, 12, 12, tr("bit depth per pixel").toAscii().data());
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

    
    
    QCam_LoadDriver();
    
    //get list of cameras
    QCam_CamListItem  camList[10];
    unsigned long camListLen = sizeof(camList) / sizeof( camList[Qcam_CamListItem]);
    QCam_ListCameras(list, &camListLen); //camListLen is now the number of cameras available. It may be larger than your QCam_CamListItem array length!
    
    if (camListLen > 0)
    {
        int i = 0;
        while (i < 10 && camList[i].isOpen == false)
        {
            ++i;
        }
        
        if (i < 10)
        {
            Qcam_OpenCamera(camList[i].cameraId, &m_camHandle);
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
        QCam_CloseCamera(m_camHandle);
    }
    
    QCam_ReleaseDriver();
    
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
    ito::RetVal retValue(ito::retOk);
    QString key = val->getName();

    if(key == "")
    {
        retValue += ito::RetVal(ito::retError, 0, tr("name of requested parameter is empty.").toAscii().data());
    }
    else
    {
        QMap<QString, ito::Param>::iterator paramIt = m_params.find(key);
        if (paramIt != m_params.end())
        {
            if(!key.compare("gain"))
            {
                unsigned short gainMin, gainMax, gainVal, dummy;
                m_pC1394gain->GetRange(&gainMin, &gainMax);
			    m_pC1394gain->GetValue(&gainVal, &dummy);
			    double gain = (gainVal-gainMin) / (double)(gainMax-gainMin);
                paramIt.value().setVal<double>(gain);
                *val = paramIt.value();
            }
            else if(!key.compare("offset"))
            {
                unsigned short offsMin, offsVal, offsMax, dummy;
			    m_pC1394offset->GetRange(&offsMin, &offsMax);
			    m_pC1394offset->GetValue(&offsVal, &dummy);
			    double offset = (offsVal-offsMin) / (double)(offsMax-offsMin);
                paramIt.value().setVal<double>(offset);
                *val = paramIt.value();
            }
            else
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

    if(key == "")	// Check if the key is valied
    {
        retValue += ito::RetVal(ito::retError, 0, tr("name of given parameter is empty.").toAscii().data());
    }
    else	// key valid so go on
    {
		QMap<QString, ito::Param>::iterator paramIt = m_params.find(key);	// try to find the parameter in the parameter list

		if (paramIt != m_params.end()) // Okay the camera has this parameter so go on
		{
    
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

			trigger_mode = m_params["trigger_mode"].getVal<int>();
            trigger_on = m_params["trigger_enable"].getVal<int>();
			gain = (double)(m_params["gain"].getVal<double>()); 
            offset = (double)(m_params["offset"].getVal<double>());	
            int timeout_ms = (int)(m_params["timeout"].getVal<double>()*1000);	
			//vbin = m_params["binning"].getVal<int>();
			//hbin = m_params["binning"].getVal<int>();

            if(!key.compare("gain"))
            {
                unsigned short  gainMin = 0, gainMax = 0;
                m_pC1394gain->GetRange(&gainMin, &gainMax);
			    m_pC1394gain->SetValue((unsigned short)((gainMax-gainMin)*gain)+gainMin, 0);
            }
            else if(!key.compare("offset"))
            {
                unsigned short offsMin, offsMax, offsoldVal, dummy;
			    m_pC1394offset->GetRange(&offsMin, &offsMax);
			    m_pC1394offset->GetValue(&offsoldVal, &dummy);
			    unsigned short offsVal = (unsigned short)((offsMax-offsMin)*offset)+offsMin;
			    ret = m_pC1394offset->SetValue(offsVal, 0);
			    ret = m_ptheCamera->CaptureImage();
			    if (ret)
			    {
				    m_pC1394offset->SetValue(offsoldVal, 0);
			    }
            }
            else 
            {
			    if (grabberStartedCount())
			    {
				    runningANDstopped = grabberStartedCount();
				    setGrabberStarted(1);
				    retValue += this->stopDevice(0);
			    }
                if(!key.compare("trigger_mode"))
                {
                    if (m_ptheCamera->HasFeature(FEATURE_TRIGGER_MODE))
			        {
				        ret=m_pC1394trigger->SetMode((unsigned short)trigger_mode);
			        }
			        else
                        retValue = ito::RetVal(ito::retError, 0, tr("Camera has no trigger feature").toAscii().data());
                }
                else if(!key.compare("trigger_enable"))
                {
			        if (m_ptheCamera->HasFeature(FEATURE_TRIGGER_MODE))
			        {
				        if(trigger_on > 0)
				        {
					        m_pC1394trigger->SetOnOff(true);
					        if ( (ret = m_ptheCamera->StartImageAcquisitionEx( 6, timeout_ms, ACQ_START_VIDEO_STREAM)))  
					        {
						        if (ret == -14)
                                    retValue = ito::RetVal(ito::retError, 0, tr("FireWire: StartDataCapture failed,\nmaybe video rate too high!").toAscii().data()); 
						        else if (ret==-15)
                                    retValue = ito::RetVal(ito::retError, 0, tr("FireWire: StartDataCapture failed,\ntime out!").toAscii().data());
						        return -1;
					        }
                            m_ptheCamera->StopImageAcquisition();

					        ret = m_ptheCamera->GetNode();
				        }
				        else
				        {
					        m_pC1394trigger->SetOnOff(false);
					        if ( (ret = m_ptheCamera->StartImageAcquisition() ) )  
					        {
						        if (ret == -14)
                                    retValue = ito::RetVal(ito::retError, 0, tr("FireWire: StartDataCapture failed,\nmaybe video rate too high!").toAscii().data()); 
						        else if (ret==-15)
                                    retValue = ito::RetVal(ito::retError, 0, tr("FireWire: StartDataCapture failed,\ntime out!").toAscii().data());
						        return -1;
					        }
					        ret = m_ptheCamera->GetNode();
                            m_ptheCamera->StopImageAcquisition();
				        }
			        }
			        else
				        retValue = ito::RetVal(ito::retError, 0, tr("FireWire: StartDataCapture failed,\ntime out!").toAscii().data());
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
        //start your camera
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
		m_isgrabbing = true;
		
		QCam_Trigger();
	}

    if (waitCond)
    {
        waitCond->returnValue = retValue;
        waitCond->release();  
    }
	return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal QCam::retrieveData(ito::DataObject *externalDataObject)
{
    ito::RetVal retValue(ito::retOk);

    unsigned long imglength = 0;
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
	
		if (retValue != ito::retError)
		{// Now we shoud have a picture in the camera buffer

			switch (m_params["bpp"].getVal<int>())
			{
				case 8:
                {
					unsigned char *cbuf=m_ptheCamera->GetRawData(&imglength);
					if (curxsize == maxxsize)
					{
						lsrcstrpos = y0 * maxxsize;
                        if(copyExternal) retValue += externalDataObject->copyFromData2D<ito::uint8>((ito::uint8*)cbuf+lsrcstrpos, maxxsize, curysize);
						if(!copyExternal || hasListeners) retValue += m_data.copyFromData2D<ito::uint8>((ito::uint8*)cbuf+lsrcstrpos, maxxsize, curysize);
					}
					else
					{
                        if(copyExternal) retValue += externalDataObject->copyFromData2D<ito::uint8>((ito::uint8*)cbuf, maxxsize, maxysize, x0, y0, curxsize, curysize);
						if(!copyExternal || hasListeners) retValue += m_data.copyFromData2D<ito::uint8>((ito::uint8*)cbuf, maxxsize, maxysize, x0, y0, curxsize, curysize);
					}
					break;
                }
				case 16:
				case 12:
                {
                    unsigned short *cbuf=(unsigned short*)m_ptheCamera->GetRawData(&imglength);
					if (curxsize == maxxsize)
					{
						lsrcstrpos = y0 * maxxsize;
                        if(copyExternal) retValue += externalDataObject->copyFromData2D<ito::uint16>((ito::uint16*)cbuf+lsrcstrpos, maxxsize, curysize);
						if(!copyExternal || hasListeners) retValue += m_data.copyFromData2D<ito::uint16>((ito::uint16*)cbuf+lsrcstrpos, maxxsize, curysize);
					}
					else
					{
                        if(copyExternal) retValue += externalDataObject->copyFromData2D<ito::uint16>((ito::uint16*)cbuf, maxxsize, maxysize, x0, y0, curxsize, curysize);
						if(!copyExternal || hasListeners) retValue += m_data.copyFromData2D<ito::uint16>((ito::uint16*)cbuf, maxxsize, maxysize, x0, y0, curxsize, curysize);
					}
					break;
                }
				default:
					retValue += ito::RetVal(ito::retError, 0, tr("F Wrong picture Type").toAscii().data());
					break;
			}
		
		}
		this->m_isgrabbing = false;
	}

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
