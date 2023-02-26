/* ********************************************************************
    Template for a camera / grabber plugin for the software itom
    
    You can use this template, use it in your plugins, modify it,
    copy it and distribute it without any license restrictions.
*********************************************************************** */

#define ITOM_IMPORT_API
#define ITOM_IMPORT_PLOTAPI

#include "thorlabsCCS.h"
#include "pluginVersion.h"
#include "gitVersion.h"

#include <qstring.h>
#include <qstringlist.h>
#include <qplugin.h>
#include <qmessagebox.h>
#include <qelapsedtimer.h>

#include "dockWidgetThorlabsCCS.h"

//----------------------------------------------------------------------------------------------------------------------------------
//! Constructor of Interface Class.
/*!
    \todo add necessary information about your plugin here.
*/
ThorlabsCCSInterface::ThorlabsCCSInterface()
{
    m_type = ito::typeDataIO | ito::typeGrabber; //any grabber is a dataIO device AND its subtype grabber (bitmask -> therefore the OR-combination).
    setObjectName("ThorlabsCCS");

    m_description = QObject::tr("Thorlabs CCS Series Spectrometer");

    //for the docstring, please don't set any spaces at the beginning of the line.
/*    char docstring[] = \
"This plugin can operate spectrometers from the Thorlabs CCS Series. \n\
\n\
If you start the plugin without further parameters (device=''), the first connected device is opened. \n\
Set device = '<scan>' in order to get a printed list of detected devices. Use the device string or the desired \n\
device in order to open this specific one. \n\
\n\
This plugin has been tested with CCS175. \n\
\n\
For compiling this plugin, you need to install the Thorlabs OSASW Instrumentation software, shipped with the spectrometer. \n\
Then set the CMake variable THORLABS_IVI_VISA_SDK to the 32/64 IVI_VISA directory (e.g. C:/Program Files/IVI Foundation/...) where \n\
subdirectories like include or bin are contained.";
    m_detaildescription = QObject::tr(docstring);*/
    m_detaildescription = QObject::tr("This plugin can operate spectrometers from the Thorlabs CCS Series. \n\
\n\
If you start the plugin without further parameters (device=''), the first connected device is opened. \n\
Set device = '<scan>' in order to get a printed list of detected devices. Use the device string or the desired \n\
device in order to open this specific one. \n\
\n\
This plugin has been tested with CCS175. \n\
\n\
For compiling this plugin, you need to install the Thorlabs OSASW Instrumentation software, shipped with the spectrometer. \n\
Then set the CMake variable THORLABS_IVI_VISA_SDK to the 32/64 IVI_VISA directory (e.g. C:/Program Files/IVI Foundation/...) where \n\
subdirectories like include or bin are contained.");

    m_author = "M. Gronle, ITO, University Stuttgart";
    m_version = (PLUGIN_VERSION_MAJOR << 16) + (PLUGIN_VERSION_MINOR << 8) + PLUGIN_VERSION_PATCH;
    m_minItomVer = MINVERSION;
    m_maxItomVer = MAXVERSION;
    m_license = QObject::tr("LPGL, uses Thorlabs CCS VISA Instrument Driver (LGPL licensed)");
    m_aboutThis = QObject::tr(GITVERSION); 

    //add mandatory and optional parameters for the initialization here.
    //append them to m_initParamsMand or m_initParamsOpt.
    m_initParamsMand.append(ito::Param("device", ito::ParamBase::String, "", "device name that should be opened, an empty string opens the first device that is found. Pass '<scan>' for displaying all devices"));
}

//----------------------------------------------------------------------------------------------------------------------------------
//! Destructor of Interface Class.
/*!
    
*/
ThorlabsCCSInterface::~ThorlabsCCSInterface()
{
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal ThorlabsCCSInterface::getAddInInst(ito::AddInBase **addInInst)
{
    NEW_PLUGININSTANCE(ThorlabsCCS) //the argument of the macro is the classname of the plugin
    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal ThorlabsCCSInterface::closeThisInst(ito::AddInBase **addInInst)
{
   REMOVE_PLUGININSTANCE(ThorlabsCCS) //the argument of the macro is the classname of the plugin
   return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
//! Constructor of plugin.
/*!
    \todo add internal parameters of the plugin to the map m_params. It is allowed to append or remove entries from m_params
    in this constructor or later in the init method
*/
ThorlabsCCS::ThorlabsCCS() : AddInGrabber(), m_isgrabbing(false),
    m_instrument(VI_NULL)
{
    ito::Param paramVal("name", ito::ParamBase::String | ito::ParamBase::Readonly, "ThorlabsCCS", NULL);
    m_params.insert(paramVal.getName(), paramVal);

    int roi[] = {0, 0, TLCCS_NUM_PIXELS, 1};
    paramVal = ito::Param("roi", ito::ParamBase::IntArray, 4, roi, tr("ROI (x,y,width,height)").toLatin1().data());
    ito::RectMeta *rm = new ito::RectMeta(ito::RangeMeta(0, TLCCS_NUM_PIXELS - 1), ito::RangeMeta(0, 0, 1, 1, 1, 1));
    paramVal.setMeta(rm, true);
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param("sizex", ito::ParamBase::Int | ito::ParamBase::Readonly | ito::ParamBase::In, 1, 2048, 2048, tr("width of ROI (x-direction)").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("sizey", ito::ParamBase::Int | ito::ParamBase::Readonly | ito::ParamBase::In, 1, 1, 1, tr("height of ROI (y-direction)").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param("bpp", ito::ParamBase::Int | ito::ParamBase::In, 64, 64, 64, tr("bpp").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param("integration_time", ito::ParamBase::Double | ito::ParamBase::In, TLCCS_MIN_INT_TIME, TLCCS_MAX_INT_TIME, TLCCS_DEF_INT_TIME, tr("integration time of ccd in sec").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param("manufacturer_name", ito::ParamBase::String | ito::ParamBase::Readonly, "", tr("manufacturer name").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("device_name", ito::ParamBase::String | ito::ParamBase::Readonly, "", tr("device name").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("serial_number", ito::ParamBase::String | ito::ParamBase::Readonly, "", tr("serial number").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("firmware_revision", ito::ParamBase::String | ito::ParamBase::Readonly, "", tr("firmware revision").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("instrument_driver_revision", ito::ParamBase::String | ito::ParamBase::Readonly, "", tr("instrument driver revision").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param("wavelength_data", ito::ParamBase::DoubleArray | ito::ParamBase::Readonly, NULL, tr("wavelength in nm (air) for each pixel").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    
    //the following lines create and register the plugin's dock widget. Delete these lines if the plugin does not have a dock widget.
    DockWidgetThorlabsCCS *dw = new DockWidgetThorlabsCCS(this);
    
    Qt::DockWidgetAreas areas = Qt::AllDockWidgetAreas;
    QDockWidget::DockWidgetFeatures features = QDockWidget::DockWidgetClosable | QDockWidget::DockWidgetFloatable | QDockWidget::DockWidgetMovable;
    createDockWidget(QString(m_params["name"].getVal<char *>()), features, areas, dw);   
}

//----------------------------------------------------------------------------------------------------------------------------------
ThorlabsCCS::~ThorlabsCCS()
{
}

//----------------------------------------------------------------------------------------------------------------------------------
//! initialization of plugin
/*!
    \sa close
*/
ito::RetVal ThorlabsCCS::init(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);

    QString deviceName = paramsMand->at(0).getVal<char*>();
    QList<QByteArray> foundDevices;

    ViSession  resMgr  = VI_NULL;      //resource manager
    ViUInt32   count   = 0;            //counts found devices
    ViChar     rscStr[VI_FIND_BUFLEN]; // resource string
    ViFindList findList;

    //scanning for CCS instruments
    retValue += checkError(viOpenDefaultRM(&resMgr));
    if (!retValue.containsError())
    {
        retValue += checkError(viFindRsrc(resMgr, const_cast<char*>(TLCCS_FIND_PATTERN), &findList, &count, rscStr));
        if (retValue == ito::retOk) foundDevices.append(rscStr);

        for (ViUInt32 c = 1; c < count; ++c)
        {
            retValue += checkError(viFindNext(findList, rscStr));
            if (retValue == ito::retOk) foundDevices.append(rscStr);
        }

        viClose(findList);

        if (!retValue.containsError())
        {
            if (deviceName == "<scan>")
            {
                std::cout << "Thorlabs CCS devices \n------------------------\n" << std::endl;
                for (ViUInt32 i = 0; i < std::min(static_cast<size_t>(count), static_cast<size_t>(foundDevices.size())); ++i)
                {
                    std::cout << "Dev. " << i << ": " << foundDevices[i].data() << std::endl;
                }

                retValue += ito::RetVal(ito::retError, 0, tr("The initialization is terminated since only a list of found devices has been requested ('<scan>')").toLatin1().data());
            }
            else if (deviceName == "")
            {
                if (foundDevices.size() == 0)
                {
                    retValue += ito::RetVal(ito::retError, 0, tr("no devices found").toLatin1().data());
                }
                else
                {
                    deviceName = foundDevices[0]; 
                }
            }
            else
            {
                bool found = false;
                foreach(const QByteArray &ba, foundDevices)
                {
                    if (QString::compare(deviceName, ba, Qt::CaseInsensitive) == 0)
                    {
                        deviceName = ba;
                        found = true;
                        break;
                    }
                }

                if (!found)
                {
                    retValue += ito::RetVal::format(ito::retError, 0, tr("Device %s could not be found").toLatin1().data(), deviceName.toLatin1().data());
                }
            }
        }
    }

    if (!retValue.containsError())
    {
        //try to open CCS
        retValue += checkError(tlccs_init(deviceName.toLatin1().data(), VI_OFF, VI_OFF, &m_instrument));
    }

    if (!retValue.containsError())
    {
        ViChar manufacturer[256];
        ViChar device[256];
        ViChar serial[256];
        ViChar firmware[256];
        ViChar driver[256];
        retValue += checkError(tlccs_identificationQuery(m_instrument, manufacturer, device, serial, firmware, driver));

        if (!retValue.containsError())
        {
            setIdentifier(QString("%1 (%2)").arg(device).arg(manufacturer));
            m_params["deviceName"].setVal<char*>(device);
            m_params["manufacturer_name"].setVal<char*>(manufacturer);
            m_params["serial_number"].setVal<char*>(serial);
            m_params["firmware_revision"].setVal<char*>(firmware);
            m_params["instrument_driver_revision"].setVal<char*>(driver);
        }
    }

    if (!retValue.containsError())
    {
        //get wavelength table, length of array...
        m_params["sizex"].setMeta(new ito::IntMeta(0, TLCCS_NUM_PIXELS), true);
        m_params["sizex"].setVal<int>(TLCCS_NUM_PIXELS);
        
        ViReal64 wavelengthDataArray[TLCCS_NUM_PIXELS];

        retValue += checkError(tlccs_getWavelengthData(m_instrument, TLCCS_CAL_DATA_SET_FACTORY, wavelengthDataArray, VI_NULL, VI_NULL));

        if (!retValue.containsError())
        {
            m_params["wavelength_data"].setVal<double*>(wavelengthDataArray, TLCCS_NUM_PIXELS);
        }

        ViReal64 integrationTime;
        retValue += checkError(tlccs_getIntegrationTime(m_instrument, &integrationTime));
        if (!retValue.containsError())
        {
            m_params["integration_time"].setVal<double>(integrationTime);
        }
    }
    
    if (!retValue.containsError())
    {        
        retValue += checkData();
    }
    
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

//----------------------------------------------------------------------------------------------------------------------------------
//! shutdown of plugin
/*!
    \sa init
*/
ito::RetVal ThorlabsCCS::close(ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);
    
    if (m_instrument != VI_NULL)
    {
        tlccs_close(m_instrument);
        m_instrument = VI_NULL;
    }

    if (waitCond)
    {
        waitCond->returnValue = retValue;
        waitCond->release();
    }
    
    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal ThorlabsCCS::checkError(ViStatus err)
{
    if (err)
    {
        ViChar msg[TLCCS_ERR_DESCR_BUFFER_SIZE];
        memset(msg,0,sizeof(ViChar) * TLCCS_ERR_DESCR_BUFFER_SIZE);
        tlccs_error_message(m_instrument, err, msg);

        return ito::RetVal(ito::retError, 0, msg);
    }

    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal ThorlabsCCS::getParam(QSharedPointer<ito::Param> val, ItomSharedSemaphore *waitCond)
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
ito::RetVal ThorlabsCCS::setParam(QSharedPointer<ito::ParamBase> val, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);
    QString key;
    bool hasIndex;
    int index;
    QString suffix;
    QMap<QString, ito::Param>::iterator it;

    //parse the given parameter-name (if you support indexed or suffix-based parameters)
    retValue += apiParseParamName(val->getName(), key, hasIndex, index, suffix);

    if (!retValue.containsError())
    {
        //gets the parameter key from m_params map (read-only is not allowed and leads to ito::retError).
        retValue += apiGetParamFromMapByKey(m_params, key, it, true);
    }

    if (!retValue.containsError())
    {
        //here the new parameter is checked whether its type corresponds or can be cast into the
        // value in m_params and whether the new type fits to the requirements of any possible
        // meta structure.
        retValue += apiValidateParam(*it, *val, false, true);
    }

    if (!retValue.containsError())
    {
        if (key == "integration_time")
        {
            ViReal64 integrationTime = val->getVal<double>();
            retValue += checkError(tlccs_setIntegrationTime(m_instrument, integrationTime));
            if (!retValue.containsError())
            {
                retValue += checkError(tlccs_getIntegrationTime(m_instrument, &integrationTime));
                m_params["integration_time"].setVal<double>(integrationTime);
            }
        }
        else if (key == "roi")
        {
            if (!hasIndex)
            {
                retValue += it->copyValueFrom(&(*val));
            }
            else
            {
                //index is already checked for limits
                int *roi = it->getVal<int*>();
                roi[index] = val->getVal<int>();
            }
            const int *roi = it->getVal<int*>();
            m_params["sizex"].setVal<int>(roi[2]);
            m_params["sizey"].setVal<int>(roi[3]);

            retValue += checkData();
        }
        else
        {
            //all parameters that don't need further checks can simply be assigned
            //to the value in m_params (the rest is already checked above)
            retValue += it->copyValueFrom(&(*val));
        }
    }

    if (!retValue.containsError())
    {
        emit parametersChanged(m_params); //send changed parameters to any connected dialogs or dock-widgets
    }

    if (waitCond)
    {
        waitCond->returnValue = retValue;
        waitCond->release();
    }

    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal ThorlabsCCS::startDevice(ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);
    
    incGrabberStarted(); //increment a counter to see how many times startDevice has been called

    checkData();
    
    //get test image in order to set the device into an idle state
    retValue += checkError(tlccs_startScan(m_instrument));
    ViInt32 status;

    while(1)
    {
        retValue += secureGetStatus(status);
        if (status & TLCCS_STATUS_SCAN_IDLE)
        {
            break;
        }
        //dummy read
        if (status & TLCCS_STATUS_SCAN_TRANSFER)
        {
            retValue += checkError(tlccs_getScanData(m_instrument, dummyBuffer));
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
ito::RetVal ThorlabsCCS::stopDevice(ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);

    decGrabberStarted(); //decrements the counter (see startDevice)

    if (grabberStartedCount() < 0)
    {
        retValue += ito::RetVal(ito::retWarning, 0, tr("The grabber has already been stopped.").toLatin1().data());
        setGrabberStarted(0);
    }
    
    //todo:
    // if the counter (obtained by grabberStartedCount()) drops to zero again, stop the camera, free all allocated
    // image buffers of the camera... (it is the opposite from all things that have been started, allocated... in startDevice)

    if (waitCond)
    {
        waitCond->returnValue = retValue;
        waitCond->release();
    }

    return ito::retOk;
}
         
//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal ThorlabsCCS::acquire(const int trigger, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);
    bool done;
    QElapsedTimer timer;

    if (grabberStartedCount() <= 0)
    {
        retValue += ito::RetVal(ito::retError, 0, tr("Tried to acquire an image without having started the device.").toLatin1().data());
    }
    else
    {
        m_isgrabbing = true;
    }
    
    ViInt32 status;
    retValue += secureGetStatus(status);

    
    if (!retValue.containsError())
    {
        if ((status & TLCCS_STATUS_SCAN_IDLE) == 0)
        {
            timer.start();
            done = false;

            //try to get the device into an idle state for new acquisition
            while(timer.elapsed() < 5000)
            {
                if (status & TLCCS_STATUS_SCAN_TRANSFER)
                {
                    //dummy read in order to reset it into idle state
                    retValue += checkError(tlccs_getScanData(m_instrument, dummyBuffer));
                }
                else if (status & TLCCS_STATUS_SCAN_IDLE)
                {
                    done = true;
                    break;
                }

                retValue += secureGetStatus(status);
            }

            if (!done && (status & TLCCS_STATUS_SCAN_IDLE) == 0)
            {
                retValue += ito::RetVal(ito::retError, 0, tr("device can not be triggered since it is not in an idle state").toLatin1().data());
            }
        }
    }

    if (!retValue.containsError())
    {
        retValue += checkError(tlccs_startScan(m_instrument));
    }

    m_grabbingRetVal = ito::retOk;
    
    //now the wait condition is released, such that itom (caller) stops waiting and continuous with its own execution.
    if (waitCond)
    {
        waitCond->returnValue = retValue;
        waitCond->release();  
    }

    if (!retValue.containsError())
    {
        bool done = false;

        //m_data.lockWrite();

        retValue += secureGetStatus(status);
        if (status & TLCCS_STATUS_SCAN_TRANSFER)
        {
            done = true;
            m_grabbingRetVal += checkError(tlccs_getScanData(m_instrument, buffer));
        }
        else
        {
            double integrationTimeMS = m_params["integration_time"].getVal<double>() * 1000;
            timer.start();

            while (timer.elapsed() < std::max(200.0,(integrationTimeMS * 2.0)) && (!done))
            {
                retValue += secureGetStatus(status);
                if (status & TLCCS_STATUS_SCAN_TRANSFER)
                {
                    done = true;
                    m_grabbingRetVal += checkError(tlccs_getScanData(m_instrument, buffer));
                }
            }
        }

        //m_data.unlock();

        if (!done)
        {
            m_grabbingRetVal += ito::RetVal(ito::retError, 0, tr("timeout while getting image from device").toLatin1().data());
        }
    }

    retValue += m_grabbingRetVal;

    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal ThorlabsCCS::retrieveData(ito::DataObject *externalDataObject)
{
    //todo: this is just a basic example for getting the buffered image to m_data or the externally given data object
    //enhance it and adjust it for your needs
    ito::RetVal retValue(ito::retOk);

    ito::DataObject *dataObj = externalDataObject ? externalDataObject : &m_data;
    const int *roi = m_params["roi"].getVal<int*>();

    bool hasListeners = (m_autoGrabbingListeners.size() > 0);
    bool copyExternal = (externalDataObject != NULL);

    if (m_isgrabbing == false)
    {
        retValue += ito::RetVal(ito::retWarning, 0, tr("Tried to get picture without triggering exposure").toLatin1().data());
    }
    else
    {
        retValue += m_grabbingRetVal;

        if (!retValue.containsError())
        {
            

            if (externalDataObject)
            {
                retValue += checkData(externalDataObject); //update external object
                if (retValue == ito::retOk)
                {
                    memcpy(externalDataObject->rowPtr(0,0), &(buffer[roi[0]]), roi[2] * sizeof(ito::float64));
                }
            }
            if (hasListeners || !externalDataObject)
            {
                //size of m_data is checked
                memcpy(m_data.rowPtr(0,0), &(buffer[roi[0]]), roi[2] * sizeof(ito::float64));
            }
        }       
        
        m_isgrabbing = false;
    }

    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal ThorlabsCCS::checkData(ito::DataObject *externalDataObject /*= NULL*/)
{
    int futureHeight = 1;
    int futureWidth = m_params["sizex"].getVal<int>();
    int futureType = ito::tFloat64;

    if (externalDataObject == NULL)
    {
        if (m_data.getDims() < 2 || m_data.getSize(0) != (unsigned int)futureHeight || m_data.getSize(1) != (unsigned int)futureWidth || m_data.getType() != futureType)
        {
            m_data = ito::DataObject(futureHeight,futureWidth,futureType);
        }
    }
    else
    {
        int dims = externalDataObject->getDims();
        if (externalDataObject->getDims() == 0)
        {
            *externalDataObject = ito::DataObject(futureHeight,futureWidth,futureType);
        }
        else if (externalDataObject->calcNumMats () > 1)
        {
            return ito::RetVal(ito::retError, 0, tr("Error during check data, external dataObject invalid. Object has more than 1 plane. It must be of right size and type or a uninitilized image.").toLatin1().data());            
        }
        else if (externalDataObject->getSize(dims - 2) != (unsigned int)futureHeight || externalDataObject->getSize(dims - 1) != (unsigned int)futureWidth || externalDataObject->getType() != futureType)
        {
            return ito::RetVal(ito::retError, 0, tr("Error during check data, external dataObject invalid. Object must be of right size and type or a uninitilized image.").toLatin1().data());
        }
    }

    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
// usually it is not necessary to implement the checkData method, since the default implementation from AddInGrabber is already
// sufficient.
//
// What is does:
// - it obtains the image size from sizex, sizey, bpp
// - it checks whether the rows, cols and type of m_data are unequal to the requested dimensions and type
// - if so, m_data is reallocated, else nothing is done
// - if an external data object is given (from copyVal), this object is checked in place of m_data
// - the external data object is only reallocated if it is empty, else its size or its region of interest must exactly
//    fit to the given size restrictions
//
// if you need to do further things, overload checkData and implement your version there
/*ito::RetVal ThorlabsCCS::checkData(ito::DataObject *externalDataObject)
{
    return ito::retOk;
}*/

//----------------------------------------------------------------------------------------------------------------------------------
//! Returns the grabbed camera frame as reference.
/*!
    This method returns a reference to the recently acquired image. Therefore this camera size must fit to the data structure of the 
    DataObject.
    
    This method returns a reference to the internal dataObject m_data of the camera where the currently acquired image data is copied to (either
    in the acquire method or in retrieve data). Please remember, that the reference may directly change if a new image is acquired.

    \param [in,out] vpdObj is the pointer to a given dataObject (this pointer should be cast to ito::DataObject*). After the call, the dataObject is a reference to the internal m_data dataObject of the camera.
    \param [in] waitCond is the semaphore (default: NULL), which is released if this method has been terminated
    \return retOk if everything is ok, retError is camera has not been started or no image has been acquired by the method acquire.
    
    \sa retrieveImage, copyVal
*/
ito::RetVal ThorlabsCCS::getVal(void *vpdObj, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);
    ito::DataObject *dObj = reinterpret_cast<ito::DataObject *>(vpdObj);
    
    //call retrieveData without argument. Retrieve data should then put the currently acquired image into the dataObject m_data of the camera.
    retValue += retrieveData();

    if (!retValue.containsError())
    {
        //send newly acquired image to possibly connected live images
        sendDataToListeners(0); //don't wait for live data, since user should get the data as fast as possible.

        if (dObj)
        {
            (*dObj) = m_data; //copy reference to externally given object
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
    This method copies the recently grabbed camera frame to the given DataObject. 
    
    The given dataObject must either have an empty size (then it is resized to the size and type of the camera image) or its size or adjusted region of
    interest must exactly fit to the size of the camera. Then, the acquired image is copied inside of the given region of interest (copy into a subpart of
    an image stack is possible then)

    \param [in,out] vpdObj is the pointer to a given dataObject (this pointer should be cast to ito::DataObject*) where the acquired image is deep copied to.
    \param [in] waitCond is the semaphore (default: NULL), which is released if this method has been terminated
    \return retOk if everything is ok, retError is camera has not been started or no image has been acquired by the method acquire.
    
    \sa retrieveImage, getVal
*/
ito::RetVal ThorlabsCCS::copyVal(void *vpdObj, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);
    ito::DataObject *dObj = reinterpret_cast<ito::DataObject *>(vpdObj);
    
    if (!dObj)
    {
        retValue += ito::RetVal(ito::retError, 0, tr("Empty object handle retrieved from caller").toLatin1().data());
    }
    
    if (!retValue.containsError())
    {
        //this method calls retrieveData with the passed dataObject as argument such that retrieveData is able to copy the image obtained
        //by the camera directly into the given, external dataObject
        retValue += retrieveData(dObj);  //checkData is executed inside of retrieveData
    }

    if (!retValue.containsError())
    {
        //send newly acquired image to possibly connected live images
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
//! slot called if the dock widget of the plugin becomes (in)visible
/*!
    Overwrite this method if the plugin has a dock widget. If so, you can connect the parametersChanged signal of the plugin
    with the dock widget once its becomes visible such that no resources are used if the dock widget is not visible. Right after
    a re-connection emit parametersChanged(m_params) in order to send the current status of all plugin parameters to the dock widget.
*/
void ThorlabsCCS::dockWidgetVisibilityChanged(bool visible)
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
//! method called to show the configuration dialog
/*!
    This method is called from the main thread from itom and should show the configuration dialog of the plugin.
    If the instance of the configuration dialog has been created, its slot 'parametersChanged' is connected to the signal 'parametersChanged'
    of the plugin. By invoking the slot sendParameterRequest of the plugin, the plugin's signal parametersChanged is immediately emitted with
    m_params as argument. Therefore the configuration dialog obtains the current set of parameters and can be adjusted to its values.
    
    The configuration dialog should emit reject() or accept() depending if the user wanted to close the dialog using the ok or cancel button.
    If ok has been clicked (accept()), this method calls applyParameters of the configuration dialog in order to force the dialog to send
    all changed parameters to the plugin. If the user clicks an apply button, the configuration dialog itsself must call applyParameters.
    
    If the configuration dialog is inherited from AbstractAddInConfigDialog, use the api-function apiShowConfigurationDialog that does all
    the things mentioned in this description.
    
    Remember that you need to implement hasConfDialog in your plugin and return 1 in order to signalize itom that the plugin
    has a configuration dialog.
    
    \sa hasConfDialog
*/
const ito::RetVal ThorlabsCCS::showConfDialog(void)
{
    return apiShowConfigurationDialog(this, new DialogThorlabsCCS(this));
}