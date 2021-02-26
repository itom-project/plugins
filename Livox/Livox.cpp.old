/* ********************************************************************
    Template for a camera / grabber plugin for the software itom
    
    You can use this template, use it in your plugins, modify it,
    copy it and distribute it without any license restrictions.
*********************************************************************** */

#define ITOM_IMPORT_API
#define ITOM_IMPORT_PLOTAPI

#include "Livox.h"
#include "pluginVersion.h"
#include "gitVersion.h"

#include <qstring.h>
#include <qstringlist.h>
#include <qplugin.h>
#include <qmessagebox.h>

#include "dockWidgetLivox.h"

#include <stdio.h>
#include <stdlib.h>
#ifdef WIN32
#include <windows.h>
#else
#include <unistd.h>
#endif
#include <string.h>

//----------------------------------------------------------------------------------------------------------------------------------
//! Constructor of Interface Class.
LivoxInterface::LivoxInterface()
{
    m_type = ito::typeDataIO | ito::typeGrabber; //any grabber is a dataIO device AND its subtype grabber (bitmask -> therefore the OR-combination).
    setObjectName("Livox");

    m_description = QObject::tr("Livox");

    //for the docstring, please don't set any spaces at the beginning of the line.
    char docstring[] = \
"This is the grabber plugin for a livox LIDAR\n\
\n\
Requirements:  LIVOX SDK,\n\
\n\
Put a detailed description about what the plugin is doing, what is needed to get it started, limitations...";
    m_detaildescription = QObject::tr(docstring);

    m_author = PLUGIN_AUTHOR;
    m_version = PLUGIN_VERSION;
    m_minItomVer = MINVERSION;
    m_maxItomVer = MAXVERSION;
    m_license = QObject::tr("The MIT License(MIT) Copyright(c) 2019 Livox. All rights reserved.");
    m_aboutThis = QObject::tr(GITVERSION); 

    //add mandatory and optional parameters for the initialization here.
    //append them to m_initParamsMand or m_initParamsOpt.
}

//----------------------------------------------------------------------------------------------------------------------------------
//! Destructor of Interface Class.
LivoxInterface::~LivoxInterface()
{
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal LivoxInterface::getAddInInst(ito::AddInBase **addInInst)
{
    NEW_PLUGININSTANCE(Livox) //the argument of the macro is the classname of the plugin
    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal LivoxInterface::closeThisInst(ito::AddInBase **addInInst)
{
   REMOVE_PLUGININSTANCE(Livox) //the argument of the macro is the classname of the plugin
   return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
#if QT_VERSION < 0x050000
    Q_EXPORT_PLUGIN2(Livoxinterface, LivoxInterface) //the second parameter must correspond to the class-name of the interface class, the first parameter is arbitrary (usually the same with small letters only)
#endif




//----------------------------------------------------------------------------------------------------------------------------------
//! Constructor of plugin.
/*!
    \todo add internal parameters of the plugin to the map m_params. It is allowed to append or remove entries from m_params
    in this constructor or later in the init method
*/
Livox::Livox() : AddInGrabber(), m_isgrabbing(false)
{
    ito::Param paramVal("name", ito::ParamBase::String | ito::ParamBase::Readonly, "Livox", NULL);
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param("x0", ito::ParamBase::Int | ito::ParamBase::In, 0, 2048, 0, tr("first pixel index in ROI (x-direction)").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("y0", ito::ParamBase::Int | ito::ParamBase::In, 0, 2048, 0, tr("first pixel index in ROI (y-direction)").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("x1", ito::ParamBase::Int | ito::ParamBase::In, 0, 1279, 1279, tr("last pixel index in ROI (x-direction)").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("y1", ito::ParamBase::Int | ito::ParamBase::In, 0, 1023, 1023, tr("last pixel index in ROI (y-direction)").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("sizex", ito::ParamBase::Int | ito::ParamBase::Readonly | ito::ParamBase::In, 1, 2048, 2048, tr("width of ROI (x-direction)").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("sizey", ito::ParamBase::Int | ito::ParamBase::Readonly | ito::ParamBase::In, 1, 2048, 2048, tr("height of ROI (y-direction)").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param("bpp", ito::ParamBase::Int | ito::ParamBase::In, 8, 8, 8, tr("bpp").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("integrationTime", ito::ParamBase::Double | ito::ParamBase::In, 0.0, 1.0, 0.01, tr("Integrationtime of CCD [0..1] (no unit)").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    
    //the following lines create and register the plugin's dock widget. Delete these lines if the plugin does not have a dock widget.
    DockWidgetLivox *dw = new DockWidgetLivox(this);
    
    Qt::DockWidgetAreas areas = Qt::AllDockWidgetAreas;
    QDockWidget::DockWidgetFeatures features = QDockWidget::DockWidgetClosable | QDockWidget::DockWidgetFloatable | QDockWidget::DockWidgetMovable;
    createDockWidget(QString(m_params["name"].getVal<char *>()), features, areas, dw);   
}

//----------------------------------------------------------------------------------------------------------------------------------
Livox::~Livox()
{
}

//----------------------------------------------------------------------------------------------------------------------------------
//! initialization of plugin
ito::RetVal Livox::init(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);

	devices[kMaxLidarCount];
	data_recv_count[kMaxLidarCount];


	///** Connect all the broadcast device. */
	lidar_count = 0;
	broadcast_code_list[kMaxLidarCount][kBroadcastCodeSize];
	apr_pool_t *mp = NULL;

	//if (!retValue.containsError()) {
	//	//retValue += SetProgramOption(argc, argv);
	//	//retValue += (apr_initialize() != APR_SUCCESS);
	//	//retValue += (apr_pool_create(&mp, NULL) != APR_SUCCESS);

	//}
	//printf("Livox SDK initializing...\n");

	LivoxSdkVersion _sdkversion;
	GetLivoxSdkVersion(& _sdkversion);
	printf("Livox SDK version %d.%d.%d .\n", _sdkversion.major, _sdkversion.minor, _sdkversion.patch);

	if (!retValue.containsError()) {
		retValue += Init();
		retValue += apr_initialize();
	}
	printf("Livox SDK has been initialized.\n");

	memset(devices, 0, sizeof(devices));
	memset(data_recv_count, 0, sizeof(data_recv_count));

	
	if (!retValue.containsError())
	{
		/** Set the callback function receiving broadcast message from Livox LiDAR. */
		//SetBroadcastCallback(OnDeviceBroadcast);

		/** Set the callback function called when device state change,
		 * which means connection/disconnection and changing of LiDAR state.
		 */
		//SetDeviceStateUpdateCallback(OnDeviceInfoChange);


	}


    //steps todo:
    // - get all initialization parameters
    // - try to detect your device
    // - establish a connection to the device
    // - synchronize the current parameters of the device with the current values of parameters inserted in m_params
    // - if an identifier string of the device is available, set it via setIdentifier("yourIdentifier")
    // - call checkData() in order to reconfigure the temporary image buffer m_data (or other structures) depending on the current size, image type...
    // - call emit parametersChanged(m_params) in order to propagate the current set of parameters in m_params to connected dock widgets...
    // - call setInitialized(true) to confirm the end of the initialization (even if it failed)
    
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
ito::RetVal Livox::close(ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);
    
 //   //todo:
 //   // - disconnect the device if not yet done
 //   // - this funtion is considered to be the "inverse" of init.
	//int i = 0;
	//for (i = 0; i < kMaxLidarCount; ++i) {
	//	if (devices[i].device_state == kDeviceStateSampling) {
	//		/** Stop the sampling of Livox LiDAR. */
	//		//LidarStopSampling(devices[i].handle, OnStopSampleCallback, NULL);
	//	}
	//}

	/** Uninitialize Livox-SDK. */
	//Uninit();

    if (waitCond)
    {
        waitCond->returnValue = retValue;
        waitCond->release();
    }
    
    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal Livox::getParam(QSharedPointer<ito::Param> val, ItomSharedSemaphore *waitCond)
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
ito::RetVal Livox::setParam(QSharedPointer<ito::ParamBase> val, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);
    QString key;
    bool hasIndex;
    int index;
    QString suffix;
    QMap<QString, ito::Param>::iterator it;

    //parse the given parameter-name (if you support indexed or suffix-based parameters)
    retValue += apiParseParamName( val->getName(), key, hasIndex, index, suffix );

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
        if (key == "demoKey1")
        {
            //check the new value and if ok, assign it to the internal parameter
            retValue += it->copyValueFrom( &(*val) );
        }
        else if (key == "demoKey2")
        {
            //check the new value and if ok, assign it to the internal parameter
            retValue += it->copyValueFrom( &(*val) );
        }
        else
        {
            //all parameters that don't need further checks can simply be assigned
            //to the value in m_params (the rest is already checked above)
            retValue += it->copyValueFrom( &(*val) );
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
ito::RetVal Livox::startDevice(ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);
    
    incGrabberStarted(); //increment a counter to see how many times startDevice has been called
    
    //todo:
    // if this function has been called for the first time (grabberStartedCount() == 1),
    // start the camera, allocate necessary buffers or do other work that is necessary
    // to prepare the camera for image acquisitions.
    
    if (waitCond)
    {
        waitCond->returnValue = retValue;
        waitCond->release();
    }
    return retValue;
}
         
//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal Livox::stopDevice(ItomSharedSemaphore *waitCond)
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
ito::RetVal Livox::acquire(const int trigger, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);
    bool RetCode = false;

    if (grabberStartedCount() <= 0)
    {
        retValue += ito::RetVal(ito::retError, 0, tr("Tried to acquire an image without having started the device.").toLatin1().data());
    }
    else
    {
        m_isgrabbing = true;
    }
    
    //todo:
    // trigger the camera for starting the acquisition of a new image (software trigger or make hardware trigger ready (depending on trigger (0: software trigger, default))
    
    //now the wait condition is released, such that itom (caller) stops waiting and continuous with its own execution.
    if (waitCond)
    {
        waitCond->returnValue = retValue;
        waitCond->release();  
    }
    
    //todo:
    // it is possible now, to wait here until the acquired image is ready
    // if you want to do this here, wait for the finished image, get it and save it
    // to any accessible buffer, for instance the m_data dataObject that is finally delivered
    // via getVal or copyVal.
    // 
    // you can also implement this waiting and obtaining procedure in retrieveImage.
    // If you do it here, the camera thread is blocked until the image is obtained, such that calls to getParam, setParam, stopDevice...
    // are not executed during the waiting operation. They are queued and executed once the image is acquired and transmitted to the plugin.

    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal Livox::retrieveData(ito::DataObject *externalDataObject)
{
    //todo: this is just a basic example for getting the buffered image to m_data or the externally given data object
    //enhance it and adjust it for your needs
    ito::RetVal retValue(ito::retOk);

    ito::DataObject *dataObj = externalDataObject ? externalDataObject : &m_data;

    bool hasListeners = (m_autoGrabbingListeners.size() > 0);
    bool copyExternal = (externalDataObject != NULL);
    
    const int bufferWidth = m_params["sizex"].getVal<int>();
    const int bufferHeight = m_params["sizey"].getVal<int>();



    if (m_isgrabbing == false)
    {
        retValue += ito::RetVal(ito::retWarning, 0, tr("Tried to get picture without triggering exposure").toLatin1().data());
    }
    else
    {
        //step 1: create m_data (if not yet available)
        if (externalDataObject && hasListeners)
        {
            retValue += checkData(NULL); //update m_data
            retValue += checkData(externalDataObject); //update external object
        }
        else
        {
            retValue += checkData(externalDataObject); //update external object or m_data
        }
        
        if (!retValue.containsError())
        {
            if (m_data.getType() == ito::tUInt8)
            {
                if (copyExternal)
                {
                    retValue += externalDataObject->copyFromData2D<ito::uint8>((ito::uint8*) bufferPtr, bufferWidth, bufferHeight);
                }
                if (!copyExternal || hasListeners)
                {
                    retValue += m_data.copyFromData2D<ito::uint8>((ito::uint8*) bufferPtr, bufferWidth, bufferHeight);
                }
            }
            else if (m_data.getType() == ito::tUInt16)
            {
                if (copyExternal)
                {
                    retValue += externalDataObject->copyFromData2D<ito::uint16>((ito::uint16*) bufferPtr, bufferWidth, bufferHeight);
                }
                if (!copyExternal || hasListeners)
                {
                    retValue += m_data.copyFromData2D<ito::uint16>((ito::uint16*) bufferPtr, bufferWidth, bufferHeight);            
                }
            }
            else
            {
                retValue += ito::RetVal(ito::retError, 1002, tr("copying image buffer not possible since unsupported type.").toLatin1().data());
            }
        }

        m_isgrabbing = false;
    }

    return retValue;
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
/*ito::RetVal Livox::checkData(ito::DataObject *externalDataObject)
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
ito::RetVal Livox::getVal(void *vpdObj, ItomSharedSemaphore *waitCond)
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
ito::RetVal Livox::copyVal(void *vpdObj, ItomSharedSemaphore *waitCond)
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
void Livox::dockWidgetVisibilityChanged(bool visible)
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
const ito::RetVal Livox::showConfDialog(void)
{
    return apiShowConfigurationDialog(this, new DialogLivox(this));
}



/**************LIVOX FUNC*****************************/

/** Callback function when broadcast message received.
 * You need to add listening device broadcast code and set the point cloud data callback in this function.
 */
//ito::RetVal Livox::OnDeviceBroadcast(const BroadcastDeviceInfo *info) {
//	if (info == NULL || info->dev_type == kDeviceTypeHub) {
//		return 0;
//	}
//	printf("Receive Broadcast Code %s\n", info->broadcast_code);
//	if (lidar_count > 0) {
//		bool found = false;
//		int i = 0;
//		for (i = 0; i < lidar_count; ++i) {
//			if (strncmp(info->broadcast_code, broadcast_code_list[i], kBroadcastCodeSize) == 0) {
//				found = true;
//				break;
//			}
//		}
//		if (!found) {
//			return 0;
//		}
//	}
//	bool result = false;
//	uint8_t handle = 0;
//	result = AddLidarToConnect(info->broadcast_code, &handle);
//	if (result == kStatusSuccess) {
//		/** Set the point cloud data for a specific Livox LiDAR. */
//		//SetDataCallback(handle, GetLidarData, NULL);
//		devices[handle].handle = handle;
//		devices[handle].device_state = kDeviceStateDisconnect;
//	}
//}
//
///** Receiving point cloud data from Livox LiDAR. */
//void Livox::GetLidarData(uint8_t handle, LivoxEthPacket *data, uint32_t data_num, void *client_data) {
//	if (data) {
//		data_recv_count[handle] ++;
//		if (data_recv_count[handle] % 100 == 0) {
//			/** Parsing the timestamp and the point cloud data. */
//			uint64_t cur_timestamp = *((uint64_t *)(data->timestamp));
//			if (data->data_type == kCartesian) {
//				LivoxRawPoint *p_point_data = (LivoxRawPoint *)data->data;
//			}
//			else if (data->data_type == kSpherical) {
//				LivoxSpherPoint *p_point_data = (LivoxSpherPoint *)data->data;
//			}
//			else if (data->data_type == kExtendCartesian) {
//				LivoxExtendRawPoint *p_point_data = (LivoxExtendRawPoint *)data->data;
//			}
//			else if (data->data_type == kExtendSpherical) {
//				LivoxExtendSpherPoint *p_point_data = (LivoxExtendSpherPoint *)data->data;
//			}
//			else if (data->data_type == kDualExtendCartesian) {
//				LivoxDualExtendRawPoint *p_point_data = (LivoxDualExtendRawPoint *)data->data;
//			}
//			else if (data->data_type == kDualExtendSpherical) {
//				LivoxDualExtendSpherPoint *p_point_data = (LivoxDualExtendSpherPoint *)data->data;
//			}
//			else if (data->data_type == kImu) {
//				LivoxImuPoint *p_point_data = (LivoxImuPoint *)data->data;
//			}
//			printf("data_type %d packet num %d\n", data->data_type, data_recv_count[handle]);
//		}
//	}
//}
//
///** Callback function of stopping sampling. */
//void Livox::OnStopSampleCallback(livox_status status, uint8_t handle, uint8_t response, void *data) {
//}


//int Livox::SetProgramOption(int argc, const char *argv[]) {
//	apr_status_t rv;
//	apr_pool_t *mp = NULL;
//	static const apr_getopt_option_t opt_option[] = {
//		/** Long-option, short-option, has-arg flag, description */
//		{ "code", 'c', 1, "Register device broadcast code" },
//		{ "log", 'l', 0, "Save the log file" },
//		{ "help", 'h', 0, "Show help" },
//		{ NULL, 0, 0, NULL },
//	};
//	apr_getopt_t *opt = NULL;
//	int optch = 0;
//	const char *optarg = NULL;
//
//	if (apr_initialize() != APR_SUCCESS) {
//		return -1;
//	}
//
//	if (apr_pool_create(&mp, NULL) != APR_SUCCESS) {
//		return -1;
//	}
//
//	rv = apr_getopt_init(&opt, mp, argc, argv);
//	if (rv != APR_SUCCESS) {
//		printf("Program options initialization failed.\n");
//		return -1;
//	}
//
//	/** Parse the all options based on opt_option[] */
//	bool is_help = false;
//	while ((rv = apr_getopt_long(opt, opt_option, &optch, &optarg)) == APR_SUCCESS) {
//		switch (optch) {
//		case 'c':
//			printf("Register broadcast code: %s\n", optarg);
//			char *sn_list = (char *)malloc(sizeof(char)*(strlen(optarg) + 1));
//			strncpy(sn_list, optarg, sizeof(char)*(strlen(optarg) + 1));
//			char *sn_list_head = sn_list;
//			sn_list = strtok(sn_list, "&");
//			int i = 0;
//			while (sn_list) {
//				strncpy(broadcast_code_list[i], sn_list, kBroadcastCodeSize);
//				sn_list = strtok(NULL, "&");
//				i++;
//			}
//			lidar_count = i;
//			free(sn_list_head);
//			sn_list_head = NULL;
//			break;
//		//case 'l':
//		//	printf("Save the log file.\n");
//		//	SaveLoggerFile();
//		//	break;
//		//case 'h':
//		//	printf(
//		//		" [-c] Register device broadcast code\n"
//		//		" [-l] Save the log file\n"
//		//		" [-h] Show help\n"
//		//	);
//		//	is_help = true;
//		//	break;
//		}
//	}
//	if (rv != APR_EOF) {
//		printf("Invalid options.\n");
//	}
//
//	apr_pool_destroy(mp);
//	mp = NULL;
//	apr_terminate();
//	if (is_help)
//		return 1;
//	return 0;
//}