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

/**Cmdline input broadcast code */
std::vector<std::string> cmdline_broadcast_code;
LdsLidar& read_lidar = LdsLidar::GetInstance();


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
    if (!read_lidar.DeInitLdsLidar())
    {
        std::cout << "Livox destructed"<<endl;
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
//! initialization of plugin
ito::RetVal Livox::init(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);


    if (!read_lidar.InitLdsLidar(cmdline_broadcast_code))
    {
        std::cout << "Livox init success. Start discovering devices..." << endl;
    }
    else
    {
        retValue += ito::RetVal(ito::retWarning, 0, tr("Livox Init failed.").toLatin1().data());
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
    
    if(!read_lidar.DeInitLdsLidar())
    {
        retValue += ito::RetVal(ito::retWarning, 0, tr("Livox De-init failed.").toLatin1().data());
    }

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


///////////////////////////////////////////////////////
/**************LIVOX FUNCTIONS************************/
///////////////////////////////////////////////////////

/** Const varible ------------------------------------------------------------------------------- */
/** User add broadcast code here */
static const char* local_broadcast_code_list[] = {"000000000000001",};

/** For callback use only */
LdsLidar* g_lidars = nullptr;

/** Lds lidar function */
LdsLidar::LdsLidar() {
	auto_connect_mode_ = true;
	whitelist_count_ = 0;
	is_initialized_ = false;
	lidar_count_ = 0;
	memset(broadcast_code_whitelist_, 0, sizeof(broadcast_code_whitelist_));
	memset(lidars_, 0, sizeof(lidars_));
	for (uint32_t i = 0; i < kMaxLidarCount; i++) {
		lidars_[i].handle = kMaxLidarCount;
		/** Unallocated state */
		lidars_[i].connect_state = kConnectStateOff;
	}
}

/** Lds lidar deconstructor */
LdsLidar::~LdsLidar() {}

/** Lds lidar init */
int LdsLidar::InitLdsLidar(std::vector<std::string>& broadcast_code_strs) {
	if (is_initialized_) {
		std::cout << "LiDAR data source is already inited!"<<endl;
		return -1;
	}
	if (!Init()) {
		Uninit();
		std::cout << "Livox-SDK init fail!"<<endl;
		return -1;
	}
	LivoxSdkVersion _sdkversion;
	GetLivoxSdkVersion(&_sdkversion);
    std::cout << "Livox SDK version: " << _sdkversion.major << _sdkversion.minor << _sdkversion.patch <<"\n";
	SetBroadcastCallback(LdsLidar::OnDeviceBroadcast);
	SetDeviceStateUpdateCallback(LdsLidar::OnDeviceChange);

	/** Add local broadcast code */
	LdsLidar::AddLocalBroadcastCode();
	if (whitelist_count_) {
		LdsLidar::DisableAutoConnectMode();
        std::cout << "Disable auto connect mode!\n";
        std::cout << "List all broadcast code in whiltelist:\n";
		for (uint32_t i = 0; i < whitelist_count_; i++) {
			printf("%s\n", broadcast_code_whitelist_[i]);
		}
	}
	else {
		LdsLidar::EnableAutoConnectMode();
		std::cout << "No broadcast code was added to whitelist, swith to automatic connection mode!\n";
	}
	/** Start livox sdk to receive lidar data */
	if (!Start()) {
		Uninit();
        std::cout << "Livox-SDK init failed at starting Livox SDK start routine!\n";
		return -1;
	}
	/** Add here, only for callback use */
	if (g_lidars == nullptr) {
		g_lidars = this;
	}
	is_initialized_ = true;
    std::cout << "Livox-SDK init success!\n";

	return 0;
}

/** Lds lidar deinit */
int LdsLidar::DeInitLdsLidar(void) {
	if (!is_initialized_) {
		printf("LiDAR data source is not exit");
		return -1;
	}
	Uninit(); //uninit from SDK-side
	printf("Livox SDK Deinit complete!\n");
	return 0;
}

/** Static function in LdsLidar for callback or event process ------------------------------------*/

/** Receiving point cloud data from Livox LiDAR. */
void LdsLidar::GetLidarDataCb(uint8_t handle, LivoxEthPacket *data,	uint32_t data_num, void *client_data) {

	LdsLidar* lidar_this = static_cast<LdsLidar *>(client_data);
	LivoxEthPacket* eth_packet = data;
	if (!data || !data_num || (handle >= kMaxLidarCount)) {
		return;
	}
	if (eth_packet) {
		lidar_this->data_recveive_count_[handle] ++;
		if (lidar_this->data_recveive_count_[handle] % 100 == 0) {
			printf("receive packet count %d %d\n", handle, lidar_this->data_recveive_count_[handle]);
			/** Parsing the timestamp and the point cloud data. */
			uint64_t cur_timestamp = *((uint64_t *)(data->timestamp));
			if (data->data_type == kCartesian) {
				LivoxRawPoint *p_point_data = (LivoxRawPoint *)data->data;
			}
			else if (data->data_type == kSpherical) {
				LivoxSpherPoint *p_point_data = (LivoxSpherPoint *)data->data;
			}
			else if (data->data_type == kExtendCartesian) {
				LivoxExtendRawPoint *p_point_data = (LivoxExtendRawPoint *)data->data;
			}
			else if (data->data_type == kExtendSpherical) {
				LivoxExtendSpherPoint *p_point_data = (LivoxExtendSpherPoint *)data->data;
			}
			else if (data->data_type == kDualExtendCartesian) {
				LivoxDualExtendRawPoint *p_point_data = (LivoxDualExtendRawPoint *)data->data;
			}
			else if (data->data_type == kDualExtendSpherical) {
				LivoxDualExtendSpherPoint *p_point_data = (LivoxDualExtendSpherPoint *)data->data;
			}
			else if (data->data_type == kImu) {
				LivoxImuPoint *p_point_data = (LivoxImuPoint *)data->data;
			}
			else if (data->data_type == kTripleExtendCartesian) {
				LivoxTripleExtendRawPoint *p_point_data = (LivoxTripleExtendRawPoint *)data->data;
			}
			else if (data->data_type == kTripleExtendSpherical) {
				LivoxTripleExtendSpherPoint *p_point_data = (LivoxTripleExtendSpherPoint *)data->data;
			}
		}
	}
}

/** Checking on Broadcast by device type and connect. */
void LdsLidar::OnDeviceBroadcast(const BroadcastDeviceInfo *info) {
	if (info == nullptr) {return;}
	if (info->dev_type == kDeviceTypeHub) {
		printf("In lidar mode, couldn't connect a hub : %s\n", info->broadcast_code);
		return;
	}
	if (g_lidars->IsAutoConnectMode()) {
		printf("In automatic connection mode, will connect %s\n", info->broadcast_code);
	}
	else {
		if (!g_lidars->FindInWhitelist(info->broadcast_code)) {
			printf("Not in the whitelist, please add %s to if want to connect!\n", info->broadcast_code);
			return;
		}
	}
	livox_status result = kStatusFailure;
	uint8_t handle = 0;
	result = AddLidarToConnect(info->broadcast_code, &handle);
	if (result == kStatusSuccess && handle < kMaxLidarCount) {
		SetDataCallback(handle, LdsLidar::GetLidarDataCb, (void *)g_lidars);
		LidarDevice* p_lidar = &(g_lidars->lidars_[handle]);
		p_lidar->handle = handle;
		p_lidar->connect_state = kConnectStateOff;
		p_lidar->config.enable_fan = true;
		p_lidar->config.return_mode = kStrongestReturn;
		p_lidar->config.coordinate = kCoordinateCartesian;
		p_lidar->config.imu_rate = kImuFreq200Hz;
	}
	else {
		printf("Add lidar to connect is failed : %d %d \n", result, handle);
	}
}

/** Callback function of changing of device state. */
void LdsLidar::OnDeviceChange(const DeviceInfo *info, DeviceEvent type) {
	if (info == nullptr) {return;}
	uint8_t handle = info->handle;
	if (handle >= kMaxLidarCount) {return;}

	LidarDevice* p_lidar = &(g_lidars->lidars_[handle]);
	if (type == kEventConnect) {
		QueryDeviceInformation(handle, DeviceInformationCb, g_lidars);
		if (p_lidar->connect_state == kConnectStateOff) {
			p_lidar->connect_state = kConnectStateOn;
			p_lidar->info = *info;
		}
		printf("[WARNING] Lidar sn: [%s] Connect!!!\n", info->broadcast_code);
	}
	else if (type == kEventDisconnect) {
		p_lidar->connect_state = kConnectStateOff;
		printf("[WARNING] Lidar sn: [%s] Disconnect!!!\n", info->broadcast_code);
	}
	else if (type == kEventStateChange) {
		p_lidar->info = *info;
		printf("[WARNING] Lidar sn: [%s] StateChange!!!\n", info->broadcast_code);
	}
	if (p_lidar->connect_state == kConnectStateOn) {
		printf("Device Working State %d\n", p_lidar->info.state);
		if (p_lidar->info.state == kLidarStateInit) {
			printf("Device State Change Progress %u\n", p_lidar->info.status.progress);
		}
		else {
			printf("Device State Error Code 0X%08x\n", p_lidar->info.status.status_code.error_code);
		}
		printf("Device feature %d\n", p_lidar->info.feature);
		SetErrorMessageCallback(handle, LdsLidar::LidarErrorStatusCb);
		/** Config lidar parameter */
		if (p_lidar->info.state == kLidarStateNormal) {
			if (p_lidar->config.coordinate != 0) {
				SetSphericalCoordinate(handle, LdsLidar::SetCoordinateCb, g_lidars);
			}
			else {
				SetCartesianCoordinate(handle, LdsLidar::SetCoordinateCb, g_lidars);
			}
			p_lidar->config.set_bits |= kConfigCoordinate;
			if (kDeviceTypeLidarMid40 != info->type) {
				LidarSetPointCloudReturnMode(handle, (PointCloudReturnMode)(p_lidar->config.return_mode), \
					LdsLidar::SetPointCloudReturnModeCb, g_lidars);
				p_lidar->config.set_bits |= kConfigReturnMode;
			}
			if (kDeviceTypeLidarMid40 != info->type && kDeviceTypeLidarMid70 != info->type) {
				LidarSetImuPushFrequency(handle, (ImuFreq)(p_lidar->config.imu_rate), \
					LdsLidar::SetImuRatePushFrequencyCb, g_lidars);
				p_lidar->config.set_bits |= kConfigImuRate;
			}
			p_lidar->connect_state = kConnectStateConfig;
		}
	}
}

/** Query the firmware version of Livox LiDAR. */
void LdsLidar::DeviceInformationCb(livox_status status, uint8_t handle, DeviceInformationResponse *ack, void *client_data)
{
	if (status != kStatusSuccess) {printf("Device Query Informations Failed : %d\n", status);}
	if (ack) {printf("firm ver: %d.%d.%d.%d\n",ack->firmware_version[0],ack->firmware_version[1],ack->firmware_version[2],ack->firmware_version[3]);}
}

/** Callback function of Lidar error message. */
void LdsLidar::LidarErrorStatusCb(livox_status status, uint8_t handle, ErrorMessage *message) {
	static uint32_t error_message_count = 0;
	if (message != NULL) {
		++error_message_count;
		if (0 == (error_message_count % 100)) {
			printf("handle: %u\n", handle);
			printf("temp_status : %u\n", message->lidar_error_code.temp_status);
			printf("volt_status : %u\n", message->lidar_error_code.volt_status);
			printf("motor_status : %u\n", message->lidar_error_code.motor_status);
			printf("dirty_warn : %u\n", message->lidar_error_code.dirty_warn);
			printf("firmware_err : %u\n", message->lidar_error_code.firmware_err);
			printf("pps_status : %u\n", message->lidar_error_code.device_status);
			printf("fan_status : %u\n", message->lidar_error_code.fan_status);
			printf("self_heating : %u\n", message->lidar_error_code.self_heating);
			printf("ptp_status : %u\n", message->lidar_error_code.ptp_status);
			printf("time_sync_status : %u\n", message->lidar_error_code.time_sync_status);
			printf("system_status : %u\n", message->lidar_error_code.system_status);
		}
	}
}

void LdsLidar::ControlFanCb(livox_status status, uint8_t handle, uint8_t response, void *client_data) {}

void LdsLidar::SetPointCloudReturnModeCb(livox_status status, uint8_t handle, uint8_t response, void *client_data) 
{
	LdsLidar* lds_lidar = static_cast<LdsLidar *>(client_data);
	if (handle >= kMaxLidarCount) { return;	}
	LidarDevice* p_lidar = &(lds_lidar->lidars_[handle]);
	if (status == kStatusSuccess) {
		p_lidar->config.set_bits &= ~((uint32_t)(kConfigReturnMode));
		printf("Set return mode success!\n");
		if (!p_lidar->config.set_bits) {
			LidarStartSampling(handle, LdsLidar::StartSampleCb, lds_lidar);
			p_lidar->connect_state = kConnectStateSampling;
		}
	}
	else {
		LidarSetPointCloudReturnMode(handle, (PointCloudReturnMode)(p_lidar->config.return_mode), \
			LdsLidar::SetPointCloudReturnModeCb, lds_lidar);
		printf("Set return mode fail, try again!\n");
	}
}

void LdsLidar::SetCoordinateCb(livox_status status, uint8_t handle, uint8_t response, void *client_data) 
{
	LdsLidar* lds_lidar = static_cast<LdsLidar *>(client_data);
	if (handle >= kMaxLidarCount) {	return; }
	LidarDevice* p_lidar = &(lds_lidar->lidars_[handle]);
	if (status == kStatusSuccess) {
		p_lidar->config.set_bits &= ~((uint32_t)(kConfigCoordinate));
		printf("Set coordinate success!\n");
		if (!p_lidar->config.set_bits) {
			LidarStartSampling(handle, LdsLidar::StartSampleCb, lds_lidar);
			p_lidar->connect_state = kConnectStateSampling;
		}
	}
	else {
		if (p_lidar->config.coordinate != 0) {
			SetSphericalCoordinate(handle, LdsLidar::SetCoordinateCb, lds_lidar);
		}
		else {
			SetCartesianCoordinate(handle, LdsLidar::SetCoordinateCb, lds_lidar);
		}
		printf("Set coordinate fail, try again!\n");
	}
}

void LdsLidar::SetImuRatePushFrequencyCb(livox_status status, uint8_t handle, uint8_t response, void *client_data) 
{
	LdsLidar* lds_lidar = static_cast<LdsLidar *>(client_data);
	if (handle >= kMaxLidarCount) {
		return;
	}
	LidarDevice* p_lidar = &(lds_lidar->lidars_[handle]);
	if (status == kStatusSuccess) {
		p_lidar->config.set_bits &= ~((uint32_t)(kConfigImuRate));
		printf("Set imu rate success!\n");
		if (!p_lidar->config.set_bits) {
			LidarStartSampling(handle, LdsLidar::StartSampleCb, lds_lidar);
			p_lidar->connect_state = kConnectStateSampling;
		}
	}
	else {
		LidarSetImuPushFrequency(handle, (ImuFreq)(p_lidar->config.imu_rate), LdsLidar::SetImuRatePushFrequencyCb, lds_lidar);
		printf("Set imu rate fail, try again!\n");
	}
}

/** Callback function of starting sampling. */
void LdsLidar::StartSampleCb(livox_status status, uint8_t handle, uint8_t response, void *client_data)
{
	LdsLidar* lds_lidar = static_cast<LdsLidar *>(client_data);
	if (handle >= kMaxLidarCount) {
		return;
	}
	LidarDevice* p_lidar = &(lds_lidar->lidars_[handle]);
	if (status == kStatusSuccess) {
		if (response != 0) {
			p_lidar->connect_state = kConnectStateOn;
			printf("Lidar start sample fail : state[%d] handle[%d] res[%d]\n", status, handle, response);
		}
		else {
			printf("Lidar start sample success\n");
		}
	}
	else if (status == kStatusTimeout) {
		p_lidar->connect_state = kConnectStateOn;
		printf("Lidar start sample timeout : state[%d] handle[%d] res[%d]\n", status, handle, response);
	}
}

/** Callback function of stopping sampling. */
void LdsLidar::StopSampleCb(livox_status status, uint8_t handle, uint8_t response, void *client_data) {}

/** Add broadcast code to whitelist */
int LdsLidar::AddBroadcastCodeToWhitelist(const char* bd_code)
{
	if (!bd_code || (strlen(bd_code) > kBroadcastCodeSize) || (whitelist_count_ >= kMaxLidarCount)) {
		return -1;
	}
	if (LdsLidar::FindInWhitelist(bd_code)) {
		printf("%s is alrealy exist!\n", bd_code);
		return -1;
	}
	strcpy(broadcast_code_whitelist_[whitelist_count_], bd_code);
	++whitelist_count_;
	return 0;
}

void LdsLidar::AddLocalBroadcastCode(void) {
	for (size_t i = 0; i < sizeof(local_broadcast_code_list) / sizeof(intptr_t); ++i) {
		std::string invalid_bd = "000000000";
		printf("Local broadcast code : %s\n", local_broadcast_code_list[i]);
		if ((kBroadcastCodeSize == strlen(local_broadcast_code_list[i]) + 1) && \
			(nullptr == strstr(local_broadcast_code_list[i], invalid_bd.c_str()))) 
		{
			LdsLidar::AddBroadcastCodeToWhitelist(local_broadcast_code_list[i]);
		}
		else {
			printf("Invalid local broadcast code : %s\n", local_broadcast_code_list[i]);
		}
	}
}

bool LdsLidar::FindInWhitelist(const char* bd_code) {
	if (!bd_code) {
		return false;
	}
	for (uint32_t i = 0; i < whitelist_count_; i++) {
		if (strncmp(bd_code, broadcast_code_whitelist_[i], kBroadcastCodeSize) == 0) {
			return true;
		}
	}
	return false;
}


