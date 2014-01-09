#include "Vistek.h"
#include "VistekInterface.h"
#include "VistekContainer.h"

#define _USE_MATH_DEFINES  // needs to be defined to enable standard declartions of PI constant
#include "math.h"

#include <qstring.h>
#include <qstringlist.h>
#include <QtCore/QtPlugin>
#include <qmetaobject.h>

#include <qdockwidget.h>
#include <qpushbutton.h>
#include <qmetaobject.h>
#include "dockWidgetVistek.h"


VistekContainer* VistekContainer::m_pVistekContainer = NULL;

Q_DECLARE_METATYPE(ito::DataObject)

//----------------------------------------------------------------------------------------------------------------------------------
/*!
    \class Vistek
    \brief Class that can handle SVS Vistek GigE Cameras.

    Usually every method in this class can be executed in an own thread. Only the constructor, destructor, showConfDialog will be executed by the 
    main (GUI) thread.
*/

//----------------------------------------------------------------------------------------------------------------------------------
//! Shows the configuration dialog. This method must be executed in the main (GUI) thread and is usually called by the addIn-Manager.
/*!
    Creates new instance of dialogVistek, calls the method setVals of dialogVistek, starts the execution loop and if the dialog
    is closed, reads the new parameter set and deletes the dialog.

    \return ito::RetVal retOk
    \sa dialogVistek
*/
const ito::RetVal Vistek::showConfDialog(void)
{
    ito::RetVal retValue(ito::retOk);

    DialogVistek *confDialog = new DialogVistek();
    connect(this, SIGNAL(parametersChanged(QMap<QString, ito::Param>)), confDialog, SLOT(valuesChanged(QMap<QString, ito::Param>)));
    QMetaObject::invokeMethod(this, "sendParameterRequest");

    if (confDialog->exec())
    {
        disconnect(this, SIGNAL(parametersChanged(QMap<QString, ito::Param>)), confDialog, SLOT(valuesChanged(QMap<QString, ito::Param>)));
        confDialog->sendVals(this);
    }
    else
    {
        disconnect(this, SIGNAL(parametersChanged(QMap<QString, ito::Param>)), confDialog, SLOT(valuesChanged(QMap<QString, ito::Param>)));
    }
    delete confDialog;

    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
//! constructor for Vistek
/*!
    In the constructor the m_params-vector with all parameters, which are accessible by getParam or setParam, is built.
    Additionally the optional docking widget for the Vistek's toolbar is instantiated and created by createDockWidget.

    \param [in] uniqueID is an unique identifier for this Vistek-instance
    \param [in] parent is the parent object for this grabber (default: NULL)
    \sa ito::tParam, createDockWidget, setParam, getParam
*/
Vistek::Vistek(QObject *parent) : 
    AddInGrabber(),
    m_pVistekContainer(NULL)
{

    Cam = SVGigE_NO_CAMERA;

    ito::Param paramVal("name", ito::ParamBase::String, "Vistek", NULL);
    m_params.insert(paramVal.getName(), paramVal);

    // Camera specific information
    paramVal = ito::Param("CameraModel", ito::ParamBase::String, tr("").toAscii().data(), tr("Camera Model ID").toAscii().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("CameraManufacturer", ito::ParamBase::String, tr("").toAscii().data(), tr("Camera manufacturer").toAscii().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("CameraVersion", ito::ParamBase::String, tr("").toAscii().data(), tr("Camera firmware version").toAscii().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("CameraSerialNo", ito::ParamBase::String, tr("").toAscii().data(), tr("Serial number of the camera (see camera housing)").toAscii().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("CameraIP", ito::ParamBase::String, tr("").toAscii().data(), tr("IP adress of the camera").toAscii().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("camnum", ito::ParamBase::Int, 0, 63, 0, tr("Camera Number").toAscii().data());
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param("exposure", ito::ParamBase::Double, 0.00001, 2.0, 0.0, tr("Exposure time in [s]").toAscii().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("gain", ito::ParamBase::Double, 0.0, 18.0, 0.0, tr("Gain [0..18 dB]").toAscii().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("binning", ito::ParamBase::Int, 101, 404, 101, tr("Binning of different pixel").toAscii().data());
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param("sizex", ito::ParamBase::Int, 1, 4096, 1024, tr("Width of current camera frame").toAscii().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("sizey", ito::ParamBase::Int, 1, 4096, 1024, tr("Height of current camera frame").toAscii().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("bpp", ito::ParamBase::Int, 8, 64, 8, tr("bit-depth for camera buffer").toAscii().data());
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param("timestamp", ito::ParamBase::Double, 0.0, 10000000.0, 0.0, tr("Time in ms since last image (end of exposure)").toAscii().data());
    m_params.insert(paramVal.getName(), paramVal);

    //now create dock widget for this plugin
    DockWidgetVistek *dw = new DockWidgetVistek(m_params, getID());
    connect(this, SIGNAL(parametersChanged(QMap<QString, ito::Param>)), dw, SLOT(valuesChanged(QMap<QString, ito::Param>)));
    connect(dw, SIGNAL(dataPropertiesChanged(int,int,int)), this, SLOT(dataParametersChanged(int,int,int)));
    connect(dw, SIGNAL(gainPropertiesChanged(double)), this, SLOT(gainPropertiesChanged(double)));
    connect(dw, SIGNAL(exposurePropertiesChanged(double)), this, SLOT(exposurePropertiesChanged(double)));

    Qt::DockWidgetAreas areas = Qt::AllDockWidgetAreas;
    QDockWidget::DockWidgetFeatures features = QDockWidget::DockWidgetClosable | QDockWidget::DockWidgetFloatable | QDockWidget::DockWidgetMovable;
    createDockWidget(QString(m_params["name"].getVal<char *>()), features, areas, dw);

    checkData();
    

}

//----------------------------------------------------------------------------------------------------------------------------------
//! This is the destructor of the Vistek class.
/*!
    \sa ~AddInBase
*/
Vistek::~Vistek()
{
   m_params.clear();
}

//----------------------------------------------------------------------------------------------------------------------------------
//! Returns parameter of m_params with key name.
/*!
    This method copies the string of the corresponding parameter to val with a maximum length of maxLen.

    \param [in,out] val is a shared-pointer pointing to a Param.
    \param [in] waitCond is the semaphore (default: NULL), which is released if this method has been terminated
    \return ito::RetVal retOk in case that everything is ok, else retError
    \sa ito::Param, setParam ItomSharedSemaphore
*/
ito::RetVal Vistek::getParam(QSharedPointer<ito::Param> val, ItomSharedSemaphore *waitCond)
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
//! Sets parameter of m_params with key name. 
/*!
    This method copies the given string of the char pointer val with a size of len to the m_params-parameter.

    \param [in] val is the pointer to the ParamBase.
    \param [in] waitCond is the semaphore (default: NULL), which is released if this method has been terminated
    \return ito::RetVal retOk in case that everything is ok, else retError
    \sa ito::Param, ItomSharedSemaphore
*/
ito::RetVal Vistek::setParam(QSharedPointer<ito::ParamBase> val, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);
    QString key = val->getName();

    if(key == "")
    {
        retValue += ito::RetVal(ito::retError, 0, tr("Name of given parameter is empty.").toAscii().data());
    }
    else
    {
        QMap<QString, ito::Param>::iterator paramIt = m_params.find(key);
        if (paramIt != m_params.end())
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
                /*else if(m_isgrabbing = true && (!key.compare("sizex") || !key.compare("sizey") || !key.compare("x0") || !key.compare("y0")))
                {
                    retValue += ito::RetVal(ito::retError, 0, tr("camera is grabbing, stop camera first").toAscii().data());        
                                                    
                }*/
                else 
                {
                    if(!key.compare("exposure"))
                    {
                        // Camera_setExposureTime expects µs, so multiply by 10^6
                        retValue += Camera_setExposureTime(Cam, curval*1000000);
                    };
                    if(!key.compare("gain"))
                    {
                        // Camera_setGain expects a value between 0 and 18 in dB
                        retValue += Camera_setGain(Cam, curval);
                    };
                    if(!key.compare("binning"))
                    {
                        BINNING_MODE OldBinning;
                        // Camera_setGain expects a value between 0 and 18 in dB
                        if(curval==101) BinningMode = BINNING_MODE_OFF;
                        if(curval==102) BinningMode = BINNING_MODE_HORIZONTAL;
                        if(curval==201) BinningMode = BINNING_MODE_VERTICAL;
                        if(curval==202) BinningMode = BINNING_MODE_2x2;
                        if(curval==303) BinningMode = BINNING_MODE_3x3;
                        if(curval==404) BinningMode = BINNING_MODE_4x4;
                        Camera_getBinningMode(Cam, &OldBinning);
                        if(OldBinning!=BinningMode)
                        {
                            // Set new binning mode if neccessary. On failure set curval to 101, as the camera might not accept the new binning.
                            // The API has no function to check if a binning mode is valid other than failing to set it...
                            ito::RetVal tempRet = updateStreamingChannel();
                            if(tempRet==ito::retError)
                                curval = 101;
                            m_params["binning"].setVal<int>(int(curval));
                            retValue += tempRet;
                        }
                    };
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
        }
        else if(paramIt->getType() != val->getType())
        {
            retValue += ito::RetVal(ito::retError, 0, tr("Given parameter and m_param do not have the same type").toAscii().data());
        }
        else
        {
            retValue += ito::RetVal(ito::retError, 0, tr("Parameter not found in m_params.").toAscii().data());
        }
    }
end:
    emit parametersChanged(m_params);
    retValue += checkData(); //check if data must be reallocated

    if (waitCond) 
    {
        waitCond->returnValue = retValue;
        waitCond->release();
    }

    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
//! The init method is called by the addInManager after the initiation of a new instance of Vistek.
/*!
    This init method gets the mandatory and optional parameter vectors of type tParam and must copy these given parameters to the
    internal m_params-vector. Notice that this method is called after that this instance has been moved to its own (non-gui) thread.

    \param [in] paramsMand is a pointer to the vector of mandatory ParamBase.
    \param [in] paramsOpt is a pointer to the vector of optional ParamBase.
    \param [in] waitCond is the semaphore (default: NULL), which is released if this method has been terminated
    \return ito::RetVal retOk
*/
ito::RetVal Vistek::init(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, ItomSharedSemaphore *waitCond)
{
    int NumberOfCams = 0;
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal ReturnValue = ito::retOk;
    int CamNum;
    VistekCam TempCam;

    //*********************************************
    setParam( QSharedPointer<ito::ParamBase>(new ito::ParamBase("CameraSerialNo",ito::ParamBase::String,(*paramsOpt)[0].getVal<char*>())), NULL);
    m_pVistekContainer = VistekContainer::getInstance();
    ReturnValue += m_pVistekContainer->initCameraContainer();

    CamNum = -1;

    // Check if a valid Serial number is specified
    if (m_params["CameraSerialNo"].getVal<char*>() != NULL)
    {
        CamNum = m_pVistekContainer->getCameraBySN(m_params["CameraSerialNo"].getVal<char*>());
    }

    if ( CamNum < 0)
    {
        CamNum = m_pVistekContainer->getNextFreeCam();
    }

    if ( CamNum >= 0)
    {
        setParam( QSharedPointer<ito::ParamBase>(new ito::ParamBase("camnum", m_params["camnum"].getType(), CamNum)));
        // Get camera info
        TempCam = m_pVistekContainer->getCamInfo(CamNum);
        setParam( QSharedPointer<ito::ParamBase>(new ito::ParamBase("CameraModel",ito::ParamBase::String,TempCam.camModel.toAscii().data())), NULL);
        setParam( QSharedPointer<ito::ParamBase>(new ito::ParamBase("CameraSerialNo",ito::ParamBase::String,TempCam.camSerialNo.toAscii().data())), NULL);
        m_identifier = TempCam.camSerialNo;
        setParam( QSharedPointer<ito::ParamBase>(new ito::ParamBase("CameraVersion",ito::ParamBase::String,TempCam.camVersion.toAscii().data())), NULL);
        setParam( QSharedPointer<ito::ParamBase>(new ito::ParamBase("CameraIP",ito::ParamBase::String,TempCam.camIP.toAscii().data())), NULL);
        setParam( QSharedPointer<ito::ParamBase>(new ito::ParamBase("CameraManufacturer",ito::ParamBase::String,TempCam.camManufacturer.toAscii().data())), NULL);

        ReturnValue += initCamera(CamNum);

        setParam( QSharedPointer<ito::ParamBase>(new ito::ParamBase("exposure",m_params["exposure"].getType(), 0.06)));
        setParam( QSharedPointer<ito::ParamBase>(new ito::ParamBase("gain", m_params["gain"].getType(), 0.0)));
        setParam( QSharedPointer<ito::ParamBase>(new ito::ParamBase("binning", m_params["binning"].getType(), 101)));
        //Camera_readEEPROM(Cam);
    }
    else
    {
        ReturnValue += ito::RetVal(ito::retError,0,tr("No free camera found").toAscii().data());
    }
    //*********************************************
    checkData(); //check if data must be reallocated

    if(waitCond)
    {
        waitCond->returnValue = ReturnValue;
        waitCond->release();

        setInitialized(true); //init method has been finished (independent on retval)
        return waitCond->returnValue;
    }
    else
    {
        setInitialized(true); //init method has been finished (independent on retval)
        return ReturnValue;
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
//! The close method is called before an instance is deleted by the VistekInterface
/*!
    Notice that this method is called in the actual thread of the instance.

    \param [in] waitCond is the semaphore (default: NULL), which is released if this method has been terminated
    \return ito::RetVal retOk
    \sa ItomSharedSemaphore
*/
ito::RetVal Vistek::close(ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);

    if(m_timerID > 0)
    {
        killTimer(m_timerID);
        m_timerID = 0;
    }

    // ONE TIME DEINIT HERE (Holger)
    //*********************************************
    if( Cam != SVGigE_NO_CAMERA)
    {
        std::cout << "Forcing camera shutdown.\n" << std::endl;
        Camera_closeConnection(Cam); // This is necessary to prevent an access violation if m gets terminated with an open cam connection
        StreamingChannel_delete(StreamingChannel);
        Cam = SVGigE_NO_CAMERA;
        m_pVistekContainer->freeCameraStatus(m_params["camnum"].getVal<int>());
    }
    //*********************************************

    if(waitCond)
    {
        waitCond->returnValue = ito::retOk;
        waitCond->release();

        return waitCond->returnValue;
    }
    else
    {
        return ito::retOk;
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
//! With startDevice the camera aquisition is started.
/*!
    This method sets the acquisition mode of the camera to software triggering.

    \note This method is similar to VideoCapture::open() of openCV

    \param [in] waitCond is the semaphore (default: NULL), which is released if this method has been terminated
    \return ito::RetVal retOk if starting was successfull, retWarning if startDevice has been calling at least twice.
    \sa stopDevice
*/
ito::RetVal Vistek::startDevice(ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue = ito::retOk;
    
    checkData(); //this will be reallocated in this method.

    incGrabberStarted();

    if(grabberStartedCount() == 1)
    {
        //*********************************************
        if( Cam != SVGigE_NO_CAMERA)
        {
            retValue += Camera_setAcquisitionControl(Cam, ACQUISITION_CONTROL_START);
        }
        //*********************************************
    }

    // Dont count a grabber that is not really started ...
    if ( retValue != ito::retOk )
        decGrabberStarted();

    if(waitCond)
    {
        waitCond->returnValue = retValue;
        waitCond->release();
    }

    return retValue;
}
         
//----------------------------------------------------------------------------------------------------------------------------------
//! With stopDevice the camera device is stopped (opposite to startDevice)
/*!
    This method sets the acquisition mode of the camera to off.

    \note This method is similar to VideoCapture::release() of openCV

    \param [in] waitCond is the semaphore (default: NULL), which is released if this method has been terminated
    \return ito::RetVal retOk if everything is ok, retError if camera wasn't started before
    \sa startDevice
*/
ito::RetVal Vistek::stopDevice(ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue = ito::retOk;

    decGrabberStarted();
    if(grabberStartedCount() == 0)
    {
        //*********************************************
        if( Cam != SVGigE_NO_CAMERA)
        {
            retValue += Camera_setAcquisitionControl(Cam, ACQUISITION_CONTROL_START);
        }
        //*********************************************
    }
    else if(grabberStartedCount() < 0)
    {
        retValue += ito::RetVal(ito::retError, 1001, tr("StopDevice of Vistek can not be executed, since camera has not been started.").toAscii().data());
        setGrabberStarted(0);
    }


    if(waitCond)
    {
        waitCond->returnValue = retValue;
        waitCond->release();
    }

    return retValue;
}
         
//----------------------------------------------------------------------------------------------------------------------------------
//! This method triggers a new data acquisition.
/*!
    With this method a new data acquisition is triggered by the camera, that means the acquisition of the data starts at the moment, this method is called.
    The new data is then stored either in internal camera memory or in internal memory of this class.

    \note This method is similar to VideoCapture::grab() of openCV

    \param [in] trigger may describe the trigger parameter (unused here)
    \param [in] waitCond is the semaphore (default: NULL), which is released if this method has been terminated
    \return ito::RetVal retOk if everything is ok, retError if camera has not been started or an older data lies in memory which has not be fetched by getVal, yet.
    \sa getVal, retrieveData
*/
ito::RetVal Vistek::acquire(const int trigger, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue = ito::retOk;
    
    int timeout = 0;

    if(grabberStartedCount() <= 0)
    {
        retValue += ito::RetVal(ito::retError, 1002, tr("Acquire of Vistek can not be executed, since camera has not been started.").toAscii().data());
    }
    else
    {
        if (Camera_softwareTrigger(Cam) != SVGigE_SUCCESS)
        {
            retValue += ito::RetVal(ito::retError, 1002, tr("Camera trigger failed.").toAscii().data());
        }
    }

    if(waitCond)
    {
        waitCond->returnValue = retValue;
        waitCond->release();
    }

    if (retValue == ito::retOk)
    {
        while(!FrameCompletedFlag)
        {
            if (timeout > 2000)
            {
                std::cout<<"timeout\n";
                break;
            }
            Sleep(1);
            timeout++;
        }
        FrameCompletedFlag = false;
    }

    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
//! This method copies the acquired image data to externalDataObject or m_data.
/*!
    If *externalDataObject is NULL then it will be reassigned to &m_data so the image data is copied to m_data instead.

    \param [in] *externalDataObject is the data object where the image is to be stored. (Defaults to NULL)
    \return ito::RetVal retOk if everything is ok, retError if camera has not been started or no image has been acquired.
    \sa DataObject, getVal, acquire
*/
ito::RetVal Vistek::retrieveData(ito::DataObject *externalDataObject)
{
    ito::RetVal retValue(ito::retOk);

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

    if(grabberStartedCount() <= 0)
    {
        retValue += ito::RetVal(ito::retError, 1002, tr("getVal of Vistek can not be executed, since camera has not been started.").toAscii().data());
    }
    if(ImageData==NULL)
    {
        retValue += ito::RetVal(ito::retError, 1002, tr("getVal of Vistek can not be executed, since image data is empty.").toAscii().data());
    }
    else
    {
        if (retValue != ito::retError)
        {
            if(m_data.getType() == ito::tUInt8)
            {
                if(copyExternal) retValue += externalDataObject->copyFromData2D<ito::uint8>((ito::uint8*) ImageData, SizeX, SizeY);
                if(!copyExternal || hasListeners) retValue += m_data.copyFromData2D<ito::uint8>((ito::uint8*) ImageData, SizeX, SizeY);
            }
            else if(m_data.getType() == ito::tUInt16)
            {
                if(copyExternal) retValue += externalDataObject->copyFromData2D<ito::uint16>((ito::uint16*) ImageData, SizeX, SizeY);
                if(!copyExternal || hasListeners) retValue += m_data.copyFromData2D<ito::uint16>((ito::uint16*) ImageData, SizeX, SizeY);            
            }
            else
            {
                retValue += ito::RetVal(ito::retError, 1002, tr("copy buffer during getVal of Vistek can not be executed, since no DataType unknown.").toAscii().data());
            }
        }
        else
            retValue += ito::RetVal(ito::retError, 1002, tr("getVal of Vistek can not be executed, since no Data has been acquired.").toAscii().data());
    }

    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
//! Returns the grabbed camera frame.
/*!
    This method copies the recently grabbed camera frame to the given DataObject. Therefore this camera size must fit to the data structure of the 
    DataObject.

    \note This method is similar to VideoCapture::retrieve() of openCV

    \param [in,out] vpdObj is the pointer to a given dataObject (this pointer should be cast to ito::DataObject*) where the acquired data is copied to.
    \param [in] waitCond is the semaphore (default: NULL), which is released if this method has been terminated
    \return retOk if everything is ok, retError is camera has not been started or no data has been acquired by the method acquire.
    \sa DataObject, acquire, retrieveData
*/
ito::RetVal Vistek::getVal(void *vpdObj, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::DataObject *dObj = reinterpret_cast<ito::DataObject *>(vpdObj);

    ito::RetVal retValue(ito::retOk);

    retValue += retrieveData();

    if(!retValue.containsError())
    {

        if(dObj == NULL)
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
        waitCond->returnValue=retValue;
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
ito::RetVal Vistek::copyVal(void *vpdObj, ItomSharedSemaphore *waitCond)
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
//! Slot invoked if data parameters in docking toolbox have been manually changed
/*!
    \param [in] sizex width of camera data
    \param [in] sizey height of camera data
    \param [bpp] bit depth of camera data
*/
void Vistek::dataParametersChanged(int sizex, int sizey, int bpp)
{
    m_params["sizex"].setVal<int>(sizex);
    m_params["sizey"].setVal<int>(sizey);
    m_params["bpp"].setVal<int>(bpp);

    checkData();
}

//----------------------------------------------------------------------------------------------------------------------------------
//! Slot invoked if gain parameters in docking toolbox has been manually changed
/*!
    \param [in] gain new gain value
*/
void Vistek::gainPropertiesChanged(double gain)
{
    setParam( QSharedPointer<ito::ParamBase>(new ito::ParamBase("gain", m_params["gain"].getType(), gain)));
}

//----------------------------------------------------------------------------------------------------------------------------------
//! Slot invoked if exposure parameters in docking toolbox has been manually changed
/*!
    \param [in] exposure new exposure value
*/
void Vistek::exposurePropertiesChanged(double exposure)
{
    setParam( QSharedPointer<ito::ParamBase>(new ito::ParamBase("exposure", m_params["exposure"].getType(), exposure)));
}

//----------------------------------------------------------------------------------------------------------------------------------
//! This method is only called during init and initializes the camera with the number CameraNumber.
/*!
    If the camera has no valid IP address, it is forced to a new one. The DataCallback for the camera is registered here.
    \sa DataCallback, init
*/
ito::RetVal Vistek::initCamera(int CameraNumber)
{
    // Some Variables that might be needed
    SVGigE_RETURN SVGigERet;
    unsigned int IPAddress = 0;
    unsigned int SubnetMask = 0;
    char Msg[256];

    // Check if there already is an open handle for a camera
    if( Cam != SVGigE_NO_CAMERA )
    {
        // TODO: Shutdown the cam first.
        std::cout << "Camera handle exists, trying to close an existing connection." << std::endl;
        StreamingChannel_delete(StreamingChannel);
        Camera_closeConnection(Cam);
        Cam = SVGigE_NO_CAMERA;
    }

    // Get a Camera Handle
    std::cout << "Getting camera handle ... " << std::endl;
    Cam = CameraContainer_getCamera(m_pVistekContainer->getCameraContainerHandle(), CameraNumber);
    if( SVGigE_NO_CAMERA == Cam )
    {
        return ito::RetVal(ito::retError, 1004, tr("Requested camera could not be selected.").toAscii().data());
    }
    std::cout << "done!\n" << std::endl;

    // Open the camera connection
    std::cout << "Opening camera connection ... " << std::endl;
    if( SVGigE_SUCCESS != Camera_openConnection(Cam, 30) )
    {
        std::cout << "\nOpening camera failed. Trying to enforce valid network settings.\n" << std::endl;
        // Check for valid network settings & doing necessary steps
        SVGigERet = Camera_forceValidNetworkSettings(Cam, &IPAddress, &SubnetMask);
        if( SVGigERet == SVGigE_SVCAM_STATUS_CAMERA_OCCUPIED )
        {
            return ito::RetVal(ito::retError, 1004, tr("Requested camera is occupied by another application.").toAscii().data());
        }
        
        if( SVGigERet != SVGigE_SUCCESS )
        {
            return ito::RetVal(ito::retError, 1004, tr("Enforcing valid network settings failed.").toAscii().data());
        }

        // Try to open the connection again
        SVGigERet = Camera_openConnection(Cam, 30);

        // print out the new valid network settings
        memset(Msg,0,sizeof(Msg));
        sprintf_s(Msg,255,"Camera has been forced to a valid IP '%d.%d.%d.%d'\n", 
            (IPAddress >> 24)%256, (IPAddress >> 16)%256, (IPAddress >> 8)%256, IPAddress%256);
        std::cout << Msg << std::endl;

        if( SVGigERet != SVGigE_SUCCESS )
        {
            return ito::RetVal(ito::retError, 1004, tr("Selected camera could not be opened.").toAscii().data());
        }
    }
    std::cout << "done!\n" << std::endl;

    registerCallbacks();

    return ito::retOk;
}

ito::RetVal Vistek::updateStreamingChannel()
{
    ito::RetVal ret = ito::retOk;
    SVGigE_RETURN SVGigERet;

    Camera_setAcquisitionMode(Cam, ACQUISITION_MODE_NO_ACQUISITION);

    // Unregister callbacks
    Stream_unregisterEventCallback(StreamingChannel, EventID, &MessageCallback);
    EventID = SVGigE_NO_EVENT;
    StreamingChannel_delete(StreamingChannel);
    
    SVGigERet = Camera_setBinningMode(Cam, BinningMode);
    if(SVGigERet != SVGigE_SUCCESS)
    {
        ret += ito::RetVal(ito::retError, 1004, tr("Binning mode set failed! Defaulting to 1x1.").toAscii().data());
        Camera_setBinningMode(Cam, BINNING_MODE_OFF);
    }

    ret += registerCallbacks();

    return ret;
}

void Vistek::updateTimestamp()
{
    double timestamp = (MessageTimestampStartOfTransfer - MessageTimestampLastStartOfTransfer) * 1000;
    setParam( QSharedPointer<ito::ParamBase>(new ito::ParamBase("timestamp", m_params["timestamp"].getType(), timestamp)));
}

ito::RetVal Vistek::registerCallbacks()
{
    SVGigE_RETURN SVGigERet;
    int TempInt;

    // Obtain geometry data
    Camera_getSizeX(Cam, &TempInt);
    setParam( QSharedPointer<ito::ParamBase>(new ito::ParamBase("sizex",ito::ParamBase::Int,TempInt)), NULL);
    Camera_getSizeY(Cam, &TempInt);
    setParam( QSharedPointer<ito::ParamBase>(new ito::ParamBase("sizey",ito::ParamBase::Int,TempInt)), NULL);

    // Get Tick Frequency
    Camera_getTimestampTickFrequency(Cam, &TimestampTickFrequency);
    std::cout<<"TimestampTickFrequency: "<<TimestampTickFrequency<<"\n";

    // set default parameters
    Camera_setMulticastMode(Cam, MULTICAST_MODE_NONE);
    Camera_setStreamingPacketSize(Cam, 1024);

    // Register data callback
    std::cout << "Registering data callback ..." << std::endl;
    SVGigERet =
        StreamingChannel_create (&StreamingChannel,                                // a streaming channel handle will be returned
                             m_pVistekContainer->getCameraContainerHandle(),    // a valid camera container client handle
                             Cam,                                                // a valid camera handle
                             0,                                                    // buffer count 0 => 3 buffers
                             &DataCallback,                                        // callback function pointer where datas are delivered to
                             this);                                                // current class pointer will be passed through as context

    // Check if data callback registration was successful
    if( SVGigE_SUCCESS != SVGigERet )
    {
        return ito::RetVal(ito::retError, 1004, tr("Streaming channel creation failed.").toAscii().data());
    }
    else
    {
        std::cout << "done!\n" << std::endl;
    }

    // Create event
    if(SVGigE_SUCCESS != Stream_createEvent(StreamingChannel,&EventID,100))
        return ito::RetVal(ito::retError, 1004, tr("Event creation failed.").toAscii().data());
    // Register messages
    Stream_addMessageType(StreamingChannel,EventID,SVGigE_SIGNAL_START_OF_TRANSFER);
    Stream_addMessageType(StreamingChannel,EventID,SVGigE_SIGNAL_FRAME_COMPLETED);
    Stream_addMessageType(StreamingChannel,EventID,SVGigE_SIGNAL_CAMERA_TRIGGER_VIOLATION);
    Stream_addMessageType(StreamingChannel,EventID,SVGigE_SIGNAL_FRAME_ABANDONED);
    Stream_addMessageType(StreamingChannel,EventID,SVGigE_SIGNAL_END_OF_EXPOSURE);
    // Register message callback
    std::cout << "Registering message callback ..." << std::endl;
    SVGigERet = Stream_registerEventCallback(StreamingChannel, EventID, &MessageCallback, this);
    // Check if message callback registration was successful
    if( SVGigE_SUCCESS != SVGigERet )
    {
        return ito::RetVal(ito::retError, 1004, tr("Message callback registration failed.").toAscii().data());
    }
    else
    {
        std::cout << "done!\n" << std::endl;
    }

    FrameCompletedFlag = false;

    Camera_setAcquisitionMode(Cam, ACQUISITION_MODE_SOFTWARE_TRIGGER);
    
    return ito::retOk;
}

SVGigE_RETURN __stdcall DataCallback(Image_handle Data, void* Context)
{
    // Check for valid context
    Vistek *ClassContext = reinterpret_cast<Vistek*>(Context);
    if(ClassContext == NULL)
        return SVGigE_ERROR;

    ClassContext->ImageData = (void *) Image_getDataPointer(Data);
    ClassContext->PixelType = Image_getPixelType(Data);
    ClassContext->SizeX = Image_getSizeX(Data);
    ClassContext->SizeY = Image_getSizeY(Data);
    ClassContext->DataID = Image_getImageID(Data);
    ClassContext->Timestamp = Image_getTimestamp(Data);
    ClassContext->TransferTime = Image_getTransferTime(Data);
    ClassContext->PacketCount = Image_getPacketCount(Data);
    Image_release(Data);
    ClassContext->FrameCompletedFlag = true;

    return SVGigE_SUCCESS;
};

SVGigE_RETURN __stdcall MessageCallback(Event_handle EventID, void* Context)
{
    // Check for valid context
    Vistek *ClassContext = reinterpret_cast<Vistek*>(Context);
    if(ClassContext == NULL)
        return SVGigE_ERROR;

    Message_handle MessageID;
    SVGigE_SIGNAL_TYPE MessageType;

    // Get the signal
    if( SVGigE_SUCCESS != Stream_getMessage(ClassContext->StreamingChannel,EventID,&MessageID,&MessageType) )
        return SVGigE_ERROR;

    // Get Message timestamp
    if(MessageType == SVGigE_SIGNAL_START_OF_TRANSFER)
    {
        ClassContext->MessageTimestampLastStartOfTransfer = ClassContext->MessageTimestampStartOfTransfer;
        Stream_getMessageTimestamp(ClassContext->StreamingChannel,EventID,MessageID,&ClassContext->MessageTimestampStartOfTransfer);
        ClassContext->updateTimestamp();
    }
    if(MessageType == SVGigE_SIGNAL_FRAME_COMPLETED)
    {
        Stream_getMessageTimestamp(ClassContext->StreamingChannel,EventID,MessageID,&ClassContext->MessageTimestampFrameCompleted);
    }
    if(MessageType == SVGigE_SIGNAL_CAMERA_TRIGGER_VIOLATION)
    {
        // Count trigger violations in current streaming channel
        ClassContext->TriggerViolationCount++;
    }
    if(MessageType == SVGigE_SIGNAL_FRAME_ABANDONED)
    {
        // Not implemented yet
    }
    if(MessageType == SVGigE_SIGNAL_END_OF_EXPOSURE)
    {
        Stream_getMessageTimestamp(ClassContext->StreamingChannel,EventID,MessageID, &ClassContext->MessageTimestampEndOfExposure);
    }

    // Release message
    Stream_releaseMessage(ClassContext->StreamingChannel,EventID,MessageID);

    return SVGigE_SUCCESS;
}