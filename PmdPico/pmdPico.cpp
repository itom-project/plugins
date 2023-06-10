/* ********************************************************************
Plugin "PmdPico" for itom software
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

#include "pmdPico.h"
#include "pluginVersion.h"
#include "gitVersion.h"

#include <qstring.h>
#include <qstringlist.h>
#include <qplugin.h>
#include <qmessagebox.h>
#include <iostream>

#include "dockWidgetPmdPico.h"
#include "dialogPmdPico.h"


static char InitList[5] = { 0, 0, 0, 0, 0 };  /*!<A map with successfull initialized boards (max = 5) */
static char Initnum = 0;    /*!< Number of successfull initialized cameras */
//----------------------------------------------------------------------------------------------------------------------------------
//! Constructor of Interface Class.
/*!
    \todo add necessary information about your plugin here.
*/
PmdPicoInterface::PmdPicoInterface()
{
    m_type = ito::typeDataIO | ito::typeGrabber; //any grabber is a dataIO device AND its subtype grabber (bitmask -> therefore the OR-combination).
    setObjectName("PmdPico");

    m_description = QObject::tr("PmdPico Grabber");

    //for the docstring, please don't set any spaces at the beginning of the line.
    char docstring[] = \
"This plugin can be used to control any PMD pico device. The plugin was tested with a pico flexx device. Other devices have not been tested so far. \n\
If you start the plugin without further parameters (camera Number= 0), the first connected device is opened. \n\
This Plugin gives you only control to the level 1 functions of the SDK. \n\
For compiling this plugin, you need to install the royale software, shipped with the camera. \n\
Then set the CMake variable PmdPico_ROYALE_DIR to the folder including the bin folder (e.g. C:/Program Files/royale/3.8.0.35).\n\
After that make shure that the bin folder of the royale software is added to your path variables.";

    m_detaildescription = QObject::tr(docstring);

    m_author = PLUGIN_AUTHOR;
    m_version = PLUGIN_VERSION;
    m_minItomVer = MINVERSION;
    m_maxItomVer = MAXVERSION;
    m_license = QObject::tr("LPGL, uses royale software and driver (not covered by LPGL)");
    m_aboutThis = QObject::tr(GITVERSION);

    ito::Param paramVal = ito::Param("camera Number", ito::ParamBase::Int | ito::ParamBase::In, 0, 254, 0, "The index of the addressed camera starting with 0");
    m_initParamsOpt.append(paramVal);
    //add mandatory and optional parameters for the initialization here.
    //append them to m_initParamsMand or m_initParamsOpt.
}

//----------------------------------------------------------------------------------------------------------------------------------
//! Destructor of Interface Class.
/*!

*/
PmdPicoInterface::~PmdPicoInterface()
{
    m_initParamsMand.clear();
    m_initParamsOpt.clear();
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal PmdPicoInterface::getAddInInst(ito::AddInBase **addInInst)
{
    NEW_PLUGININSTANCE(PmdPico) //the argument of the macro is the classname of the plugin
    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal PmdPicoInterface::closeThisInst(ito::AddInBase **addInInst)
{
   REMOVE_PLUGININSTANCE(PmdPico) //the argument of the macro is the classname of the plugin
   return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
ExposureListener::ExposureListener() : integrationTime(0)
{
}
//----------------------------------------------------------------------------------------------------------------------------------
void ExposureListener::onNewExposure(const uint32_t exposureTime, const royale::StreamId streamId)
{
    integrationTime = exposureTime;
    emit integrationTimeChanged(integrationTime);
}
//----------------------------------------------------------------------------------------------------------------------------------
ito::int32 ExposureListener::getIntegrationTime() const
{
    return integrationTime;
}
//----------------------------------------------------------------------------------------------------------------------------------
DataListener::DataListener(PmdPico* inst) : m_singleShot(false), m_host(inst), m_updateParams(false), m_mutex(NULL), m_takeNext(false), m_data(NULL)
{

}
//----------------------------------------------------------------------------------------------------------------------------------
//! listener for new data
/*!
if new datas are available this function is called. This function will stop the acquisition if m_singleShot is true.
The incomming data are copied to m_data.

\param data points to DepthData
\sa captureSingleImage
*/
void DataListener::onNewData(const royale::DepthData *data)
{
    if (m_takeNext)
    {
        m_takeNext = false;
        if (m_singleShot)
        {
            m_host->setCapturingState(false);
            m_singleShot = false;
        }
        m_data = data;
        m_host->copyDataToBuffer();

        if (m_updateParams)
        {
            m_host->updateParamsFromImage();
            m_updateParams = false;
        }
        if (m_mutex)
        {
            m_mutex->unlock();
            m_mutex = NULL;
        }
    }

}
//----------------------------------------------------------------------------------------------------------------------------------
//! starts the acquisition of a single picture
/*!
since the sdk has no function to capture a single image this function will do it.
Not all parameters can be obtained by the sdk, therefore they have to be read out of the image.


\param updateParams update the params of the plugin if true
\return RetVal
\sa onNewData
*/
ito::RetVal DataListener::captureSingleImage(bool updateParams)
{
    ito::RetVal retValue(ito::retOk);
    bool capture;
    if (m_host)
    {
        retValue += m_host->getCapturingState(capture);
        if (!retValue.containsError())
        {
            m_singleShot = true;
            m_takeNext = true;
            m_updateParams = updateParams;
            retValue += m_host->setCapturingState(true);

            return retValue;
        }
    }
    else
    {
        retValue += ito::RetVal(ito::retError,0, QString("plugin instance not available").toLatin1().data());
    }
    return retValue;
}
ito::RetVal DataListener::freeRunCapture()
{
    ito::RetVal retValue(ito::retOk);
    retValue += m_host->setCapturingState(true);
    m_takeNext = true;
    if (retValue.containsError())
    {
        retValue += m_host->setCapturingState(false);
    }
    return retValue;


}
void DataListener::setLockMutex(QMutex* mutex)
{
    if (!m_mutex)//if there is a mutex release the current one
    {
        m_mutex = mutex;
        m_mutex->lock();
    }
    else
    {
        m_mutex->unlock();
        m_mutex = mutex;
    }
}
//----------------------------------------------------------------------------------------------------------------------------------
//! Constructor of plugin.
/*!
    \todo add internal parameters of the plugin to the map m_params. It is allowed to append or remove entries from m_params
    in this constructor or later in the init method
*/
PmdPico::PmdPico() : AddInGrabber(), m_isgrabbing(false), m_exposureListener(), m_dataListener(this), m_currentBuffer(0), m_deliverState(0), m_camInit(false)
{
    ito::Param paramVal("name", ito::ParamBase::String | ito::ParamBase::Readonly, "PmdPico", NULL);
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param("x0", ito::ParamBase::Int | ito::ParamBase::Readonly, 0, 2048, 0, tr("first pixel index (x-direction)").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("y0", ito::ParamBase::Int | ito::ParamBase::Readonly, 0, 2048, 0, tr("first pixel index (y-direction)").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("x1", ito::ParamBase::Int | ito::ParamBase::Readonly, 0, 1279, 1279, tr("last pixel index (x-direction)").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("y1", ito::ParamBase::Int | ito::ParamBase::Readonly, 0, 1023, 1023, tr("last pixel index (y-direction)").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("sizex", ito::ParamBase::Int | ito::ParamBase::Readonly | ito::ParamBase::In, 1, 2048, 2048, tr("width (x-direction)").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("sizey", ito::ParamBase::Int | ito::ParamBase::Readonly | ito::ParamBase::In, 1, 2048, 2048, tr("height (y-direction)").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param("bpp", ito::ParamBase::Int | ito::ParamBase::Readonly, 16, 16, 16, tr("bpp of gray value image").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("integration_time", ito::ParamBase::Double , 0.0, 1.0, 0.0, tr("integration time of [sec]").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param("cam_number", ito::ParamBase::Int | ito::ParamBase::Readonly | ito::ParamBase::NoAutosave, 0, 4, 0, tr("index of the camera device.").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("serial_number", ito::ParamBase::String | ito::ParamBase::Readonly, "unknown", tr("serial number of device.").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("sensor_name", ito::ParamBase::String | ito::ParamBase::Readonly, "unknown", tr("sensor name of the attached camera").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("auto_exposure", ito::ParamBase::Int, 0, 1, 1, tr("indicates if the integration time is set automatically (1) or Manual (0)").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("framerate", ito::ParamBase::Int, 0, 5000, 0, tr("framerate of image acquisition (in fps). This parameter reflects the current framerate.").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("use_case", ito::ParamBase::String , "unknown", tr("current use case of camera. To get a list of all available use cases call the getUseCases exec function. Note the mixed modes are not supported yet.").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("acquisition_mode", ito::ParamBase::Int, 0, 3, 3, tr("indicates which data should be recorded. 0: depth data, 1: gray value, 2: confidence of depth, 3: all").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("data_mode", ito::ParamBase::Int, 0, 2, 0, tr("indicates whether depth data (0), gray value (1) or confidence map (2) is transfered when using copyVal, getVal or the live image").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);

    DockWidgetPmdPico *dw = new DockWidgetPmdPico(this);



    Qt::DockWidgetAreas areas = Qt::AllDockWidgetAreas;
    QDockWidget::DockWidgetFeatures features = QDockWidget::DockWidgetClosable | QDockWidget::DockWidgetFloatable | QDockWidget::DockWidgetMovable;
    createDockWidget(QString(m_params["name"].getVal<char *>()), features, areas, dw);
    QVector<ito::Param> pMand;
    pMand << ito::Param("xCoordinate", ito::ParamBase::DObjPtr | ito::ParamBase::In| ito::ParamBase::Out, NULL,tr("x-coordinate map of the same shape as the current image").toLatin1().data());
    pMand << ito::Param("yCoordinate", ito::ParamBase::DObjPtr | ito::ParamBase::In | ito::ParamBase::Out, NULL,tr("y-coordinate map of the same shape as the current image").toLatin1().data());
    QVector<ito::Param> pOpt;
    QVector<ito::Param> pOut;
    registerExecFunc("getCoordinates", pMand, pOpt, pOut, tr("returns the coordinates map of the acquired image.").toLatin1().data());
    pMand.clear();
    registerExecFunc("getUseCases", pMand, pOpt, pOut, tr("prints out the available use cases."));

}

//----------------------------------------------------------------------------------------------------------------------------------
PmdPico::~PmdPico()
{
}

//----------------------------------------------------------------------------------------------------------------------------------
//! initialization of plugin
/*!
    \sa close
*/
ito::RetVal PmdPico::init(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);
    int icam_number = (*paramsOpt)[0].getVal<int>();

    if (!retValue.containsError())
    {

        m_params["cam_number"].getVal<int>(icam_number);
        Initnum++;  // so we have a new running instance of this grabber (or not)

        if (++InitList[icam_number] > 1)    // It does not matter if the rest works or not. The close command will fix this anyway
        {
            retValue = ito::RetVal(ito::retError, 0, tr("Camera already initialized. Try another camera number.").toLatin1().data());
        }
        else
        {
            royale::CameraManager manager;
            royale::Vector<royale::String> camlist(manager.getConnectedCameraList());
            qDebug() << "Detected " << camlist.size() << " camera(s).";
            if (!camlist.empty())
            {
                m_cameraDevice = manager.createCamera(camlist[0]).release(); //the pointer is no longer guarded by a unique ptr
                if(!m_cameraDevice)
                {
                    retValue += ito::RetVal(ito::retError, 0, tr("could not open camera").toLatin1().data());
                }
                camlist.clear();
            }
            else
            {
                retValue += ito::RetVal(ito::retError, 0, tr("no device detected").toLatin1().data());
            }
            if (!retValue.containsError())
            {
                royale::CameraStatus status(m_cameraDevice->initialize());
                retValue+=getErrStr(status);
                if (!retValue.containsError())
                {
                    m_camInit=true;
                    retValue+=getErrStr(m_cameraDevice->registerExposureListener(&m_exposureListener));
                    retValue += getErrStr(m_cameraDevice->registerDataListener(&m_dataListener));
                    if (!retValue.containsError())
                    {
                        royale::String id;
                        royale::String name;
                        retValue += getErrStr(m_cameraDevice->getCameraName(name));
                        if (!retValue.containsError())
                        {
                            m_params["sensor_name"].setVal<const char*>(name.data());
                        }
                        retValue += getErrStr(m_cameraDevice->getId(id));
                        if (!retValue.containsError())
                        {
                            m_params["serial_number"].setVal<const char*>(id.data());
                            m_identifier = QString("%1 (SN:%2)").arg(name.data()).arg(id.data());
                            setIdentifier(m_identifier);
                            qDebug() << QString("%1 (SN:%2)").arg(name.data()).arg(id.data());
                            retValue += synchronizeCameraSettings();
                            if (!retValue.containsError())
                            {
                                m_cameraDevice->setExternalTrigger(false); // hardware trigger not implemented yet
                            }
                        }
                    }
                    //m_cameraDevice->registerExposureListener(m_exposureListener);

                }
            }
        }
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
        QMutex mutex;
        m_dataListener.setLockMutex(&mutex);
        retValue+=m_dataListener.captureSingleImage(true); //this is needed to get all params
        if (!retValue.containsError())
        {
            mutex.lock();
            synchronizeCameraSettings(sExposure);
            retValue += checkData();
            switchDataObj();
        }
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
ito::RetVal PmdPico::checkForCoordinatesObj(ito::DataObject* externalObj)
{
    int futureHeight = m_params["sizey"].getVal<int>();
    int futureWidth = m_params["sizex"].getVal<int>();
    int dims = externalObj->getDims();
    if (externalObj->getDims() == 0)
    {
        *externalObj = ito::DataObject(futureHeight, futureWidth, ito::tFloat32);
    }
    else if (externalObj->calcNumMats() != 1)
    {
        return ito::RetVal(ito::retError, 0, tr("Error during check data, external dataObject invalid. Object has more or less than 1 plane. It must be of right size and type or an uninitilized image.").toLatin1().data());
    }
    else {
        if (externalObj->getSize(dims - 2) != (unsigned int)futureHeight || externalObj->getSize(dims - 1) != (unsigned int)futureWidth || externalObj->getType() != ito::tFloat32)
        {
            return ito::RetVal(ito::retError, 0, tr("Error during check data, external dataObject invalid. Object must be of right size and type or a uninitilized image.").toLatin1().data());
        }
    }
    return ito::retOk;

}
ito::RetVal copyToExternalObj(ito::DataObject * externalDataObject, const ito::DataObject* data)
{
    if (data)
    {
        int planes = data->getNumPlanes();
        for (int i = 0; i < planes; ++i)
        {
            const cv::Mat* internalMat = data->getCvPlaneMat(i);
            cv::Mat* externalMat = externalDataObject->getCvPlaneMat(i);

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

    }
    else
    {
        return ito::RetVal(ito::retError, 0, QString("data buffer not initialized").toLatin1().data());
    }
    return ito::retOk;
}
//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal PmdPico::execFunc(const QString funcName, QSharedPointer<QVector<ito::ParamBase> >paramsMand, QSharedPointer<QVector<ito::ParamBase> > paramsOpt, QSharedPointer<QVector<ito::ParamBase> > paramsOut, ItomSharedSemaphore *waitCond)
{
    ito::RetVal retval(ito::retOk);
    if (funcName == "getCoordinates")
    {
        if (paramsMand->length() != 2)
        {
            retval += ito::RetVal(ito::retError, 0, tr("getCoordinates requires two dataObjects as argument.").toLatin1().data());
        }
        ito::DataObject* x(paramsMand->at(0).getVal<ito::DataObject*>());
        ito::DataObject* y(paramsMand->at(0).getVal<ito::DataObject*>());
        if (!(x && y))
        {
            ito::RetVal(ito::retError, 0, tr("the incomming dataObject is not available").toLatin1().data());
        }
        if (!retval.containsError())
        {
            retval += checkForCoordinatesObj(paramsMand->at(0).getVal<ito::DataObject*>());
            retval += checkForCoordinatesObj(paramsMand->at(1).getVal<ito::DataObject*>());
            if (!retval.containsError())
            {
                if (grabberStartedCount() <= 0)
                {
                    retval += ito::RetVal(ito::retWarning, 0, tr("Tried to get picture without starting device").toLatin1().data());
                }
                if (!(m_currentBuffer & cXCoordinate && m_currentBuffer & cYCoordinate))
                {
                    retval += ito::RetVal(ito::retError, 0, tr("Tried to get data which was not recorded with in the last acquire").toLatin1().data());
                }
                if (!(m_deliverState & cXCoordinate && m_deliverState & cYCoordinate))
                {
                    retval += ito::RetVal(ito::retWarning, 0, tr("Tried to get picture without triggering exposure").toLatin1().data());
                }
                if (!retval.containsError())
                {
                            retval += copyToExternalObj(paramsMand->at(0).getVal<ito::DataObject*>(), &m_dataXCoordinate);
                            if (!retval.containsError())
                            {
                                retval += copyToExternalObj(paramsMand->at(1).getVal<ito::DataObject*>(), &m_dataYCoordinate);
                            }
                }
                m_deliverState ^= cXCoordinate;
                m_deliverState ^= cYCoordinate;
            }

        }

    }
    else if(funcName == "getUseCases")
    {
        if (m_cameraDevice)
        {
            royale::Vector<royale::String> vec;
            retval += getErrStr(m_cameraDevice->getUseCases(vec));
            std::cout << "PmdPico use cases: \n" << std::endl;
            for (int i = 0; i < vec.size(); ++i)
            {
                std::cout << vec[i].data()<<"\n" << std::endl;
            }
        }
    }
    if (waitCond)
    {
        waitCond->returnValue = retval;
        waitCond->release();
        waitCond->deleteSemaphore();
        waitCond = NULL;
    }
    return retval;
}
//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal PmdPico::synchronizeCameraSettings(const int &paramMask /*=sAll*/)
{
    ito::RetVal retValue(ito::retOk);
    bool connected;
    retValue += getErrStr(m_cameraDevice->isConnected(connected));
    if (connected && !retValue.containsError())
    {
        if (paramMask & sExposure)
        {

            royale::Pair<uint32_t, uint32_t> limits;
            retValue += getErrStr(m_cameraDevice->getExposureLimits(limits));
            if (!retValue.containsError())
            {
                ito::DoubleMeta* meta = new ito::DoubleMeta (musecToSec(limits.first), musecToSec(limits.second));
                meta->setDisplayPrecision(6);
                m_params["integration_time"].setMeta(meta, true);
                m_params["integration_time"].setVal<double>(musecToSec(m_exposureListener.getIntegrationTime()));
            }
        }
        if (paramMask & sAutoExposure)
        {
            royale::ExposureMode mode;
            retValue += getErrStr(m_cameraDevice->getExposureMode(mode));
            if (!retValue.containsError())
            {
                if (mode == royale::ExposureMode::MANUAL)
                {
                    m_params["auto_exposure"].setVal<int>(0);
                }
                else
                {
                    m_params["auto_exposure"].setVal<int>(1);
                }

            }
        }
        if (paramMask & sFrameRate)
        {
            uint16_t framerate, maxFrameRate;
            retValue += getErrStr(m_cameraDevice->getMaxFrameRate(maxFrameRate));
            if (!retValue.containsError())
            {
                m_params["framerate"].setMeta(new ito::IntMeta(0, maxFrameRate), true);
            }
            retValue += getErrStr(m_cameraDevice->getFrameRate(framerate));
            if (!retValue.containsError())
            {
                m_params["framerate"].setVal<int>(framerate);
            }
        }
        if (paramMask & sRoi)
        {
            QMutex mutex;
            uint16_t maxWidth, maxHeight;
            retValue += getErrStr(m_cameraDevice->getMaxSensorWidth(maxWidth));

            if (!retValue.containsError())
            {
                m_params["sizex"].setMeta(new ito::IntMeta(1, maxWidth, 1), true);
            }
            retValue += getErrStr(m_cameraDevice->getMaxSensorHeight(maxHeight));
            if (!retValue.containsError())
            {
                m_params["sizey"].setMeta(new ito::IntMeta(1, maxHeight, 1), true);
            }
            if (!m_isgrabbing)
            {
                m_dataListener.setLockMutex(&mutex); //will be unlocked if picture is ready
                m_dataListener.captureSingleImage(true); //this will update sizex and sizey
            }
            else
            {
                retValue += ito::RetVal(ito::retError, 0, tr("failed to synchronize plugin params with the camera since the camera is grabbing").toLatin1().data());
                mutex.unlock();
            }
            mutex.lock();
        }
        if (paramMask & sUseCase)
        {
            royale::String str;
            retValue += getErrStr(m_cameraDevice->getCurrentUseCase(str));
            if (!retValue.containsError())
            {
                m_params["use_case"].setVal<const char*>(QString(str.data()).toLatin1().data());
            }
        }
    }
    return retValue;
}
//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal PmdPico::getErrStr(const royale::CameraStatus& status)
{
    if (status == royale::CameraStatus::SUCCESS)
    {
        return ito::retOk;
    }
    else{
        royale::String msg(royale::getErrorString(status));
        if (msg.empty())
        {
            return ito::RetVal(ito::retError, 0, tr("PmdPico royale error: an unknown error occured").toLatin1().data());
        }
        else
        {
            QString msg_final;

            msg_final = tr("PmdPico royale error: %1").arg(msg.data());


            return ito::RetVal(ito::retError, 0, msg_final.toLatin1().data());
        }
    }
}
//----------------------------------------------------------------------------------------------------------------------------------
//! shutdown of plugin
/*!
    \sa init
*/
ito::RetVal PmdPico::close(ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);
    int nr = m_params["cam_number"].getVal<int>();
    InitList[nr] = 0;
    Initnum--;
    if (this->m_cameraDevice == NULL)
    {
        retValue += ito::RetVal(ito::retWarning, 0, tr("Camera handle deleted before closing procedure").toLatin1().data());
    }
    else
    {
        if (m_cameraDevice && m_camInit)
        {
            bool connected=false;
            m_cameraDevice->isConnected(connected);
            if (connected)
            {
                retValue += setCapturingState(false);
                retValue += getErrStr(m_cameraDevice->unregisterDepthImageListener());
                retValue += getErrStr(m_cameraDevice->unregisterExposureListener());
                delete m_cameraDevice;
            }
        }

        m_cameraDevice = NULL;

    }
    if (waitCond)
    {
        waitCond->returnValue = retValue;
        waitCond->release();
    }

    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal PmdPico::getParam(QSharedPointer<ito::Param> val, ItomSharedSemaphore *waitCond)
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
        if (key == "integration_time")
        {
            retValue += synchronizeCameraSettings(sExposure);
        }
        if (key == "framerate")
        {
            retValue += synchronizeCameraSettings(sFrameRate);
        }
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
ito::RetVal PmdPico::setParam(QSharedPointer<ito::ParamBase> val, ItomSharedSemaphore *waitCond)
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
        if (key == "integration_time")
        {
            //check the new value and if ok, assign it to the internal parameter
            if (!m_params["auto_exposure"].getVal<int>())
            {
                retValue += it->copyValueFrom(&(*val));
                uint32_t time = secToMusec(it->getVal<double>());
                bool cap;
                m_cameraDevice->isCapturing(cap);
                if (!cap) //the camera can only set the integration time if capturing
                {
                    retValue +=setCapturingState(true);
                }
                retValue += getErrStr(m_cameraDevice->setExposureTime(time));

                m_exposureListener.onNewExposure(time,0); //set this value to the listener since a manuel set does not call the onNewExposure

                synchronizeCameraSettings(sExposure);

            }
            else
            {
                retValue += ito::RetVal(ito::retError, 0, tr("Integration time only settable if auto_exposure is set to 0").toLatin1().data());
            }
        }
        else if (key == "auto_exposure")
        {
            //check the new value and if ok, assign it to the internal parameter
            retValue += it->copyValueFrom( &(*val) );
            royale::ExposureMode mode;
            it->getVal<int>() == 1 ? mode = royale::ExposureMode::AUTOMATIC : mode = royale::ExposureMode::MANUAL;
            retValue += getErrStr(m_cameraDevice->setExposureMode(mode));
            synchronizeCameraSettings(sAutoExposure);
        }
        else if (key == "framerate")
        {
            retValue += it->copyValueFrom(&(*val));
            bool cap;
            m_cameraDevice->isCapturing(cap);
            if (!cap) //the camera can only set the framerate if capturing
            {
                retValue += setCapturingState(true);
            }
            retValue += getErrStr(m_cameraDevice->setFrameRate(it->getVal<int>()));
            synchronizeCameraSettings(sFrameRate);
        }
        else if (key == "data_mode")
        {
            retValue += it->copyValueFrom(&(*val));
            switchDataObj();
        }
        else if (key == "use_case")
        {
            royale::Vector<royale::String> vec;
            retValue += getErrStr(m_cameraDevice->getUseCases(vec));
            QString str(QString::fromLatin1(val->getVal<char*>()));
            if (str.contains("MIXED", Qt::CaseInsensitive))
            {
                retValue += ito::RetVal(ito::retError, 0, tr("mixed modes not supported yet.").toLatin1().data());
            }
            if (!retValue.containsError())
            {
                m_cameraDevice->setUseCase(str.toLatin1().data());
                synchronizeCameraSettings(sRoi | sFrameRate |sUseCase);
            }
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
ito::RetVal PmdPico::startDevice(ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);

    incGrabberStarted();


    if (waitCond)
    {
        waitCond->returnValue = retValue;
        waitCond->release();
    }
    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal PmdPico::stopDevice(ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);

    decGrabberStarted(); //decrements the counter (see startDevice)

    if (grabberStartedCount() < 0)
    {
        retValue += ito::RetVal(ito::retWarning, 0, tr("The grabber has already been stopped.").toLatin1().data());
        setGrabberStarted(0);
    }

    if (grabberStartedCount() < 0)
    {
        retValue += ito::RetVal(ito::retWarning, 0, tr("stopDevice ignored since camera was not started.").toLatin1().data());
        setGrabberStarted(0);
    }
    bool cap;
    m_cameraDevice->isCapturing(cap);
    if(cap)
    {
        m_cameraDevice->stopCapture();
    }

    if (waitCond)
    {
        waitCond->returnValue = retValue;
        waitCond->release();
    }
    return ito::retOk;
}
//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal PmdPico::acquire(const int trigger, ItomSharedSemaphore *waitCond)
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
        switch (m_params["acquisition_mode"].getVal<int>())
        {
        case 0:
            m_currentBuffer = cZValue | cXCoordinate | cYCoordinate;
            break;
        case 1:
            m_currentBuffer = cGrayImage | cXCoordinate | cYCoordinate;
            break;
        case 2:
            m_currentBuffer = cConfidenceMap | cXCoordinate | cYCoordinate;
            break;
        case 3:
            m_currentBuffer = cAll;
            break;
        }
        m_deliverState = m_currentBuffer;


            QMutex mutex;
            m_dataListener.setLockMutex(&mutex);
            retValue += m_dataListener.freeRunCapture();
            if (retValue.containsError())
            {
                m_isgrabbing = false;
            }
            mutex.lock();


    }


    if (waitCond)
    {
        waitCond->returnValue = retValue;
        waitCond->release();
    }

    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal PmdPico::retrieveData(ito::DataObject *externalDataObject)
{

    ito::RetVal retValue(ito::retOk);

    bool copyExternal = externalDataObject != NULL;
    if (grabberStartedCount() <= 0)
    {
        retValue += ito::RetVal(ito::retWarning, 0, tr("Tried to get picture without starting device").toLatin1().data());
    }

    int mode = m_params["data_mode"].getVal<int>();
    switch (mode)
    {
    case 0:
        if (!(m_currentBuffer & cZValue))
        {
            retValue += ito::RetVal(ito::retError, 0, tr("Tried to get data which was not recorded with in the last acquire").toLatin1().data());
        }
        if (!(m_deliverState & cZValue))
        {
            retValue += ito::RetVal(ito::retWarning, 0, tr("Tried to get picture without triggering exposure").toLatin1().data());
        }
        break;
    case 1:
        if (!(m_currentBuffer & cGrayImage))
        {
            retValue += ito::RetVal(ito::retError, 0, tr("Tried to get data which was not recorded with in the last acquire").toLatin1().data());
        }
        if (!(m_deliverState & cGrayImage))
        {
            retValue += ito::RetVal(ito::retWarning, 0, tr("Tried to get picture without triggering exposure").toLatin1().data());
        }
        break;
    case 2:
        if ((m_currentBuffer & cConfidenceMap))
        {
            retValue += ito::RetVal(ito::retError, 0, tr("Tried to get data which was not recorded with in the last acquire").toLatin1().data());
        }
        if (!(m_deliverState & cGrayImage))
        {
            retValue += ito::RetVal(ito::retWarning, 0, tr("Tried to get picture without triggering exposure").toLatin1().data());
        }

    }
    if (!retValue.containsError())
    {
        if (copyExternal)
        {
            //here we wait until the Event is set to signaled state
            //or the timeout runs out

            switch (mode)
            {
            case 0:
                retValue += copyToExternalObj(externalDataObject, &m_dataZValue);
                break;
            case 1:
                retValue += copyToExternalObj(externalDataObject, &m_dataGray);
                break;
            case 2:
                retValue += copyToExternalObj(externalDataObject, &m_dataConfidence);
                break;
            }
        }
    }
    if (!retValue.containsWarning()) //if there is a warning dont set the bitmask to available...
    {
        switch (mode)
        {
        case 0:
            m_deliverState ^= cZValue;
            break;
        case 1:
            m_deliverState ^= cGrayImage;
            break;
        case 2:
            m_deliverState ^= cConfidenceMap;
        }
    }
    m_isgrabbing = false;
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
ito::RetVal PmdPico::checkData(ito::DataObject *externalDataObject)
{
    int futureHeight = m_params["sizey"].getVal<int>();
    int futureWidth = m_params["sizex"].getVal<int>();
    if (externalDataObject == NULL)
    {
        switch (m_params["acquisition_mode"].getVal<int>())
        {
            case 0 :
                if (m_dataZValue.getDims() < 2 || m_dataZValue.getSize(0) != (unsigned int)futureHeight || m_dataZValue.getSize(1) != (unsigned int)futureWidth || m_dataZValue.getType() != ito::tFloat32)
                {
                    m_dataZValue = ito::DataObject(futureHeight, futureWidth, ito::tFloat32);

                }
                break;
            case 1:
                if (m_dataGray.getDims() < 2 || m_dataGray.getSize(0) != (unsigned int)futureHeight || m_dataGray.getSize(1) != (unsigned int)futureWidth || m_dataGray.getType() != ito::tUInt16)
                {
                    m_dataGray = ito::DataObject(futureHeight, futureWidth, ito::tUInt16);
                }
                break;
            case 2:
                if (m_dataConfidence.getDims() < 2 || m_dataConfidence.getSize(0) != (unsigned int)futureHeight || m_dataConfidence.getSize(1) != (unsigned int)futureWidth || m_dataConfidence.getType() != ito::tUInt8)
                {
                    m_dataConfidence = ito::DataObject(futureHeight, futureWidth, ito::tUInt8);
                }
                break;
            case 3:
                if (m_dataConfidence.getDims() < 2 || m_dataConfidence.getSize(0) != (unsigned int)futureHeight || m_dataConfidence.getSize(1) != (unsigned int)futureWidth || m_dataConfidence.getType() != ito::tUInt8)
                {
                    m_dataConfidence = ito::DataObject(futureHeight, futureWidth, ito::tUInt8);
                }
                if (m_dataGray.getDims() < 2 || m_dataGray.getSize(0) != (unsigned int)futureHeight || m_dataGray.getSize(1) != (unsigned int)futureWidth || m_dataGray.getType() != ito::tUInt16)
                {
                    m_dataGray = ito::DataObject(futureHeight, futureWidth, ito::tUInt16);
                }
                if (m_dataZValue.getDims() < 2 || m_dataZValue.getSize(0) != (unsigned int)futureHeight || m_dataZValue.getSize(1) != (unsigned int)futureWidth || m_dataZValue.getType() != ito::tFloat32)
                {
                    m_dataZValue = ito::DataObject(futureHeight, futureWidth, ito::tFloat32);
                }

         }
        // the coordiantes are always buffered
        if (m_dataXCoordinate.getDims() < 2 || m_dataXCoordinate.getSize(0) != (unsigned int)futureHeight || m_dataXCoordinate.getSize(1) != (unsigned int)futureWidth || m_dataXCoordinate.getType() != ito::tFloat32)
        {
            m_dataXCoordinate = ito::DataObject(futureHeight, futureWidth, ito::tFloat32);
        }
        if (m_dataYCoordinate.getDims() < 2 || m_dataYCoordinate.getSize(0) != (unsigned int)futureHeight || m_dataYCoordinate.getSize(1) != (unsigned int)futureWidth || m_dataYCoordinate.getType() != ito::tFloat32)
        {
            m_dataYCoordinate = ito::DataObject(futureHeight, futureWidth, ito::tFloat32);
        }
    }
    else
    {
        int dims = externalDataObject->getDims();
        if (externalDataObject->getDims() == 0)
        {
            switch (m_params["data_mode"].getVal<int>())
            {
            case 0:
                *externalDataObject = ito::DataObject(futureHeight, futureWidth, ito::tFloat32);
                break;
            case 1:
                *externalDataObject = ito::DataObject(futureHeight, futureWidth, ito::tUInt16);
                break;
            case 2:
                *externalDataObject = ito::DataObject(futureHeight, futureWidth, ito::tUInt8);
            }
        }
        else if (externalDataObject->calcNumMats() != 1)
        {
            return ito::RetVal(ito::retError, 0, tr("Error during check data, external dataObject invalid. Object has more or less than 1 plane. It must be of right size and type or an uninitilized image.").toLatin1().data());
        }
        else {
            switch (m_params["data_mode"].getVal<int>())
            {
            case 0:
                if (externalDataObject->getSize(dims - 2) != (unsigned int)futureHeight || externalDataObject->getSize(dims - 1) != (unsigned int)futureWidth || externalDataObject->getType() != ito::tFloat32)
                {
                    return ito::RetVal(ito::retError, 0, tr("Error during check data, external dataObject invalid. Object must be of right size and type or a uninitilized image.").toLatin1().data());
                }
                break;
            case 1:
                if (externalDataObject->getSize(dims - 2) != (unsigned int)futureHeight || externalDataObject->getSize(dims - 1) != (unsigned int)futureWidth || externalDataObject->getType() != ito::tUInt16)
                {
                    return ito::RetVal(ito::retError, 0, tr("Error during check data, external dataObject invalid. Object must be of right size and type or a uninitilized image.").toLatin1().data());
                }
                break;
            case 2:
                if (externalDataObject->getSize(dims - 2) != (unsigned int)futureHeight || externalDataObject->getSize(dims - 1) != (unsigned int)futureWidth || externalDataObject->getType() != ito::tUInt8)
                {
                    return ito::RetVal(ito::retError, 0, tr("Error during check data, external dataObject invalid. Object must be of right size and type or a uninitilized image.").toLatin1().data());
                }
                break;
            }
        }

    }
    return ito::retOk;
}

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
ito::RetVal PmdPico::getVal(void *vpdObj, ItomSharedSemaphore *waitCond)
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
ito::RetVal PmdPico::copyVal(void *vpdObj, ItomSharedSemaphore *waitCond)
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
void PmdPico::dockWidgetVisibilityChanged(bool visible)
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
const ito::RetVal PmdPico::showConfDialog(void)
{
    return apiShowConfigurationDialog(this, new DialogPmdPico(this));
}
//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal PmdPico::setCapturingState(const bool capture) const
{
    ito::RetVal retval(ito::retOk);
    if (m_cameraDevice)
    {
        bool cap;
        m_cameraDevice->isCapturing(cap);
        if (capture != cap)
        {
            if (capture)
            {
                retval += getErrStr(m_cameraDevice->startCapture());
            }
            else
            {
                retval += getErrStr(m_cameraDevice->stopCapture());
            }
        }
    }
    else
    {
        retval += ito::RetVal(ito::retError, 0, tr("camera handle is not available").toLatin1().data());
    }
    return retval;
}
//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal PmdPico::getCapturingState(bool &state) const
{
    ito::RetVal retval(ito::retOk);
    if (m_cameraDevice)
    {
        retval += getErrStr(m_cameraDevice->isCapturing(state));
    }
    else
    {
        retval += ito::RetVal(ito::retError, 0, tr("camera handle is not available").toLatin1().data());
    }
    return retval;
}
//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal PmdPico::updateParamsFromImage()
{
    ito::RetVal retval(ito::retOk);
    if (m_dataListener.m_data)
    {
        m_params["sizex"].setVal<int>(m_dataListener.m_data->width);
        m_params["sizey"].setVal<int>(m_dataListener.m_data->height);
    }
    return retval;
}
//----------------------------------------------------------------------------------------------------------------------------------
template<typename _Tp> void copyDataToImageBufferFunc(const royale::DepthData* src, ito::DataObject* dst, int mode, const QPair<ito::DataObject*, ito::DataObject*>* coordinates = NULL)
{
    int dims = dst->getDims();
    int numRow = dst->getSize(dims - 2);
    int numCol = dst->getSize(dims - 1);
    uint32_t nrPts = src->height * src->width;
    long num = 0;
    _Tp* rowPtr;
    cv::Mat* mat = dst->getCvPlaneMat(0);



    if (coordinates)
    {
        float* x;
        float* y;
        cv::Mat* xMat = coordinates->first->getCvPlaneMat(0);
        cv::Mat* yMat = coordinates->second->getCvPlaneMat(0);
        switch (mode)
        {
        case 0:
            if (numCol * numRow == nrPts)
            {
                int row, col;
                for (row = 0; row < numRow; ++row)
                {
                    rowPtr = (_Tp*)mat->ptr(row);
                    x = (float*)xMat->ptr(row);
                    y = (float*)yMat->ptr(row);
                    for (col = 0; col < numCol; ++col)
                    {
                        rowPtr[col] = src->points[num].z;
                        x[col] = src->points[num].x;
                        y[col] = src->points[num++].y;
                    }
                }
            }
            break;
        case 1:
            if (numCol * numRow == nrPts)
            {
                int row, col;
                for (row = 0; row < numRow; ++row)
                {
                    rowPtr = (_Tp*)mat->ptr(row);
                    x = (float*)xMat->ptr(row);
                    y = (float*)yMat->ptr(row);
                    for (col = 0; col < numCol; ++col)
                    {
                        rowPtr[col] = src->points[num].grayValue;
                        x[col] = src->points[num].x;
                        y[col] = src->points[num++].y;
                    }
                }
            }
            break;
        case 2:
            if (numCol * numRow == nrPts)
            {
                int row, col;
                for (row = 0; row < numRow; ++row)
                {
                    rowPtr = (_Tp*)mat->ptr(row);
                    x = (float*)xMat->ptr(row);
                    y = (float*)yMat->ptr(row);
                    for (col = 0; col < numCol; ++col)
                    {
                        rowPtr[col] = src->points[num].depthConfidence;
                        x[col] = src->points[num].x;
                        y[col] = src->points[num++].y;
                    }
                }
            }
            break;

        }
    }
    else
    {
        switch (mode)
        {
        case 0:
            if (numCol * numRow == nrPts)
            {
                int row, col;
                for (row = 0; row < numRow; ++row)
                {
                    rowPtr = (_Tp*)mat->ptr(row);
                    for (col = 0; col < numCol; ++col)
                    {
                        rowPtr[col] = src->points[num++].z;
                    }
                }
            }
            break;
        case 1:
            if (numCol * numRow == nrPts)
            {
                int row, col;
                for (row = 0; row < numRow; ++row)
                {
                    rowPtr = (_Tp*)mat->ptr(row);
                    for (col = 0; col < numCol; ++col)
                    {
                        rowPtr[col] = src->points[num++].grayValue;
                    }
                }
            }
            break;
        case 2:
            if (numCol * numRow == nrPts)
            {
                int row, col;
                for (row = 0; row < numRow; ++row)
                {
                    rowPtr = (_Tp*)mat->ptr(row);
                    for (col = 0; col < numCol; ++col)
                    {
                        rowPtr[col] = src->points[num++].depthConfidence;
                    }
                }
            }
            break;

        }
    }
}
//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal PmdPico::copyDataToBuffer()
{
    if (m_dataListener.m_data)
    {
        QPair<ito::DataObject*, ito::DataObject*> coord(&m_dataXCoordinate, &m_dataYCoordinate);
        switch (m_params["acquisition_mode"].getVal<int>())
        {
        case 0:
            copyDataToImageBufferFunc<ito::float32>(m_dataListener.m_data, &m_dataZValue, 0, &coord);
            break;
        case 1:
            copyDataToImageBufferFunc<ito::uint16>(m_dataListener.m_data, &m_dataGray, 1, &coord);
            break;
        case 2: //confidence
            copyDataToImageBufferFunc<ito::uint8>(m_dataListener.m_data, &m_dataConfidence, 2, &coord);
            break;
        case 3:
            copyDataToImageBufferFunc<ito::float32>(m_dataListener.m_data, &m_dataZValue, 0);
            copyDataToImageBufferFunc<ito::uint16>(m_dataListener.m_data, &m_dataGray, 1);
            copyDataToImageBufferFunc<ito::uint8>(m_dataListener.m_data, &m_dataConfidence, 2, &coord);
        }
    }
    else
    {
        return ito::RetVal(ito::retError, 0, tr("no data on device available").toLatin1().data());
    }
    return ito::retOk;
}
//----------------------------------------------------------------------------------------------------------------------------------
//! shallow copy the buffer to m_data
/*!
every time the data_mode is changed set the m_data to the corresponding buffer
*/
ito::RetVal PmdPico::switchDataObj()
{
    switch (m_params["data_mode"].getVal<int>())
    {
    case 0:
        m_data = m_dataZValue;
        break;
    case 1:
        m_data = m_dataGray;
        break;
    case 2:
        m_data = m_dataConfidence;
        break;

    }
    return ito::retOk;

}
