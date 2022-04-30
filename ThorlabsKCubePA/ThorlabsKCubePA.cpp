/* ********************************************************************
Plugin "ThorlabsKCubePA" for itom software
Copyright (C) 2018, TRUMPF Laser- & Systemtechnik GmbH, Ditzingen

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

#include "thorlabsKCubePA.h"

#define NOMINMAX

#include "pluginVersion.h"
#include "gitVersion.h"

#include <qstring.h>
#include <qstringlist.h>
#include <QtCore/QtPlugin>
#include <qtimer.h>
#include <qwaitcondition.h>
#include <qelapsedtimer.h>
#include <qdatetime.h>

#include "Thorlabs.MotionControl.KCube.PositionAligner.h"

#include <qdebug.h>

QList<QByteArray> ThorlabsKCubePA::openedDevices = QList<QByteArray>();


//----------------------------------------------------------------------------------------------------------------------------------
/*!
    \detail creates new instance of ThorlabsKCubePAInterface and returns the instance-pointer
    \param [in,out] addInInst is a double pointer of type ito::AddInBase. The newly created ThorlabsKCubePAInterface-instance is stored in *addInInst
    \return retOk
    \sa ThorlabsKCubePA
*/
ito::RetVal ThorlabsKCubePAInterface::getAddInInst(ito::AddInBase **addInInst)
{
    ito::RetVal retValue;
    NEW_PLUGININSTANCE(ThorlabsKCubePA)
    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
/*!
    \detail Closes an instance of of ThorlabsKCubePAInterface. This instance is given by parameter addInInst.
    \param [in] double pointer to the instance which should be deleted.
    \return retOk
    \sa ThorlabsKCubePA
*/
ito::RetVal ThorlabsKCubePAInterface::closeThisInst(ito::AddInBase **addInInst)
{

    ito::RetVal retValue;
    REMOVE_PLUGININSTANCE(ThorlabsKCubePA)
    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
/*! \detail defines the plugin type (typeActuator) and sets the plugins object name. Theplugin is initialized (e.g. by a Python call) 
    with mandatory or optional parameters (m_initParamsMand and m_initParamsOpt).
*/
ThorlabsKCubePAInterface::ThorlabsKCubePAInterface()
{
    m_type = ito::typeADDA | ito::typeDataIO;

    setObjectName("ThorlabsKCubePA");

    m_description = QObject::tr("ThorlabsKCubePA");
    m_detaildescription = QObject::tr("ThorlabsKCubePA");

    m_author = "M. Gronle, TRUMPF Laser- & Systemtechnik GmbH";
    m_version = (PLUGIN_VERSION_MAJOR << 16) + (PLUGIN_VERSION_MINOR << 8) + PLUGIN_VERSION_PATCH;
    m_minItomVer = MINVERSION;
    m_maxItomVer = MAXVERSION;
    m_license = QObject::tr("licensed under LGPL");
    m_aboutThis = QObject::tr(GITVERSION);    
    
    m_initParamsOpt.append(ito::Param("serialNo", ito::ParamBase::String, "", tr("Serial number of the device to be loaded, if empty, the first device that can be opened will be opened").toLatin1().data()));
    m_initParamsOpt.append(ito::Param("includeSumSignal", ito::ParamBase::Int, 0, 1, 0, tr("If 1, a 3x1 dataObject with (dx, dy and sum signal) is returned via getVal / copyVal. Else (0, default), only the dx, dy signals are returned within a 2x1 dataObject.").toLatin1().data()));
}

//----------------------------------------------------------------------------------------------------------------------------------
// this makro registers the class ThorlabsKCubePAInterface with the name ThorlabsKCubePAInterface as plugin for the Qt-System (see Qt-DOC)


//----------------------------------------------------------------------------------------------------------------------------------
/*! \detail defines the name and sets the plugins parameters (m_parans). The plugin is initialized (e.g. by a Python call) 
    with mandatory or optional parameters (m_initParamsMand and m_initParamsOpt) by the ThorlabsKCubePA::init. The widged window is created at this position.
*/
ThorlabsKCubePA::ThorlabsKCubePA() :
	AddInDataIO(),
	m_isgrabbing(false),
    m_includeSumSignal(false)
{
    m_params.insert("name", ito::Param("name", ito::ParamBase::String | ito::ParamBase::Readonly, "ThorlabsKCubePA", tr("Name of the plugin").toLatin1().data()));
    m_params.insert("deviceName", ito::Param("deviceName", ito::ParamBase::String | ito::ParamBase::Readonly, "", tr("Description of the device").toLatin1().data()));
    m_params.insert("serialNumber", ito::Param("serialNumber", ito::ParamBase::String | ito::ParamBase::Readonly, "", tr("Serial number of the device").toLatin1().data()));


    if (hasGuiSupport())
    {
        //now create dock widget for this plugin
		DockWidgetThorlabsKCubePA *dockWidget = new DockWidgetThorlabsKCubePA(this);
        Qt::DockWidgetAreas areas = Qt::AllDockWidgetAreas;
        QDockWidget::DockWidgetFeatures features = QDockWidget::DockWidgetClosable | QDockWidget::DockWidgetFloatable | QDockWidget::DockWidgetMovable;
        createDockWidget(QString(m_params["name"].getVal<char *>()), features, areas, dockWidget);
    }

    memset(m_serialNo, '\0', sizeof(m_serialNo));
}

//---------------------------------------------------------------------------------------------------------------------------------
const ito::RetVal ThorlabsKCubePA::showConfDialog(void)
{
	return apiShowConfigurationDialog(this, new DialogThorlabsKCubePA(this));
}

//----------------------------------------------------------------------------------------------------------------------------------
/*! \detail Init method which is called by the addInManager after the initiation of a new instance of DummyGrabber.
    This init method gets the mandatory and optional parameter vectors of type tParam and must copy these given parameters to the
    internal m_params-vector. Notice that this method is called after that this instance has been moved to its own (non-gui) thread.

    \param [in] paramsMand is a pointer to the vector of mandatory tParams.
    \param [in] paramsOpt is a pointer to the vector of optional tParams.
    \param [in] waitCond is the semaphore (default: NULL), which is released if this method has been terminated
    \todo check if (*paramsMand)[0] is a serial port
    \return retOk
    */
ito::RetVal ThorlabsKCubePA::init(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> * paramsOpt, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retval = ito::retOk;

    QByteArray serial = paramsOpt->at(0).getVal<char*>();
    m_includeSumSignal = paramsOpt->at(1).getVal<int>() > 0;

    retval += checkError(TLI_BuildDeviceList(), "build device list");
    QByteArray existingSerialNumbers("", 256);
    TLI_DeviceInfo deviceInfo;

    if (!retval.containsError())
    {
        short numDevices = TLI_GetDeviceListSize();

        if (numDevices == 0)
        {
            retval += ito::RetVal(ito::retError, 0, "no Thorlabs devices detected.");
        }
        else
        {
			int allowedTypeIDs[] = { 69 }; //69: KCube Position Aligner
			int numAllowedTypeIDs = 1;
            retval += checkError(TLI_GetDeviceListByTypesExt(existingSerialNumbers.data(), existingSerialNumbers.size(), allowedTypeIDs, numAllowedTypeIDs), "get device list");
        }
    }

    if (!retval.containsError())
    {
        int idx = existingSerialNumbers.indexOf('\0');
        if (idx > 0)
        {
            existingSerialNumbers = existingSerialNumbers.left(idx);
        }

        QList<QByteArray> serialNumbers = existingSerialNumbers.split(',');

        if (serial == "")
        {
            bool found = false;
            for (int i = 0; i < serialNumbers.size(); ++i)
            {
                if (serialNumbers[i] != "" && !openedDevices.contains(serialNumbers[i]))
                {
                    serial = serialNumbers[i];
                    found = true;
                    break;
                }
            }
            
            if (!found)
            {
                retval += ito::RetVal(ito::retError, 0, "no free Thorlabs devices found.");
            }
        }
        else
        {
            bool found = false;
            foreach(const QByteArray &s, serialNumbers)
            {
                if (s == serial && s != "")
                {
                    found = true;
                    break;
                }
            }

            if (!found)
            {
                retval += ito::RetVal::format(ito::retError, 0, "no device with the serial number '%s' found", serial.data());
            }
            else if (openedDevices.contains(serial))
            {
                retval += ito::RetVal::format(ito::retError, 0, "Thorlabs device with the serial number '%s' already in use.", serial.data());
            }
        }
    }

    if (!retval.containsError())
    {
        if (TLI_GetDeviceInfo(serial.data(), &deviceInfo) == 0)
        {
            retval += ito::RetVal(ito::retError, 0, "error obtaining device information.");
        }
        else
        {
            m_params["serialNumber"].setVal<char*>(serial.data()); // bug: deviceInfo.serialNo is sometimes wrong if more than one of the same devices are connected
            setIdentifier(QLatin1String(serial.data())); // bug: deviceInfo.serialNo is sometimes wrong if more than one of the same devices are connected
            m_params["deviceName"].setVal<char*>(deviceInfo.description);
        }
    }

    if (!retval.containsError())
    {
        if (deviceInfo.isKnownType)
        {
            memcpy(m_serialNo, serial.data(), std::min((size_t)serial.size(), sizeof(m_serialNo)));
            retval += checkError(QD_Open(m_serialNo), "open device");

            if (!retval.containsError())
            {
                m_opened = true;
                openedDevices.append(m_serialNo);
            }
        }
        else
        {
            retval += ito::RetVal(ito::retError, 0, "the type of the device is not among the supported devices (Long Travel Stage, Labjack, Cage Rotator)");
        }
    }

    if (!retval.containsError())
    {
        if (!QD_LoadSettings(m_serialNo))
        {
            retval += ito::RetVal(ito::retWarning, 0, "settings of device could not be loaded.");
        }

		QD_OperatingMode operatingMode = QD_GetOperatingMode(m_serialNo);
		retval += checkError(QD_SetOperatingMode(m_serialNo, QD_OperatingMode::QD_OpenLoop, true), "Set operating mode to open loop");
		QD_Position demandPosition;
		demandPosition.x = 0;
		demandPosition.y = 0;
		retval += checkError(QD_SetPosition(m_serialNo, &demandPosition), "Set demanded position to (0,0)");

		if (!QD_StartPolling(m_serialNo, 50))
		{
			retval += ito::RetVal(ito::retError, 0, "error starting position and status polling.");
		}
    }

	if (!retval.containsError())
	{
		checkData();
	}
    
    if (waitCond)
    {
        waitCond->returnValue = retval;
        waitCond->release();
    }

    setInitialized(true); //init method has been finished (independent on retval)
    return retval;
}



//----------------------------------------------------------------------------------------------------------------------------------
/*! \detail close method which is called before that this instance is deleted by the ThorlabsKCubePAInterface
    notice that this method is called in the actual thread of this instance.

    \param [in] waitCond is the semaphore (default: NULL), which is released if this method has been terminated
    \return retOk
    \sa ItomSharedSemaphore
*/
ito::RetVal ThorlabsKCubePA::close(ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retval = ito::retOk;

    if (m_opened)
    {
        QD_StopPolling(m_serialNo);
        Sleep(300);

        QD_Close(m_serialNo);
        m_opened = false;
        openedDevices.removeOne(m_serialNo);
    }


    if (waitCond)
    {
        waitCond->returnValue = retval;
        waitCond->release();
    }

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
/*!
    \detail It is used to set the parameter of type int/double with key "name" stored in m_params and the corresponding member variabels. 
            This function is defined by the actuator class and overwritten at this position.

    \param[in] *name        Name of parameter
    \param[out] val            New parameter value as double
    \param[in/out] *waitCond    Waitcondition between this thread and the callers tread

    \return retOk
*/
ito::RetVal ThorlabsKCubePA::getParam(QSharedPointer<ito::Param> val, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);
    QString key;
    bool hasIndex = false;
    int index;
    QString suffix;
    QMap<QString,ito::Param>::iterator it;

    retValue += apiParseParamName(val->getName(), key, hasIndex, index, suffix);

    if (retValue == ito::retOk)
    {
        retValue += apiGetParamFromMapByKey(m_params, key, it, false);
    }

    if (!retValue.containsError())
    {
        *val = apiGetParam(*it, hasIndex, index, retValue);
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
    \detail It is used to set the parameter of type char* with key "name" stored in m_params and the corresponding member variabels. 
            This function is defined by the actuator class and overwritten at this position.
            If the "ctrl-type" is set, ThorlabsKCubePA::SMCSwitchType is executed.

    \param[in] *name        Name of parameter
    \param[in] *val            String with parameter
    \param[in] len            Length of the string
    \param[in/out] *waitCond    Waitcondition between this thread and the callers tread

    \return retOk
*/
ito::RetVal ThorlabsKCubePA::setParam(QSharedPointer<ito::ParamBase> val, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);
    QString key;
    bool hasIndex;
    int index = 0;
    QString suffix;
    QMap<QString, ito::Param>::iterator it;
    QVector<QPair<int, QByteArray> > lastitError;

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
        retValue += apiValidateAndCastParam(*it, *val, false, true, true);
    }

    if (!retValue.containsError())
    {
        retValue += it->copyValueFrom(&(*val));
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
ito::RetVal ThorlabsKCubePA::startDevice(ItomSharedSemaphore *waitCond)
{
	ito::RetVal retval = ito::retOk;

	if (waitCond)
	{
		waitCond->returnValue = retval;
		waitCond->release();
		waitCond->deleteSemaphore();
	}

	return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal ThorlabsKCubePA::stopDevice(ItomSharedSemaphore *waitCond)
{
	ito::RetVal retval = ito::retOk;

	if (waitCond)
	{
		waitCond->returnValue = retval;
		waitCond->release();
		waitCond->deleteSemaphore();
	}
	return retval;
}


//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal ThorlabsKCubePA::acquire(const int trigger, ItomSharedSemaphore *waitCond)
{
	ito::RetVal retval(ito::retOk);
	m_isgrabbing = false;

	QD_Readings readPosition = { 0,0 };
	retval += checkError(QD_GetReading(m_serialNo, &readPosition), "get current position");

	if (!retval.containsError())
	{
		m_isgrabbing = true;
		ito::float64 *ptr = m_data.rowPtr<ito::float64>(0, 0);
		ptr[0] = (10.0 * (ito::int16)readPosition.posDifference.x) / std::numeric_limits<ito::int16>::max();
		ptr[1] = (10.0 * (ito::int16)readPosition.posDifference.y) / std::numeric_limits<ito::int16>::max();

        if (m_includeSumSignal)
        {
            ptr[2] = (10.0 * (ito::uint16)readPosition.sum) / std::numeric_limits<ito::uint16>::max();
        }

	}

	if (waitCond)
	{
		waitCond->returnValue = retval;
		waitCond->release();
		waitCond->deleteSemaphore();
	}

	return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal ThorlabsKCubePA::retrieveData(ito::DataObject *externalDataObject)
{
	//todo: this is just a basic example for getting the buffered image to m_data or the externally given data object
	//enhance it and adjust it for your needs
	ito::RetVal retValue(ito::retOk);

	if (m_isgrabbing == false)
	{
		retValue += ito::RetVal(ito::retWarning, 0, tr("Tried to get picture without triggering exposure").toLatin1().data());
		return retValue;
	}

	m_isgrabbing = false;

	if (externalDataObject == NULL)
	{
		return retValue;
	}
	else
	{
		retValue += checkData(externalDataObject);

		if (!retValue.containsError())
		{
			retValue += m_data.deepCopyPartial(*externalDataObject);
		}
	}

	return retValue;
}

//-------------------------------------------------------------------------------------------------
ito::RetVal ThorlabsKCubePA::checkData(ito::DataObject *externalDataObject)
{
	int futureHeight = m_includeSumSignal ? 3 : 2; //x and y position
	int futureWidth = 1;
	int futureType = ito::tFloat64;
	ito::RetVal retval;

	if (externalDataObject == NULL)
	{
		if (m_data.getDims() < 2 || m_data.getSize(0) != (unsigned int)futureHeight || m_data.getSize(1) != (unsigned int)futureWidth || m_data.getType() != futureType)
		{
			m_data = ito::DataObject(futureHeight, futureWidth, futureType);
		}
	}
	else
	{
		int dims = externalDataObject->getDims();
		if (externalDataObject->getDims() == 0)
		{
			*externalDataObject = ito::DataObject(futureHeight, futureWidth, futureType);
		}
		else if (externalDataObject->calcNumMats() != 1)
		{
			return ito::RetVal(ito::retError, 0, tr("Error during check data, external dataObject invalid. Object has more than 1 plane or zero planes. It must be of right size and type or an uninitialized image.").toLatin1().data());
		}
		else if (externalDataObject->getSize(dims - 2) != (unsigned int)futureHeight || externalDataObject->getSize(dims - 1) != (unsigned int)futureWidth || externalDataObject->getType() != futureType)
		{
			return ito::RetVal(ito::retError, 0, tr("Error during check data, external dataObject invalid. Object must be of right size and type or a uninitialized image.").toLatin1().data());
		}
	}

	return retval;
}


//----------------------------------------------------------------------------------------------------------------------------------
//! Returns the grabbed camera frame as reference.
/*!
This method returns a reference to the recently acquired data. Therefore this data size must fit to the data structure of the
DataObject.

This method returns a reference to the internal dataObject m_data of the camera where the currently acquired image data is copied to (either
in the acquire method or in retrieve data). Please remember, that the reference may directly change if a new image is acquired.

\param [in,out] vpdObj is the pointer to a given dataObject (this pointer should be cast to ito::DataObject*). After the call, the dataObject is a reference to the internal m_data dataObject of the camera.
\param [in] waitCond is the semaphore (default: NULL), which is released if this method has been terminated
\return retOk if everything is ok, retError is camera has not been started or no image has been acquired by the method acquire.

\sa retrieveImage, copyVal
*/
ito::RetVal ThorlabsKCubePA::getVal(void *vpdObj, ItomSharedSemaphore *waitCond)
{
	ItomSharedSemaphoreLocker locker(waitCond);
	ito::RetVal retValue(ito::retOk);
	ito::DataObject *dObj = reinterpret_cast<ito::DataObject *>(vpdObj);

	//call retrieveData without argument. Retrieve data should then put the currently acquired image into the dataObject m_data of the camera.
	retValue += retrieveData();

	if (!retValue.containsError())
	{
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
ito::RetVal ThorlabsKCubePA::copyVal(void *vpdObj, ItomSharedSemaphore *waitCond)
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
		retValue += retrieveData(dObj);
	}

	if (waitCond)
	{
		waitCond->returnValue = retValue;
		waitCond->release();
	}

	return retValue;
}

//---------------------------------------------------------------------------------------------------------------------------------- 
void ThorlabsKCubePA::dockWidgetVisibilityChanged(bool visible)
{
    if (getDockWidget())
    {
        QWidget *w = getDockWidget()->widget();

        if (w)
        {
            if (visible)
            {
                connect(this, SIGNAL(parametersChanged(QMap<QString, ito::Param>)), w, SLOT(parametersChanged(QMap<QString, ito::Param>)));
                emit parametersChanged(m_params);

            }
            else
            {
                QObject::disconnect(this, SIGNAL(parametersChanged(QMap<QString, ito::Param>)), w, SLOT(parametersChanged(QMap<QString, ito::Param>)));
            }
        }
    }
}

//---------------------------------------------------------------------------------------------------------------------------------- 
ito::RetVal ThorlabsKCubePA::checkError(short value, const char* message)
{
    if (value == 0)
    {
        return ito::retOk;
    }
    else
    {
        switch (value)
        {
        case 1: return ito::RetVal::format(ito::retError, 1, "%s: The FTDI functions have not been initialized.", message);
        case 2: return ito::RetVal::format(ito::retError, 1, "%s: The Device could not be found. This can be generated if the function TLI_BuildDeviceList() has not been called.", message);
        case 3: return ito::RetVal::format(ito::retError, 1, "%s: The Device must be opened before it can be accessed. See the appropriate Open function for your device.", message);
        case 4: return ito::RetVal::format(ito::retError, 1, "%s: An I/O Error has occured in the FTDI chip.", message);
        case 5: return ito::RetVal::format(ito::retError, 1, "%s: There are Insufficient resources to run this application.", message);
        case 6: return ito::RetVal::format(ito::retError, 1, "%s: An invalid parameter has been supplied to the device.", message);
        case 7: return ito::RetVal::format(ito::retError, 1, "%s: The Device is no longer present. The device may have been disconnected since the last TLI_BuildDeviceList() call.", message);
        case 8: return ito::RetVal::format(ito::retError, 1, "%s: The device detected does not match that expected.", message);
        case 16: return ito::RetVal::format(ito::retError, 1, "%s: The library for this device could not be found.", message);
        case 17: return ito::RetVal::format(ito::retError, 1, "%s: No functions available for this device.", message);
        case 18: return ito::RetVal::format(ito::retError, 1, "%s: The function is not available for this device.", message);
        case 19: return ito::RetVal::format(ito::retError, 1, "%s: Bad function pointer detected.", message);
        case 20: return ito::RetVal::format(ito::retError, 1, "%s: The function failed to complete succesfully.", message);
        case 21: return ito::RetVal::format(ito::retError, 1, "%s: The function failed to complete succesfully.", message);
        case 32: return ito::RetVal::format(ito::retError, 1, "%s: Attempt to open a device that was already open.", message);
        case 33: return ito::RetVal::format(ito::retError, 1, "%s: The device has stopped responding.", message);
        case 34: return ito::RetVal::format(ito::retError, 1, "%s: This function has not been implemented.", message);
        case 35: return ito::RetVal::format(ito::retError, 1, "%s: The device has reported a fault.", message);
        case 36: return ito::RetVal::format(ito::retError, 1, "%s: The function could not be completed at this time.", message);
        case 40: return ito::RetVal::format(ito::retError, 1, "%s: The function could not be completed because the device is disconnected.", message);
        case 41: return ito::RetVal::format(ito::retError, 1, "%s: The firmware has thrown an error", message);
        case 42: return ito::RetVal::format(ito::retError, 1, "%s: The device has failed to initialize", message);
        case 43: return ito::RetVal::format(ito::retError, 1, "%s: An Invalid channel address was supplied", message);
        case 37: return ito::RetVal::format(ito::retError, 1, "%s: The device cannot perform this function until it has been Homed.", message);
        case 38: return ito::RetVal::format(ito::retError, 1, "%s: The function cannot be performed as it would result in an illegal position.", message);
        case 39: return ito::RetVal::format(ito::retError, 1, "%s: An invalid velocity parameter was supplied. The velocity must be greater than zero.", message);
        case 44: return ito::RetVal::format(ito::retError, 1, "%s: This device does not support Homing. Check the Limit switch parameters are correct.", message);
        case 45: return ito::RetVal::format(ito::retError, 1, "%s: An invalid jog mode was supplied for the jog function.", message);
        case 46: return ito::RetVal::format(ito::retError, 1, "%s: There is no Motor Parameters available to convert Real World Units.", message);
        case 47: return ito::RetVal::format(ito::retError, 1, "%s: Command temporarily unavailable, Device may be busy.", message);
        default:
            return ito::RetVal::format(
                ito::retError, value, "%s: unknown error %i.", message, value);
        }
    }
}
