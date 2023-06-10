/* ********************************************************************
    Template for a camera / grabber plugin for the software itom

    You can use this template, use it in your plugins, modify it,
    copy it and distribute it without any license restrictions.
*********************************************************************** */

#define ITOM_IMPORT_API
#define ITOM_IMPORT_PLOTAPI

#include "newport2936.h"
#include "pluginVersion.h"
#include "gitVersion.h"

#include <qstring.h>
#include <qstringlist.h>
#include <qplugin.h>
#include <qmessagebox.h>

#include "dockWidgetNewport2936.h"
#include <NewpDll.h>

//----------------------------------------------------------------------------------------------------------------------------------
//! Constructor of Interface Class.
/*!
    \todo add necessary information about your plugin here.
*/
Newport2936Interface::Newport2936Interface()
{
    m_type = ito::typeDataIO | ito::typeADDA; //any grabber is a dataIO device AND its subtype grabber (bitmask -> therefore the OR-combination).
    setObjectName("Newport2936");

    m_description = QObject::tr("Newport2936");

    //for the docstring, please don't set any spaces at the beginning of the line.
    char docstring[] = \
"This plugin can be used to control Newport 1931-C, Newport 1936-R and Newport 2936-R devices.\
\n\
The plugin was tested with a 1931-C and 2936-R device. \n\
\n\
For compiling this plugin, you need to install the Newport USB Driver V 4.2.2";
    m_detaildescription = QObject::tr(docstring);

    m_author = PLUGIN_AUTHOR;
    m_version = PLUGIN_VERSION;
    m_minItomVer = MINVERSION;
    m_maxItomVer = MAXVERSION;
    m_license = QObject::tr("LPGL, uses Newport USB Driver software and driver (not covered by LPGL)");
    m_aboutThis = QObject::tr(GITVERSION);

    //add mandatory and optional parameters for the initialization here.
    //append them to m_initParamsMand or m_initParamsOpt.
}

//----------------------------------------------------------------------------------------------------------------------------------
//! Destructor of Interface Class.
/*!

*/
Newport2936Interface::~Newport2936Interface()
{
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal Newport2936Interface::getAddInInst(ito::AddInBase **addInInst)
{
    NEW_PLUGININSTANCE(Newport2936) //the argument of the macro is the classname of the plugin
    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal Newport2936Interface::closeThisInst(ito::AddInBase **addInInst)
{
   REMOVE_PLUGININSTANCE(Newport2936) //the argument of the macro is the classname of the plugin
   return ito::retOk;
}



//----------------------------------------------------------------------------------------------------------------------------------
//! Constructor of plugin.
/*!
    \todo add internal parameters of the plugin to the map m_params. It is allowed to append or remove entries from m_params
    in this constructor or later in the init method
*/
Newport2936::Newport2936() : AddInGrabber(), m_isgrabbing(false), m_faileIdx(0)
{
    ito::Param paramVal("name", ito::ParamBase::String | ito::ParamBase::Readonly, "Newport2936", NULL);
    m_params.insert(paramVal.getName(), paramVal);

	paramVal = ito::Param("manufacturer_name", ito::ParamBase::String | ito::ParamBase::Readonly, "", tr("Manufacturer name").toLatin1().data());
	m_params.insert(paramVal.getName(), paramVal);
	paramVal = ito::Param("model_name", ito::ParamBase::String | ito::ParamBase::Readonly, "", tr("Model name").toLatin1().data());
	m_params.insert(paramVal.getName(), paramVal);
	paramVal = ito::Param("firmware_version", ito::ParamBase::String | ito::ParamBase::Readonly, "", tr("Firmware version #").toLatin1().data());
	m_params.insert(paramVal.getName(), paramVal);
	paramVal = ito::Param("firmware_date", ito::ParamBase::String | ito::ParamBase::Readonly, "", tr("Firmware date (mm/dd/yy)").toLatin1().data());
	m_params.insert(paramVal.getName(), paramVal);
	paramVal = ito::Param("serial_number", ito::ParamBase::String | ito::ParamBase::Readonly, "", tr("Controller serial number").toLatin1().data());
	m_params.insert(paramVal.getName(), paramVal);

	paramVal = ito::Param("channels", ito::ParamBase::Int | ito::ParamBase::Readonly | ito::ParamBase::In, 1, 2, 2, tr("Number of channels (single: 1, dual: 2)").toLatin1().data());
	m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param("wavelengthA", ito::ParamBase::Int | ito::ParamBase::In, NULL, tr("Wavelength [nm] for channel A").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("attenuatorA", ito::ParamBase::Int | ito::ParamBase::In, 0, new ito::IntMeta(0, 1), tr("Attenuator enabled (1) or disabled (0) for channel A").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("offsetValueA", ito::ParamBase::Double | ito::ParamBase::In, -std::numeric_limits<ito::float64>::max(), std::numeric_limits<ito::float64>::max(), 0.0, tr("Offset for zero value channel A [W]").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("filterTypeA", ito::ParamBase::Int | ito::ParamBase::In, 0, 3, 3, tr("Filter type of channel A: (0) no filtering, (1) analog filter, (2) digital filter, (3) analog and digital filter").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("filterTypeB", ito::ParamBase::Int | ito::ParamBase::In, 0, 3, 3, tr("Filter type of channel B: (0) no filtering, (1) analog filter, (2) digital filter, (3) analog and digital filter").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);

	paramVal = ito::Param("wavelengthB", ito::ParamBase::Int | ito::ParamBase::In, NULL, tr("Wavelength [nm] for channel B").toLatin1().data());
	m_params.insert(paramVal.getName(), paramVal);
	paramVal = ito::Param("attenuatorB", ito::ParamBase::Int | ito::ParamBase::In, 0, new ito::IntMeta(0, 1), tr("Attenuator enabled (1) or disabled (0) for channel B").toLatin1().data());
	m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("offsetValueB", ito::ParamBase::Double | ito::ParamBase::In, -std::numeric_limits<ito::float64>::max(), std::numeric_limits<ito::float64>::max(), 0.0, tr("Offset for zero value channel 2 [W]").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);


    paramVal = ito::Param("bpp", ito::ParamBase::Int | ito::ParamBase::In, 8, 8, 8, tr("bpp").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    //paramVal = ito::Param("integrationTime", ito::ParamBase::Double | ito::ParamBase::In, 0.0, 1.0, 0.01, tr("Integrationtime of CCD [0..1] (no unit)").toLatin1().data());
    //m_params.insert(paramVal.getName(), paramVal);

    QVector<ito::Param> pMand;
    QVector<ito::Param> pOpt;
    QVector<ito::Param> pOut;
    pMand.append(ito::Param("channel", ito::ParamBase::Int | ito::ParamBase::In, 1, 2, 1, tr("Channel").toLatin1().data()));
    registerExecFunc("zero_device", pMand, pOpt, pOut, tr("Function to set the zero value of the device to the current Value").toLatin1().data());
    pMand.clear();
    pMand.append(ito::Param("value", ito::ParamBase::Double | ito::ParamBase::In, NULL, tr("Zero Value of Device in Watts").toLatin1().data()));
    pMand.append(ito::Param("channel", ito::ParamBase::Int | ito::ParamBase::In, 1, 2, 1, tr("Channel").toLatin1().data()));
    registerExecFunc("zero_device_to", pMand, pOpt, pOut, tr("Zero device to a specific value").toLatin1().data());

    //the following lines create and register the plugin's dock widget. Delete these lines if the plugin does not have a dock widget.
    DockWidgetNewport2936 *dw = new DockWidgetNewport2936(this);

    Qt::DockWidgetAreas areas = Qt::AllDockWidgetAreas;
    QDockWidget::DockWidgetFeatures features = QDockWidget::DockWidgetClosable | QDockWidget::DockWidgetFloatable | QDockWidget::DockWidgetMovable;
    createDockWidget(QString(m_params["name"].getVal<char *>()), features, areas, dw);
}

//----------------------------------------------------------------------------------------------------------------------------------
Newport2936::~Newport2936()
{
}
//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal Newport2936::charToInt(const char* str, int &val)
{
    QLocale locale("C");
    bool ok = true;
    val = locale.toInt(str,&ok);
    if (!ok)
    {
        return ito::RetVal(ito::retError, 0, tr("could not convert char* to int").toLatin1().data());
    }
    return ito::retOk;
}

ito::RetVal Newport2936::charToDouble(const char* str, double &val)
{
    bool ok = true;
    QLocale locale("C");
    val = locale.toDouble(str,&ok);
    if (!ok)
    {
        return ito::RetVal(ito::retError, 0, tr("could not convert char* to int").toLatin1().data());
    }
    return ito::retOk;


}
//----------------------------------------------------------------------------------------------------------------------------------
//! initialization of plugin
/*!
    \sa close
*/
ito::RetVal Newport2936::init(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);

    char strBuf[1024];
    long deviceFound = newp_usb_init_system();



	if (deviceFound != 0) {

		retValue += ito::RetVal(ito::retError, 0, tr("No devices found.").toLatin1().data());
	}

	if (!retValue.containsError())
	{

		newp_usb_get_device_info(strBuf);

		char szDelimiters[] = ",;";
		char* pToken = strtok(strBuf, szDelimiters);
		int cnt = 0;


		// get device ID and its name from info buffer
		while (pToken != NULL)
		{

			cnt++;

			if (cnt % 2 == 1) {

                retValue += charToInt(pToken, devID);

			}
			else
			{

				QString devName = pToken;

				char* pTokenNew = strtok(pToken, " ");

				m_params["manufacturer_name"].setVal<char*>(pTokenNew);
				pTokenNew = strtok(NULL, " ");
				m_params["model_name"].setVal<char*>(pTokenNew);
				pTokenNew = strtok(NULL, " ");
				m_params["firmware_version"].setVal<char*>(pTokenNew);
				pTokenNew = strtok(NULL, " ");
				m_params["firmware_date"].setVal<char*>(pTokenNew);
				pTokenNew = strtok(NULL, " \r");
				m_params["serial_number"].setVal<char*>(pTokenNew);
                m_identifier = QString("%1 (SN:%2)").arg(m_params["model_name"].getVal<char*>()).arg(pTokenNew);
                setIdentifier(m_identifier);
			}

			pToken = strtok(NULL, szDelimiters);
		}

	}

	// Check number of channels
	if (!retValue.containsError())
	{
		retValue += sendCommand(devID, "PM:CHAN 2");
		char rBuffer[64];
		retValue += sendCommand(devID, "PM:ERRors?");
		retValue += readResponse(devID, rBuffer,64);
        int channel = 0;
        retValue += charToInt(rBuffer, channel);
		if (!channel)
		{
			retValue += sendCommand(devID, "PM:CHAN 1");
			retValue += m_params["channels"].setVal<int>(2);


			retValue += checkData();
		}
		else
		{
			retValue += sendCommand(devID, "PM:CHAN 1");
			retValue += m_params["channels"].setVal<int>(1);
            m_params["wavelengthB"].setFlags(ito::ParamBase::Readonly);
            m_params["filterTypeB"].setFlags(ito::ParamBase::Readonly);
            m_params["attenuatorB"].setFlags(ito::ParamBase::Readonly);
            m_params["offsetValueB"].setFlags(ito::ParamBase::Readonly);

			retValue += checkData();
		}

	}

	if (!retValue.containsError())
	{
		retValue += synchronizeParams(bAll);
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
/*!
    \sa init
*/
ito::RetVal Newport2936::close(ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);

	newp_usb_uninit_system();


	newp_usb_uninit_system();

    if (waitCond)
    {
        waitCond->returnValue = retValue;
        waitCond->release();
    }

    return retValue;
}
//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal Newport2936::execFunc(const QString funcName, QSharedPointer<QVector<ito::ParamBase> >paramsMand, QSharedPointer<QVector<ito::ParamBase> > paramsOpt, QSharedPointer<QVector<ito::ParamBase> > paramsOut, ItomSharedSemaphore *waitCond)
{
    ito::RetVal retval(ito::retOk);
    if (funcName == "zero_device")
    {
        retval += zeroDevice(paramsMand->at(0).getVal<int>());
    }
    else if (funcName == "zero_device_to")
    {
        retval += zeroDeviceTo(paramsMand->at(0).getVal<double>(), paramsMand->at(1).getVal<int>());

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
ito::RetVal Newport2936::getParam(QSharedPointer<ito::Param> val, ItomSharedSemaphore *waitCond)
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
        if (key == "attenuatorA" || key == "attenuatorB")
        {
            retValue += synchronizeParams(bAttenuator);
        }
        if (key == "wavelengthA" || key == "wavelengthB")
        {
            retValue += synchronizeParams(bWavelength);
        }
        if (key == "offsetValueA" || key == "offsetValueB")
        {
            retValue += synchronizeParams(bPowerOffset);
        }
        if (key == "filterTypeA" || key == "filterTypeB")
        {
            retValue += synchronizeParams(bFilterType);
        }

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
ito::RetVal Newport2936::setParam(QSharedPointer<ito::ParamBase> val, ItomSharedSemaphore *waitCond)
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
        if (key == "wavelengthA")
        {
            retValue += sendCommand(devID, "PM:CHAN 1");
            int lambda = val->getVal<int>();
            char rBuffer[64];
            retValue += sendCommand(devID, QString("PM:Lambda %1").arg(lambda).toLatin1().data());
            if (!retValue.containsError())
            {
                retValue += synchronizeParams(bWavelength);
            }
        }
        else if (key == "wavelengthB")
        {
            retValue += sendCommand(devID, "PM:CHAN 2");
            int lambda = val->getVal<int>();
            char rBuffer[64];
            retValue += sendCommand(devID, QString("PM:Lambda %1").arg(lambda).toLatin1().data());
            if (!retValue.containsError())
            {
                retValue += synchronizeParams(bWavelength);
            }
        }
        else if (key == "attenuatorA")
        {
            retValue += sendCommand(devID, "PM:CHAN 1");
            int value = val->getVal<int>();
            retValue += sendCommand(devID, QString("PM:ATT %1").arg(value).toLatin1().data());
            if (!retValue.containsError())
            {
                retValue += synchronizeParams(bAttenuator);
            }
        }
        else if (key == "attenuatorB")
        {
            retValue += sendCommand(devID, "PM:CHAN 2");
            int value = val->getVal<int>();
            retValue += sendCommand(devID, QString("PM:ATT %1").arg(value).toLatin1().data());
            if (!retValue.containsError())
            {
                retValue += synchronizeParams(bAttenuator);
            }
        }
        else if (key == "offsetValueA")
        {
            retValue += sendCommand(devID, "PM:CHAN 1");
            double value = val->getVal<double>();
            retValue += sendCommand(devID, QString("PM:ZEROVALue %1").arg(value).toLatin1().data());
            if (!retValue.containsError())
            {
                retValue += synchronizeParams(bPowerOffset);
            }

        }
        else if (key == "offsetValueB")
        {
            retValue += sendCommand(devID, "PM:CHAN 2");
            double value = val->getVal<double>();
            retValue += sendCommand(devID, QString("PM:ZEROVALue %1").arg(value).toLatin1().data());
            if (!retValue.containsError())
            {
                retValue += synchronizeParams(bPowerOffset);
            }
        }
        else if (key == "filterTypeA")
        {
            retValue += sendCommand(devID, "PM:CHAN 1");
            int value = val->getVal<int>();
            retValue += sendCommand(devID, QString("PM:FILTer %1").arg(value).toLatin1().data());
            retValue += synchronizeParams(bFilterType);
        }
        else if (key == "filterTypeB")
        {
            retValue += sendCommand(devID, "PM:CHAN 2");
            int value = val->getVal<int>();
            retValue += sendCommand(devID, QString("PM:FILTer %1").arg(value).toLatin1().data());
            retValue += synchronizeParams(bFilterType);
        }
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
ito::RetVal Newport2936::startDevice(ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
	ito::RetVal retValue = ito::RetVal(ito::retWarning, 0, tr("startDevice not necessary.").toLatin1().data());

    if (waitCond)
    {
        waitCond->returnValue = retValue;
        waitCond->release();
    }
    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal Newport2936::stopDevice(ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
	ito::RetVal retValue = ito::RetVal(ito::retWarning, 0, tr("stopDevice not necessary.").toLatin1().data());

    if (waitCond)
    {
        waitCond->returnValue = retValue;
        waitCond->release();
    }
    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal Newport2936::acquire(const int trigger, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);

	char rBuffer[128];
	retValue += sendCommand(devID, "PM:PWS?");
	retValue += readResponse(devID, rBuffer,128);

	char* pToken = strtok(rBuffer, " \r");
	int cnt = 0;

	if(!retValue.containsError())
	{
		while (pToken != NULL)
		{

			if (cnt % 2 == 0)
			{
                if (m_params["channels"].getVal<int>() >= cnt)
                {
                    double val;
                    retValue += charToDouble(pToken, val);
                    if (!retValue.containsError())
                    {
                        m_data.at<ito::float64>(0, cnt / 2) = val;
                    }
                }
			}

			pToken = strtok(NULL, " \r");
			cnt++;
		}
	}

	if (!retValue.containsError())
	{
		m_isgrabbing = true;
		m_data.setValueUnit("W");
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
ito::RetVal Newport2936::retrieveData(ito::DataObject *externalDataObject)
{
    //todo: this is just a basic example for getting the buffered image to m_data or the externally given data object
    //enhance it and adjust it for your needs
    ito::RetVal retValue(ito::retOk);

	if (m_isgrabbing == false)
	{
		retValue += ito::RetVal(ito::retWarning, 0, tr("Tried to get data without acquisition.").toLatin1().data());
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

ito::RetVal Newport2936::sendCommand(long DeviceID, const char* commandBuffer)
{
	ito::RetVal retValue(ito::retOk);
	char szBuffer[64];
	unsigned long commandLength = (unsigned long)strlen(commandBuffer) + 1;

	strncpy(szBuffer, commandBuffer, commandLength);

	try
	{
		newp_usb_send_ascii(DeviceID, szBuffer, commandLength);
	}
	catch (...)
	{
		retValue += ito::RetVal(ito::retError, 0, tr("Failed to send command.").toLatin1().data());
	}

	return retValue;
}

ito::RetVal Newport2936::readResponse(long DeviceID, char* responseBuffer, const unsigned long& length)
{
	ito::RetVal retValue(ito::retOk);
	unsigned long lBytesRead = 0;
	//unsigned long responseLength = (unsigned long)strlen(responseBuffer) + 1;

	try
	{
		newp_usb_get_ascii( DeviceID, responseBuffer, length, &lBytesRead);
		int i = lBytesRead;

		while (--i >= 0)
		{
			if (responseBuffer[i] == '\n')
			{
				if (responseBuffer[i - 1] == '\r')
				{
					i--;
				}

				lBytesRead = i;

				responseBuffer[i] = '\0';
				break;

			}

		}

	}
	catch (...)
	{
		retValue += ito::RetVal(ito::retError, 0, tr("Failed to read response.").toLatin1().data());
	}

	return retValue;
}

ito::RetVal Newport2936::synchronizeParams(int what)
{
    ito::RetVal retValue(ito::retOk);
    char rBuffer[64];
    if (m_faileIdx < 2)
    {

        ito::RetVal retValue(ito::retOk);
        char rBuffer[64];



        if (what & bWavelength)
        {
            int lambda;
            int lambdaMax;
            int lambdaMin;

            retValue += sendCommand(devID, "PM:CHAN 1");
            retValue += sendCommand(devID, "PM:Lambda?");
            retValue += readResponse(devID, rBuffer, 64);
            retValue += charToInt(rBuffer, lambda);
            memset(rBuffer, -52, sizeof(rBuffer)); // clear buffer, but can't be set to 0 for some reason...

            retValue += sendCommand(devID, "PM:MAX:Lambda?");
            retValue += readResponse(devID, rBuffer, 64);
            retValue += charToInt(rBuffer, lambdaMax);
            memset(rBuffer, -52, sizeof(rBuffer));

            retValue += sendCommand(devID, "PM:MIN:Lambda?");
            retValue += readResponse(devID, rBuffer, 64);
            retValue += charToInt(rBuffer, lambdaMin);
            memset(rBuffer, -52, sizeof(rBuffer));

            if (!retValue.containsError())
            {
                m_params["wavelengthA"].setMeta(new ito::IntMeta(lambdaMin, lambdaMax, 1), true);
                retValue += m_params["wavelengthA"].setVal<int>(lambda);
            }

            if (m_params["channels"].getVal<int>() == 2)
            {
                retValue += sendCommand(devID, "PM:CHAN 2");

                retValue += sendCommand(devID, "PM:Lambda?");
                retValue += readResponse(devID, rBuffer, 64);
                retValue += charToInt(rBuffer, lambda);
                memset(rBuffer, -52, sizeof(rBuffer));

                retValue += sendCommand(devID, "PM:MAX:Lambda?");
                retValue += readResponse(devID, rBuffer, 64);
                retValue += charToInt(rBuffer, lambdaMax);
                memset(rBuffer, -52, sizeof(rBuffer));

                retValue += sendCommand(devID, "PM:MIN:Lambda?");
                retValue += readResponse(devID, rBuffer, 64);
                retValue += charToInt(rBuffer, lambdaMin);

                if (!retValue.containsError())
                {
                    if (!retValue.containsError())
                    {
                        m_params["wavelengthB"].setMeta(new ito::IntMeta(lambdaMin, lambdaMax, 1), true);
                        retValue += m_params["wavelengthB"].setVal<int>(lambda);
                    }
                }
            }
            else
            {
                retValue += m_params["wavelengthB"].setVal<int>(-1);
                m_params["wavelengthB"].setMeta(new ito::IntMeta(-1, -1, 1), true);
            }

        }
        if (what & bAttenuator)
        {
            memset(rBuffer, -52, sizeof(rBuffer));
            int state;
            retValue += sendCommand(devID, "PM:CHAN 1");
            retValue += sendCommand(devID, "PM:ATT?");
            retValue += readResponse(devID, rBuffer, 64);
            retValue += charToInt(rBuffer, state);
            if (!retValue.containsError())
            {
                retValue += m_params["attenuatorA"].setVal<int>(state);
            }
            if (m_params["channels"].getVal<int>() == 2)
            {
                memset(rBuffer, -52, sizeof(rBuffer));
                retValue += sendCommand(devID, "PM:CHAN 2");
                retValue += sendCommand(devID, "PM:ATT?");
                retValue += readResponse(devID, rBuffer, 64);
                retValue += charToInt(rBuffer, state);
                if (!retValue.containsError())
                {
                    retValue += m_params["attenuatorB"].setVal<int>(state);
                }
            }
        }
        if (what & bPowerOffset)
        {
            memset(rBuffer, -52, sizeof(rBuffer));
            retValue += sendCommand(devID, "PM:CHAN 1");
            retValue += sendCommand(devID, "PM:ZEROVALue?");
            retValue += readResponse(devID, rBuffer, 64);
            if (!retValue.containsError())
            {
                double val;
                retValue += charToDouble(rBuffer,val);
                if (!retValue.containsError())
                {
                    retValue += m_params["offsetValueA"].setVal<double>(val);
                }
            }
            if (m_params["channels"].getVal<int>() == 2)
            {
                memset(rBuffer, -52, sizeof(rBuffer));
                retValue += sendCommand(devID, "PM:CHAN 2");
                retValue += sendCommand(devID, "PM:ZEROVALue?");
                retValue += readResponse(devID, rBuffer, 64);
                if (!retValue.containsError())
                {
                    double val;
                    retValue += charToDouble(rBuffer,val);
                    if (!retValue.containsError())
                    {
                        retValue += m_params["offsetValueB"].setVal<double>(val);
                    }
                }
            }
        }
        if (what & bFilterType)
        {
            int val;
            memset(rBuffer, -52, sizeof(rBuffer));
            retValue += sendCommand(devID, "PM:CHAN 1");
            retValue += sendCommand(devID, "PM:FILTer?");
            retValue += readResponse(devID, rBuffer, 64);
            if (!retValue.containsError());
            {
                retValue += charToInt(rBuffer,val);
                if (retValue.containsError())
                {
                    retValue += m_params["filterTypeA"].setVal<int>(val);
                }
            }
            if (m_params["channels"].getVal<int>() == 2)
            {
                memset(rBuffer, -52, sizeof(rBuffer));
                retValue += sendCommand(devID, "PM:CHAN 2");
                retValue += sendCommand(devID, "PM:FILTer?");
                retValue += readResponse(devID, rBuffer, 64);
                if (!retValue.containsError());
                {
                    retValue = charToInt(rBuffer, val);
                    if (retValue.containsError())
                    {
                        retValue += m_params["filterTypeB"].setVal<int>(val);
                    }
                }
            }
        }
    }
    if (retValue.containsError() && m_faileIdx++ < 1)
    {
        retValue = ito::RetVal(ito::retWarning, 0, tr("Failed to synchronize parameters with device. Retry to sync params.").toLatin1().data());
        retValue +=Newport2936::synchronizeParams(what);
        m_faileIdx = 0;

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
ito::RetVal Newport2936::checkData(ito::DataObject *externalDataObject)
{

	int futureHeight = 1;
	int futureWidth;
	int futureType = ito::tFloat64;
	ito::RetVal retval;

	if (m_params["channels"].getVal<int>() == 2)
	{
		futureWidth = 2;
	}
	else
	{
		futureWidth = 1;
	}


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
    This method returns a reference to the recently acquired image. Therefore this camera size must fit to the data structure of the
    DataObject.

    This method returns a reference to the internal dataObject m_data of the camera where the currently acquired image data is copied to (either
    in the acquire method or in retrieve data). Please remember, that the reference may directly change if a new image is acquired.

    \param [in,out] vpdObj is the pointer to a given dataObject (this pointer should be cast to ito::DataObject*). After the call, the dataObject is a reference to the internal m_data dataObject of the camera.
    \param [in] waitCond is the semaphore (default: NULL), which is released if this method has been terminated
    \return retOk if everything is ok, retError is camera has not been started or no image has been acquired by the method acquire.

    \sa retrieveImage, copyVal
*/
ito::RetVal Newport2936::getVal(void *vpdObj, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);
    ito::DataObject *dObj = reinterpret_cast<ito::DataObject *>(vpdObj);

    //call retrieveData without argument. Retrieve data should then put the currently acquired image into the dataObject m_data of the camera.
    retValue += retrieveData(dObj);

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
ito::RetVal Newport2936::copyVal(void *vpdObj, ItomSharedSemaphore *waitCond)
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

    if (waitCond)
    {
        waitCond->returnValue = retValue;
        waitCond->release();
    }

    return retValue;
}
//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal Newport2936::zeroDevice(int channel, ItomSharedSemaphore * waitCond)
{
    ito::RetVal retValue(ito::retOk);
    char rBuffer[64];
    retValue += sendCommand(devID, QString("PM:CHAN %1").arg(channel).toLatin1().data());
    if (!retValue.containsError())
    {
        retValue += sendCommand(devID, "PM:ZEROSTOre");
        retValue += readResponse(devID, rBuffer,64);
    }
    if (waitCond)
    {
        waitCond->returnValue = retValue;
        waitCond->release();
    }
    return retValue;
}
//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal Newport2936::zeroDeviceTo(double val, int channel,ItomSharedSemaphore * waitCond)
{
    ito::RetVal retValue(ito::retOk);
    char rBuffer[64];
    retValue += sendCommand(devID, QString("PM:CHAN %1").arg(channel).toLatin1().data());
    if (!retValue.containsError())
    {
        retValue += sendCommand(devID, QString("PM:ZEROVALue %1").arg(val).toLatin1().data());
        retValue += readResponse(devID, rBuffer,64);
    }

    return retValue;
}
//----------------------------------------------------------------------------------------------------------------------------------
//! method called to aqcuire and get a image
/*!
This method is invoked from the dock widget to get a value in the autograbbing mode.
\param [in,out] QSharedPointer to return the measured value.
\param [in] waitCond is the semaphore (default: NULL), which is released if this method has been terminated
\return retOk if everything is ok, retError if the occurred any error .
*/
ito::RetVal Newport2936::acquireAutograbbing(QSharedPointer<QList<double> > value, ItomSharedSemaphore *waitCond)
{
    ito::RetVal retValue(ito::retOk);

    char rBuffer[128];
    retValue += sendCommand(devID, "PM:PWS?");
    retValue += readResponse(devID, rBuffer,128);

    char* pToken = strtok(rBuffer, " \r");
    int cnt = 0;

    if (!retValue.containsError())
    {
        while (pToken != NULL)
        {

            if (cnt % 2 == 0)
            {
                double val;
                retValue += charToDouble(pToken, val);
                    if (!retValue.containsError())
                    {
                        value->append(val);
                    }
            }

            pToken = strtok(NULL, " \r");
            cnt++;
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
//! slot called if the dock widget of the plugin becomes (in)visible
/*!
    Overwrite this method if the plugin has a dock widget. If so, you can connect the parametersChanged signal of the plugin
    with the dock widget once its becomes visible such that no resources are used if the dock widget is not visible. Right after
    a re-connection emit parametersChanged(m_params) in order to send the current status of all plugin parameters to the dock widget.
*/
void Newport2936::dockWidgetVisibilityChanged(bool visible)
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
const ito::RetVal Newport2936::showConfDialog(void)
{
    return apiShowConfigurationDialog(this, new DialogNewport2936(this));
}
