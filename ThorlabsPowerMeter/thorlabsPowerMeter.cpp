/* ********************************************************************
itom software
URL: http://www.uni-stuttgart.de/ito
Copyright (C) 2018, Institut fuer Technische Optik (ITO),,
Universität Stuttgart, Germany

This file is part of itom and its software development toolkit (SDK).

itom is free software; you can redistribute it and/or modify it
under the terms of the GNU Library General Public Licence as published by
the Free Software Foundation; either version 2 of the Licence, or (at
your option) any later version.

In addition, as a special exception, the Institut für Technische
Optik (ITO) gives you certain additional rights.
These rights are described in the ITO LGPL Exception version 1.0,
which can be found in the file LGPL_EXCEPTION.txt in this package.

itom is distributed in the hope that it will be useful, but
WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU Library
General Public Licence for more details.

You should have received a copy of the GNU Library General Public License
along with itom. If not, see <http://www.gnu.org/licenses/>.
*********************************************************************** */

#define ITOM_IMPORT_API
#define ITOM_IMPORT_PLOTAPI

#include "thorlabsPowerMeter.h"
#include "pluginVersion.h"
#include "gitVersion.h"



#include <qstring.h>
#include <qstringlist.h>
#include <qplugin.h>
#include <qmessagebox.h>
#include <iostream>
#include <QElapsedTimer> 

#include "dockWidgetThorlabsPowerMeter.h"
#if defined(USE_API_1_02) 
    #include <PM100D.h>
#define PM(name) PM100D_##name
#else
    #include <TLPM.h>
    #define PM(name) TLPM_##name
#endif

//----------------------------------------------------------------------------------------------------------------------------------
//! Constructor of Interface Class.
/*!
    \todo add necessary information about your plugin here.
*/
ThorlabsPowerMeterInterface::ThorlabsPowerMeterInterface()
{
    m_type = ito::typeDataIO | ito::typeADDA;
    setObjectName("ThorlabsPowerMeter");

    m_description = QObject::tr("ThorlabsPowerMeter");

    //for the docstring, please don't set any spaces at the beginning of the line.
    char docstring[] = \
"This plugin can be used to control any Thorlabs PM100x, PM200 or PM400 Power Console with a connected photodiode power sensors. Other sensors like thermal or pyroelectric sensors \
are currently not supported by this plugin.\n\
If you start the plugin without further parameters (device=''), the first connected device is opened. \n\
Set device = '<scan>' in order to get a printed list of detected devices. Use the device string or the desired \n\
device in order to open this specific one. \n\
\n\
The plugin was tested with a PM100A console and a standard photodiode power sensor. \n\
\n\
This plugin gives full control over the connected device. For high attenuation there is a known issue when setting the power range via the slider widget (this belongs to the great range of possible values) in the config dialog or the dock widget.\
The power range is at any time fully accessible over the setParam command.\n\
\n\
For compiling this plugin, you need to install the Thorlabs PM100x Instrument Driver or the Optical Power Meter Utility (including the driver) , shipped with the spectrometer. \n\
Then set the CMake variable Thorlabs_IVI_VISA_INCLUDE_DIR to the include directory (e.g. C:/Program Files/IVI Foundation/VISA/Win64/Include)";
    m_detaildescription = QObject::tr(docstring);

    m_author = "Robin Hahn, ITO, University Stuttgart";
    m_version = (PLUGIN_VERSION_MAJOR << 16) + (PLUGIN_VERSION_MINOR << 8) + PLUGIN_VERSION_PATCH;
    m_minItomVer = MINVERSION;
    m_maxItomVer = MAXVERSION;
    m_license = QObject::tr("LPGL, uses Thorlabs CCS VISA Instrument Driver (LGPL licensed)");
    m_aboutThis = QObject::tr(GITVERSION); 

    //add mandatory and optional parameters for the initialization here.
    //append them to m_initParamsMand or m_initParamsOpt.
    m_initParamsOpt.append(ito::Param("device", ito::ParamBase::String, "", tr("device name that should be opened, an empty string opens the first device that is found (default). Pass '<scan>' for displaying all detected devices.").toLatin1().data()));


}

//----------------------------------------------------------------------------------------------------------------------------------
//! Destructor of Interface Class.
/*!
    
*/
ThorlabsPowerMeterInterface::~ThorlabsPowerMeterInterface()
{
}


//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal ThorlabsPowerMeterInterface::getAddInInst(ito::AddInBase **addInInst)
{
    NEW_PLUGININSTANCE(ThorlabsPowerMeter) //the argument of the macro is the classname of the plugin
    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal ThorlabsPowerMeterInterface::closeThisInst(ito::AddInBase **addInInst)
{
   REMOVE_PLUGININSTANCE(ThorlabsPowerMeter) //the argument of the macro is the classname of the plugin
   return ito::retOk;
}




//----------------------------------------------------------------------------------------------------------------------------------
//! Constructor of plugin.
/*!
    \todo add internal parameters of the plugin to the map m_params. It is allowed to append or remove entries from m_params
    in this constructor or later in the init method
    */
    ThorlabsPowerMeter::ThorlabsPowerMeter() : AddInDataIO(), m_isgrabbing(false),
    m_instrument(VI_NULL)
{
    ito::Param paramVal("name", ito::ParamBase::String | ito::ParamBase::Readonly, "ThorlabsPowerMeter", tr("name of the device").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param("manufacturer_name", ito::ParamBase::String | ito::ParamBase::Readonly, "", tr("manufacturer name").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("device_name", ito::ParamBase::String | ito::ParamBase::Readonly, "", tr("device name").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("serial_number", ito::ParamBase::String | ito::ParamBase::Readonly, "", tr("serial number").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("firmware_revision", ito::ParamBase::String | ito::ParamBase::Readonly, "", tr("firmware revision").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("calibration_message", ito::ParamBase::String | ito::ParamBase::Readonly, "", tr("calibration message").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param("wavelength", ito::ParamBase::Double | ito::ParamBase::In, NULL, tr("wavelength [nm]").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("attenuation", ito::ParamBase::Double | ito::ParamBase::In, NULL, tr("attenuation [db]. -1 if not supported by device.").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("dark_offset", ito::ParamBase::Double | ito::ParamBase::Readonly, NULL, tr("dark offset, read-out from the device [unit unknown]").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    

    paramVal = ito::Param("line_frequency", ito::ParamBase::Int | ito::ParamBase::In, 50 , new ito::IntMeta(50, 60, 10), tr(" line frequency of 50Hz or 60Hz. -1 if not supported by device.").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("power_range", ito::ParamBase::Double | ito::ParamBase::In, NULL, tr("power range [W]; will be set to the next bigger possible value").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param("auto_range", ito::ParamBase::Int | ito::ParamBase::In, 0, new ito::IntMeta(0, 1), tr(" shows if the auto power range is wether on (1) or off(2) ").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param("measurement_mode", ito::ParamBase::String | ito::ParamBase::In, NULL , tr("Get / set the current measurement mode (values 'absolute' or 'relative')").toLatin1().data());
	ito::StringMeta measurement_mode_meta(ito::StringMeta::String);
	measurement_mode_meta.addItem("absolute");
	measurement_mode_meta.addItem("relative");
	paramVal.setMeta(&measurement_mode_meta, false);
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param("reference_power", ito::ParamBase::Double | ito::ParamBase::In, NULL, tr("reference power for relative measurements").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param("average_number", ito::ParamBase::Int | ito::ParamBase::In, 1, new ito::IntMeta(1, std::numeric_limits<ito::int32>::max()), tr("defines the number of measurements to be averaged").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param("bandwidth", ito::ParamBase::Int | ito::ParamBase::In, 0, new ito::IntMeta(0, 1), tr("defines if the input filter state is whether High (0) or Low (1). -1 if not supported by device.").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);

    //register exec functions
    QVector<ito::Param> pMand;
    QVector<ito::Param> pOpt;
    QVector<ito::Param> pOut;
    registerExecFunc("zero_device", pMand, pOpt, pOut, tr("function to set the zero value of the device").toLatin1().data());


	if (hasGuiSupport())
	{
		//the following lines create and register the plugin's dock widget. Delete these lines if the plugin does not have a dock widget.
		DockWidgetThorlabsPowerMeter *dw = new DockWidgetThorlabsPowerMeter(this);

		Qt::DockWidgetAreas areas = Qt::AllDockWidgetAreas;
		QDockWidget::DockWidgetFeatures features = QDockWidget::DockWidgetClosable | QDockWidget::DockWidgetFloatable | QDockWidget::DockWidgetMovable;
		createDockWidget(QString(m_params["name"].getVal<char *>()), features, areas, dw);
	}
}

//----------------------------------------------------------------------------------------------------------------------------------
ThorlabsPowerMeter::~ThorlabsPowerMeter()
{
}

//----------------------------------------------------------------------------------------------------------------------------------
//! initialization of plugin
/*!
    \sa close
*/
ito::RetVal ThorlabsPowerMeter::init(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retval(ito::retOk);

    m_isgrabbing = false;
    QString deviceName = paramsOpt->at(0).getVal<char*>();
    QList<QByteArray> foundDevices;
    QList<QString> deviceInfo;
    ViSession	defaultRM = VI_NULL;
    ViSession  resMgr = VI_NULL;      //resource manager
    ViUInt32   count = 0;            //counts found devices
    ViStatus status;

#ifdef USE_API_1_02 //Thorlabs Power Meter (1.02)
    ViChar     rscStr[VI_FIND_BUFLEN]; // resource string
    ViFindList findList;

    status = viOpenDefaultRM(&resMgr);
    if (status != VI_SUCCESS && status != VI_WARN_CONFIG_NLOADED)
    {
        retval += checkError(status);

    }
    retval += checkError(viFindRsrc(resMgr, PMxxx_FIND_PATTERN, &findList, &count, rscStr));
    if (retval == ito::retOk)
        foundDevices.append(rscStr);
    for (ViUInt32 c = 1; c < count; ++c)
    {
        retval += checkError(viFindNext(findList, rscStr));
        if (retval == ito::retOk) foundDevices.append(rscStr);
    }
    viClose(findList);
#else //Thorlabs Optical Power Meter (1.1)
    ViBoolean available;
    ViChar name[TLPM_BUFFER_SIZE], sernr[TLPM_BUFFER_SIZE];
    ViChar rsrcDescr[TLPM_BUFFER_SIZE];
    retval += checkError(PM(findRsrc)(0, &count));
    if (!retval.containsError())
    {
        for (int i = 0; i < count; ++i)
        {
            retval += checkError(PM(getRsrcInfo)(i, 0, name, sernr, VI_NULL, &available));
            retval += checkError(PM(getRsrcName)(i, 0, rsrcDescr));
            if (!retval.containsError())
            {
                foundDevices.append(rsrcDescr);
                deviceInfo.append(QString("%1 (%2): S/N%3\t%4\tadress: %5\n").arg(i + 1).arg((available) ? "FREE" : "LOCK").arg(sernr).arg(name).arg(rsrcDescr));
            }

        }
    }


#endif
    if (!retval.containsError())
    {
        if (deviceName == "<scan>")
        {
            std::cout << "Thorlabs power meter devices \n" << std::endl;

            for (ViUInt32 i = 0; i < std::min<int>((int)count, foundDevices.size()); ++i)
            {
#if defined(USE_API_3_02)
                std::cout << "Dev. " << i << ": " << foundDevices[i].data() << std::endl;
#else
                std::cout << deviceInfo[i].toLatin1().data() << std::endl;
#endif
            }



            retval += ito::RetVal(ito::retError, 0, tr("The initialization is terminated since only a list of found devices has been requested ('<scan>')").toLatin1().data());
        }
        else if (deviceName == "")
        {
            if (foundDevices.size() == 0)
            {
                retval += ito::RetVal(ito::retError, 0, tr("no devices found").toLatin1().data());
            }
            else
            {
                deviceName = foundDevices[0];
            }
        }
        else
        {
            bool gotIt(false);

            foreach(const QByteArray &string, foundDevices)
            {
                if (QString::compare(deviceName, string, Qt::CaseInsensitive) == 0)
                {
                    deviceName = string;
                    gotIt = true;
                    break;
                }
            }
            if (!gotIt)
            {
                retval += ito::RetVal::format(ito::retError, 0, tr("Device %s could not be found").toLatin1().data(), deviceName.toLatin1().data());
            }
        }


        if (!retval.containsError())
        {
            //status = viOpen(resMgr, deviceName.toLatin1().data(), VI_EXCLUSIVE_LOCK, 2000, &m_instrument);

            //try to open device
            status = PM(init)(deviceName.toLatin1().data(), VI_OFF, VI_OFF, &m_instrument);
#if defined(USE_API_3_02)
            if (status != VI_SUCCESS && status != VI_WARN_CONFIG_NLOADED)
            {
                retval += checkError(status);
            }
#else
            if (status != VI_SUCCESS)
            {
                retval += checkError(status);
            }
#endif
        }

        ViChar name[256];
        status = PM(getRsrcName)(m_instrument, 0, name);
        if (!retval.containsError())
        {
            //check if a photo diode is connected
            ViChar name[256];
            ViChar snr[256];
            ViChar message[256];
            ViInt16 pType;
            ViInt16 pStype;
            ViInt16 pFlags;

            retval += checkError(PM(getSensorInfo)(m_instrument, name, snr, message, &pType, &pStype, &pFlags));
            if (retval.containsError())
            {
                switch (pType)
                {
                case SENSOR_TYPE_PD_SINGLE:
                    break;
                case SENSOR_TYPE_THERMO:
                    retval += ito::RetVal(ito::retError, 0, tr("This plugin supports only Photodiode Power Sensor but a Thermal Sensor was detected.").toLatin1().data());
                    break;
                case SENSOR_TYPE_PYRO:
                    retval += ito::RetVal(ito::retError, 0, tr("This plugin supports only Photodiode Power Sensor but a Pyroelectric Sensors was detected.").toLatin1().data());
                    break;
                case SENSOR_TYPE_NONE:
                    retval += ito::RetVal(ito::retError, 0, tr("There is no detector connected to the device.").toLatin1().data());
                }
            }
        }
        if (!retval.containsError())
        {
            ViChar manufacturer[256];
            ViChar device[256];
            ViChar serial[256];
            ViChar firmware[256];
            ViChar message[256];
            retval += checkError(PM(identificationQuery)(m_instrument, manufacturer, device, serial, firmware));

            if (!retval.containsError())
            {
                setIdentifier(QString("%1 (%2)").arg(device).arg(manufacturer));
                m_params["device_name"].setVal<char*>(device);
                m_params["manufacturer_name"].setVal<char*>(manufacturer);
                m_params["serial_number"].setVal<char*>(serial);
                m_params["firmware_revision"].setVal<char*>(firmware);


                retval += checkError(PM(getCalibrationMsg)(m_instrument, message));
                if (!retval.containsError())
                {
                    m_params["calibration_message"].setVal<char*>(message);
                }


            }
        }
        if (!retval.containsError())
        {
            m_data = ito::DataObject(1, 1, ito::tFloat64);
        }
        if (!retval.containsError())
        {
            //get all changing parameters of the plugin
            retval += synchronizeParams(bAll);
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



    if (!retval.containsError())
    {
        emit parametersChanged(m_params);
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
//! shutdown of plugin
/*!
    \sa init
*/
ito::RetVal ThorlabsPowerMeter::close(ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);
    
    if (m_instrument)
    {
        PM(close)(m_instrument);
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
ito::RetVal ThorlabsPowerMeter::getParam(QSharedPointer<ito::Param> val, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue;
    QString key;
    bool hasIndex = false;
    int index;
    QString suffix;
    QMap<QString,ito::Param>::iterator it;

    retValue += synchronizeParams(bPowerRange); // make shure that power_range is up to date... this is important since the power_range is continously changed if auto_range is true

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
ito::RetVal ThorlabsPowerMeter::setParam(QSharedPointer<ito::ParamBase> val, ItomSharedSemaphore *waitCond)
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
        if (key == "wavelength")
        {
            ViReal64 wavelength = val->getVal<double>();
            retValue += checkError(PM(setWavelength)(m_instrument, wavelength));
            if (!retValue.containsError())
            {
                retValue += synchronizeParams(bWavelength);
            }
        }
        else if (key == "attenuation")
        {
            ViReal64 atten = val->getVal<double>();
            retValue += checkError(PM(setAttenuation)(m_instrument, atten));
            if (!retValue.containsError())
            {
                retValue += synchronizeParams(bAttenuation | bPowerRange); //update the power range since the attenuation influences the  power_range boundaries
            }

        }
        else if (key == "line_frequency")
        {
            ViInt16 freq(val->getVal<int>());
            retValue += checkError(PM(setLineFrequency)(m_instrument, freq));
            if (!retValue.containsError())
            {
                retValue += synchronizeParams(bLineFrequency);
            }
        }
        else if (key == "power_range")
        {
            ViReal64 range(val->getVal<ito::float64>());
            retValue += checkError(PM(setPowerRange)(m_instrument, range));
            if (!retValue.containsError())
            {
                retValue += synchronizeParams(bPowerRange);
            }
        }
        else if (key == "auto_range")
        {
            ViBoolean autoRange(val->getVal<ito::int32>());
            retValue += checkError(PM(setPowerAutoRange)(m_instrument, autoRange));
            if (!retValue.containsError())
            {
                retValue += synchronizeParams(bAutoRange | bPowerRange);//also update power range to obtain the last range
            }
        }
        else if (key == "measurement_mode")
        {
            QString mode = val->getVal<char*>();
            if (!QString::compare(mode, "absolute", Qt::CaseInsensitive))
            {
                retValue += checkError(PM(setPowerRefState)(m_instrument, PM(POWER_REF_OFF)));
                if (!retValue.containsError())
                {
                    retValue += synchronizeParams(bMeasurementMode);
                }
            }
            else if (!QString::compare(mode, "relative", Qt::CaseInsensitive))
            {
                retValue += checkError(PM(setPowerRefState)(m_instrument, PM(POWER_REF_ON)));
                if (!retValue.containsError())
                {
                    retValue += synchronizeParams(bMeasurementMode);
                }
            }
            else
            {
                retValue += ito::RetVal(ito::retError, 0, tr("invalid value, 'absolute' or 'relative' required.").toLatin1().data());
            }

        }
        else if (key == "reference_power")
        {
            ViReal64  valRef(val->getVal<ito::float64>());
            retValue += checkError(PM(setPowerRef)(m_instrument, valRef));
            if (!retValue.containsError())
            {
                retValue += synchronizeParams(bPowerReference);
            }
        }
        else if (key == "bandwidth")
        {
            ViInt16 bandwidth(val->getVal<ito::int32>());
            retValue += checkError(PM(setInputFilterState)(m_instrument, bandwidth));
            if (!retValue.containsError())
            {
                retValue += synchronizeParams(bBandwidth);
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
ito::RetVal ThorlabsPowerMeter::startDevice(ItomSharedSemaphore *waitCond)
{
    ito::RetVal retval = ito::RetVal(ito::retWarning, 0, tr("StartDevice not necessary").toLatin1().data());

    if (waitCond)
    {
        waitCond->returnValue = retval;
        waitCond->release();
        waitCond->deleteSemaphore();
    }

    return retval;
}
         
//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal ThorlabsPowerMeter::stopDevice(ItomSharedSemaphore *waitCond)
{
    ito::RetVal retval = ito::RetVal(ito::retWarning, 0, tr("StopDevice not necessary").toLatin1().data());

    if (waitCond)
    {
        waitCond->returnValue = retval;
        waitCond->release();
        waitCond->deleteSemaphore();
    }
    return retval;
}
         
//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal ThorlabsPowerMeter::acquire(const int trigger, ItomSharedSemaphore *waitCond)
{
    ito::RetVal retval(ito::retOk);
    ito::int32 num = m_params["average_number"].getVal<ito::int32>();
    if (num == 1)
    {
        retval += checkError(PM(measPower)(m_instrument, (ViPReal64) &m_data.at<ito::float64>(0, 0)));

    }
    else
    {
        ito::float64 sum(0);
        ViReal64 val;
        for (int i(0); i < num; ++i)
        {
            retval += checkError(PM(measPower)(m_instrument, &val));
            sum += val;
            AddInBase::setAlive();
        }
        m_data.at<ito::float64>(0, 0) = sum / num;
    }
    if (!retval.containsError())
    {
        m_isgrabbing = true;
        m_data.setValueUnit("W");
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
ito::RetVal ThorlabsPowerMeter::retrieveData(ito::DataObject *externalDataObject)
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
ito::RetVal ThorlabsPowerMeter::checkData(ito::DataObject *externalDataObject)
{
    int futureHeight = 1;
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
/*ito::RetVal ThorlabsPowerMeter::checkData(ito::DataObject *externalDataObject)
{
    return ito::retOk;
}*/

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
ito::RetVal ThorlabsPowerMeter::getVal(void *vpdObj, ItomSharedSemaphore *waitCond)
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
ito::RetVal ThorlabsPowerMeter::copyVal(void *vpdObj, ItomSharedSemaphore *waitCond)
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
//! slot called if the dock widget of the plugin becomes (in)visible
/*!
    Overwrite this method if the plugin has a dock widget. If so, you can connect the parametersChanged signal of the plugin
    with the dock widget once its becomes visible such that no resources are used if the dock widget is not visible. Right after
    a re-connection emit parametersChanged(m_params) in order to send the current status of all plugin parameters to the dock widget.
*/
void ThorlabsPowerMeter::dockWidgetVisibilityChanged(bool visible)
{
    if (getDockWidget())
    {
        QWidget *widget = getDockWidget()->widget();
        if (visible)
        {
            connect(this, SIGNAL(parametersChanged(QMap<QString, ito::Param>)), widget, SLOT(parametersChanged(QMap<QString, ito::Param>)));
            connect(this, SIGNAL(visibilityChanged(bool)), widget, SLOT(manageTimer(bool)));
            

            emit visibilityChanged(visible);
            emit parametersChanged(m_params);
        }
        else
        {
            disconnect(this, SIGNAL(parametersChanged(QMap<QString, ito::Param>)), widget, SLOT(parametersChanged(QMap<QString, ito::Param>)));
            emit visibilityChanged(visible);
            disconnect(this, SIGNAL(visibilityChanged(bool)), widget, SLOT(manageTimer(bool)));
        }
    }
}
//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal ThorlabsPowerMeter::checkError(ViStatus err)
{
    if (err)
    {
        ViChar msg[PM(ERR_DESCR_BUFFER_SIZE)];
        memset(msg, 0, sizeof(ViChar) * PM(ERR_DESCR_BUFFER_SIZE));
        PM(errorMessage)(m_instrument, err, msg);

        return ito::RetVal(ito::retError, 0, msg);
    }

    return ito::retOk;
}
//----------------------------------------------------------------------------------------------------------------------------------
//! method called to aqcuire and get a image
/*!
    This method is invoked from the dock widget to get a value in the autograbbing mode. 
    \param [in,out] QSharedPointer to return the measured value.
    \param [in] waitCond is the semaphore (default: NULL), which is released if this method has been terminated
    \return retOk if everything is ok, retError if the occurred any error .
*/
ito::RetVal ThorlabsPowerMeter::acquireAutograbbing(QSharedPointer<double> value, ItomSharedSemaphore *waitCond)
{
    ito::RetVal retval(ito::retOk);
    ito::int32 num = m_params["average_number"].getVal<ito::int32>();
    if (num == 1)
    {
        retval += checkError(PM(measPower)(m_instrument, (ViPReal64) (value.data())));

    }
    else
    {
        ito::float64 sum(0);
        ViReal64 val;
        for (int i(0); i < num; ++i)
        {
            retval += checkError(PM(measPower)(m_instrument, &val));
            sum += val;
            AddInBase::setAlive();
        }
        *value = sum / num;
    }
    if (waitCond)
    {
        waitCond->returnValue = retval;
        waitCond->release();
    }

    return retval;

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
const ito::RetVal ThorlabsPowerMeter::showConfDialog(void)
{
    return apiShowConfigurationDialog(this, new DialogThorlabsPowerMeter(this));
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal ThorlabsPowerMeter::execFunc(const QString funcName, QSharedPointer<QVector<ito::ParamBase> >paramsMand, QSharedPointer<QVector<ito::ParamBase> > paramsOpt, QSharedPointer<QVector<ito::ParamBase> > paramsOut, ItomSharedSemaphore *waitCond)
{
    ito::RetVal retval(ito::retOk);
    if (funcName == "zero_device")
    {
        retval += zeroDevice();
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
ito::RetVal ThorlabsPowerMeter::zeroDevice(ItomSharedSemaphore *waitCond/*=NULL*/)
{
    ito::RetVal retval(ito::retOk);
    retval += checkError(PM(startDarkAdjust)(m_instrument));
    ViInt16 state = PM(STAT_DARK_ADJUST_RUNNING);
    QElapsedTimer timer;
    timer.start();
    while (state == PM(STAT_DARK_ADJUST_RUNNING) && timer.elapsed() < 10000)
    {
        retval += checkError(PM(getDarkAdjustState)(m_instrument, &state));
    }
    if (timer.elapsed() > 10000)
    {
        retval += ito::RetVal(ito::retError, 0,tr("Timeout while dark adjustment ").toLatin1().data());
    }
    if (!retval.containsError())
    {
        synchronizeParams(bDarkOffset);
    }
    if (waitCond)
    {
        waitCond->returnValue = retval;
        waitCond->release();
    }
    return retval;
}
//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal ThorlabsPowerMeter::checkFunctionCompatibility(bool* compatibility)
{
    ito::RetVal retval;

    ViChar manufacturer[256];
    ViChar device[256];
    ViChar serial[256];
    ViChar firmware[256];
    retval += checkError(PM(identificationQuery)(m_instrument, manufacturer, device, serial, firmware));

    QString str = device;
    //QString str = "PM100A";   //For testing purposes

    if (!QString::compare(str, "PM100A", Qt::CaseInsensitive) ||
        !QString::compare(str, "PM100D", Qt::CaseInsensitive) || 
        !QString::compare(str, "PM100USB", Qt::CaseInsensitive) || 
        !QString::compare(str, "PM200", Qt::CaseInsensitive) || 
        !QString::compare(str, "PM400", Qt::CaseInsensitive))
    {
        *compatibility = true;
    }
    else
    {
        *compatibility = false;
    }

    return retval;
}
//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal ThorlabsPowerMeter::synchronizeParams(int what /*=sAll*/)
{
    ito::RetVal retval(ito::retOk);
    bool compatible = false;

    if (what & bWavelength)
    {        
        ViReal64 wavelength;
        ViReal64 min;
        ViReal64 max;

        retval += checkError(PM(getWavelength)(m_instrument, PM(ATTR_SET_VAL), &wavelength));
        retval += checkError(PM(getWavelength)(m_instrument, PM(ATTR_MIN_VAL), &min));
        retval += checkError(PM(getWavelength)(m_instrument, PM(ATTR_MAX_VAL), &max));
        
        if (!retval.containsError())
        {
            retval += m_params["wavelength"].setVal<double>(wavelength);
            m_params["wavelength"].setMeta(new ito::DoubleMeta(min, max, 0.0), true);
        }
    }
	


    if (what & bAttenuation)
    {
        retval = checkFunctionCompatibility(&compatible);   //Check compatibility to Sensors PM100A, PM100D, PM100USB, PM200 and PM400

        ViReal64 attenuation;
        ViReal64 minAttenuation;
        ViReal64 maxAttenuation;

        if (compatible)
        {
            retval += checkError(PM(getAttenuation)(m_instrument, PM(ATTR_SET_VAL), &attenuation));
            retval += checkError(PM(getAttenuation)(m_instrument, PM(ATTR_MAX_VAL), &maxAttenuation));
            retval += checkError(PM(getAttenuation)(m_instrument, PM(ATTR_MIN_VAL), &minAttenuation));
        }
        else
        {
            attenuation = -1;
            maxAttenuation = -1;
            minAttenuation = -1;
            m_params["attenuation"].setFlags(ito::ParamBase::Readonly);
        }

        if (!retval.containsError())
        {
            retval += m_params["attenuation"].setVal<double>(attenuation);
            m_params["attenuation"].setMeta(new ito::DoubleMeta(minAttenuation, maxAttenuation, 0.0), true);
        }
    }
	


    if (what & bDarkOffset)
    {
        ViReal64 dark;
        retval += checkError(PM(getDarkOffset)(m_instrument, &dark));

        if (!retval.containsError())
        {
            retval += m_params["dark_offset"].setVal<double>(dark);
        }
    }
	


    if (what & bLineFrequency)
    {
        retval = checkFunctionCompatibility(&compatible);   //Check compatibility to Sensors PM100A, PM100D, PM100USB, PM200 and PM400

        ViInt16 frequency;

        if (compatible)
        {
            retval += checkError(PM(getLineFrequency)(m_instrument, &frequency));
        }
        else
        {
            frequency = -1;
            m_params["line_frequency"].setFlags(ito::ParamBase::Readonly);
        }

        if (!retval.containsError())
        {
            retval += m_params["line_frequency"].setVal<int>(frequency);
        }
    }
	


    if (what & bPowerRange)
    {
        ViReal64 powerRange;
        ViReal64 minPowerRange;
        ViReal64 maxPowerRange;

        retval += checkError(PM(getPowerRange)(m_instrument, PM(ATTR_SET_VAL), &powerRange));
        retval += checkError(PM(getPowerRange)(m_instrument, PM(ATTR_MAX_VAL), &maxPowerRange));
        retval += checkError(PM(getPowerRange)(m_instrument, PM(ATTR_MIN_VAL), &minPowerRange));
        if (!retval.containsError())
        {
            retval += m_params["power_range"].setVal<double>(powerRange);

            m_params["power_range"].setMeta(new ito::DoubleMeta(minPowerRange, maxPowerRange, 0.0), true);
        }

    }



    if (what & bAutoRange)
    {
        ViBoolean autoRange;
        retval += checkError(PM(getPowerAutorange)(m_instrument, &autoRange));
        if (!retval.containsError())
        {
            retval += m_params["auto_range"].setVal<ito::int32>(autoRange);
            if (autoRange)
            {
                m_params["power_range"].setFlags(ito::ParamBase::Readonly);
            }
            else
            {
                m_params["power_range"].setFlags(ito::ParamBase::In);
            }

        }
    }



    if (what & bMeasurementMode)
    {
        ViBoolean mode;
        retval += checkError(PM(getPowerRefState)(m_instrument, &mode));
        if (!retval.containsError())
        {
			if (mode == PM(POWER_REF_ON))
			{
				retval += m_params["measurement_mode"].setVal<const char*>("relative");
			}
			else if (mode == PM(POWER_REF_OFF))
			{
				retval += m_params["measurement_mode"].setVal<const char*>("absolute");
			}
			else
			{
				retval += ito::RetVal(ito::retError, 0, tr("received invalid mode from device").toLatin1().data());
			}
        }
    }



    if (what & bPowerReference)
    {
        ViReal64 refPower;
        ViReal64 maxRefPower;
        ViReal64 minRefPower;
        retval += checkError(PM(getPowerRef)(m_instrument, PM(ATTR_SET_VAL), &refPower));
        retval += checkError(PM(getPowerRef)(m_instrument, PM(ATTR_MAX_VAL), &maxRefPower));
        retval += checkError(PM(getPowerRef)(m_instrument, PM(ATTR_MIN_VAL), &minRefPower));
        if (!retval.containsError())
        {
            retval += m_params["reference_power"].setVal<double>(refPower);
            m_params["reference_power"].setMeta(new ito::DoubleMeta(minRefPower, maxRefPower, 0.0), true);
        }
    }


	
    if (what & bBandwidth)
    {
        retval = checkFunctionCompatibility(&compatible);   //Check compatibility to Sensors PM100A, PM100D, PM100USB, PM200 and PM400

        ViBoolean bandwidth;

        if (compatible)
        {
            retval += checkError(PM(getInputFilterState)(m_instrument, &bandwidth));
        }
        else{
            bandwidth = -1;
            m_params["bandwidth"].setFlags(ito::ParamBase::Readonly);
        }
        
        if (!retval.containsError())
        {
               retval += m_params["bandwidth"].setVal<ito::int32>(bandwidth);
        }

    }
    
    return retval;
}
