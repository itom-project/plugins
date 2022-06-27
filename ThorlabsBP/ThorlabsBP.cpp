/* ********************************************************************
    Plugin "ThorlabsBP" for itom software
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

#include "ThorlabsBP.h"

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

#include "Thorlabs.MotionControl.Benchtop.Piezo.h"

#include <qdebug.h>

QList<QByteArray> ThorlabsBP::openedDevices = QList<QByteArray>();


//----------------------------------------------------------------------------------------------------------------------------------
/*!
    \detail creates new instance of ThorlabsBPInterface and returns the instance-pointer
    \param [in,out] addInInst is a double pointer of type ito::AddInBase. The newly created ThorlabsBPInterface-instance is stored in *addInInst
    \return retOk
    \sa ThorlabsBP
*/
ito::RetVal ThorlabsBPInterface::getAddInInst(ito::AddInBase **addInInst)
{
    ito::RetVal retValue;
    NEW_PLUGININSTANCE(ThorlabsBP)
    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
/*!
    \detail Closes an instance of of ThorlabsBPInterface. This instance is given by parameter addInInst.
    \param [in] double pointer to the instance which should be deleted.
    \return retOk
    \sa ThorlabsBP
*/
ito::RetVal ThorlabsBPInterface::closeThisInst(ito::AddInBase **addInInst)
{

    ito::RetVal retValue;
    REMOVE_PLUGININSTANCE(ThorlabsBP)
    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
/*! \detail defines the plugin type (typeActuator) and sets the plugins object name. Theplugin is initialized (e.g. by a Python call) 
    with mandatory or optional parameters (m_initParamsMand and m_initParamsOpt).
*/
ThorlabsBPInterface::ThorlabsBPInterface()
{
    m_type = ito::typeActuator;

    setObjectName("ThorlabsBP");

    m_description = QObject::tr("ThorlabsBP");
    m_detaildescription = QObject::tr("ThorlabsBP is an acutator plugin to control the following integrated devices from Thorlabs: \n\
\n\
* Benchtop Piezo (1 channel) \n\
* Benchtop Piezo (3 channels) \n\
\n\
It requires the new Kinesis driver package from Thorlabs and implements the interface Thorlabs.MotionControl.Benchtop.Piezo.\n\
\n\
Please install the Kinesis driver package in advance with the same bit-version (32/64bit) than itom. \n\
\n\
This plugin has been tested with the Benchtop Piezo with 1 and 3 channels. \n\
\n\
The position values are always in mm if the corresponding axis is in closed-loop mode and if a strain gauge feedback is connected. Else the values are always in volts.");

    m_author = "M. Gronle, ITO, University Stuttgart";
    m_version = (PLUGIN_VERSION_MAJOR << 16) + (PLUGIN_VERSION_MINOR << 8) + PLUGIN_VERSION_PATCH;
    m_minItomVer = MINVERSION;
    m_maxItomVer = MAXVERSION;
    m_license = QObject::tr("licensed under LGPL");
    m_aboutThis = QObject::tr(GITVERSION);    
    
    m_initParamsOpt.append(ito::Param("serialNo", ito::ParamBase::String, "", tr("Serial number of the device to be loaded, if empty, the first device that can be opened will be opened").toLatin1().data()));
    m_initParamsOpt.append(ito::Param("channels", ito::ParamBase::IntArray, NULL, tr("list of channels to connect to. If an empty list is given, all connected channels are used. The plugin axis indices are then mapped to the channels.").toLatin1().data()));
    m_initParamsOpt[1].setMeta(new ito::IntArrayMeta(1, 3, 1, 0, 3, 1), true);
}

//----------------------------------------------------------------------------------------------------------------------------------
// this makro registers the class ThorlabsBPInterface with the name ThorlabsBPInterface as plugin for the Qt-System (see Qt-DOC)


//----------------------------------------------------------------------------------------------------------------------------------
/*! \detail defines the name and sets the plugins parameters (m_parans). The plugin is initialized (e.g. by a Python call) 
    with mandatory or optional parameters (m_initParamsMand and m_initParamsOpt) by the ThorlabsBP::init. The widged window is created at this position.
*/
ThorlabsBP::ThorlabsBP() :
    AddInActuator(),
    m_async(0),
    m_opened(false),
    m_numChannels(0)
{
    m_params.insert("name", ito::Param("name", ito::ParamBase::String | ito::ParamBase::Readonly, "ThorlabsBP", tr("name of plugin").toLatin1().data()));
    m_params.insert("numaxis", ito::Param("numaxis", ito::ParamBase::Int | ito::ParamBase::Readonly, 0, 100, 0, tr("number of axes (channels)").toLatin1().data()));
    m_params.insert("deviceName", ito::Param("deviceName", ito::ParamBase::String | ito::ParamBase::Readonly, "", tr("Description of the device").toLatin1().data()));
    m_params.insert("serialNumber", ito::Param("serialNumber", ito::ParamBase::String | ito::ParamBase::Readonly, "", tr("Serial number of the device").toLatin1().data()));
    m_params.insert("channel", ito::Param("channel", ito::ParamBase::IntArray | ito::ParamBase::Readonly, NULL, tr("Channel number of each axis.").toLatin1().data()));
    m_params.insert("timeout", ito::Param("timeout", ito::ParamBase::Double, 0.0, 200.0, 5.0, tr("Timeout for positioning in seconds.").toLatin1().data()));
    
    m_params.insert("enabled", ito::Param("enabled", ito::ParamBase::IntArray, NULL, tr("If 1, the axis is enabled and power is applied to the motor. 0: disabled, the motor can be turned by hand.").toLatin1().data()));
    m_params.insert("zeroed", ito::Param("zeroed", ito::ParamBase::IntArray | ito::ParamBase::Readonly, NULL, tr("If 0, the axis is not zeroed. 1: zeroed. If the axis is not zeroed, it is possible that position values at the edge of the valid range can not be reached.").toLatin1().data()));
    m_params.insert("hasFeedback", ito::Param("hasFeedback", ito::ParamBase::IntArray | ito::ParamBase::Readonly, NULL, tr("If 0, the axis is not equipped with a strain gauge feedback, else 1.").toLatin1().data()));
    m_params.insert("controlMode", ito::Param("controlMode", ito::ParamBase::IntArray, NULL, tr("Open loop (0), closed loop (1)").toLatin1().data()));
    m_params.insert("maximumTravelRange", ito::Param("maximumTravelRange", ito::ParamBase::DoubleArray | ito::ParamBase::Readonly, NULL, tr("Maximum travel range for each axis in mm. This requires an actuator with built in position sensing. These values might not be correct if the motor is in open loop mode.").toLatin1().data()));
    m_params.insert("maximumVoltage", ito::Param("maximumVoltage", ito::ParamBase::IntArray, NULL, tr("Maximum output voltage (75, 100 or 150 V).").toLatin1().data()));
    
    m_params.insert("async", ito::Param("async", ito::ParamBase::Int, 0, 1, m_async, tr("asychronous (1) or synchronous (0) mode").toLatin1().data()));

    m_currentPos.fill(0.0, 1);
    m_currentStatus.fill(0, 1);
    m_targetPos.fill(0.0, 1);

    if (hasGuiSupport())
    {
        //now create dock widget for this plugin
        DockWidgetThorlabsBP *dockWidget = new DockWidgetThorlabsBP(this);
        Qt::DockWidgetAreas areas = Qt::AllDockWidgetAreas;
        QDockWidget::DockWidgetFeatures features = QDockWidget::DockWidgetClosable | QDockWidget::DockWidgetFloatable | QDockWidget::DockWidgetMovable;
        createDockWidget(QString(m_params["name"].getVal<char *>()), features, areas, dockWidget);
    }

    memset(m_serialNo, '\0', sizeof(m_serialNo));
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal ThorlabsBP::init(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> * paramsOpt, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retval = ito::retOk;

    QByteArray serial = paramsOpt->at(0).getVal<char*>();
    const int* channels = paramsOpt->at(1).getVal<int*>();
    int numChannels = paramsOpt->at(1).getLen();

    retval += checkError(TLI_BuildDeviceList(), "build device list");
    QByteArray existingSerialNumbers("", 256);
    TLI_DeviceInfo deviceInfo;
    int maxChannels = 0;

    if (!retval.containsError())
    {
        short numDevices = TLI_GetDeviceListSize();

        if (numDevices == 0)
        {
            retval += ito::RetVal(ito::retError, 0, "no Thorlabs devices detected.");
        }
        else
        {
            retval += checkError(TLI_GetDeviceListExt(existingSerialNumbers.data(), existingSerialNumbers.size()), "get device list");
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
				if (!openedDevices.contains(serialNumbers[i]))
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
        if (deviceInfo.isKnownType && (deviceInfo.typeID == 41 /*Benchtop Piezo (1 channel)*/ || \
            deviceInfo.typeID == 71 /*Benchtop Piezo (3 channels)*/))
        {
            maxChannels = (deviceInfo.typeID == 71) ? 3 : 1;
            memcpy(m_serialNo, serial.data(), std::min((size_t)serial.size(), sizeof(m_serialNo)));
            retval += checkError(PBC_Open(m_serialNo), "open device");

            if (!retval.containsError())
            {
                m_opened = true;
				openedDevices.append(m_serialNo);
            }
        }
        else
        {
            retval += ito::RetVal(ito::retError, 0, "the type of the device is not among the supported devices (Benchtop Piezo with 1 or 3 channels)");
        }
    }

    if (!retval.containsError())
    {
        //qDebug() << PBC_MaxChannelCount(m_serialNo); bug in kinesis: this method always returns 3.
        if (PBC_GetNumChannels(m_serialNo) <= 0)
        {
            retval += ito::RetVal(ito::retError, 0, "The device has no connected channels.");
        }
        else
        {
            if (numChannels <= 0)
            {
                m_channelIndices.resize(PBC_GetNumChannels(m_serialNo));
                int i = 0;
                for (int c = 1; c <= maxChannels; ++c)
                {
                    if (PBC_EnableChannel(m_serialNo, c) == 0)
                    {
                        m_channelIndices[i] = c;
                        i++;
                    }
                }
            }
            else if (numChannels < maxChannels)
            {
                m_channelIndices.resize(numChannels);
                m_channelIndices.fill(0, numChannels);
                bool found;

                for (int c = 0; c < numChannels; ++c)
                {
                    if (PBC_EnableChannel(m_serialNo, channels[c]) == 0)
                    {
                    /*if (PBC_IsChannelValid(serial.data(), channels[c]))
                    {*/
                        found = false;
                        foreach(int channel, m_channelIndices)
                        {
                            if (channel == channels[c])
                            {
                                found = true;
                                retval += ito::RetVal(ito::retError, 0, "Every channel can only be connected once.");
                            }
                        }

                        if (!found)
                        {
                            m_channelIndices[c] = channels[c];
                        }
                    }
                    else
                    {
                        retval += ito::RetVal::format(ito::retError, 0, "Channel %i is not connected", channels[c]);
                    }
                }
            }
            else
            {
                retval += ito::RetVal(ito::retError, 0, "There are more channels given than connected to the device.");
            }
        }
    }

    if (!retval.containsError())
    {
        foreach (int channel, m_channelIndices)
        {
            //this method is not available in current SDK (Kinesis 1.6.0)
            int err = PBC_LoadSettings(m_serialNo, channel);
            if (!err)
            {
                retval += ito::RetVal(ito::retWarning, 0, "settings of device could not be loaded.");
            }

            if (!PBC_StartPolling(m_serialNo, channel, 100))
            {
                retval += ito::RetVal(ito::retError, 0, "error starting position and status polling.");
            }
        }

        m_numChannels = m_channelIndices.size();
        m_dummyValues = QSharedPointer<QVector<double> >(new QVector<double>(m_numChannels, 0.0));
       

        m_params["numaxis"].setVal<int>(m_numChannels);
        m_currentPos.fill(0.0, m_numChannels);
        m_currentStatus.fill(ito::actuatorAvailable | ito::actuatorEnabled | ito::actuatorAtTarget, m_numChannels);
        m_targetPos.fill(0.0, m_numChannels);

        int *dummy = new int[m_numChannels];
        memset(dummy, 0, m_numChannels * sizeof(int));
        m_params["enabled"].setMeta(new ito::IntArrayMeta(0, 1, 1, m_numChannels, m_numChannels, 1), true);
        m_params["enabled"].setVal<int*>(dummy, m_numChannels);
        m_params["zeroed"].setMeta(new ito::IntArrayMeta(0, 1, 1, m_numChannels, m_numChannels, 1), true);
        m_params["zeroed"].setVal<int*>(dummy, m_numChannels);
        m_params["hasFeedback"].setMeta(new ito::IntArrayMeta(0, 1, 1, m_numChannels, m_numChannels, 1), true);
        m_params["hasFeedback"].setVal<int*>(dummy, m_numChannels);
        m_params["controlMode"].setMeta(new ito::IntArrayMeta(0, 1, 1, m_numChannels, m_numChannels, 1), true);
        m_params["controlMode"].setVal<int*>(dummy, m_numChannels);
        m_params["maximumTravelRange"].setMeta(new ito::DoubleArrayMeta(0.0, std::numeric_limits<double>::max(), 0.0, m_numChannels, m_numChannels, 1), true);
        m_params["maximumVoltage"].setMeta(new ito::IntArrayMeta(75, 150, 25, m_numChannels, m_numChannels, 1), true);
        m_params["slewRateOpenLoop"].setMeta(new ito::DoubleArrayMeta(0.00, 258.68, 0.0, m_numChannels, m_numChannels, 1), true); //max is 32767 * max-voltage / 19000 (we assume the max-voltage to be 150V)
        m_params["slewRateClosedLoop"].setMeta(new ito::DoubleArrayMeta(0.00, 258.68, 0.0, m_numChannels, m_numChannels, 1), true); //max is 32767 * max-voltage / 19000 (we assume the max-voltage to be 150V)
        for (int i = 0; i < m_numChannels; ++i)
        {
            dummy[i] = m_channelIndices[i];
        }
        m_params["channel"].setVal<int*>(dummy, m_numChannels);
        DELETE_AND_SET_NULL_ARRAY(dummy);
    }

    if (!retval.containsError())
    {
        Sleep(160);
        QSharedPointer<QVector<int> > status(new QVector<int>(m_numChannels, 0));
        retval += getStatus(status, NULL);
        updateRanges();
    }
    
    if (waitCond)
    {
        waitCond->returnValue = retval;
        waitCond->release();
    }

    setInitialized(true); //init method has been finished (independent on retval)
    return retval;
}

//---------------------------------------------------------------------------------------------------------------------------------
void ThorlabsBP::updateRanges()
{
    int *vals = new int[m_numChannels];
    ito::float64 *values = new ito::float64[m_numChannels];
    for (int i = 0; i < m_numChannels; ++i)
    {
        values[i] = (ito::float64)PBC_GetMaximumTravel(m_serialNo, m_channelIndices[i]) / 10000.0;
    }
    m_params["maximumTravelRange"].setVal<ito::float64*>(values, m_numChannels);

    for (int i = 0; i < m_numChannels; ++i)
    {
        vals[i] = qRound(PBC_GetMaxOutputVoltage(m_serialNo, m_channelIndices[i]) / 10.0);
    }
    m_params["maximumVoltage"].setVal<int*>(vals, m_numChannels);

    DELETE_AND_SET_NULL_ARRAY(vals);
    DELETE_AND_SET_NULL_ARRAY(values);
}

//---------------------------------------------------------------------------------------------------------------------------------
const ito::RetVal ThorlabsBP::showConfDialog(void)
{
    return apiShowConfigurationDialog(this, new DialogThorlabsBP(this));
}

//----------------------------------------------------------------------------------------------------------------------------------
/*! \detail close method which is called before that this instance is deleted by the ThorlabsBPInterface
    notice that this method is called in the actual thread of this instance.

    \param [in] waitCond is the semaphore (default: NULL), which is released if this method has been terminated
    \return retOk
    \sa ItomSharedSemaphore
*/
ito::RetVal ThorlabsBP::close(ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retval = ito::retOk;

    if (m_opened)
    {
        foreach(int channel, m_channelIndices)
        {
            PBC_StopPolling(m_serialNo, channel);
        }
        Sleep(200);

        PBC_Close(m_serialNo); //Kinesis 1.6.0 and Kinesis 1.7.0 crash in this line! bug. todo.
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
ito::RetVal ThorlabsBP::getParam(QSharedPointer<ito::Param> val, ItomSharedSemaphore *waitCond)
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
            If the "ctrl-type" is set, ThorlabsBP::SMCSwitchType is executed.

    \param[in] *name        Name of parameter
    \param[in] *val            String with parameter
    \param[in] len            Length of the string
    \param[in/out] *waitCond    Waitcondition between this thread and the callers tread

    \return retOk
*/
ito::RetVal ThorlabsBP::setParam(QSharedPointer<ito::ParamBase> val, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);
    QString key;
    bool hasIndex;
    int index = 0;
    QString suffix;
    ParamMapIterator it;

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

    if (!retValue.containsError() && hasIndex)
    {
        if (index < 0 || index >= m_numChannels)
        {
            retValue += ito::RetVal(ito::retError, 0, "The given index is out of bounds.");
        }
    }

    if (!retValue.containsError())
    {
        //---------------------------
        if (key == "async")
        {
            m_async = val->getVal<int>();
            retValue += it->copyValueFrom(&(*val));
        }
        else if (key == "enabled")
        {
            if (hasIndex)
            {
                if (val->getVal<int>())
                {
                    retValue += checkError(PBC_EnableChannel(m_serialNo, m_channelIndices[index]), "enable channel");
                }
                else
                {
                    retValue += checkError(PBC_DisableChannel(m_serialNo, m_channelIndices[index]), "disable channel");
                }

                if (!retValue.containsError())
                {
                    it->getVal<int*>()[index] = val->getVal<int>();
                }
                else
                {
                    Sleep(160);
                    QSharedPointer<QVector<int> > status(new QVector<int>(m_numChannels, 0));
                    retValue += getStatus(status, NULL);
                }
            }
            else
            {
                const int *values = val->getVal<int*>(); //are alway m_numChannels values due to meta information
                for (int i = 0; i < m_numChannels; ++i)
                {
                    if (values[i])
                    {
                        retValue += checkError(PBC_EnableChannel(m_serialNo, m_channelIndices[i]), "enable channel");
                    }
                    else
                    {
                        retValue += checkError(PBC_DisableChannel(m_serialNo, m_channelIndices[i]), "disable channel");
                    }
                }

                if (!retValue.containsError())
                {
                    it->copyValueFrom(&(*val));
                }
                else
                {
                    Sleep(160);
                    QSharedPointer<QVector<int> > status(new QVector<int>(m_numChannels, 0));
                    retValue += getStatus(status, NULL);
                }
            }
        }
        else if (key == "controlMode")
        {
            if (hasIndex)
            {
                if (m_params["hasFeedback"].getVal<int*>()[index])
                {
                    retValue += checkError(PBC_SetPositionControlMode(m_serialNo, m_channelIndices[index], val->getVal<int>() ? PZ_CloseLoop : PZ_OpenLoop), "set control mode");
                }
                else
                {
                    retValue += ito::RetVal(ito::retError, 0, "The axis has no position feedback. No closed loop is possible.");
                }

                if (!retValue.containsError())
                {
                    it->getVal<int*>()[index] = val->getVal<int>();
                }
                else
                {
                    Sleep(160);
                    QSharedPointer<QVector<int> > status(new QVector<int>(m_numChannels, 0));
                    retValue += getStatus(status, NULL);
                }
            }
            else
            {
                const int *values = val->getVal<int*>(); //are alway m_numChannels values due to meta information
                for (int i = 0; i < m_numChannels; ++i)
                {
                    if (m_params["hasFeedback"].getVal<int*>()[i])
                    {
                        retValue += checkError(PBC_SetPositionControlMode(m_serialNo, m_channelIndices[i], values[i] ? PZ_CloseLoop : PZ_OpenLoop), "set control mode");
                    }
                    else
                    {
                        retValue += ito::RetVal::format(ito::retError, 0, "The axis %i  has no position feedback. No closed loop is possible.", i);
                    }
                }

                if (!retValue.containsError())
                {
                    it->copyValueFrom(&(*val));
                }
                else
                {
                    Sleep(160);
                    QSharedPointer<QVector<int> > status(new QVector<int>(m_numChannels, 0));
                    retValue += getStatus(status, NULL);
                }
            }

            Sleep(160);

            updateRanges(); //if control mode becomes closed loop, the maximum travel range can be changed!

            QVector<int> allAxes;
            for (int i = 0; i < m_numChannels; ++i)
            {
                allAxes << i;
            }

            retValue += getPos(allAxes, m_dummyValues, NULL);

            m_targetPos = m_currentPos; //if control mode is changed, the unit of the positions might change
            sendTargetUpdate();
        }
        else if (key == "maximumVoltage")
        {
            if (hasIndex)
            {
                if (val->getVal<int>() == 125)
                {
                    retValue += ito::RetVal(ito::retError, 0, "Maximum output voltage of 125 V is not allowed.");
                }
                else
                {
                    retValue += checkError(PBC_SetMaxOutputVoltage(m_serialNo, m_channelIndices[index], val->getVal<int>() * 10.0), "set maximum voltage");
                }

                if (!retValue.containsError())
                {
                    it->getVal<int*>()[index] = val->getVal<int>();
                }
                else
                {
                    it->getVal<int*>()[index] = qRound(PBC_GetMaxOutputVoltage(m_serialNo, m_channelIndices[index]) / 10.0);
                }
            }
            else
            {
                int *values = val->getVal<int*>(); //are alway m_numChannels values due to meta information
                for (int i = 0; i < m_numChannels; ++i)
                {
                    if (values[i] == 125)
                    {
                        retValue += ito::RetVal(ito::retError, 0, "Maximum output voltage of 125 V is not allowed.");
                    }
                    else
                    {
                        retValue += checkError(PBC_SetMaxOutputVoltage(m_serialNo, m_channelIndices[i], values[i] * 10.0), "set maximum voltage");
                    }
                }

                if (!retValue.containsError())
                {
                    it->copyValueFrom(&(*val));
                }
                else
                {
                    for (int i = 0; i < m_numChannels; ++i)
                    {
                        values[i] = qRound(PBC_GetMaxOutputVoltage(m_serialNo, m_channelIndices[i]) / 10.0);
                    }
                }
            }

            Sleep(160);

            updateRanges();

            QVector<int> allAxes;
            for (int i = 0; i < m_numChannels; ++i)
            {
                allAxes << i;
            }

            retValue += getPos(allAxes, m_dummyValues, NULL);

            QSharedPointer<QVector<int> > status(new QVector<int>(m_numChannels, 0));
            retValue += getStatus(status, NULL);
        }

        //---------------------------
        else
        {
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
/*! \detail This function executes a calibration routine for one axis spezified by "axis". In the case of this device the function body is nearly empty and has no effect.

    \param [in] axis    Number of axis to calibrate
    \param [in] waitCond is the semaphore (default: NULL), which is released if this method has been terminated
    \return retOk
    \sa ItomSharedSemaphore
*/
ito::RetVal ThorlabsBP::calib(const int axis, ItomSharedSemaphore *waitCond)
{
    return calib(QVector<int>(1, axis), waitCond);
}

//----------------------------------------------------------------------------------------------------------------------------------
/*! \detail This function executes a calibration routine for a set of axis spezified by "axis". In the case of this device the function body is nearly empty and has no effect.

    \param [in] axis    Vector this numbers of axis to calibrate
    \param [in] waitCond is the semaphore (default: NULL), which is released if this method has been terminated
    \return retOk
    \sa ItomSharedSemaphore
*/
ito::RetVal ThorlabsBP::calib(const QVector<int> axis, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retval;
    bool done = false;
    DWORD s;

    foreach(int ax, axis)
    {
        if (ax < 0 || ax >= m_numChannels)
        {
            retval += ito::RetVal::format(ito::retError, 0, "invalid axis index %i.", ax);
        }
        else
        {
            s = PBC_GetStatusBits(m_serialNo, m_channelIndices[ax]);
            if ((s & 0x80000000) == 0)
            {
                retval += ito::RetVal::format(ito::retError, 0, "axis %i is disabled and cannot be zeroed.", ax);
            }
        }
    }

    if (!retval.containsError())
    {
        foreach(int ax, axis)
        {
            retval += checkError(PBC_SetZero(m_serialNo, m_channelIndices[ax]), "set zero");
        }

        replaceStatus(axis, ito::actuatorAtTarget, ito::actuatorMoving);
        sendStatusUpdate(true);

        Sleep(400); //to be sure that the first requested status is correct

        while (!done && !retval.containsError())
        {
            done = true;

            foreach(int ax, axis)
            {
                s = PBC_GetStatusBits(m_serialNo, m_channelIndices[ax]);
                if (s & 0x00000020)
                {
                    done = false;
                }
            }

            if (!done)
            {
                setAlive();
                Sleep(160);
            }
        }

        if (done)
        {   // Position reached and movement done
            replaceStatus(axis, ito::actuatorMoving, ito::actuatorAtTarget);
            sendStatusUpdate(true);

            QSharedPointer<QVector<int> > status(new QVector<int>(m_numChannels, 0));
            getStatus(status, NULL);

            QVector<int> allAxes;
            for (int i = 0; i < m_numChannels; ++i)
            {
                allAxes << i;
            }

            getPos(allAxes, m_dummyValues, NULL);
        }
    }

    if (waitCond)
    {
        waitCond->returnValue = retval;
        waitCond->release();
    }

    return retval;
}

//----------------------------------------------------------------------------------------------------------------
ito::RetVal ThorlabsBP::setOrigin(const int axis, ItomSharedSemaphore * waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retval = ito::RetVal(ito::retWarning, 0, "set origin not implemented.");

    if (waitCond)
    {
        waitCond->returnValue = retval;
        waitCond->release();
    }

    return retval;
}

//----------------------------------------------------------------------------------------------------------------
ito::RetVal ThorlabsBP::setOrigin(QVector<int> axis, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retval = ito::RetVal(ito::retWarning, 0, "set origin not implemented.");

    if (waitCond)
    {
        waitCond->returnValue = retval;
        waitCond->release();
    }

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
/*! \detail This function gets the status of the device. The SMCStatus function is called internally.

    \param [out] status        Status of System. 0: okay, 1: error
    \param [in] waitCond is the semaphore (default: NULL), which is released if this method has been terminated
    \return retOk
    \todo define the status value
*/
ito::RetVal ThorlabsBP::getStatus(QSharedPointer<QVector<int> > status, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retval;
    DWORD s;

    int *zeroed = new int[m_numChannels];
    int *enabled = new int[m_numChannels];
    int *controlMode = new int[m_numChannels];
    int *hasFeedback = new int[m_numChannels];

    for (int idx = 0; idx < m_numChannels; ++idx)
    {
        s = PBC_GetStatusBits(m_serialNo, m_channelIndices[idx]);

        m_currentStatus[idx] = ito::actuatorAtTarget;
        m_currentStatus[idx] |= ito::actuatorAvailable; //the connected flag is not always set

        if (s & 0x80000000)
        {
            m_currentStatus[idx] |= ito::actuatorEnabled;
        }

        zeroed[idx] =      (s & 0x00000010) ? 1 : 0;
        enabled[idx] =     (s & 0x80000000) ? 1 : 0;
        controlMode[idx] = (s & 0x00000400) ? 1 : 0; //1 -> closed loop, 0: open loop
        hasFeedback[idx] = (s & 0x00000100) ? 1 : 0;

        if (memcmp(m_params["zeroed"].getVal<int*>(), zeroed, m_numChannels * sizeof(int)) != 0 || \
            memcmp(m_params["enabled"].getVal<int*>(), enabled, m_numChannels * sizeof(int)) != 0 || \
            memcmp(m_params["hasFeedback"].getVal<int*>(), hasFeedback, m_numChannels * sizeof(int)) != 0 || \
            memcmp(m_params["controlMode"].getVal<int*>(), controlMode, m_numChannels * sizeof(int)) != 0)
        {
            m_params["zeroed"].setVal<int*>(zeroed, m_numChannels);
            m_params["enabled"].setVal<int*>(enabled, m_numChannels);
            m_params["controlMode"].setVal<int*>(controlMode, m_numChannels);
            m_params["hasFeedback"].setVal<int*>(hasFeedback, m_numChannels);
            emit parametersChanged(m_params); //send changed parameters to any connected dialogs or dock-widgets
        }

        *status = m_currentStatus;
    }

    DELETE_AND_SET_NULL_ARRAY(zeroed);
    DELETE_AND_SET_NULL_ARRAY(enabled);
    DELETE_AND_SET_NULL_ARRAY(controlMode);
    DELETE_AND_SET_NULL_ARRAY(hasFeedback);

    if (waitCond)
    {
        waitCond->returnValue = retval;
        waitCond->release();
    }

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
/*! \detail Get the Position of a single axis spezified by axis. The value in device independet in mm.

    \param [in] axis        Axisnumber
    \param [out] pos        Current position in mm
    \param [in] waitCond is the semaphore (default: NULL), which is released if this method has been terminated
    \return retOk
*/
ito::RetVal ThorlabsBP::getPos(const int axis, QSharedPointer<double> pos, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    QVector<int> axes(1, axis);
    ito::RetVal retval = getPos(axes, m_dummyValues, NULL);
    *pos = m_dummyValues->at(0);

    if (waitCond)
    {
        waitCond->returnValue = retval;
        waitCond->release();
    }

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
/*! \detail Get the Position of a set of axis spezified by "axis". The value in device independet in mm. 
            In this case if more than one axis is specified this function returns an error.

    \param [in] axis        Vector with axis numbers
    \param [out] pos        Current positions in mm
    \param [in] waitCond is the semaphore (default: NULL), which is released if this method has been terminated
    \return retOk
*/
ito::RetVal ThorlabsBP::getPos(const QVector<int> axis, QSharedPointer<QVector<double> > pos, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retval;

    const int* controlMode = m_params["controlMode"].getVal<int*>();
    const int* hasFeedback = m_params["hasFeedback"].getVal<int*>();
    const ito::float64* maximumTravelRange = m_params["maximumTravelRange"].getVal<ito::float64*>();
    const int* maximumVoltage = m_params["maximumVoltage"].getVal<int*>();
    int idx;

    for (int i = 0; i < axis.size(); ++i)
    {
        idx = axis[i];

        if (idx < 0 || idx >= m_numChannels)
        {
            retval += ito::RetVal::format(ito::retError, 0, "invalid axis index %i", idx);
            break;
        }

        if (controlMode[idx] && hasFeedback[idx])
        {
            //in mm
            m_currentPos[idx] = maximumTravelRange[0] * (ito::float64)PBC_GetPosition(m_serialNo, m_channelIndices[idx]) / 32767.0;
            (*pos)[i] = m_currentPos[idx];
        }
        else
        {
            //in V
            m_currentPos[idx] = maximumVoltage[0] * (ito::float64)PBC_GetOutputVoltage(m_serialNo, m_channelIndices[idx]) / 32767.0;
            (*pos)[i] = m_currentPos[idx];
        }
    }

    sendStatusUpdate(false);

    if (waitCond)
    {
        waitCond->returnValue = retval;
        waitCond->release();
    }

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
/*! \detail Set the absolute position of a one axis spezified by "axis" to the position "pos" . The value in device independet in mm. 
            This function calls ThorlabsBP::SMCSetPos(axis, pos, "ABSOLUTCOMMAND")

    \param [in] axis     axis number
    \param [in] pos      absolute target position in mm
    \param [in] waitCond is the semaphore (default: NULL), which is released if this method has been terminated
    \sa SMCSetPos
    \return retOk
*/
ito::RetVal ThorlabsBP::setPosAbs(const int axis, const double pos, ItomSharedSemaphore *waitCond)
{
    QVector<int> axes(1, axis);
    QVector<double> positions(1, pos);
    return setPosAbs(axes, positions, waitCond);
}

//----------------------------------------------------------------------------------------------------------------------------------
/*! \detail Set the absolute position of a number of axis spezified by "axis" to the position "pos" . The value in device independet in mm.
            If the size of the vector is more then 1 element, this function returns an error.
            This function calls ThorlabsBP::SMCSetPos(axis, pos, "ABSOLUTCOMMAND")

    \param [in] axis    1 Element Vector with axis numbers
    \param [in] pos        1 Element Vector with absolute positions in mm
    \param [in] waitCond is the semaphore (default: NULL), which is released if this method has been terminated
    \sa SMCSetPos
    \return retOk
*/
ito::RetVal ThorlabsBP::setPosAbs(const QVector<int> axis, QVector<double> pos, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retval;
    int idx;
    const int* controlMode = m_params["controlMode"].getVal<int*>();
    const int* hasFeedback = m_params["hasFeedback"].getVal<int*>();
    const ito::float64* maximumTravelRange = m_params["maximumTravelRange"].getVal<ito::float64*>();
    const int* maximumVoltage = m_params["maximumVoltage"].getVal<int*>();
    int flags = 0;
    DWORD s;

    if (isMotorMoving())
    {
        retval += ito::RetVal(ito::retError, 0, tr("Any motor axis is moving. The motor is locked.").toLatin1().data());
    }
    else
    {
        foreach(int a, axis)
        {
            if (a < 0 || a >= m_numChannels)
            {
                retval += ito::RetVal::format(ito::retError, 0, "invalid axis index %i.", a);
            }
            else
            {
                s = PBC_GetStatusBits(m_serialNo, m_channelIndices[a]);
                if ((s & 0x80000000) == 0)
                {
                    retval += ito::RetVal::format(ito::retError, 0, "axis %i is disabled.", a);
                }
            }
        }
    }

    if (!retval.containsError())
    {
        for (int i = 0; i < axis.size(); ++i)
        {
            idx = axis[i];

            if (controlMode[idx] && hasFeedback[idx]) //set in mm
            {
                if (pos[i] < 0 || pos[i] > maximumTravelRange[idx])
                {
                    retval += ito::RetVal::format(ito::retError, 0, "target of axis %i out of bounds.", idx);
                    break;
                }
                else
                {
                    m_targetPos[idx] = pos[i];
                    //PBC_SetPosition(m_serialNo, m_channelIndices[idx], 32767 * (pos[i] / maximumTravelRange[idx]));
                }
            }
            else
            {
                if (pos[i] < -maximumVoltage[idx] || pos[i] > maximumVoltage[idx])
                {
                    retval += ito::RetVal::format(ito::retError, 0, "target of axis %i out of bounds.", idx);
                    break;
                }
                else
                {
                    m_targetPos[idx] = pos[i];
                    //PBC_SetOutputVoltage(m_serialNo, m_channelIndices[idx], 32767 * (pos[i] / maximumVoltage[idx]));
                }

                flags += (1 << idx);
            }
        }
    }

    if (!retval.containsError())
    {
        sendTargetUpdate();

        for (int i = 0; i < axis.size(); ++i)
        {
            idx = axis[i];

            if (controlMode[idx] && hasFeedback[idx]) //set in mm
            {
                retval += checkError(PBC_SetPosition(m_serialNo, m_channelIndices[idx], 32767 * (m_targetPos[idx] / maximumTravelRange[idx])), "set position");
            }
            else
            {
                retval += checkError(PBC_SetOutputVoltage(m_serialNo, m_channelIndices[idx], 32767 * (m_targetPos[idx] / maximumVoltage[idx])), "set output voltage");
            }
        }

        if (!retval.containsError())
        {
            setStatus(axis, ito::actuatorMoving, ito::actSwitchesMask | ito::actStatusMask);
            sendStatusUpdate();

            if (m_async && waitCond)
            {
                waitCond->returnValue = retval;
                waitCond->release();
                waitCond = NULL;
            }

            retval += waitForDone(m_params["timeout"].getVal<double>() * 1000, axis, flags); //drops into timeout
        }

        if (!retval.containsError())
        {
            replaceStatus(axis, ito::actuatorMoving, ito::actuatorAtTarget);
            sendStatusUpdate();
        }

        if (!m_async && waitCond)
        {
            waitCond->returnValue = retval;
            waitCond->release();
            waitCond = NULL;
        }
    }

    if (waitCond)
    {
        waitCond->returnValue = retval;
        waitCond->release();
    }

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
/*! \detail Set the relativ position of a one axis spezified by "axis" to the position "pos" . The value in device independet in mm. 
            This function calls ThorlabsBP::SMCSetPos(axis, pos, "ABSOLUTCOMMAND")

    \param [in] axis    axis number
    \param [in] pos        relative target position in mm
    \param [in] waitCond is the semaphore (default: NULL), which is released if this method has been terminated
    \sa SMCSetPos
    \return retOk
*/
ito::RetVal ThorlabsBP::setPosRel(const int axis, const double pos, ItomSharedSemaphore *waitCond)
{
    QVector<int> axes(1, axis);
    QVector<double> positions(1, pos);
    return setPosRel(axes, positions, waitCond);

    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retval;

    if (waitCond)
    {
        waitCond->returnValue = retval;
        waitCond->release();
    }

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
/*! \detail Set the absolute position of a number of axis spezified by "axis" to the position "pos" . The value in device independet in mm.
            If the size of the vector is more then 1 element, this function returns an error.
            This function calls ThorlabsBP::SMCSetPos(axis, pos, "ABSOLUTCOMMAND")

    \param [in] axis    1 Element Vector with axis numbers
    \param [in] pos        1 Element Vector with relative positions in mm
    \param [in] waitCond is the semaphore (default: NULL), which is released if this method has been terminated
    \sa SMCSetPos
    \return retOk
*/
ito::RetVal ThorlabsBP::setPosRel(const QVector<int> axis, QVector<double> pos, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retval;
    int idx;
    double newAbsPos;
    const int* controlMode = m_params["controlMode"].getVal<int*>();
    const int* hasFeedback = m_params["hasFeedback"].getVal<int*>();
    const ito::float64* maximumTravelRange = m_params["maximumTravelRange"].getVal<ito::float64*>();
    const int* maximumVoltage = m_params["maximumVoltage"].getVal<int*>();
    int flags = 0;
    DWORD s;

    if (isMotorMoving())
    {
        retval += ito::RetVal(ito::retError, 0, tr("Any motor axis is moving. The motor is locked.").toLatin1().data());
    }
    else
    {
        foreach(int a, axis)
        {
            if (a < 0 || a >= m_numChannels)
            {
                retval += ito::RetVal::format(ito::retError, 0, "invalid axis index %i.", a);
            }
            else
            {
                s = PBC_GetStatusBits(m_serialNo, m_channelIndices[a]);
                if ((s & 0x80000000) == 0)
                {
                    retval += ito::RetVal::format(ito::retError, 0, "axis %i is disabled.", a);
                }
            }
        }

        retval += getPos(axis, m_dummyValues, NULL);
    }

    if (!retval.containsError())
    {
        for (int i = 0; i < axis.size(); ++i)
        {
            idx = axis[i];
            newAbsPos = m_currentPos[idx] + pos[i];

            if (controlMode[idx] && hasFeedback[idx]) //set in mm
            {
                if (newAbsPos < 0 || newAbsPos > maximumTravelRange[idx])
                {
                    retval += ito::RetVal::format(ito::retError, 0, "target of axis %i out of bounds.", idx);
                    break;
                }
                else
                {
                    m_targetPos[idx] = newAbsPos;
                    //PBC_SetPosition(m_serialNo, m_channelIndices[idx], 32767 * (pos[i] / maximumTravelRange[idx]));
                }
            }
            else
            {
                if (newAbsPos < -maximumVoltage[idx] || newAbsPos > maximumVoltage[idx])
                {
                    retval += ito::RetVal::format(ito::retError, 0, "target of axis %i out of bounds.", idx);
                    break;
                }
                else
                {
                    m_targetPos[idx] = newAbsPos;
                    //PBC_SetOutputVoltage(m_serialNo, m_channelIndices[idx], 32767 * (pos[i] / maximumVoltage[idx]));
                }

                flags += (1 << idx);
            }
        }
    }

    if (!retval.containsError())
    {
        sendTargetUpdate();

        for (int i = 0; i < axis.size(); ++i)
        {
            idx = axis[i];

            if (controlMode[idx] && hasFeedback[idx]) //set in mm
            {
                retval += checkError(PBC_SetPosition(m_serialNo, m_channelIndices[idx], 32767 * (m_targetPos[idx] / maximumTravelRange[idx])), "set position");
            }
            else
            {
                retval += checkError(PBC_SetOutputVoltage(m_serialNo, m_channelIndices[idx], 32767 * (m_targetPos[idx] / maximumVoltage[idx])), "set output voltage");
            }
        }

        if (!retval.containsError())
        {
            setStatus(axis, ito::actuatorMoving, ito::actSwitchesMask | ito::actStatusMask);
            sendStatusUpdate();

            if (m_async && waitCond)
            {
                waitCond->returnValue = retval;
                waitCond->release();
                waitCond = NULL;
            }

            retval += waitForDone(m_params["timeout"].getVal<double>() * 1000, axis, flags); //drops into timeout
        }

        if (!retval.containsError())
        {
            replaceStatus(axis, ito::actuatorMoving, ito::actuatorAtTarget);
            sendStatusUpdate();
        }

        if (!m_async && waitCond)
        {
            waitCond->returnValue = retval;
            waitCond->release();
            waitCond = NULL;
        }
    }

    if (waitCond)
    {
        waitCond->returnValue = retval;
        waitCond->release();
    }

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
/*! \detail This slot is triggerd by the request signal from the dockingwidged dialog to update the position after ever positioning command.
            It sends the current postion and the status to the world.

    \sa SMCSetPos
    \return retOk
*/
ito::RetVal ThorlabsBP::requestStatusAndPosition(bool sendCurrentPos, bool sendTargetPos)
{
    ito::RetVal retval(ito::retOk);

    

    if (sendCurrentPos)
    {
        QVector<int> axes(m_numChannels, 0);
        for (int i = 0; i < m_numChannels; ++i)
        {
            axes[i] = i;
        }
        retval += getPos(axes, m_dummyValues, NULL);
    }
    else
    {
        sendStatusUpdate(true);
    }

    if (sendTargetPos)
    {
        sendTargetUpdate();
    }

    return retval;
}


//----------------------------------------------------------------------------------------------------------------
ito::RetVal ThorlabsBP::waitForDone(const int timeoutMS, const QVector<int> axis /*if empty -> all axis*/, const int flags /*for your use*/)
{
    ito::RetVal retval(ito::retOk);
    bool done = false;
    Sleep(120); //to be sure that the first requested status is correct
    QVector<double> oldCurrentPos = m_currentPos;
    QElapsedTimer timer;
    timer.start();

    while (!done && !retval.containsError())
    {   
        memcpy(oldCurrentPos.data(), m_currentPos.data(), sizeof(double)*m_currentPos.size());

        done = true;
        retval += getPos(axis, m_dummyValues, NULL);

        foreach(int a, axis)
        {
            if (flags && (1 << a)) //voltage, precision is 0.05 V
            {
                if ((std::abs(m_targetPos[a] - m_currentPos[a]) > 0.05) || (std::abs(oldCurrentPos[a] - m_currentPos[a]) > 0.01))
                {
                    done = false;
                }
            }
            else //mm, precision is 50nm
            {
                if ((std::abs(m_targetPos[a] - m_currentPos[a]) > 50.0e-6) || (std::abs(oldCurrentPos[a] - m_currentPos[a]) > 10.0e-6))
                {
                    done = false;
                }
            }
        }

        if (timer.elapsed() > timeoutMS)
        {
            replaceStatus(axis, ito::actuatorMoving, ito::actuatorTimeout);
            retval += ito::RetVal(ito::retError, 0, tr("timeout occurred").toLatin1().data());
            done = true;
        }

        if (done)
        {   // Position reached and movement done
            replaceStatus(axis, ito::actuatorMoving, ito::actuatorAtTarget);
            sendStatusUpdate(true);
            break;
        }
        else
        {
            sendStatusUpdate(false);
        }

        if (!done)
        {
            if (!retval.containsError())
            {
                setAlive();
            }

            Sleep(120);
        }
    }

    if (retval.containsError())
    {
        replaceStatus(axis, ito::actuatorMoving, ito::actuatorUnknown);
        sendStatusUpdate(true);
    }

    return retval;
}

//---------------------------------------------------------------------------------------------------------------------------------- 
void ThorlabsBP::dockWidgetVisibilityChanged(bool visible)
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
ito::RetVal ThorlabsBP::checkError(short value, const char* message)
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
