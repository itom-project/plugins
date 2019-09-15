/* ********************************************************************
    Plugin "NI-DAQmx" for itom software
    URL: http://www.uni-stuttgart.de/ito
    Copyright (C) 2018, Institut fuer Technische Optik (ITO),setT
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



// includes
#include <NI-PeripheralClasses.h>

//*****************************************//
//   NiBaseChannel Class implementations   //
//*****************************************//

//------------------------------------------------------------------------------------------------------
// Constructor
NiBaseChannel::NiBaseChannel(const QString &physicalName, NiChannelType channelType, NiChannelIoType ioType) :
    m_chIo(ioType),
    m_chType(channelType),
    m_physicalName(physicalName)
{
}

//------------------------------------------------------------------------------------------------------
// Destructor
NiBaseChannel::~NiBaseChannel()
{

}


//************************************************//
//   NiAnalogInputChannel Class implementations   //
//************************************************//

//------------------------------------------------------------------------------------------------------
// Constructor, protected
NiAnalogInputChannel::NiAnalogInputChannel(const QString &physicalName) :
    NiBaseChannel(physicalName, NiBaseChannel::ChTypeAnalog, NiBaseChannel::ChIoInput)
{
}

//------------------------------------------------------------------------------------------------------
// Destructor
NiAnalogInputChannel::~NiAnalogInputChannel()
{

}

//------------------------------------------------------------------------------------------------------
/*static*/ NiBaseChannel* NiAnalogInputChannel::fromConfigurationString(const QString &configString, ito::RetVal &retValue)
{
    // (dev-channel,inConfig, MinOutputVoltage, MaxOutputVoltage)
    QRegExp regExp(QString("^(\\w+)/(\\w+),([0-%1]),([+-]?\\d+),([+-]?\\d+)$").arg(NiAnalogInputChannel::NiAnInConfEndValue - 1));
    if (regExp.indexIn(configString) == -1)
    {
        retValue += ito::RetVal::format(ito::retError, 0, "Errorneous digital input channel format '%s'. Required format: device/channel,configMode [0-%i],minOutputVoltage,maxOutputVoltage",
            configString.toLatin1().data(), NiAnalogInputChannel::NiAnInConfEndValue - 1);
        return NULL;
    }
    else
    {
        QString physicalName = regExp.cap(1) + "/" + regExp.cap(2);

        if (physicalName.contains(":"))
        {
            retValue += ito::RetVal(ito::retError, 0, "A colon (:) in the physical channel name (range of channels) is currently not supported");
            return NULL;
        }

        int inConfig = regExp.cap(3).toInt();
        int minOutputVoltage = regExp.cap(4).toInt();
        int maxOutputVoltage = regExp.cap(5).toInt();
        NiAnalogInputChannel *ai = new NiAnalogInputChannel(physicalName);
        ai->setAnalogInputConfig((NiAnalogInputConfig)inConfig);
        ai->setMinOutputLim(minOutputVoltage);
        ai->setMaxOutputLim(maxOutputVoltage);
        return ai;
    }
}

//------------------------------------------------------------------------------------------------------
// applyParameters
ito::RetVal NiAnalogInputChannel::addChannelToTask(TaskHandle taskHandle)
{
    ito::RetVal retValue;

    int config = 0;
    switch (m_analogInputConfig)
    {
        case NiAnInConfDefault:
        {
            config = DAQmx_Val_Cfg_Default;
            break;
        }
        case NiAnInConfDifferential:
        {
            config = DAQmx_Val_Diff;
            break;
        }
        case NiAnInConfRSE:
        {
            config = DAQmx_Val_RSE;
            break;
        }
        case NiAnInConfNRSE:
        {
            config = DAQmx_Val_NRSE;
            break;
        }
        case NiAnInConfPseudoDiff:
        {
            config = DAQmx_Val_PseudoDiff;
            break;
        }
        default:
        {
            retValue += ito::RetVal::format(ito::retError, 0, "NiAnalogInputChannel::addChannelToTask: Configmode %i is not in range of 0 to %i", m_analogInputConfig, NiAnalogInputConfig::NiAnInConfEndValue - 1);
        }
    }

    if (!retValue.containsError())
    {
        // TODO: Check if parameters are in Range and min is smaller than max
        QByteArray name = physicalName().toLatin1();
        int err = DAQmxCreateAIVoltageChan(taskHandle, name.constData(), "", config, m_minOutputLim, m_maxOutputLim, DAQmx_Val_Volts, NULL);
        retValue += checkError(err, "NiAnalogInputChannel::addChannelToTask: NI routine reported a create channel abnormality -");
    }

    return retValue;
}

//------------------------------------------------------------------------------------------------------
// returnParameters
QString NiAnalogInputChannel::getConfigurationString() const
{
    QString config = m_physicalName + QString(",%1,%2,%3").arg(m_analogInputConfig).arg(m_minOutputLim).arg(m_maxOutputLim);
    return config;
}

//************************************************//
//   NiAnalogOutputChannel Class implementations   //
//************************************************//

//------------------------------------------------------------------------------------------------------
// Constructor
NiAnalogOutputChannel::NiAnalogOutputChannel(const QString &physicalName) :
    NiBaseChannel(physicalName, NiBaseChannel::ChTypeAnalog, NiBaseChannel::ChIoOutput)
{
}

//------------------------------------------------------------------------------------------------------
// Destructor
NiAnalogOutputChannel::~NiAnalogOutputChannel()
{

}

//------------------------------------------------------------------------------------------------------
/*static*/ NiBaseChannel* NiAnalogOutputChannel::fromConfigurationString(const QString &configString, ito::RetVal &retValue)
{
    // (dev-channel,minInLim,maxInLim)
    QRegExp regExp(QString("^(\\w+)/(\\w+),([+-]?\\d+),([+-]?\\d+)$"));
    if (regExp.indexIn(configString) == -1)
    {
        retValue += ito::RetVal::format(ito::retError, 0, "Errorneous analog output channel format '%s'. Required format: device/channel,minOutputVoltage,maxOutputVoltage", configString.toLatin1().data());
        return NULL;
    }
    else
    {
        QString physicalName = regExp.cap(1) + "/" + regExp.cap(2);

        if (physicalName.contains(":"))
        {
            retValue += ito::RetVal(ito::retError, 0, "A colon (:) in the physical channel name (range of channels) is currently not supported");
            return NULL;
        }

        int minOutputVoltage = regExp.cap(3).toInt();
        int maxOutputVoltage = regExp.cap(4).toInt();
        NiAnalogOutputChannel *ai = new NiAnalogOutputChannel(physicalName);
        ai->setMinOutputLim(minOutputVoltage);
        ai->setMaxOutputLim(maxOutputVoltage);
        return ai;
    }
}

//------------------------------------------------------------------------------------------------------
// applyParameters
ito::RetVal NiAnalogOutputChannel::addChannelToTask(TaskHandle taskHandle)
{
    ito::RetVal retValue;

    // TODO: Check if parameters are in Range and min is smaller than max
    QByteArray name = physicalName().toLatin1();
    int err = DAQmxCreateAOVoltageChan(taskHandle, name.constData(), "", m_minOutputLim, m_maxOutputLim, DAQmx_Val_Volts, NULL);
    retValue += checkError(err, "NiAnalogOutputChannel::addChannelToTask: NI function returend create channel abnormality");

    return retValue;
}

//------------------------------------------------------------------------------------------------------
// returnParameters
QString NiAnalogOutputChannel::getConfigurationString() const
{
    QString config = m_physicalName + QString(",%1,%2").arg(m_minOutputLim).arg(m_maxOutputLim);
    return config;
}

//************************************************//
//   NiDigitalInputChannel Class implementations   //
//************************************************//

//------------------------------------------------------------------------------------------------------
// Constructor
NiDigitalInputChannel::NiDigitalInputChannel(const QString &physicalName) :
    NiBaseChannel(physicalName, NiBaseChannel::ChTypeDigital, NiBaseChannel::ChIoInput)
{
}

//------------------------------------------------------------------------------------------------------
// Destructor
NiDigitalInputChannel::~NiDigitalInputChannel()
{
}

//------------------------------------------------------------------------------------------------------
/*static*/ NiBaseChannel* NiDigitalInputChannel::fromConfigurationString(const QString &configString, ito::RetVal &retValue)
{
    // (dev-channel)
    if (configString == "" || configString.contains(","))
    {
        retValue += ito::RetVal::format(ito::retError, 0, "Errorneous digital input channel format '%s'. Required format: device/channel", configString.toLatin1().data());
        return NULL;
    }
    else
    {
        QString physicalName = configString;

        if (physicalName.contains(":"))
        {
            retValue += ito::RetVal(ito::retError, 0, "A colon (:) in the physical channel name (range of channels) is currently not supported");
            return NULL;
        }

        NiDigitalInputChannel *ai = new NiDigitalInputChannel(physicalName);
        return ai;
    }
}

//------------------------------------------------------------------------------------------------------
// applyParameters
ito::RetVal NiDigitalInputChannel::addChannelToTask(TaskHandle taskHandle)
{
    ito::RetVal retValue;
    QByteArray name = physicalName().toLatin1();
    int err = DAQmxCreateDIChan(taskHandle, name.constData(), "", DAQmx_Val_ChanForAllLines);
    retValue += checkError(err, "NiDigitalInputChannel::addChannelToTask: NI function returned create channel abnormality");
    return retValue;
}

//------------------------------------------------------------------------------------------------------
// returnParameters
QString NiDigitalInputChannel::getConfigurationString() const
{
    return m_physicalName; //the digital input channel has no further configurations (besides the physical name)
}

//************************************************//
//  NiDigitalOutputChannel Class implementations  //
//************************************************//

//------------------------------------------------------------------------------------------------------
// Constructor
NiDigitalOutputChannel::NiDigitalOutputChannel(const QString &physicalName) :
    NiBaseChannel(physicalName, NiBaseChannel::ChTypeDigital, NiBaseChannel::ChIoOutput)
{
}

//------------------------------------------------------------------------------------------------------
// Destructor
NiDigitalOutputChannel::~NiDigitalOutputChannel()
{
}

//------------------------------------------------------------------------------------------------------
/*static*/ NiBaseChannel* NiDigitalOutputChannel::fromConfigurationString(const QString &configString, ito::RetVal &retValue)
{
    // (dev-channel)
    if (configString == "" || configString.contains(","))
    {
        retValue += ito::RetVal::format(ito::retError, 0, "Errorneous digital output channel format '%s'. Required format: device/channel", configString.toLatin1().data());
        return NULL;
    }
    else
    {
        QString physicalName = configString;

        if (physicalName.contains(":"))
        {
            retValue += ito::RetVal(ito::retError, 0, "A colon (:) in the physical channel name (range of channels) is currently not supported");
            return NULL;
        }

        NiDigitalOutputChannel *ai = new NiDigitalOutputChannel(physicalName);
        return ai;
    }
}

//------------------------------------------------------------------------------------------------------
// applyParameters
ito::RetVal NiDigitalOutputChannel::addChannelToTask(TaskHandle taskHandle)
{
    ito::RetVal retValue;
    QByteArray name = physicalName().toLatin1();
    int err = DAQmxCreateDOChan(taskHandle, name.constData(), "", DAQmx_Val_ChanForAllLines);
    retValue += checkError(err, "NiDigitalOutputChannel::addChannelToTask: NI function returned create channel abnormality");
    return retValue;
}

//------------------------------------------------------------------------------------------------------
// returnParameters
QString NiDigitalOutputChannel::getConfigurationString() const
{
    return m_physicalName; //the digital output channel has no further configurations (besides the physical name)
}