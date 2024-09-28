/* ********************************************************************
    Plugin "NI-DAQmx" for itom software
    URL: http://www.uni-stuttgart.de/ito
    Copyright (C) 2018, Institut für Technische Optik (ITO),setT
    Universität Stuttgart, Germany

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
#include <qregularexpression.h>
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
    NiBaseChannel(physicalName, NiBaseChannel::ChTypeAnalog, NiBaseChannel::ChIoInput),
    m_maxInputLim(0.0),
    m_minInputLim(0.0),
    m_terminalConfig(NiTerminalConfDefault)
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
    QRegularExpression regExp(QString("^(\\w+)/(\\w+),([0-%1]),([+-]?\\d+\\.?\\d*),([+-]?\\d+\\.?\\d*)$").arg(NiAnalogInputChannel::NiTerminalConfEndValue - 1));
    QRegularExpressionMatch match = regExp.match(configString);
    if (!match.hasMatch())
    {
        retValue += ito::RetVal::format(ito::retError, 0, "Erroneous digital input channel format '%s'. Required format: device/channel,terminalMode [0-%i],minInputVoltage,maxInputVoltage",
            configString.toLatin1().data(), NiAnalogInputChannel::NiTerminalConfEndValue - 1);
        return NULL;
    }
    else
    {
        QString physicalName = match.captured(1) + "/" + match.captured(2);

        if (physicalName.contains(":"))
        {
            retValue += ito::RetVal(ito::retError, 0, "A colon (:) in the physical channel name (range of channels) is currently not supported");
            return NULL;
        }

        int inConfig = match.captured(3).toInt();
        double minInputVoltage = match.captured(4).toDouble();
        double maxInputVoltage = match.captured(5).toDouble();
        NiAnalogInputChannel *ai = new NiAnalogInputChannel(physicalName);
        ai->setTerminalConfig((NiAITerminalConfig)inConfig);
        ai->setMinInputLim(minInputVoltage);
        ai->setMaxInputLim(maxInputVoltage);
        return ai;
    }
}

//------------------------------------------------------------------------------------------------------
// applyParameters
ito::RetVal NiAnalogInputChannel::addChannelToTask(TaskHandle taskHandle)
{
    ito::RetVal retValue;

    int config = 0;
    switch (m_terminalConfig)
    {
        case NiTerminalConfDefault:
        {
            config = DAQmx_Val_Cfg_Default;
            break;
        }
        case NiTerminalConfDifferential:
        {
            config = DAQmx_Val_Diff;
            break;
        }
        case NiTerminalConfRSE:
        {
            config = DAQmx_Val_RSE;
            break;
        }
        case NiTerminalConfNRSE:
        {
            config = DAQmx_Val_NRSE;
            break;
        }
        case NiTerminalConfPseudoDiff:
        {
            config = DAQmx_Val_PseudoDiff;
            break;
        }
        default:
        {
            retValue += ito::RetVal::format(ito::retError, 0, "NiAnalogInputChannel::addChannelToTask: TerminalMode %i is not in range of 0 to %i", m_terminalConfig, NiAITerminalConfig::NiTerminalConfEndValue - 1);
        }
    }

    if (!retValue.containsError())
    {
        // TODO: Check if parameters are in Range and min is smaller than max
        QByteArray name = physicalName().toLatin1();
        int err = DAQmxCreateAIVoltageChan(taskHandle, name.constData(), "", config, m_minInputLim, m_maxInputLim, DAQmx_Val_Volts, NULL);
        retValue += checkError(err, "NiAnalogInputChannel::addChannelToTask: NI routine reported a create channel abnormality -");
    }

    return retValue;
}

//------------------------------------------------------------------------------------------------------
// returnParameters
QString NiAnalogInputChannel::getConfigurationString() const
{
    QString config = m_physicalName + QString(",%1,%2,%3").arg(m_terminalConfig).arg(m_minInputLim).arg(m_maxInputLim);
    return config;
}

//************************************************//
//   NiAnalogOutputChannel Class implementations   //
//************************************************//

//------------------------------------------------------------------------------------------------------
// Constructor
NiAnalogOutputChannel::NiAnalogOutputChannel(const QString &physicalName) :
    NiBaseChannel(physicalName, NiBaseChannel::ChTypeAnalog, NiBaseChannel::ChIoOutput),
    m_maxOutputLim(0.0),
    m_minOutputLim(0.0)
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
    QRegularExpression regExp(QString("^(\\w+)/(\\w+),([+-]?\\d+\\.?\\d*),([+-]?\\d+\\.?\\d*)$"));
    QRegularExpressionMatch match = regExp.match(configString);
    if (!match.hasMatch())
    {
        retValue += ito::RetVal::format(ito::retError, 0, "Erroneous analog output channel format '%s'. Required format: device/channel,minOutputVoltage,maxOutputVoltage", configString.toLatin1().data());
        return NULL;
    }
    else
    {
        QString physicalName = match.captured(1) + "/" + match.captured(2);

        if (physicalName.contains(":"))
        {
            retValue += ito::RetVal(ito::retError, 0, "A colon (:) in the physical channel name (range of channels) is currently not supported");
            return NULL;
        }

        double minOutputVoltage = match.captured(3).toDouble();
        double maxOutputVoltage = match.captured(4).toDouble();
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
    retValue += checkError(err, "NiAnalogOutputChannel::addChannelToTask: NI function returned create channel abnormality");

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
        retValue += ito::RetVal::format(ito::retError, 0, "Erroneous digital input channel format '%s'. Required format: device/channel", configString.toLatin1().data());
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
        retValue += ito::RetVal::format(ito::retError, 0, "Erroneous digital output channel format '%s'. Required format: device/channel", configString.toLatin1().data());
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
