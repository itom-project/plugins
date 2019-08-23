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
//      NiTask Class implementations       //
//*****************************************//

/*
//------------------------------------------------------------------------------------------------------
// Constructor
NiTask::NiTask(QString name)
{
    m_name = name;
    m_task = NULL;
    this->resetTaskHandle();
    m_mode = NiTaskModeFinite;
}

//------------------------------------------------------------------------------------------------------
// Destructor
NiTask::~NiTask()
{
    DAQmxClearTask(*this->getTaskHandle());
}

//------------------------------------------------------------------------------------------------------
bool NiTask::setIgnoreTaskParamsInitialization()
{
    return m_ignoreTaskParamsInitialization = true;
}

//------------------------------------------------------------------------------------------------------
bool NiTask::getIgnoreTaskParamsInitialization()
{
    return m_ignoreTaskParamsInitialization;
}

//------------------------------------------------------------------------------------------------------
bool NiTask::setPassThroughToPeripheralClasses()
{
    return m_passThroughToPeripheralClasses = true;
}

//------------------------------------------------------------------------------------------------------
bool NiTask::getPassThroughToPeripheralClasses()
{
    return m_passThroughToPeripheralClasses;
}

//------------------------------------------------------------------------------------------------------
bool NiTask::taskParamsValid()
{
    if ( !getIgnoreTaskParamsInitialization() && ((m_taskParametersInitialized == false) || (m_rateHz < 1 || m_samplesToRW < 1 || m_mode < 0 || m_mode > 2)) )
    {
        return false;
    }
    else
    {
        return true;
    }
}

//------------------------------------------------------------------------------------------------------
bool NiTask::setTaskParamsInitialized()
{
    return m_taskParametersInitialized = true;
}

//------------------------------------------------------------------------------------------------------
bool NiTask::getTaskParamsInitialized()
{
    return m_taskParametersInitialized;
}

//------------------------------------------------------------------------------------------------------
bool NiTask::setTaskParamsSet()
{
    return m_taskParametersSet = true;
}

//------------------------------------------------------------------------------------------------------
bool NiTask::getTaskParamsSet()
{
    return m_taskParametersSet;
}

//------------------------------------------------------------------------------------------------------
bool NiTask::setChParamsInitialized()
{
    return m_chParametersInitialized = true;
}

//------------------------------------------------------------------------------------------------------
bool NiTask::getChParamsInitialized()
{
    return m_chParametersInitialized;
}

//------------------------------------------------------------------------------------------------------
uInt32 NiTask::getChCount()
{
    DAQmxGetTaskNumChans(*this->getTaskHandle(), &m_chCount);
    return m_chCount;
}

//------------------------------------------------------------------------------------------------------
QStringList NiTask::getChList()
{
    return m_chList;
}*/

//------------------------------------------------------------------------------------------------------
//QList<NiBaseChannel*> NiTask::getChannelPointer()
//{
//
//}

/*
//------------------------------------------------------------------------------------------------------
void NiTask::channelAdded(const QString &name)
{
    m_chList.append(name);
}

////------------------------------------------------------------------------------------------------------
//// sets the task in the right mode
//ito::RetVal NiTask::applyParameters()
//{
//    ito::RetVal retValue = ito::retOk;
//    int err = 0;
//    switch(m_mode)
//    {
//        // TODO: Up to now, all tasks are finite
//        case NiTaskModeFinite:
//        {
//            // Notice that the onboardclock is not supported for DigitalIn. Either you implement it, or do not use it.*/
//            //err = DAQmxCfgSampClkTiming(m_task, NULL /*OnboardClock*/, m_rateHz, DAQmx_Val_Rising, DAQmx_Val_FiniteSamps, m_samplesToRW);
//            break;
//        }
//        case NiTaskModeContinuous:
//        {
//            //err = DAQmxCfgSampClkTiming(m_task, NULL /*OnboardClock*/, m_rateHz, DAQmx_Val_Rising, DAQmx_Val_ContSamps, m_samplesToRW);
//            break;
//        }
//        case NiTaskModeOnDemand: // Hardware Timed Single Point
//        {
//            //err = DAQmxCfgSampClkTiming(m_task, NULL /*OnboardClock*/, m_rateHz, DAQmx_Val_Rising, DAQmx_Val_HWTimedSinglePoint, m_samplesToRW);
//            break;
//        }
//        default:
//        {
//            // TODO: throw an error!
//            retValue += ito::RetVal::format(ito::retError, 0, "NiTask::applyParameters: Task mode %i is undefined. \n", m_mode);
//        }
//    }
//    // If a triggerport is defined, set task to triggermode
//    if (this->getTriggerPort() != "")
//    {
//        err = DAQmxCfgDigEdgeStartTrig(m_task, this->getTriggerPort().toLatin1().data(), this->getTriggerEdge());
//    }
//
//    retValue += checkError(err, "NiTask::applyParameters: NI function returned configure channel abnormality");
//
//    return retValue;
//}
/*
//--------------------------------------------------------------------------------
ito::RetVal NiTask::resetTaskHandle()
{
    ito::RetVal retValue;
    if (m_task != NULL)
    {    // if task was already in use, reset everything
        this->free();
        m_rateHz = 0;
        m_samplesToRW = 0;
        m_mode = -1;
        m_chList.clear();
    }
    int err = DAQmxCreateTask(m_name.toLatin1(), &m_task);

    retValue += checkError(err, "NiTask::resetTaskHandle: NI function returned create task abnormality");

    m_chCount = 0;
    return retValue;
}

//--------------------------------------------------------------------------------
ito::RetVal NiTask::run()
{
    ito::RetVal retValue = ito::retOk;
    int err = 0;

    // getIgnoreTaskParamsInitialization() should be true only when unittesting error reporting functionality

    if ( this->getTaskParamsInitialized() || this->getIgnoreTaskParamsInitialization() )
    {
         if (!retValue.containsError())
         {
             err = DAQmxStartTask(*this->getTaskHandle());

	     retValue += checkError(err, "NiTask::run: NI function returned start task abnormality");
         }
    }
    else
    {
        retValue += ito::RetVal(ito::retError, 0, "NiTask::run: Task cannot be started, since it is not initialized");
    }
    
    return retValue;
}

//--------------------------------------------------------------------------------
bool NiTask::isDone()
{
    bool32 done;
    DAQmxIsTaskDone(*this->getTaskHandle(), &done);
    return done;
}

//--------------------------------------------------------------------------------
ito::RetVal NiTask::stop()
{
    return DAQmxStopTask(*this->getTaskHandle());
}

//--------------------------------------------------------------------------------
ito::RetVal NiTask::free()
{
    ito::RetVal retValue;
    retValue = DAQmxClearTask(*this->getTaskHandle());
    return retValue;
}*/

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
    QRegExp regExp(QString("(\\w+)/(\\w+),([0-%1]),([+-]?\\d+),([+-]?\\d+)").arg(NiAnalogInputChannel::NiAnInConfEndValue - 1));
    if (regExp.indexIn(configString) == -1)
    {
        retValue += ito::RetVal::format(ito::retError, 0, "NiDAQmx::setParam - aiChParams. Format must be: device/channel,configMode [0-%i],minOutputVoltage,maxOutputVoltage",
            NiAnalogInputChannel::NiAnInConfEndValue - 1);
        return NULL;
    }
    else
    {
        QString physicalName = regExp.cap(1) + "/" + regExp.cap(2);
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
    QRegExp regExp(QString("(\\w+)/(\\w+),([+-]?\\d+),([+-]?\\d+)"));
    if (regExp.indexIn(configString) == -1)
    {
        retValue += ito::RetVal(ito::retError, 0, "niDAQmx::setParam - aoChParams. Format must be: device/channel,minOutputVoltage,maxOutputVoltage");
        return NULL;
    }
    else
    {
        QString physicalName = regExp.cap(1) + "/" + regExp.cap(2);
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
    QRegExp regExp(QString("(\\w+)/(\\w+)"));
    if (regExp.indexIn(configString) == -1)
    {
        retValue += ito::RetVal(ito::retError, 0, "niDAQmx::setParam - aoChParams. Format must be: device/channel");
        return NULL;
    }
    else
    {
        QString physicalName = regExp.cap(1) + "/" + regExp.cap(2);
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
    QRegExp regExp(QString("(\\w+)/(\\w+)"));
    if (regExp.indexIn(configString) == -1)
    {
        retValue += ito::RetVal(ito::retError, 0, "niDAQmx::setParam - aoChParams. Format must be: device/channel");
        return NULL;
    }
    else
    {
        QString physicalName = regExp.cap(1) + "/" + regExp.cap(2);
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