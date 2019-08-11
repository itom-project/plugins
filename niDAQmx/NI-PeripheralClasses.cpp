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

//------------------------------------------------------------------------------------------------------
// Constructor
NiTask::NiTask(QString name)
{
    m_name = name;
    m_task = NULL;
    this->resetTaskHandle();
    m_mode = niTaskModeFinite;
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
}

//------------------------------------------------------------------------------------------------------
//QList<NiBaseChannel*> NiTask::getChannelPointer()
//{
//
//}

//------------------------------------------------------------------------------------------------------
void NiTask::channelAdded(const QString &name)
{
    m_chList.append(name);
}

//------------------------------------------------------------------------------------------------------
// sets the task in the right mode
ito::RetVal NiTask::applyParameters()
{
    ito::RetVal retval = ito::retOk;
    int err = 0;
    switch(m_mode)
    {
        // TODO: Up to now, all tasks are finite
        case niTaskModeFinite:
        {
            // Notice that the onboardclock is not supported for DigitalIn. Either you implement it, or do not use it.
            err = DAQmxCfgSampClkTiming(m_task, NULL /*OnboardClock*/, m_rateHz, DAQmx_Val_Rising, DAQmx_Val_FiniteSamps, m_samplesToRW);
            break;
        }
        case niTaskModeContinuous:
        {
            err = DAQmxCfgSampClkTiming(m_task, NULL /*OnboardClock*/, m_rateHz, DAQmx_Val_Rising, DAQmx_Val_ContSamps, m_samplesToRW);
            break;
        }
        case niTaskModeOnDemand: // Hardware Timed Single Point
        {
            err = DAQmxCfgSampClkTiming(m_task, NULL /*OnboardClock*/, m_rateHz, DAQmx_Val_Rising, DAQmx_Val_HWTimedSinglePoint, m_samplesToRW);
            break;
        }
        default:
        {
            // TODO: throw an error!
            retval += ito::RetVal::format(ito::retError, 0, "NiTask::applyParameters: Task mode %i is undefined. \n", m_mode);
        }
    }
    // If a triggerport is defined, set task to triggermode
    if (this->getTriggerPort() != "")
    {
        err = DAQmxCfgDigEdgeStartTrig(m_task, this->getTriggerPort().toLatin1().data(), this->getTriggerEdge());
    }

    retval += checkError(err, "NiTask::applyParameters: NI function returned configure channel abnormality");

    return retval;
}

//--------------------------------------------------------------------------------
ito::RetVal NiTask::resetTaskHandle()
{
    ito::RetVal retval;
    if (m_task != NULL)
    {    // if task was already in use, reset everything
        this->free();
        m_rateHz = 0;
        m_samplesToRW = 0;
        m_mode = -1;
        m_chList.clear();
        m_triggerPort = "";
        m_triggerEdge = 0;
    }
    int err = DAQmxCreateTask(m_name.toLatin1(), &m_task);

    retval += checkError(err, "NiTask::resetTaskHandle: NI function returned create task abnormality");

    m_chCount = 0;
    return retval;
}

//--------------------------------------------------------------------------------
ito::RetVal NiTask::run()
{
    ito::RetVal retVal = ito::retOk;
    int err = 0;

    // getIgnoreTaskParamsInitialization() should be true only when unittesting error reporting functionality

    if ( this->getTaskParamsInitialized() || this->getIgnoreTaskParamsInitialization() )
    {
         if (!retVal.containsError())
         {
             err = DAQmxStartTask(*this->getTaskHandle());

	     retVal += checkError(err, "NiTask::run: NI function returned start task abnormality");
         }
    }
    else
    {
        retVal += ito::RetVal(ito::retError, 0, "NiTask::run: Task cannot be started, since it is not initialized");
    }
    
    return retVal;
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
    ito::RetVal retVal;
    retVal = DAQmxClearTask(*this->getTaskHandle());
    return retVal;
}

//*****************************************//
//   NiBaseChannel Class implementations   //
//*****************************************//

//------------------------------------------------------------------------------------------------------
// Constructor
NiBaseChannel::NiBaseChannel(const QString virtChannelName)
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
// Constructor
NiAnalogInputChannel::NiAnalogInputChannel()
{
    this->setChType(NiBaseChannel::chTypeAnalog);
    this->setIoType(NiBaseChannel::chIoInput);
}

//------------------------------------------------------------------------------------------------------
// Destructor
NiAnalogInputChannel::~NiAnalogInputChannel()
{

}

//------------------------------------------------------------------------------------------------------
// applyParameters
ito::RetVal NiAnalogInputChannel::applyParameters(NiTask *task)
{
    ito::RetVal retval;
    QString physName = m_devId + "/" + m_chId;
    int config = 0;
    switch(m_analogInputConfig)
    {
        case niAnInConfDefault:
        {
            config = DAQmx_Val_Cfg_Default;
            break;
        }
        case niAnInConfDifferential:
        {
            config = DAQmx_Val_Diff ;
            break;
        }
        case niAnInConfRSE:
        {
            config = DAQmx_Val_RSE;
            break;
        }
        case niAnInConfNRSE:
        {
            config = DAQmx_Val_NRSE;
            break;
        }
        case niAnInConfPseudoDiff:
        {
            config = DAQmx_Val_PseudoDiff;
            break;
        }
        default:
        {
            retval += ito::RetVal::format(ito::retError, 0, "NiAnalogInputChannel::applyParameters: Configmode %i is not in range of 0 to %i", m_analogInputConfig, NiAnalogInputConfig::niAnInConfEndValue - 1);
        }
    }

    if (!retval.containsError())
    {
        // getIgnoreTaskParamsInitialization() should return true only if setParam(configForTesting) is given with the flag ignoreTaskParamInit
        if (!(task->getTaskParamsInitialized()) && !(task->getIgnoreTaskParamsInitialization()))
        {
            retval += ito::RetVal(ito::retError, 0, "NiAnalogInputChannel::applyParameters: task parameters must be set before setting channel parameters.");
        }
        else
        {
            int err = DAQmxCreateAIVoltageChan(*task->getTaskHandle(), physName.toLatin1().data(), m_chName.toLatin1().data(), config, m_minOutputLim, m_maxOutputLim, DAQmx_Val_Volts, NULL);
            task->channelAdded(physName);
            retval += checkError(err, "NiAnalogInputChannel::applyParameters: NI routine reported a create channel abnormality -");
        }
    }

    // It may seem odd that after the channel parameters are set, we then set the task parameters. This arises because the plugin requires
    // that task parameters are set before channel parameters. However, the underlying NI functions require the opposite. Task parameters
    // cannot be set until there is at least one device associated with the task. Otherwise the NI task call (DAQmxCfgSampClkTiming() )
    // returns an error. So, NiTask::applyParameters() is called immediately after the first call to NiAnalogInputChannel::applyParameters().
    // To ensure NiTask::applyParameters() is called only once, the boolean *task->getTaskParamsSet() is tested. If false, the call to
    // NiTask::applyParameters() is made. Otherwise, it is not.

    if ( !retval.containsError() && task->getTaskParamsInitialized() )
        {
        if (task->getTaskParamsSet() == false)
            {
            retval += task->applyParameters();
            task->setTaskParamsSet();
            }
        }

    return retval;
}

//------------------------------------------------------------------------------------------------------
// returnParameters
QStringList NiAnalogInputChannel::getParameters()
{
    QStringList p;
    p.append(this->getDevID());
    p.append(this->getChID());
    p.append(QString::number(this->getAnalogInputConfig()));
    p.append(QString::number(this->getMinOutputLim()));
    p.append(QString::number(this->getMaxOutputLim()));
    return p;
}

//************************************************//
//   NiAnalogOutputChannel Class implementations   //
//************************************************//

//------------------------------------------------------------------------------------------------------
// Constructor
NiAnalogOutputChannel::NiAnalogOutputChannel()
{
    this->setChType(NiBaseChannel::chTypeAnalog);
    this->setIoType(NiBaseChannel::chIoOutput);
}

//------------------------------------------------------------------------------------------------------
// Destructor
NiAnalogOutputChannel::~NiAnalogOutputChannel()
{

}

//------------------------------------------------------------------------------------------------------
// applyParameters
ito::RetVal NiAnalogOutputChannel::applyParameters(NiTask *task)
{
    ito::RetVal retval;
    QString physName = m_devId + "/" + m_chId;
    // TODO: Check if parameters are in Range and min is smaller than max
    int err = DAQmxCreateAOVoltageChan(*task->getTaskHandle(), physName.toLatin1().data(), m_chName.toLatin1().data(), m_minOutputLim, m_maxOutputLim, DAQmx_Val_Volts, NULL);
    task->channelAdded(physName);

    retval += checkError(err, "NiAnalogOutputChannel::applyParameters: NI function returend create channel abnormality");

    return retval;
}

//------------------------------------------------------------------------------------------------------
// returnParameters
QStringList NiAnalogOutputChannel::getParameters()
{
    QStringList p;
    p.append(this->getDevID());
    p.append(this->getChID());
    p.append(QString::number(this->getMinOutputLim()));
    p.append(QString::number(this->getMaxOutputLim()));
    return p;
}

//************************************************//
//   NiDigitalInputChannel Class implementations   //
//************************************************//

//------------------------------------------------------------------------------------------------------
// Constructor
NiDigitalInputChannel::NiDigitalInputChannel()
{
    this->setChType(NiBaseChannel::chTypeAnalog);
    this->setIoType(NiBaseChannel::chIoOutput);
}

//------------------------------------------------------------------------------------------------------
// Destructor
NiDigitalInputChannel::~NiDigitalInputChannel()
{

}

//------------------------------------------------------------------------------------------------------
// applyParameters
ito::RetVal NiDigitalInputChannel::applyParameters(NiTask *task)
{
    ito::RetVal retval;
    QString physName = m_devId + "/" + m_chId;
    // TODO: Check if parameters are in Range and min is smaller than max
    int err = DAQmxCreateDIChan(*task->getTaskHandle(), physName.toLatin1().data(), m_chName.toLatin1().data(), DAQmx_Val_ChanForAllLines);
    task->channelAdded(physName);


    retval += checkError(err, "NiDigitalInputChannel::applyParameters: NI function returned create channel abnormality");

    return retval;
}

//------------------------------------------------------------------------------------------------------
// returnParameters
QStringList NiDigitalInputChannel::getParameters()
{
    return QStringList("notImplementedYet");
}

//************************************************//
//  NiDigitalOutputChannel Class implementations  //
//************************************************//

//------------------------------------------------------------------------------------------------------
// Constructor
NiDigitalOutputChannel::NiDigitalOutputChannel()
{
    this->setChType(NiBaseChannel::chTypeAnalog);
    this->setIoType(NiBaseChannel::chIoOutput);
}

//------------------------------------------------------------------------------------------------------
// Destructor
NiDigitalOutputChannel::~NiDigitalOutputChannel()
{

}

//------------------------------------------------------------------------------------------------------
// applyParameters
ito::RetVal NiDigitalOutputChannel::applyParameters(NiTask *task)
{
    ito::RetVal retval;
    QString physName = m_devId + "/" + m_chId;
    // TODO: Check if parameters are in Range and min is smaller than max
    int err = DAQmxCreateDOChan(*task->getTaskHandle(), physName.toLatin1().data(), m_chName.toLatin1().data(), DAQmx_Val_ChanForAllLines);
    task->channelAdded(physName);

    retval += checkError(err, "NiDigitalOutputChannel::applyParameters: NI function returned create channel abnormality");

    return retval;
}

//------------------------------------------------------------------------------------------------------
// returnParameters
QStringList NiDigitalOutputChannel::getParameters()
{
    return QStringList("notImplementedYet");
}

//*************************************//
// NiChannelList Class implementations //
//*************************************//

NiChannelList::NiChannelList()
{
    clear();
}

//------------------------------------------------------------------------------------------------------
// Constructor
NiChannelList::NiChannelList(const QString &device)
{
    clear();
    updateChannelsOfDevice(device);
}

//------------------------------------------------------------------------------------------------------
// Destructor
NiChannelList::~NiChannelList()
{

}

//------------------------------------------------------------------------------------------------------
// returns a vector of pointer, pointing to all elements of the requested type
QVector<NiBaseChannel*> NiChannelList::getAllChannelOfType(NiBaseChannel::NiChType chType)
{
    QVector<NiBaseChannel*> retVector;  
    foreach(NiBaseChannel* c, this->values())
    {
        if (c->getChType() == chType)
        {
            retVector.append(c);
        }
    }
    return retVector;
}

//------------------------------------------------------------------------------------------------------
int NiChannelList::getNrOfChannels(NiBaseChannel::NiChType chType)
{
    return getAllChannelOfType(chType).size();
}

//------------------------------------------------------------------------------------------------------
// updateChannelsOfDevice
void NiChannelList::updateChannelsOfDevice(const QString &dev)
{
    const int ARRAY_lENGTH = 5120;
    char buffer[ARRAY_lENGTH]="";
    this->clear();

    DAQmxGetDevAIPhysicalChans(qPrintable(dev), buffer, sizeof(buffer));
    foreach(const QString &s, QString(buffer).split(", "))
    {
        this->insert(s, NULL);
    }
    memset(buffer, '\0', sizeof(char)* strlen(buffer));
    DAQmxGetDevAOPhysicalChans(qPrintable(dev), buffer, sizeof(buffer));
    foreach(const QString &s, QString(buffer).split(", "))
    {
        this->insert(s, NULL);
    }
    memset(buffer, '\0', sizeof(char)* strlen(buffer));
    DAQmxGetDevDIPorts(qPrintable(dev), buffer, sizeof(buffer));
    foreach(const QString &s, QString(buffer).split(", "))
    {
        this->insert(s, NULL);
    }
    memset(buffer, '\0', sizeof(char)* strlen(buffer));
    DAQmxGetDevDOPorts(qPrintable(dev), buffer, sizeof(buffer));
    foreach(const QString &s, QString(buffer).split(", "))
    {
        this->insert(s, NULL);
    }
    memset(buffer, '\0', sizeof(char)* strlen(buffer));
    DAQmxGetDevCIPhysicalChans(qPrintable(dev), buffer, sizeof(buffer));
    foreach(const QString &s, QString(buffer).split(", "))
    {
        this->insert(s, NULL);
    }
    memset(buffer, '\0', sizeof(char)* strlen(buffer));
    DAQmxGetDevCOPhysicalChans(qPrintable(dev), buffer, sizeof(buffer));
    foreach(const QString &s, QString(buffer).split(", "))
    {
        this->insert(s, NULL);
    }
}

//------------------------------------------------------------------------------------------------------
int NiChannelList::getNrOfOutputs()
{  
    int count = 0;
    foreach(NiBaseChannel* c, this->values())
    {
        if (c->getIoType() == NiBaseChannel::chIoOutput)
        {
            ++count;
        }
    }
    return count;
}

//------------------------------------------------------------------------------------------------------
int NiChannelList::getNrOfInputs()
{
    int count = 0;
    foreach(NiBaseChannel* c, this->values())
    {
        if (c->getIoType() == NiBaseChannel::chIoInput)
        {
            ++count;
        }
    }
    return count;
}

//------------------------------------------------------------------------------------------------------
QStringList NiChannelList::getAllChannelAsString()
{
    QStringList sl;
    foreach(const QString &c, this->keys())
    {
        sl.append(c);
    }
    return sl;
}

//------------------------------------------------------------------------------------------------------
QStringList NiChannelList::getAllChParameters(NiBaseChannel::NiChType type,  NiBaseChannel::NiChIoType io)
{
    QStringList params;

    foreach(NiBaseChannel* c, this->values())
    {
        if (c != NULL)
        {
            if (c->getChType() == type && c->getIoType() == io)
            {
                params.append(c->getParameters().join(","));
            }
        }
    }

    return params;
}
        
////------------------------------------------------------------------------------------------------------
//void NiChannelList::setChannelType(QString hardwareCh, NiBaseChannel::NiChIoType io, NiBaseChannel::NiChType type)
//{
//
//}

//------------------------------------------------------------------------------------------------------

//void NiChannelList::setMultipleChannelType(int idx, NiBaseChannel::NiChIoType io, NiBaseChannel::NiChType type)
//{
//
//}

