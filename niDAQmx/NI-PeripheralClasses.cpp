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
//      niTask Class implementations       //
//*****************************************//

//------------------------------------------------------------------------------------------------------
// Constructor
niTask::niTask(QString name)
{
    m_name = name;
    m_task = NULL;
    this->resetTaskHandle();
    m_mode = niTaskModeFinite;
}

//------------------------------------------------------------------------------------------------------
// Destructor
niTask::~niTask()
{
    DAQmxClearTask(*this->getTaskHandle());
}

//------------------------------------------------------------------------------------------------------
bool niTask::setIgnoreTaskParamsInitialization()
{
    return m_ignoreTaskParamsInitialization = true;
}

//------------------------------------------------------------------------------------------------------
bool niTask::getIgnoreTaskParamsInitialization()
{
    return m_ignoreTaskParamsInitialization;
}

//------------------------------------------------------------------------------------------------------
bool niTask::setPassThroughToPeripheralClasses()
{
    return m_passThroughToPeripheralClasses = true;
}

//------------------------------------------------------------------------------------------------------
bool niTask::getPassThroughToPeripheralClasses()
{
    return m_passThroughToPeripheralClasses;
}

//------------------------------------------------------------------------------------------------------
bool niTask::taskParamsValid()
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
bool niTask::setTaskParamsInitialized()
{
    return m_taskParametersInitialized = true;
}

//------------------------------------------------------------------------------------------------------
bool niTask::getTaskParamsInitialized()
{
    return m_taskParametersInitialized;
}

//------------------------------------------------------------------------------------------------------
bool niTask::setTaskParamsSet()
{
    return m_taskParametersSet = true;
}

//------------------------------------------------------------------------------------------------------
bool niTask::getTaskParamsSet()
{
    return m_taskParametersSet;
}

//------------------------------------------------------------------------------------------------------
bool niTask::setChParamsInitialized()
{
    return m_chParametersInitialized = true;
}

//------------------------------------------------------------------------------------------------------
bool niTask::getChParamsInitialized()
{
    return m_chParametersInitialized;
}

//------------------------------------------------------------------------------------------------------
uInt32 niTask::getChCount()
{
    DAQmxGetTaskNumChans(*this->getTaskHandle(), &m_chCount);
    return m_chCount;
}

//------------------------------------------------------------------------------------------------------
QStringList niTask::getChList()
{
    return m_chList;
}

//------------------------------------------------------------------------------------------------------
//QList<niBaseChannel*> niTask::getChannelPointer()
//{
//
//}

//------------------------------------------------------------------------------------------------------
void niTask::channelAdded(const QString name)
{
    m_chList.append(name);
}

//------------------------------------------------------------------------------------------------------
// sets the task in the right mode
ito::RetVal niTask::applyParameters()
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
            retval += ito::RetVal::format(ito::retError, 0, "niTask::applyParameters: Task mode %i is undefined. \n", m_mode);
        }
    }
    // If a triggerport is defined, set task to triggermode
    if (this->getTriggerPort() != "")
    {
        err = DAQmxCfgDigEdgeStartTrig(m_task, this->getTriggerPort().toLatin1().data(), this->getTriggerEdge());
    }

    retval += checkError(err, "niTask::applyParameters: NI function returned configure channel abnormality");

    return retval;
}

ito::RetVal niTask::resetTaskHandle()
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

    retval += checkError(err, "niTask::resetTaskHandle: NI function returned create task abnormality");

    m_chCount = 0;
    return retval;
}

ito::RetVal niTask::run()
{
    ito::RetVal retVal = ito::retOk;
    int err = 0;

    // getIgnoreTaskParamsInitialization() should be true only when unittesting error reporting functionality

    if ( this->getTaskParamsInitialized() || this->getIgnoreTaskParamsInitialization() )
    {
         if (!retVal.containsError())
         {
             err = DAQmxStartTask(*this->getTaskHandle());

	     retVal += checkError(err, "niTask::run: NI function returned start task abnormality");
         }
    }
    else
    {
        retVal += ito::RetVal(ito::retError, 0, "niTask::run: Task cannot be started, since it is not initialized");
    }
    
    return retVal;
}

bool niTask::isDone()
{
    bool32 done;
    DAQmxIsTaskDone(*this->getTaskHandle(), &done);
    return done;
}

ito::RetVal niTask::stop()
{
    return DAQmxStopTask(*this->getTaskHandle());
}

ito::RetVal niTask::free()
{
    ito::RetVal retVal;
    retVal = DAQmxClearTask(*this->getTaskHandle());
    return retVal;
}

//*****************************************//
//   niBaseChannel Class implementations   //
//*****************************************//

//------------------------------------------------------------------------------------------------------
// Constructor
niBaseChannel::niBaseChannel(const QString virtChannelName)
{

}

//------------------------------------------------------------------------------------------------------
// Destructor
niBaseChannel::~niBaseChannel()
{

}


//************************************************//
//   niAnalogInputChannel Class implementations   //
//************************************************//

//------------------------------------------------------------------------------------------------------
// Constructor
niAnalogInputChannel::niAnalogInputChannel()
{
    this->setChType(niBaseChannel::chTypeAnalog);
    this->setIoType(niBaseChannel::chIoInput);
}

//------------------------------------------------------------------------------------------------------
// Destructor
niAnalogInputChannel::~niAnalogInputChannel()
{

}

//------------------------------------------------------------------------------------------------------
// applyParameters
ito::RetVal niAnalogInputChannel::applyParameters(niTask *task)
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
            retval += ito::RetVal::format(ito::retError, 0, "niAnalogInputChannel::applyParameters: Configmode %i is not in range of 0 to 4", m_analogInputConfig);
        }
    }
    if( !(task->getTaskParamsInitialized()) )
    {
        retval += ito::RetVal(ito::retError, 0, "niAnalogInputChannel::applyParameters: task parameters must be set before setting channel parameters.");
    }
	
	// getIgnoreTaskParamsInitialization() should return true only if setParam(configForTesting) is given with the flag ignoreTaskParamInit

	else if ( ((m_analogInputConfig >= 0 && m_analogInputConfig <= 4) || task->getIgnoreTaskParamsInitialization()) && !retval.containsError() )
    {
        int err = DAQmxCreateAIVoltageChan(*task->getTaskHandle(), physName.toLatin1().data(), m_chName.toLatin1().data(), config, m_minOutputLim, m_maxOutputLim, DAQmx_Val_Volts, NULL);
        task->channelAdded(physName);
        retval += checkError(err, "niAnalogInputChannel::applyParameters: NI routine reported a create channel abnormality -");
    }

    // It may seem odd that after the channel parameters are set, we then set the task parameters. This arises because the plugin requires
    // that task parameters are set before channel parameters. However, the underlying NI functions require the opposite. Task parameters
    // cannot be set until there is at least one device associated with the task. Otherwise the NI task call (DAQmxCfgSampClkTiming() )
    // returns an error. So, niTask::applyParameters() is called immediately after the first call to niAnalogInputChannel::applyParameters().
    // To ensure niTask::applyParameters() is called only once, the boolean *task->getTaskParamsSet() is tested. If false, the call to
    // niTask::applyParameters() is made. Otherwise, it is not.

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
QStringList niAnalogInputChannel::getParameters()
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
//   niAnalogOutputChannel Class implementations   //
//************************************************//

//------------------------------------------------------------------------------------------------------
// Constructor
niAnalogOutputChannel::niAnalogOutputChannel()
{
    this->setChType(niBaseChannel::chTypeAnalog);
    this->setIoType(niBaseChannel::chIoOutput);
}

//------------------------------------------------------------------------------------------------------
// Destructor
niAnalogOutputChannel::~niAnalogOutputChannel()
{

}

//------------------------------------------------------------------------------------------------------
// applyParameters
ito::RetVal niAnalogOutputChannel::applyParameters(niTask *task)
{
    ito::RetVal retval;
    QString physName = m_devId + "/" + m_chId;
    // TODO: Check if parameters are in Range and min is smaller than max
    int err = DAQmxCreateAOVoltageChan(*task->getTaskHandle(), physName.toLatin1().data(), m_chName.toLatin1().data(), m_minOutputLim, m_maxOutputLim, DAQmx_Val_Volts, NULL);
    task->channelAdded(physName);

    retval += checkError(err, "niAnalogOutputChannel::applyParameters: NI function returend create channel abnormality");

    return retval;
}

//------------------------------------------------------------------------------------------------------
// returnParameters
QStringList niAnalogOutputChannel::getParameters()
{
    QStringList p;
    p.append(this->getDevID());
    p.append(this->getChID());
    p.append(QString::number(this->getMinOutputLim()));
    p.append(QString::number(this->getMaxOutputLim()));
    return p;
}

//************************************************//
//   niDigitalInputChannel Class implementations   //
//************************************************//

//------------------------------------------------------------------------------------------------------
// Constructor
niDigitalInputChannel::niDigitalInputChannel()
{
    this->setChType(niBaseChannel::chTypeAnalog);
    this->setIoType(niBaseChannel::chIoOutput);
}

//------------------------------------------------------------------------------------------------------
// Destructor
niDigitalInputChannel::~niDigitalInputChannel()
{

}

//------------------------------------------------------------------------------------------------------
// applyParameters
ito::RetVal niDigitalInputChannel::applyParameters(niTask *task)
{
    ito::RetVal retval;
    QString physName = m_devId + "/" + m_chId;
    // TODO: Check if parameters are in Range and min is smaller than max
    int err = DAQmxCreateDIChan(*task->getTaskHandle(), physName.toLatin1().data(), m_chName.toLatin1().data(), DAQmx_Val_ChanForAllLines);
    task->channelAdded(physName);


    retval += checkError(err, "niDigitalInputChannel::applyParameters: NI function returned create channel abnormality");

    return retval;
}

//------------------------------------------------------------------------------------------------------
// returnParameters
QStringList niDigitalInputChannel::getParameters()
{
    return QStringList("notImplementedYet");
}

//************************************************//
//  niDigitalOutputChannel Class implementations  //
//************************************************//

//------------------------------------------------------------------------------------------------------
// Constructor
niDigitalOutputChannel::niDigitalOutputChannel()
{
    this->setChType(niBaseChannel::chTypeAnalog);
    this->setIoType(niBaseChannel::chIoOutput);
}

//------------------------------------------------------------------------------------------------------
// Destructor
niDigitalOutputChannel::~niDigitalOutputChannel()
{

}

//------------------------------------------------------------------------------------------------------
// applyParameters
ito::RetVal niDigitalOutputChannel::applyParameters(niTask *task)
{
    ito::RetVal retval;
    QString physName = m_devId + "/" + m_chId;
    // TODO: Check if parameters are in Range and min is smaller than max
    int err = DAQmxCreateDOChan(*task->getTaskHandle(), physName.toLatin1().data(), m_chName.toLatin1().data(), DAQmx_Val_ChanForAllLines);
    task->channelAdded(physName);

    retval += checkError(err, "niDigitalOutputChannel::applyParameters: NI function returned create channel abnormality");

    return retval;
}

//------------------------------------------------------------------------------------------------------
// returnParameters
QStringList niDigitalOutputChannel::getParameters()
{
    return QStringList("notImplementedYet");
}

//*************************************//
// niChannelList Class implementations //
//*************************************//

//------------------------------------------------------------------------------------------------------
// Constructor
niChannelList::niChannelList(QString device)
{
    getChannelsOfDevice(device);
}

//------------------------------------------------------------------------------------------------------
// Destructor
niChannelList::~niChannelList()
{

}

//------------------------------------------------------------------------------------------------------
// returns a vector of pointer, pointing to all elements of the requested type
QVector<niBaseChannel*> niChannelList::getAllChannelOfType(niBaseChannel::niChType chType)
{
    QVector<niBaseChannel*> retVector;  
    foreach(niBaseChannel* c, this->values())
    {
        if (c->getChType() == chType)
        {
            retVector.append(c);
        }
    }
    return retVector;
}

//------------------------------------------------------------------------------------------------------
int niChannelList::getNrOfChannels(niBaseChannel::niChType chType)
{
    return getAllChannelOfType(chType).size();
}

//------------------------------------------------------------------------------------------------------
// getChannelsOfDevice
void niChannelList::getChannelsOfDevice(const QString dev)
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
int niChannelList::getNrOfOutputs()
{  
    int count = 0;
    foreach(niBaseChannel* c, this->values())
    {
        if (c->getIoType() == niBaseChannel::chIoOutput)
        {
            ++count;
        }
    }
    return count;
}

//------------------------------------------------------------------------------------------------------
int niChannelList::getNrOfInputs()
{
    int count = 0;
    foreach(niBaseChannel* c, this->values())
    {
        if (c->getIoType() == niBaseChannel::chIoInput)
        {
            ++count;
        }
    }
    return count;
}

//------------------------------------------------------------------------------------------------------
QStringList niChannelList::getAllChannelAsString()
{
    QStringList sl;
    foreach(const QString &c, this->keys())
    {
        sl.append(c);
    }
    return sl;
}

//------------------------------------------------------------------------------------------------------
QStringList niChannelList::getAllChParameters(niBaseChannel::niChType type,  niBaseChannel::niChIoType io)
{
    QStringList params;

    foreach(niBaseChannel* c, this->values())
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
//void niChannelList::setChannelType(QString hardwareCh, niBaseChannel::niChIoType io, niBaseChannel::niChType type)
//{
//
//}

//------------------------------------------------------------------------------------------------------

//void niChannelList::setMultipleChannelType(int idx, niBaseChannel::niChIoType io, niBaseChannel::niChType type)
//{
//
//}

