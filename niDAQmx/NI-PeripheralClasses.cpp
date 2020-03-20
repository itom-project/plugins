/* ********************************************************************
    Plugin "NI-DAQmx" for itom software
    URL: http://www.uni-stuttgart.de/ito
    Copyright (C) 2020, Institut fuer Technische Optik (ITO),
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
// Constructor I
niTask::niTask(QString name)
{
    m_name = name;
    m_task = NULL;
    this->resetTaskHandle();
    m_mode = niTaskModeFinite;
}

//------------------------------------------------------------------------------------------------------
// Destructor I
niTask::~niTask()
{
    DAQmxClearTask(*this->getTaskHandle());
}

//------------------------------------------------------------------------------------------------------
bool niTask::isInitialized()
{
    if (m_rateHz < 1 || m_samplesToRW < 1 || m_mode < 0 || m_mode > 2)
    {
        return false;
    }
    else
    {
        return true;
    }
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
        case niTaskModeOnDemand:
        {
            err = DAQmxCfgSampClkTiming(m_task, NULL /*OnboardClock*/, m_rateHz, DAQmx_Val_Rising, DAQmx_Val_HWTimedSinglePoint, m_samplesToRW);
            break;
        }
        default:
        {
            // TODO: through an error!
            retval += ito::RetVal::format(ito::retError, 0, "The given mode does not exist. \n Mode: %i", m_mode);
        }
    }
    // If a triggerport is defined, set task to triggermode
    if (this->getTriggerPort() != "")
    {
        err = DAQmxCfgDigEdgeStartTrig(m_task, this->getTriggerPort().toLatin1().data(), this->getTriggerEdge());
    }
    if (err > 0 && retval == ito::retOk)
    {
        retval += ito::RetVal::format(ito::retWarning, 0, "Warning occured while configuring channel. \n Code: %i", err);
    }
    else if (err < 0 && retval == ito::retOk)
    {
        retval += ito::RetVal::format(ito::retError, 0, "Error occured while configuring channel. \n Code: %i", err);
    }
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
    if (err > 0)
    {
        retval += ito::RetVal::format(ito::retWarning, 0, "Warning occured while creating Task. \n Code: %i", err);
    }
    else if (err < 0)
    {
        retval += ito::RetVal::format(ito::retError, 0, "Error occured while creating Task. \n Code: %i", err);
    }    
    m_chCount = 0;
    return retval;
}

ito::RetVal niTask::run()
{
    ito::RetVal retVal = ito::retOk;
    int err = 0;
    if (this->isInitialized())
    {
         retVal += this->applyParameters();
         if (!retVal.containsError())
         {
             err = DAQmxStartTask(*this->getTaskHandle());
             if (err != 0)
             {
                 retVal += ito::RetVal::format(ito::retError, 0, "Error occured while starting the task. Code: %i.", err);
             }
         }
    }
    else
    {
        retVal += ito::RetVal(ito::retError, 0, "Task cannot be started, since it is not initialized");
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
// Constructor I
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
// Constructor I
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
    DAQmx_Val_RSE;
    QString physName = m_devId + "/" + m_chId;
    int config;
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
            retval += ito::RetVal::format(ito::retError, 0, "Configmode %i is not in range of 0 to 4", m_analogInputConfig);
        }
    }
    int ret = DAQmxCreateAIVoltageChan(*task->getTaskHandle(), physName.toLatin1().data(), m_chName.toLatin1().data(), config, -m_inputLim, m_inputLim, DAQmx_Val_Volts, NULL);
    task->channelAdded(physName);
    if (ret > 0)
    {
        retval += ito::RetVal::format(ito::retWarning, 0, "Warning occured while creating channel. \n Code: %i", ret);
    }
    else if (ret < 0)
    {
        retval += ito::RetVal::format(ito::retError, 0, "Error occured while creating channel. \n Code: %i", ret);
    }
    else
    {
        retval += ito::retOk;
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
    p.append(QString::number(this->getInputLim()));
    return p;
}

//************************************************//
//   niAnalogOutputChannel Class implementations   //
//************************************************//

//------------------------------------------------------------------------------------------------------
// Constructor I
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
    int ret = DAQmxCreateAOVoltageChan(*task->getTaskHandle(), physName.toLatin1().data(), m_chName.toLatin1().data(), m_minOutputLim, m_maxOutputLim, DAQmx_Val_Volts, NULL);
    task->channelAdded(physName);
    if (ret > 0)
    {
        retval += ito::RetVal::format(ito::retWarning, 0, "Warning occured while creating channel. \n Code: %i", ret);
    }
    else if (ret < 0)
    {
        retval += ito::RetVal::format(ito::retError, 0, "Error occured while creating channel. \n Code: %i", ret);
    }
    else
    {
        retval += ito::retOk;
    }
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
// Constructor I
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
    int ret = DAQmxCreateDIChan(*task->getTaskHandle(), physName.toLatin1().data(), m_chName.toLatin1().data(), DAQmx_Val_ChanForAllLines);
    task->channelAdded(physName);
    if (ret > 0)
    {
        retval += ito::RetVal::format(ito::retWarning, 0, "Warning occured while creating channel. \n Code: %i", ret);
    }
    else if (ret < 0)
    {
        retval += ito::RetVal::format(ito::retError, 0, "Error occured while creating channel. \n Code: %i", ret);
    }
    else
    {
        retval += ito::retOk;
    }
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
// Constructor I
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
    int ret = DAQmxCreateDOChan(*task->getTaskHandle(), physName.toLatin1().data(), m_chName.toLatin1().data(), DAQmx_Val_ChanForAllLines);
    task->channelAdded(physName);
    if (ret > 0)
    {
        retval += ito::RetVal::format(ito::retWarning, 0, "Warning occured while creating channel. \n Code: %i", ret);
    }
    else if (ret < 0)
    {
        retval += ito::RetVal::format(ito::retError, 0, "Error occured while creating channel. \n Code: %i", ret);
    }
    else
    {
        retval += ito::retOk;
    }
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
// Constructor I
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
    char buffer[5120]="";
    this->clear();

    DAQmxGetDevAIPhysicalChans(qPrintable(dev), buffer, sizeof(buffer));
    foreach(const QString &s, QString(buffer).split(", "))
    {
        this->insert(s, NULL);
    }
    DAQmxGetDevAOPhysicalChans(qPrintable(dev), buffer, sizeof(buffer));
    foreach(const QString &s, QString(buffer).split(", "))
    {
        this->insert(s, NULL);
    }
    DAQmxGetDevDIPorts(qPrintable(dev), buffer, sizeof(buffer));
    foreach(const QString &s, QString(buffer).split(", "))
    {
        this->insert(s, NULL);
    }
    DAQmxGetDevCIPhysicalChans(qPrintable(dev), buffer, sizeof(buffer));
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

