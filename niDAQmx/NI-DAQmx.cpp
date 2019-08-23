/* ********************************************************************
    Plugin "NI-DAQmx" for itom software
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

#include "NI-DAQmx.h"
#include "pluginVersion.h"
#include "gitVersion.h"

#include <qstring.h>
#include <qstringlist.h>
#include <qplugin.h>
#include <qmessagebox.h>
#include "DataObject/dataobj.h"
#include <qvarlengtharray.h>
#include <qregexp.h>
#include <QRegularExpression>
#include <iostream>
#include <qlist.h>
#include "NI-PeripheralClasses.h"

//----------------------------------------------------------------------------------------------------------------------------------
#if QT_VERSION < 0x050000
    Q_EXPORT_PLUGIN2(niDAQmxinterface, NiDAQmxInterface) //the second parameter must correspond to the class-name of the interface class, the first parameter is arbitrary (usually the same with small letters only)
#endif

//#include "dockWidgetniDAQmx.h"

//----------------------------------------------------------------------------------------------------------------------------------
//! Constructor of Interface Class.
/*!
    \todo add necessary information about your plugin here.
*/
NiDAQmxInterface::NiDAQmxInterface()
{
    m_type = ito::typeDataIO | ito::typeADDA; //any grabber is a dataIO device AND its subtype grabber (bitmask -> therefore the OR-combination).
    setObjectName("NI-DAQmx");

    m_description = QObject::tr("NI-DAQmx");

    char pathSeparator[] =
    #if defined(__linux__) || defined(__APPLE__)
        "/";
    #else
        "\\";
    #endif

    char docstring[512];
    memset(&docstring, '\0', sizeof(docstring));
    std::string filepath (__FILE__);
    std::string dirpath = filepath.substr(0, filepath.rfind(pathSeparator));
    char first[] = "The plugin implements the DAQmx functions for analog-digital-converters from National Instruments. \n\
The installation needs the NI-DAQmx Library that can be downloaded from the NI website \n(https://www.ni.com/en-us/support/downloads/drivers/download.ni-daqmx.html).\n\n\
Basic plugin documentation is found in ";
    char last[] = ". \nOnline help is available through <plugin_ref>.exec('help').";
    strcpy (docstring, first);
    strcat (docstring, dirpath.c_str());
    strcat (docstring, pathSeparator);
    strcat (docstring, "doc");
    strcat (docstring, pathSeparator);
    strcat (docstring, "NiDAQmx.rst");
    strcat (docstring, last);

    m_detaildescription = QObject::tr(docstring);

    m_author = "Martin Hoppe, ITO, University Stuttgart; Dan Nessett, Unaffiliated; M. Gronle, Unaffiliated";
    m_version = (PLUGIN_VERSION_MAJOR << 16) + (PLUGIN_VERSION_MINOR << 8) + PLUGIN_VERSION_PATCH;
    m_minItomVer = MINVERSION;
    m_maxItomVer = MAXVERSION;
    m_license = QObject::tr("licensed under LGPL");
    m_aboutThis = QObject::tr(GITVERSION); 
    
    ito::Param paramVal("taskType", ito::ParamBase::String, "analogInput", tr("type of the task related to this instance of the NI-DAQmx plugin (analogInput, digitalInput, analogOutput, digitalOutput)").toLatin1().data());
    ito::StringMeta *sm = new ito::StringMeta(ito::StringMeta::String);
    sm->addItem("analogInput");
    sm->addItem("digitalInput");
    sm->addItem("analogOutput");
    sm->addItem("digitalOutput");
    paramVal.setMeta(sm, true);
    m_initParamsMand << paramVal;

    m_initParamsOpt << ito::Param("taskName", ito::ParamBase::String, "", tr("desired name of the underlying NI task (this might be changed by the NI task creation method)").toLatin1().data());

    paramVal = ito::Param("taskMode", ito::ParamBase::String, "finite", tr("mode of the task recording / data generation: finite, continuous, onDemand").toLatin1().data());
    sm = new ito::StringMeta(ito::StringMeta::String);
    sm->addItem("finite");
    sm->addItem("continuous");
    sm->addItem("onDemand");
    paramVal.setMeta(sm, true);
    m_initParamsOpt << paramVal;

    paramVal = ito::Param("samplingRate", ito::ParamBase::Double | ito::ParamBase::In, 0.0, 100000000.0, 100.0, tr("The sampling rate in samples per second per channel. If you use an external source for the Sample Clock, set this value to the maximum expected rate of that clock.").toLatin1().data());
    m_initParamsOpt << paramVal;

    paramVal = ito::Param("samplesPerChannel", ito::ParamBase::Int | ito::ParamBase::In, 0, std::numeric_limits<int>::max(),
        20000,
        tr("The number of samples to acquire or generate for each channel in the task (if taskMode is 'finite'). If taskMode is 'continuous', NI-DAQmx uses this value to determine the buffer size.").toLatin1().data());
    m_initParamsOpt << paramVal;
}

//----------------------------------------------------------------------------------------------------------------------------------
NiDAQmxInterface::~NiDAQmxInterface()
{

}

//----------------------------------------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------------------------------------

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal NiDAQmxInterface::getAddInInst(ito::AddInBase **addInInst)
{
    NEW_PLUGININSTANCE(NiDAQmx) //the argument of the macro is the classname of the plugin
    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal NiDAQmxInterface::closeThisInst(ito::AddInBase **addInInst)
{
   REMOVE_PLUGININSTANCE(NiDAQmx) //the argument of the macro is the classname of the plugin
   return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------------------------------------
/*static*/ int NiDAQmx::InstanceCounter = 0;

//----------------------------------------------------------------------------------------------------------------------------------
NiDAQmx::NiDAQmx() : 
    AddInDataIO(), 
    m_isgrabbing(false),
    m_taskHandle(NULL)
{
    InstanceCounter++;

    // General Parameters
    ito::Param paramVal("name", ito::ParamBase::String | ito::ParamBase::Readonly, "NI-DAQmx", NULL);    
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param("taskName", ito::ParamBase::String | ito::ParamBase::Readonly, "", tr("name of the NI task that is related to this instance").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param("availableDevices", ito::ParamBase::String | ito::ParamBase::Readonly, "", tr("comma-separated list of all detected and available devices").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param("supportedChannels", ito::ParamBase::String | ito::ParamBase::Readonly, "", tr("comma-separated list of all detected and supported channels with respect to the task type. Every item consists of the device name / channel name").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param("channels", ito::ParamBase::String, "", tr("semicolon-separated list of all channels that should be part of this task. Every item is a comma separated string that defines and parameterizes every channel.").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param("taskMode", ito::ParamBase::String, "finite", tr("mode of the task recording / data generation: finite, continuous, onDemand").toLatin1().data());
    ito::StringMeta *sm = new ito::StringMeta(ito::StringMeta::String);
    sm->addItem("finite");
    sm->addItem("continuous");
    sm->addItem("onDemand");
    paramVal.setMeta(sm, true);
    m_params.insert(paramVal.getName(), paramVal);
    m_taskMode = NiTaskModeFinite;

    paramVal = ito::Param("samplingRate", ito::ParamBase::Double | ito::ParamBase::In, 0.0, 100000000.0, 100.0, tr("The sampling rate in samples per second per channel. If you use an external source for the Sample Clock, set this value to the maximum expected rate of that clock.").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    
    paramVal = ito::Param("samplesPerChannel", ito::ParamBase::Int | ito::ParamBase::In, 0,std::numeric_limits<int>::max(), 
                                            20000, 
                                            tr("The number of samples to acquire or generate for each channel in the task (if taskMode is 'finite'). If taskMode is 'continuous', NI-DAQmx uses this value to determine the buffer size.").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);

    // Register Exec Functions
    QVector<ito::Param> pMand = QVector<ito::Param>();
    QVector<ito::Param> pOpt = QVector<ito::Param>();
    QVector<ito::Param> pOut = QVector<ito::Param>();
    registerExecFunc("help", pMand, pOpt, pOut, tr("Prints information on plugin methods."));
    registerExecFunc("help:name", pMand, pOpt, pOut, tr("Prints information about name plugin method."));
    registerExecFunc("help:TaskParams", pMand, pOpt, pOut, tr("Prints information about TaskParameters."));
    registerExecFunc("help:aiTaskParams", pMand, pOpt, pOut, tr("Prints information about aiTaskParameters."));
    registerExecFunc("help:channel", pMand, pOpt, pOut, tr("Prints information about channel plugin method."));
    registerExecFunc("help:chAssociated", pMand, pOpt, pOut, tr("Prints information about chAssociated plugin method."));
    registerExecFunc("help:ChParams", pMand, pOpt, pOut, tr("Prints information about ChannelParameters."));
    registerExecFunc("help:aiChParams", pMand, pOpt, pOut, tr("Prints information aiChParameters."));
    registerExecFunc("help:getParam", pMand, pOpt, pOut, tr("Prints information about getParam plugin method."));
    registerExecFunc("help:setParam", pMand, pOpt, pOut, tr("Prints information setParam plugin method."));
    registerExecFunc("help:startDevice", pMand, pOpt, pOut, tr("Prints information startDevice plugin method."));
    registerExecFunc("help:stopDevice", pMand, pOpt, pOut, tr("Prints information stopDevice plugin method."));
    registerExecFunc("help:setValMode", pMand, pOpt, pOut, tr("Prints information setValMode plugin method."));
    registerExecFunc("help:acquire", pMand, pOpt, pOut, tr("Prints information acquire plugin method."));
    registerExecFunc("help:getVal", pMand, pOpt, pOut, tr("Prints information getVal plugin method."));
    registerExecFunc("help:copyVal", pMand, pOpt, pOut, tr("Prints information copyVal plugin method."));

    //end register Exec Functions

}

//----------------------------------------------------------------------------------------------------------------------------------
NiDAQmx::~NiDAQmx()
{
}



//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal NiDAQmx::init(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);

    QByteArray taskName = paramsOpt->at(0).getVal<const char*>();
    m_params["taskMode"].setVal<const char*>(paramsOpt->at(1).getVal<const char*>());

    switch (m_params["taskMode"].getVal<const char*>()[0])
    {
    case 'f': //finite
    {
        m_taskMode = NiTaskModeFinite;
        break;
    }
    case 'c': //continuous
    {
        m_taskMode = NiTaskModeContinuous;
        break;
    }
    case 'o': // Hardware Timed Single Point
    {
        m_taskMode = NiTaskModeOnDemand;
        break;
    }
    default:
    {
        retValue += ito::RetVal::format(ito::retError, 0, "configure sample clock timing: Task mode '%s' is not supported.", m_params["taskMode"].getVal<const char*>());
        break;
    }
    }

    m_params["samplingRate"].setVal<double>(paramsOpt->at(2).getVal<double>());
    m_params["samplesPerChannel"].setVal<int>(paramsOpt->at(3).getVal<int>());

    QString taskType = paramsMand->at(0).getVal<const char*>();
    if (taskType == "analogInput")
    {
        m_taskType = TaskType::AnalogInput;
        if (taskName == "") { taskName = QString("AnalogInput %1").arg(InstanceCounter).toLatin1(); }
    }
    else if (taskType == "digitalInput")
    {
        m_taskType = TaskType::DigitalInput;
        if (taskName == "") { taskName = QString("DigitalInput %1").arg(InstanceCounter).toLatin1(); }
    }
    else if (taskType == "analogOutput")
    {
        m_taskType = TaskType::AnalogOutput;
        if (taskName == "") { taskName = QString("AnalogOutput %1").arg(InstanceCounter).toLatin1(); }
    }
    else if (taskType == "digitalOutput")
    {
        m_taskType = TaskType::DigitalOutput;
        if (taskName == "") { taskName = QString("DigitalOutput %1").arg(InstanceCounter).toLatin1(); }
    }
    else
    {
        retValue += ito::RetVal(ito::retError, 0, "invalid taskType");
    }

    m_params["taskName"].setVal<const char*>(taskName.constData());

    if (!retValue.containsError())
    {
        retValue += scanForAvailableDevicesAndSupportedChannels();
    }

    if (!retValue.containsError())
    {
        retValue += createTask();
    }

    if (!retValue.containsError())
    {
        retValue += configTask();
    }

    setIdentifier(m_params["taskName"].getVal<const char*>());

    if (!retValue.containsError())
    {
        emit parametersChanged(m_params);
    }

    if (waitCond)
    {
        waitCond->returnValue = retValue;
        waitCond->release();
    }

    setInitialized(true); //init method has been finished (independent of retValue)
    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal NiDAQmx::close(ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);

    ito::RetVal retValue = stopAndDeleteTask();

    if (waitCond)
    {
        waitCond->returnValue = retValue;
        waitCond->release();
    }
    
    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal NiDAQmx::stopAndDeleteTask()
{
    ito::RetVal retValue;

    if (m_channels.size() > 0)
    {
        foreach(NiBaseChannel* nbc, m_channels)
        {
            delete nbc;
        }
        m_channels.clear();
    }

    if (m_taskHandle)
    {
        DAQmxStopTask(m_taskHandle);

        retValue += checkError(DAQmxClearTask(m_taskHandle), "Clear task");
        m_taskHandle = NULL;
    }
    
    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal NiDAQmx::createTask()
{
    ito::RetVal retValue;

    if (m_taskHandle)
    {
        retValue += stopAndDeleteTask();
    }

    retValue += checkError(DAQmxCreateTask(m_params["taskName"].getVal<const char*>(), &m_taskHandle), "Creating new task");

    if (m_taskHandle == NULL)
    {
        retValue += ito::RetVal(ito::retError, 0, "task could not be created (unknown reason)");
    }

    if (!retValue.containsError())
    {
        char buf[1024];
        if (DAQmxGetTaskName(m_taskHandle, buf, sizeof(buf)) == DAQmxSuccess)
        {
            m_params["taskName"].setVal<const char*>(buf);
        }
    }

    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal NiDAQmx::createChannelsForTask()
{
    ito::RetVal retValue;

    if (!m_taskHandle)
    {
        retValue += ito::RetVal(ito::retError, 0, "no task created");
    }
    else
    {
        QStringList channels = QString(m_params["channels"].getVal<const char*>()).split(";");
        NiBaseChannel *nbc;
        
        foreach(const QString &channel, channels)
        {
            nbc = NULL;

            switch (m_taskType)
            {
            case TaskType::AnalogInput:
                nbc = NiAnalogInputChannel::fromConfigurationString(channel, retValue);
                break;
            case TaskType::AnalogOutput:
                nbc = NiAnalogOutputChannel::fromConfigurationString(channel, retValue);
                break;
            case TaskType::DigitalInput:
                nbc = NiDigitalInputChannel::fromConfigurationString(channel, retValue);
                break;
            case TaskType::DigitalOutput:
                nbc = NiDigitalOutputChannel::fromConfigurationString(channel, retValue);
                break;
            default:
                retValue += ito::RetVal(ito::retError, 0, "invalid task type");
                break;
            }

            if (!retValue.containsError() && nbc)
            {
                retValue += nbc->addChannelToTask(m_taskHandle);
                m_channels.append(nbc);
            }

            if (retValue.containsError())
            {
                break;
            }
        }
    }

    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal NiDAQmx::configTask()
{
    ito::RetVal retValue;

    if (!m_taskHandle)
    {
        retValue += ito::RetVal(ito::retError, 0, "no task created");
    }
    else if (m_channels.size() > 0)
    {
        double rateHz = m_params["samplingRate"].getVal<double>();
        int samplesPerChannel = m_params["samplesPerChannel"].getVal<int>();
        //configure task
        switch (m_taskMode)
        {
        case NiTaskModeFinite: //finite
        {
            // Notice that the onboardclock is not supported for DigitalIn. Either you implement it, or do not use it.
            retValue += checkError(DAQmxCfgSampClkTiming(m_taskHandle, NULL /*OnboardClock*/, rateHz, DAQmx_Val_Rising, DAQmx_Val_FiniteSamps, samplesPerChannel), "configure sample clock timing");
            break;
        }
        case NiTaskModeContinuous: //continuous
        {
            retValue += checkError(DAQmxCfgSampClkTiming(m_taskHandle, NULL /*OnboardClock*/, rateHz, DAQmx_Val_Rising, DAQmx_Val_ContSamps, samplesPerChannel), "configure sample clock timing");
            break;
        }
        case NiTaskModeOnDemand: // Hardware Timed Single Point
        {
            retValue += checkError(DAQmxCfgSampClkTiming(m_taskHandle, NULL /*OnboardClock*/, rateHz, DAQmx_Val_Rising, DAQmx_Val_HWTimedSinglePoint, samplesPerChannel), "configure sample clock timing");
            break;
        }
        }

        //if (!retValue.containsError())
        //{
        //    // If a triggerport is defined, set task to triggermode
        //    if (this->getTriggerPort() != "")
        //    {
        //        err = DAQmxCfgDigEdgeStartTrig(m_taskHandle, this->getTriggerPort().toLatin1().data(), this->getTriggerEdge());
        //    }

        //}
    }

    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal NiDAQmx::getParam(QSharedPointer<ito::Param> val, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue;
    QString key;
    bool hasIndex = false;
    int index;
    QString suffix;
    ParamMapIterator it;

    retValue += apiParseParamName(val->getName(), key, hasIndex, index, suffix);

    if (retValue == ito::retOk)
    { //gets the parameter key from m_params map (read-only is allowed, since we only want to get the value).
        retValue += apiGetParamFromMapByKey(m_params, key, it, false);
    }
    if (!retValue.containsError())
    {
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
ito::RetVal NiDAQmx::setParam(QSharedPointer<ito::ParamBase> val, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);
    QString key;
    bool hasIndex;
    int index;
    QString suffix;
    ParamMapIterator it;
    bool restartTask = false;

    ////parse the given parameter-name (if you support indexed or suffix-based parameters)
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
        retValue += apiValidateParam(*it, *val, false, true);
    }

    if (!retValue.containsError())
    {
        if (key == "channels")
        {
            QString currentValue = it->getVal<const char*>();
            QString newValue = val->getVal<const char*>();

            retValue += it->copyValueFrom(&(*val));

            if (!retValue.containsError() && (currentValue != newValue))
            {
                restartTask = true;
            }
        }
        else if (key == "samplingRate" || key == "samplesPerChannel")
        {
            retValue += it->copyValueFrom(&(*val));
            if (!retValue.containsError())
            {
                retValue += configTask();
            }
        }
        else if (key == "taskMode")
        {
            retValue += it->copyValueFrom(&(*val));

            switch (it->getVal<const char*>()[0])
            {
                case 'f': //finite
                {
                    m_taskMode = NiTaskModeFinite;
                    break;
                }
                case 'c': //continuous
                {
                    m_taskMode = NiTaskModeContinuous;
                    break;
                }
                case 'o': // Hardware Timed Single Point
                {
                    m_taskMode = NiTaskModeOnDemand;
                    break;
                }
                default:
                {
                    retValue += ito::RetVal::format(ito::retError, 0, "configure sample clock timing: Task mode '%s' is not supported.", m_params["taskMode"].getVal<const char*>());
                    break;
                }
            }

            restartTask = true;
        }
        else
        {
            //all parameters that don't need further checks can simply be assigned
            //to the value in m_params (the rest is already checked above)
            retValue += it->copyValueFrom(&(*val));
        }
    }

    if (restartTask)
    {
        retValue += stopAndDeleteTask();
        retValue += createTask();
        if (!retValue.containsError())
        {
            retValue += createChannelsForTask();
        }

        if (!retValue.containsError())
        {
            retValue += configTask();
        }

        const uInt32 bufferSize = 1024;
        char buffer[bufferSize];
        ito::RetVal retValue2 = checkError(DAQmxGetTaskChannels(m_taskHandle, buffer, bufferSize), "read all current task channels");

        QStringList availableChannels = QString(buffer).split(",");

        QList<NiBaseChannel*>::iterator iter = m_channels.begin();

        while (iter != m_channels.end())
        {
            if (availableChannels.contains((*iter)->physicalName()))
            {
                ++iter;
            }
            else
            {
                delete (*iter);
                iter = m_channels.erase(iter);
            }
        }

        //update parameters
        QStringList channelParams;
        foreach(NiBaseChannel* nbc, m_channels)
        {
            channelParams << nbc->getConfigurationString();
        }
        QByteArray channels = channelParams.join(";").toLatin1();
        m_params["channels"].setVal<const char*>(channels.constData());
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
ito::RetVal NiDAQmx::startDevice(ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue;

    // startDevice is not used in this plugin.

    retValue += ito::RetVal(ito::retError, 0, tr("NiDAQmx::startDevice. Warning: startDevice() is not used in NiDAQmx. It is a NOOP").toLatin1().data());;

    if (waitCond)
    {
        waitCond->returnValue = retValue;
        waitCond->release();
    }

    return retValue;
}
         
//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal NiDAQmx::stopDevice(ItomSharedSemaphore *waitCond)
{
    //ItomSharedSemaphoreLocker locker(waitCond);
   ito::RetVal retValue;

    // stopDevice is not used in this plugin.

    retValue += ito::RetVal(ito::retError, 0, tr("NiDAQmx::stopDevice. Warning: stopDevice() is not used in NiDAQmx It is a NOOP").toLatin1().data());;

    if (waitCond)
    {
        waitCond->returnValue = retValue;
        waitCond->release();
    }

    return retValue;
}
         
//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal NiDAQmx::acquire(const int /*trigger*/, ItomSharedSemaphore *waitCond)
{
    // The trigger is used here in another meaning. It s a bitmask and defines
    // which tasks are started! (all decimal)
    //  1 = Analog Input
    //  2 = Digital Input
    //  4 = Counter Input
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);
    m_isgrabbing = false;

    if (!m_taskHandle || m_channels.size() == 0)
    {
        retValue += ito::RetVal(ito::retError, 0, "No task started or no channels assigned to the task");
    }
    else if (m_taskType & TaskSubTypes::Output)
    {
        retValue += ito::RetVal(ito::retError, 0, "Task is an output task. No data acquisition is possible.");
    }
    else
    {
        retValue += checkError(DAQmxStartTask(m_taskHandle), "DAQmxStartTask");
    }

    if (!retValue.containsError())
    {
        m_isgrabbing = true;
    }

    if (waitCond)
    {
        waitCond->returnValue = retValue;
        waitCond->release();
    }

    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal NiDAQmx::readAnalog()
{
    ito::RetVal retValue(ito::retOk);

    int channels = m_channels.size();
    int samples = m_params["samplesPerChannel"].getVal<int>();
    double samplingRate = m_params["samplingRate"].getVal<double>();

    if (m_isgrabbing == false)
    {
        retValue += ito::RetVal(ito::retError, 0, tr("NiDAQmx::readAnalog - Tried to return data without first calling acquire").toLatin1().data());
    }
    else
    {
        //step 1: create and check m_data (if not yet available)
        retValue += checkData(NULL, channels, samples); //update external object or m_data

        if (!retValue.containsError())
        {
            if (m_taskMode == NiTaskModeFinite)
            {
                int32 retSize = -1;

                int run = 0;
                int err;

                //provide a set-alive feature for long acquisitions!
                while (1)
                {
                    err = DAQmxWaitUntilTaskDone(m_taskHandle, 4);
                    if (err == 0)
                    {
                        break;
                    }
                    else if (err != DAQmxErrorWaitUntilDoneDoesNotIndicateDone)
                    {
                        retValue += checkError(err, "Wait till task is done");
                        break;
                    }

                    run++;
                    if (run > (0.5 * samples / samplingRate))
                    {
                        retValue += ito::RetVal(ito::retError, 0, "timeout while waiting for finished acquisition");
                    }
                    else
                    {
                        setAlive();
                    }
                }

                if (!retValue.containsError())
                {
                    retValue += checkError(DAQmxReadAnalogF64(m_taskHandle, samples, DAQmx_Val_WaitInfinitely, DAQmx_Val_GroupByChannel, m_data.rowPtr<ito::float64>(0, 0), m_data.getTotal(), &retSize, NULL), "DAQmxReadAnalogF64");
                }
                
                m_data.setAxisScale(1, 1.0 / samplingRate);
                m_data.setAxisUnit(1, "s");
                m_data.setAxisDescription(1, "time");

                m_data.setValueUnit("volt"); //marc: todo: is the unit 'volt' correct? If there is a need to scale the values to a certain unit, this has to be done in a for loop, since DataObjects don't have a valueScale or valueOffset property (only for axes)
                m_data.setValueDescription("voltage");
                m_isgrabbing = false;

                retValue += checkError(DAQmxStopTask(m_taskHandle), "stop task");
            }
            else if (m_taskMode == NiTaskModeContinuous)
            {
                retValue += ito::RetVal(ito::retError, 0, tr("NiDAQmx::readAnalog - Continuous mode is not yet supported").toLatin1().data());
            }
            else if (m_taskMode == NiTaskModeOnDemand)
            {
                retValue += ito::RetVal(ito::retError, 0, tr("NiDAQmx::readAnalog - On demand mode is not yet supported").toLatin1().data());
            }
            else
            {
                retValue += ito::RetVal(ito::retError, 0, tr("NiDAQmx::readAnalog - internal error invalid mode passed to readAnalog").toLatin1().data());
            }
        }
    }

    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
// not supported yet, because an external clock source is neccesary, marc: todo: similar to readAnalog
ito::RetVal NiDAQmx::readDigital()
{
    // right now I only support 8 line Ports! Some devices have 16 or 32 lines per port! 
    // just use: DAQmxReadDigitalU16, DAQmxReadDigitalU32
    ito::RetVal retValue(ito::retOk);

    int channels = m_channels.size();
    int samples = m_params["samplesPerChannel"].getVal<int>();
    double samplingRate = m_params["samplingRate"].getVal<double>();

    if (m_isgrabbing == false)
    {
        retValue += ito::RetVal(ito::retError, 0, tr("NiDAQmx::readAnalog - Tried to return data without first calling acquire").toLatin1().data());
    }
    else
    {
        //step 1: create and check m_data (if not yet available)
        retValue += checkData(NULL, channels, samples); //update external object or m_data

        if (!retValue.containsError())
        {
            if (m_taskMode == NiTaskModeFinite)
            {
                int requestedSamples = 0;
                int size = channels * samples;
                int32 retSize = -1;
                int maxSamplesPerTurn = samplingRate * 4; //max 4seconds per turn
                ito::uint8* ptr = m_data.rowPtr<ito::uint8>(0, 0);

                while (requestedSamples < samples)
                {
                    size = std::min(samples - requestedSamples, maxSamplesPerTurn);
                    requestedSamples += size;
                    size *= channels;
                    retValue += checkError(DAQmxReadDigitalU8(m_taskHandle, samples, DAQmx_Val_WaitInfinitely, DAQmx_Val_GroupByChannel, ptr, size, &retSize, NULL), "DAQmxReadDigitalU8");
                    ptr += size;

                    setAlive();

                    if (!retValue.containsError())
                    {
                        break;
                    }
                }

                m_data.setAxisScale(1, 1.0 / samplingRate);
                m_data.setAxisUnit(1, "s");
                m_data.setAxisDescription(1, "time");

                m_data.setValueUnit("digit"); //marc: todo: is the unit 'volt' correct? If there is a need to scale the values to a certain unit, this has to be done in a for loop, since DataObjects don't have a valueScale or valueOffset property (only for axes)
                m_data.setValueDescription("value");
                m_isgrabbing = false;
            }
            else if (m_taskMode == NiTaskModeContinuous)
            {
                retValue += ito::RetVal(ito::retError, 0, tr("NiDAQmx::readDigital - Continuous mode is not yet supported").toLatin1().data());
            }
            else if (m_taskMode == NiTaskModeOnDemand)
            {
                retValue += ito::RetVal(ito::retError, 0, tr("NiDAQmx::readDigital - On demand mode is not yet supported").toLatin1().data());
            }
            else
            {
                retValue += ito::RetVal(ito::retError, 0, tr("NiDAQmx::readDigital - internal error invalid mode passed to readDigital").toLatin1().data());
            }
        }
    }
    
    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
//marc: todo: similar to readAnalog
ito::RetVal NiDAQmx::readCounter()
{
    // TODO: Add Implementation
    return ito::RetVal(ito::retError, 0, "read from counter task not implemented yet");
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal NiDAQmx::writeAnalog(const ito::DataObject *dataObj)
{
    ito::RetVal retValue(ito::retOk);
    int channels = m_channels.size();
    int samples = m_params["samplesPerChannel"].getVal<int>();
    double samplingRate = m_params["samplingRate"].getVal<double>();
    int32 smplW = -1;
    
    if (dataObj->getDims() != 2 ||
        dataObj->getSize(0) != channels ||
        dataObj->getSize(1) > samples ||
        dataObj->getType() != ito::tFloat64)
    {
        retValue += ito::RetVal::format(ito::retError, 0, "%i x M, float64 dataObject required with M <= %i", channels, samples);
    }
    else
    {
        samples = dataObj->getSize(1);

        int err = DAQmxWriteAnalogF64(m_taskHandle, samples, true, 0, DAQmx_Val_GroupByChannel, (ito::float64*)dataObj->rowPtr(0, 0), &smplW, NULL);

        // If DAQmxWriteAnalogF64 returned an error or warning, translate the numerical code into text and attach a prefix
        retValue += checkError(err, "NiDAQmx::writeAnalog - DAQmxWriteAnalogF64 abnormal return");
    }

    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal NiDAQmx::writeDigital(const ito::DataObject *dataObj)
{
    return ito::RetVal(ito::retError, 0, "write to digital task not implemented yet");
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal NiDAQmx::writeCounter(const ito::DataObject *dataObj)
{
    return ito::RetVal(ito::retError, 0, "write to counter task not implemented yet");
}

//----------------------------------------------------------------------------------------------------------------------------------
//! Returns the grabbed NiDAQmx device data as reference.
/*!
    This method returns a reference to the recently acquired NiDAQmx data. Therefore this data must fit into the data structure of the 
    DataObject.
    
    This method returns a reference to the internal dataObject m_data of the NiDAQmx device where the currently acquired data is copied to (either
    in the acquire method or in retrieve data). Please remember, that the reference may directly change if a set of data is acquired.

    \param [in,out] vpdObj is the pointer to a given dataObject (this pointer should be cast to ito::DataObject*). After the call, the dataObject is a reference to the internal m_data dataObject of the NiDAQmx device.
    \param [in] waitCond is the semaphore (default: NULL), which is released if this method has been terminated
    \return retOk if everything is ok, retError occurs when the NiDAQmx device has not been started or no data has been acquired by the method acquire.
    
    \sa retrieveImage, copyVal
*/
ito::RetVal NiDAQmx::getVal(void *vpdObj, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);
    int error = -1;
    ito::DataObject *dObj = reinterpret_cast<ito::DataObject *>(vpdObj);

    if (!retValue.containsError())
    {
        retValue += retrieveData();
    }

    if (!retValue.containsError())
    {
        *dObj = m_data; //dObj is now a shallow copy of the internal object m_data.
    }

    if (waitCond) 
    {
        waitCond->returnValue = retValue;
        waitCond->release();
    }

    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
//! Returns the grabbed NiDAQmx device data as a deep copy.
/*!
    This method copies the recently grabbed NiDAQmx device data to the given DataObject. 
    
    The given dataObject must either have an empty size (then it is resized to the size and type of the NiDAQmx device data) or its size or adjusted region of
    interest must exactly fit to the size of the NiDAQmx device data. Then, the acquired data is copied inside of the given region of interest.

    \param [in,out] vpdObj is the pointer to a given dataObject (this pointer should be cast to ito::DataObject*) where the acquired data is deep copied to.
    \param [in] waitCond is the semaphore (default: NULL), which is released if this method has been terminated
    \return retOk if everything is ok, retError is NiDAQmx device has not been started or no data has been acquired by the method acquire.
    
    \sa retrieveImage, getVal
*/
ito::RetVal NiDAQmx::copyVal(void *vpdObj, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);
    ito::DataObject *externalDataObject = reinterpret_cast<ito::DataObject *>(vpdObj);

    if (!externalDataObject)
    {
        retValue += ito::RetVal(ito::retError, 0, tr("NiDAQmx::copyVal - Empty object handle provided by caller").toLatin1().data());
    }

    if (!retValue.containsError())
    {
        retValue += retrieveData(); //retrieve data from device and store it in m_data
    }

    if (!retValue.containsError())
    {
        int channels = m_channels.size();
        int samples = m_params["samplesPerChannel"].getVal<int>();
        retValue += checkData(externalDataObject, channels, samples);
    }

    if (!retValue.containsError())
    {
        retValue += m_data.deepCopyPartial(*externalDataObject); //deeply copies the content of m_data to the current roi of externalDataObject

        //copy the scalings, offsets, axis descriptions, ... from the two last axes to the externalDataObject
        // (since the externalDataObject might have more dimensions than m_data)
        m_data.copyAxisTagsTo(*externalDataObject);
    }

    if (waitCond) 
    {
        waitCond->returnValue = retValue;
        waitCond->release();
    }
    
    return retValue;
}

//---------------------------------------------------------------------------------------------
/* retrieveData gets recently acquired data from the NI-card and stores them into the internal object m_data.
   Scalings, Offsets, axes descriptions etc. will be set to m_data accordingly.
*/
ito::RetVal NiDAQmx::retrieveData()
{
    ito::RetVal retValue;

    if (!m_taskHandle || m_channels.size() == 0)
    {
        retValue += ito::RetVal(ito::retError, 0, "No task started or no channels assigned to the task.");
    }
    else if (m_taskType & TaskSubTypes::Output)
    {
        retValue += ito::RetVal(ito::retError, 0, "Task is an output task. No data acquisition is possible.");
    }
    else if( !m_isgrabbing )
    {
        retValue += ito::RetVal(ito::retError, 0, tr("NiDAQmx::retrieveData - cannot retrieve data because none was acquired.").toLatin1().data());
    }
    else
    {
        if (m_taskType & TaskSubTypes::Analog)
        {
            retValue += readAnalog();
        }
        else if (m_taskType & TaskSubTypes::Digital)
        {
            retValue += readDigital();
        }
        else if (m_taskType & TaskSubTypes::Counter)
        {
            retValue += readCounter();
        }
        else
        {
            retValue += ito::RetVal(ito::retError, 0, "Invalid input task type");
        }
    }

    m_isgrabbing = false;

    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal NiDAQmx::setVal(const char *data, const int length, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);

    
    
    const ito::DataObject *dObj = reinterpret_cast<const ito::DataObject*>(data);

    if (!m_taskHandle || m_channels.size() == 0)
    {
        retValue += ito::RetVal(ito::retError, 0, "No task started or no channels assigned to the task.");
    }
    else if (m_taskType & TaskSubTypes::Input)
    {
        retValue += ito::RetVal(ito::retError, 0, "Task is an input task. No data output is possible.");
    }

    int error = 0;

    if (!retValue.containsError())
    {
        if (m_taskType & TaskSubTypes::Analog)
        {
            retValue += writeAnalog(dObj);
        }
        else if (m_taskType & TaskSubTypes::Digital)
        {
            retValue += writeDigital(dObj);
        }
        else if (m_taskType & TaskSubTypes::Counter)
        {
            retValue += writeCounter(dObj);
        }
        else
        {
            retValue += ito::RetVal(ito::retError, 0, "Invalid input task type");
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
void NiDAQmx::dockWidgetVisibilityChanged(bool visible)
{
    //if (getDockWidget())
    //{
    //    QWidget *widget = getDockWidget()->widget();
    //    if (visible)
    //    {
    //        connect(this, SIGNAL(parametersChanged(QMap<QString, ito::Param>)), widget, SLOT(parametersChanged(QMap<QString, ito::Param>)));

    //        emit parametersChanged(m_params);
    //    }
    //    else
    //    {
    //        disconnect(this, SIGNAL(parametersChanged(QMap<QString, ito::Param>)), widget, SLOT(parametersChanged(QMap<QString, ito::Param>)));
    //    }
    //}
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
const ito::RetVal NiDAQmx::showConfDialog(void)
{
    return apiShowConfigurationDialog(this, new DialogNiDAQmx(this, (void*)this));
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal NiDAQmx::checkData(ito::DataObject *externalDataObject, int channels, int samples)
{
    ito::tDataType futureType = m_taskType & TaskSubTypes::Analog ? ito::tFloat64 : ito::tUInt8;

    if (externalDataObject == NULL)
    {
        //check internal object m_data
        if (m_data.getDims() != 2 || m_data.getSize(0) != (unsigned int)channels || m_data.getSize(1) != (unsigned int)samples || m_data.getType() != futureType)
        {
            m_data = ito::DataObject(channels, samples, futureType);
        }
    }
    else
    {
        int dims = externalDataObject->getDims();
        if (externalDataObject->getDims() == 0)
        {
            *externalDataObject = ito::DataObject(channels, samples, futureType);
        }
        else if (externalDataObject->calcNumMats () > 1)
        {
            return ito::RetVal(ito::retError, 0, tr("NiDAQmx::checkData - Error during check data, external dataObject invalid. Object has more than 1 plane. It must be of right size and type or a uninitilized image.").toLatin1().data());            
        }
        else if (externalDataObject->getSize(dims - 2) != (unsigned int)channels || externalDataObject->getSize(dims - 1) != (unsigned int)samples || externalDataObject->getType() != futureType)
        {
            return ito::RetVal(ito::retError, 0, tr("NiDAQmx::checkData - Error during check data, external dataObject invalid. Object must be of right size and type or a uninitilized image.").toLatin1().data());
        }
    }

    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal NiDAQmx::scanForAvailableDevicesAndSupportedChannels()
{
    ito::RetVal retValue;
    ito::RetVal tempRetValue;
    m_availableDevices.clear();
    m_supportedChannels.clear();

    //scan for devices
    char buffer[5120];
    memset(buffer, 0, 5120 * sizeof(char));
    retValue += checkError(DAQmxGetSysDevNames(buffer, sizeof(buffer)), tr("Detect available devices."));

    if (!retValue.containsError())
    {
        QString deviceNames = QString(buffer);
        m_availableDevices = deviceNames.split(", ");
        if (m_availableDevices.size() == 0)
        {
            retValue += ito::RetVal(ito::retWarning, 0, "Could not detect any devices. Probably, no channels can be added to the task.");
        }
    }

    //scan for channels
    foreach(const QString &deviceName, m_availableDevices)
    {
        switch (m_taskType)
        {
        case AnalogInput:
            tempRetValue = checkError(DAQmxGetDevAIPhysicalChans(qPrintable(deviceName), buffer, sizeof(buffer)), QString("Detect supported channels for device %1").arg(deviceName));
            break;
        case AnalogOutput:
            tempRetValue = checkError(DAQmxGetDevAOPhysicalChans(qPrintable(deviceName), buffer, sizeof(buffer)), QString("Detect supported channels for device %1").arg(deviceName));
            break;
        case DigitalInput:
            tempRetValue = checkError(DAQmxGetDevDIPorts(qPrintable(deviceName), buffer, sizeof(buffer)), QString("Detect supported channels for device %1").arg(deviceName));
            break;
        case DigitalOutput:
            tempRetValue = checkError(DAQmxGetDevDOPorts(qPrintable(deviceName), buffer, sizeof(buffer)), QString("Detect supported channels for device %1").arg(deviceName));
            break;
        default:
            buffer[0] = '\0';
            break;
        }

        if (!tempRetValue.containsError())
        {
            m_supportedChannels << QString(buffer).split(", ");
        }
        else
        {
            //convert an error into a warning
            tempRetValue = ito::RetVal(ito::retWarning, tempRetValue.errorCode(), tempRetValue.errorMessage());
        }

        retValue += tempRetValue;
    }

    if (m_supportedChannels.size() == 0)
    {
        retValue += ito::RetVal(ito::retWarning, 0, "Could not detect any supported channels. Probably, no channels can be added to the task.");
    }

    m_params["availableDevices"].setVal<char*>(m_availableDevices.join(",").toLatin1().data());
    m_params["supportedChannels"].setVal<char*>(m_supportedChannels.join(",").toLatin1().data());

    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal NiDAQmx::execFunc(const QString helpCommand, QSharedPointer<QVector<ito::ParamBase> > paramsMand, QSharedPointer<QVector<ito::ParamBase> > paramsOpt, QSharedPointer<QVector<ito::ParamBase> > /*paramsOut*/, ItomSharedSemaphore *waitCond)
{
    ito::RetVal retValue = ito::retOk;

    QStringList parts = helpCommand.split(":");
    QString function = parts[0];
    QString empty("");

    if (function == "help")
    {
        if(parts.size() == 1)
        {
             retValue += help(empty);
        }
        else
        {
            retValue += help(parts[1]);
        }
    }
    else
    {
        retValue += ito::RetVal::format(ito::retError,0,tr("function name '%s' does not exist").toLatin1().data(), helpCommand.toLatin1().data());
    }

    if (waitCond)
    {
        waitCond->returnValue = retValue;
        waitCond->release();
        waitCond->deleteSemaphore();
        waitCond = NULL;
    }

    return retValue;
}

//--------------------------------------------------------------------------------------------------------
ito::RetVal NiDAQmx::help(const QString &helpTopic)
{
    ito::RetVal retValue = ito::retOk;

    if (helpTopic == "")
    {
        std::cout << "The NiDAQmx plugin provides data acquistion and control using\n" << std::endl;
        std::cout << "National Instruments computer interface hardware.\n" << std::endl;
        std::cout << "The plugin supports the following methods: name, getParam, setParam,\n" << std::endl;
        std::cout << "startDevice, stopDevice, acquire, setValMode, getVal, copyVal,\n" << std::endl;
        std::cout << "channel, and chAssociated. startDevice and stopDevice are NOOPs in this plugin.\n\n" << std::endl;
        std::cout << "Each method is documented separately with online help commands.\n" << std::endl;
        std::cout << "For example, setParam method documentation is accessed using\n" << std::endl;
        std::cout << "the command: <plugin ref>.exec('help:setParam').\n\n" << std::endl;
        std::cout << "An example acquisition would start with plugin=dataIO('NiDAQmx');\n" << std::endl;
        std::cout << "Followed by plugin.setParam('aiTaskParams','20000,100,0');\n" << std::endl;
        std::cout << "Then, plugin.setParam('aiChParams','Dev1/ai0,3,-10,10');\n" << std::endl;
        std::cout << "After setting these parameters, execute the command plugin.acquire(1);\n" << std::endl;
        std::cout << "(the number 1 indicates that the analog input task is the acquire target)\n" << std::endl;
        std::cout << "This call will immediately return and at a later time the following\n" << std::endl;
        std::cout << "commands retrieve the data: d=dataObject(); followed by plugin.getVal(d);\n" << std::endl;
        std::cout << "If getVal is called before the acquistion is finished, it will wait until\n" << std::endl;
        std::cout << "all data is acquired before returning.\n\n" << std::endl;
    }
    else if (helpTopic == "name")
    {
        std::cout << "\nTODO: name description.\n\n" << std::endl;
    }
    else if (helpTopic == "getParam")
    {
        std::cout << "\nTODO: getParam description.\n\n" << std::endl;
    }
    else if (helpTopic == "setParam")
    {
        std::cout << "\nTODO: setParam description.\n\n" << std::endl;
    }
    else if (helpTopic == "startDevice")
    {
        std::cout << "\nTODO: startDevice description.\n\n" << std::endl;
    }
    else if (helpTopic == "stopDevice")
    {
        std::cout << "\nTODO: stopDevice description.\n\n" << std::endl;
    }
    else if (helpTopic == "setValMode")
    {
        std::cout << "\nTODO: setValMode description.\n\n" << std::endl;
    }
    else if (helpTopic == "acquire")
    {
        std::cout << "\nTODO: acquire description.\n\n" << std::endl;
    }
    else if (helpTopic == "getVal")
    {
        std::cout << "\nTODO: getVal description.\n\n" << std::endl;
    }
    else if (helpTopic == "copyVal")
    {
        std::cout << "\nTODO: copyVal description.\n\n" << std::endl;
    }
    else if (helpTopic == "channel")
    {
        std::cout << "\nTODO: channel description.\n\n" << std::endl;
    }
    else if (helpTopic == "chAssociated")
    {
        std::cout << "\nTODO: chAssociated description.\n\n" << std::endl;
    }
    else if (helpTopic == "TaskParams")
    {
        std::cout << "TaskParams are specified as follows:\n\n" << std::endl;
        std::cout << "xxTaskParams, where xx is one of 'ai', 'ao', 'di', 'do', 'ci', 'do',\n" << std::endl;
        std::cout << "these abbreviations corresponding respectively to 'analog input', 'analog output',\n" << std::endl;
        std::cout << "'digital input', 'digital output', 'counter input', and 'counter output'.\n" << std::endl;
        std::cout << "These are set using setParam() and read using getParam().For more information\n" << std::endl;
        std::cout << "on specific TaskParmeters, type <plugin ref>.exec('help:xxTaskParams');\n" << std::endl;
        std::cout << "for example, <plugin ref>.exec('help:aiTaskParams').\n\n" << std::endl;
    }
    else if (helpTopic == "aiTaskParams")
    {
        std::cout << "The aiTaskParams argument takes the following parameters:\n\n" << std::endl;
        std::cout << "'SR, S, M, TC(optional)', where SR is the sample rate in Hz; S is the number \n" << std::endl;
        std::cout << "of samples to collect; M is the mode the acquisition will use;\n" << std::endl;
        std::cout << "and TC, an optional parameter, specifies an external trigger channel.\n" << std::endl;
        std::cout << "Mode may take one of the following values: 0 - finite, which means\n" << std::endl;
        std::cout << "stop when S samples are collected; 1 - continuous, which means continue\n" << std::endl;
        std::cout << "collecting samples until the task is stopped; and 2 - on demand, which\n" << std::endl;
        std::cout << "means (What?). TC is specified as: <TriggerChannel>,<rising/falling>,\n" << std::endl;
        std::cout << "where <TriggerChannel> is of the form /Device/. <rising/falling>\n" << std::endl;
        std::cout << "is either the word 'rising' or 'falling' (no quotes). The <TriggerChannel> \n" << std::endl;
        std::cout << "value may also have Programmable Function Interface identifier attached, \n" << std::endl;
        std::cout << "for example, PFI0 or PFI1. An example aiTaskParams argument is:\n" << std::endl;
        std::cout << "'20000,100,0,/Dev1/PFI0,rising', which means sample at 20,000 samples per second;\n" << std::endl;
        std::cout << "collect 100 samples using mode 0 and use the 0th Programmable Function Interface\n" << std::endl;
        std::cout << "on Device 1 with rising triggering. For example, this argument could be used in the setParam\n" << std::endl;
        std::cout << "command as follows: <plugin ref>.setParam('aiTaskParams','20000,100,0,/Dev1/PFI0,rising').\n" << std::endl;
        std::cout << "Without the TC optional parameter the command would be:\n" << std::endl;
        std::cout << "<plugin ref>.setParam('aiTaskParams','20000,100,0')\n\n" << std::endl;
    }
    else if (helpTopic == "ChParams")
    {
        std::cout << "ChParams are specified as follows:\n\n" << std::endl;
        std::cout << "xxChParams, where xx is one of 'ai', 'ao', 'di', 'do', 'ci', 'do',\n" << std::endl;
        std::cout << "these abbreviations corresponding respectively to 'analog input', 'analog output',\n" << std::endl;
        std::cout << "'digital input', 'digital output', 'counter input', and 'counter output'.\n" << std::endl;
        std::cout << "These are set using setParam() and read using getParam().For more information\n" << std::endl;
        std::cout << "on specific ChParmeters, type <plugin ref>.exec('help:xxChParams');\n" << std::endl;
        std::cout << "for example, <plugin ref>.exec('help:aiChParams').\n\n" << std::endl;
    }
    else if (helpTopic == "aiChParams")
    {
        std::cout << "The aiChParams argument takes the following parameters:\n\n" << std::endl;
        std::cout << "'Dev/Ch, M, minV, maxV', where Dev/Ch is the device and channel to apply the remaining\n" << std::endl;
        std::cout << "parameters; M is the mode; minV is the minimum of the voltage range and\n" << std::endl;
        std::cout << "maxV is the maximum of the voltage range. Mode may take one of the\n" << std::endl;
        std::cout << "following values: 0 -  default; 1 - differential; 2 - RSE mode; 3 - NRSE; 4 - Pseudodiff.\n" << std::endl;
        std::cout << "For example, these arguments could be used in the setParam command as follows:\n" << std::endl;
        std::cout << "<plugin ref>.setParam('aiChParams','Dev1/ai0,3,-10,10')\n\n" << std::endl;
    }

    return retValue;
}
