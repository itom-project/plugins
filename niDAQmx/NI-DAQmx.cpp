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

//----------------------------------------------------------------------------------------------------------------------------------
//! Constructor of Interface Class.
/*!
    \todo add necessary information about your plugin here.
*/
niDAQmxInterface::niDAQmxInterface()
{
    m_type = ito::typeDataIO | ito::typeADDA; //any grabber is a dataIO device AND its subtype grabber (bitmask -> therefore the OR-combination).
    setObjectName("niDAQmx");

    m_description = QObject::tr("niDAQmx");

    //for the docstring, please don't set any spaces at the beginning of the line.
/*    char docstring[] = \
"The plugin implements the DAQmx functions for analog-digital-converters from National Instruments. \n\
The installation needs the NI-DAQmx Library that can be downloaded from the NI website (http://www.ni.com/download/ni-daqmx-14.2/5046/en/)";
    m_detaildescription = QObject::tr(docstring);*/
    m_detaildescription = QObject::tr(
"The plugin implements the DAQmx functions for analog-digital-converters from National Instruments. \n\
The installation needs the NI-DAQmx Library that can be downloaded from the NI website (http://www.ni.com/download/ni-daqmx-14.2/5046/en/)");

    m_author = "Martin Hoppe, ITO, University Stuttgart";
    m_version = (PLUGIN_VERSION_MAJOR << 16) + (PLUGIN_VERSION_MINOR << 8) + PLUGIN_VERSION_PATCH;
    m_minItomVer = MINVERSION;
    m_maxItomVer = MAXVERSION;
    m_license = QObject::tr("licensed under LGPL");
    m_aboutThis = QObject::tr(GITVERSION); 
    
    m_initParamsMand.clear();

    ito::Param paramVal("device", ito::ParamBase::String, "Dev1", tr("Name of the target Device, Dev1 as default, cDAQ1Mod1 for compactDAQ single module Device. Other names see device description in NI MAX").toLatin1().data());
    m_initParamsOpt.append(paramVal);
}

//----------------------------------------------------------------------------------------------------------------------------------
niDAQmxInterface::~niDAQmxInterface()
{

}

//----------------------------------------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------------------------------------

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal niDAQmxInterface::getAddInInst(ito::AddInBase **addInInst)
{
    NEW_PLUGININSTANCE(niDAQmx) //the argument of the macro is the classname of the plugin
    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal niDAQmxInterface::closeThisInst(ito::AddInBase **addInInst)
{
   REMOVE_PLUGININSTANCE(niDAQmx) //the argument of the macro is the classname of the plugin
   return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------------------------------------

//----------------------------------------------------------------------------------------------------------------------------------
niDAQmx::niDAQmx() : AddInDataIO(), m_isgrabbing(false)
{
    ito::Param paramVal("device", ito::ParamBase::String | ito::ParamBase::In | ito::ParamBase::Readonly, "Dev1", tr("Name of the target device").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);

    // Create the six tasks
    m_taskMap.insert("ai", new niTask("ai"));
    m_taskMap.insert("ao", new niTask("ao"));
    m_taskMap.insert("di", new niTask("di"));
    m_taskMap.insert("do", new niTask("do"));
    m_taskMap.insert("ci", new niTask("ci"));
    m_taskMap.insert("co", new niTask("co"));

    // General Parameters
    paramVal = ito::Param("name", ito::ParamBase::String | ito::ParamBase::Readonly, "NI-DAQmx", NULL);    
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("channel", ito::ParamBase::String | ito::ParamBase::Readonly, "", NULL);    
    m_params.insert(paramVal.getName(), paramVal);

    // Channel Parameters
    paramVal = ito::Param("chAssociated", ito::ParamBase::String | ito::ParamBase::Readonly, "", tr("returns a list of all channels that are associated with their tasks").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal); // ai0,ai1,port0,... only the associated ports are listed
    paramVal = ito::Param("aiChParams", ito::ParamBase::String | ito::ParamBase::In, "", tr("Set Parameterset for analog input channel").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("aoChParams", ito::ParamBase::String | ito::ParamBase::In, "", tr("Set Parameterset for analog output channel").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("diChParams", ito::ParamBase::String | ito::ParamBase::In, "", tr("Set Parameterset for digital input channel").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("doChParams", ito::ParamBase::String | ito::ParamBase::In, "", tr("Set Parameterset for digital output channel").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("ciChParams", ito::ParamBase::String | ito::ParamBase::In, "", tr("Set Parameterset for counter input channel").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("coChParams", ito::ParamBase::String | ito::ParamBase::In, "", tr("Set Parameterset for counter output channel").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);

    // Task Parameters
    paramVal = ito::Param("taskStatus", ito::ParamBase::String | ito::ParamBase::Readonly, "Comma list of the six tasks aiX,aoX,diX,doX,...", NULL);    
    m_params.insert(paramVal.getName(), paramVal); // (X: -1=not initialized, 0=initialized, 1=running, further status possible
    paramVal = ito::Param("setValMode", ito::ParamBase::Int | ito::ParamBase::In, 1, 3, 0, tr("Defines whether the setVal data is ment for the analog, digital or counter task").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("aiTaskParams", ito::ParamBase::String | ito::ParamBase::In, "", tr("Set Parameterset for the analog input Task").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("aoTaskParams", ito::ParamBase::String | ito::ParamBase::In, "", tr("Set Parameterset for the analog output Task").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("diTaskParams", ito::ParamBase::String | ito::ParamBase::In, "", tr("Set Parameterset for the digital input Task").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("doTaskParams", ito::ParamBase::String | ito::ParamBase::In, "", tr("Set Parameterset for the digital output Task").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("ciTaskParams", ito::ParamBase::String | ito::ParamBase::In, "", tr("Set Parameterset for the counter input Task").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("coTaskParams", ito::ParamBase::String | ito::ParamBase::In, "", tr("Set Parameterset for the counter output Task").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
}

//----------------------------------------------------------------------------------------------------------------------------------
niDAQmx::~niDAQmx()
{
    foreach(niTask *val, m_taskMap)
    {
        delete val;
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal niDAQmx::init(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);

    retValue += m_params["device"].copyValueFrom(&(paramsOpt->at(0)));
    QString device = m_params["device"].getVal<char *>(); //borrowed reference

    char buffer[1024];
    memset(buffer, '\0', 1024 * sizeof(char));
    DAQmxGetSysDevNames(buffer, sizeof(buffer));

    if (!QString(buffer).split(",").contains(device, Qt::CaseSensitive))
    {
        retValue += ito::RetVal::format(ito::retError, 0, "Your device does not exist in your system. The following devices were found: %s", buffer);
    }

    // populate m_channels
    m_channels = niChannelList(device);

    QStringList dummyReads;
    dummyReads << "channel" << "chAssociated" << "aiChParams" << "aoChParams" << "diChParams" << "doChParams" << "ciChParams" << "coChParams" 
               << "taskStatus" << "aiTaskParams" << "aoTaskParams"<< "diTaskParams" << "doTaskParams" << "ciTaskParams" << "coTaskParams";

    foreach(const QString &p, dummyReads)
    {
        QSharedPointer<ito::Param> val(new ito::Param(p.toLatin1().data()));
        getParam(val, NULL);
    }

    emit parametersChanged(m_params);    

    if (waitCond)
    {
        waitCond->returnValue = retValue;
        waitCond->release();
    }

    setInitialized(true); //init method has been finished (independent on retval)
    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal niDAQmx::close(ItomSharedSemaphore *waitCond)
{
    //ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);
    //
    ////todo:
    //// - disconnect the device if not yet done
    //// - this funtion is considered to be the "inverse" of init.

    if (waitCond)
    {
        waitCond->returnValue = retValue;
        waitCond->release();
    }
    
    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal niDAQmx::getParam(QSharedPointer<ito::Param> val, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue;
    QString key;
    bool hasIndex = false;
    int index;
    QString suffix;
    QMap<QString,ito::Param>::iterator it;

    retValue += apiParseParamName(val->getName(), key, hasIndex, index, suffix);

    if (retValue == ito::retOk)
    { //gets the parameter key from m_params map (read-only is allowed, since we only want to get the value).
        retValue += apiGetParamFromMapByKey(m_params, key, it, false);
    }
    if (!retValue.containsError())
    {
        // General Parameters
        if (key == "name")
        {
            *val = it.value();
        }
        else if (key == "channel")
        {
            // Hier weitermachen
            QString ch = m_channels.getAllChannelAsString().join(",");
            m_params["channel"].setVal<char*>(ch.toLatin1().data(), ch.size());
            *val = it.value();
        }
        // Channel Paramteters
        else if (key == "chAssociated")
        {
            QStringList as;
            foreach(niTask* t, m_taskMap)
            {
                if (t->getChCount() > 0)
                {
                    as.append(t->getChList().join(","));
                }
            }
            QString res;
            res = as.join(";");
            m_params["chAssociated"].setVal<char*>(res.toLatin1().data(), res.size());
            *val = it.value();
        }
        else if (key == "aiChParams")
        {
            QString para = m_channels.getAllChParameters(niBaseChannel::chTypeAnalog, niBaseChannel::chIoInput).join(";");
            m_params["aiChParams"].setVal<char*>(para.toLatin1().data(), para.size());
            *val = it.value();
        }
        else if (key == "aoChParams")
        {
            QString para = m_channels.getAllChParameters(niBaseChannel::chTypeAnalog, niBaseChannel::chIoOutput).join(";");
            m_params["aoChParams"].setVal<char*>(para.toLatin1().data(), para.size());
            *val = it.value();
        }
        else if (key == "diChParams")
        {
            QString para = m_channels.getAllChParameters(niBaseChannel::chTypeDigital, niBaseChannel::chIoInput).join(";");
            m_params["diChParams"].setVal<char*>(para.toLatin1().data(), para.size());
            *val = it.value();
        }
        else if (key == "doChParams")
        {
            QString para = m_channels.getAllChParameters(niBaseChannel::chTypeDigital, niBaseChannel::chIoOutput).join(";");
            m_params["doChParams"].setVal<char*>(para.toLatin1().data(), para.size());
            *val = it.value();
        }
        else if (key == "ciChParams")
        {
            QString para = m_channels.getAllChParameters(niBaseChannel::chTypeCounter, niBaseChannel::chIoInput).join(";");
            m_params["ciChParams"].setVal<char*>(para.toLatin1().data(), para.size());
            *val = it.value();
        }
        else if (key == "coChParams")
        {
            QString para = m_channels.getAllChParameters(niBaseChannel::chTypeCounter, niBaseChannel::chIoOutput).join(";");
            m_params["coChParams"].setVal<char*>(para.toLatin1().data(), para.size());
            *val = it.value();
        }
        // Task Paramters
        else if (key == "taskStatus")
        {
            QStringList res;
            foreach(niTask* t, m_taskMap)
            {            
                QStringList ch;
                ch.append(t->getName());
                if (t->isInitialized())
                {
                    // Hier muss noch der status ausgelesen werden
                    // ist der Task running oder was auch immer
                    if (t->isDone())
                    {
                        ch.append("0");
                    }
                    else
                    {
                        ch.append("1");
                    }
                }
                else
                {
                    ch.append("-1");
                }
                res.append(ch.join(","));
            }
            QString resS = res.join(";");
            m_params["taskStatus"].setVal<char*>(resS.toLatin1().data(), resS.size());
            *val = it.value();
        }
        else if (key.right(10) == "TaskParams")
        {
            QStringList tl;                
            QString ch = key.left(2);
            if (m_taskMap.contains(ch))
            {
                if (m_taskMap.value(ch)->isInitialized())
                {
                    tl.append(QString::number(m_taskMap.value(ch)->getRateHz()));
                    tl.append(QString::number(m_taskMap.value(ch)->getSamplesToRW()));
                    tl.append(QString::number(m_taskMap.value(ch)->getMode()));
                    ch = tl.join(",");
                }
                else
                {
                    ch = "-1";
                }
            }
            m_params[key].setVal<char*>(ch.toLatin1().data(), ch.size());
            *val = it.value();
        }
        else
        {
            *val = it.value();
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
ito::RetVal niDAQmx::setParam(QSharedPointer<ito::ParamBase> val, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);
    QString key;
    bool hasIndex;
    int index;
    QString suffix;
    QMap<QString, ito::Param>::iterator it;

    ////parse the given parameter-name (if you support indexed or suffix-based parameters)
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
        if (key == "aiChParams")
        { // (dev-channel,inConfig,minInLim,maxInLim)
            QStringList in = QString(val->getVal<char*>()).split(",");
            if (m_channels.contains(in[0]) && in.size() == 3 && m_taskMap.value("ai")->isInitialized()) 
            {
                niAnalogInputChannel *ai = NULL;
                if (m_channels.value(in[0]) == NULL)
                { // channel does not exist yet
                    ai = new niAnalogInputChannel();
                }
                else
                { // channel already exists
                    ai = dynamic_cast<niAnalogInputChannel*>(m_channels.value(in[0]));
                }
                ai->setDevID(in[0].split('/')[0]);
                ai->setChID(in[0].split('/')[1]);
                ai->setAnalogInputConfig(in[1].toInt());
                ai->setInputLim(in[2].toInt());
                // Channel is finished and added to the corresponding input task
                retValue += ai->applyParameters(m_taskMap.value("ai"));
                // increase the corresponding counter
                m_channels.insert(in[0], ai);
                QString para = m_channels.getAllChParameters(niBaseChannel::chTypeAnalog, niBaseChannel::chIoInput).join(";");
                m_params["aiChParams"].setVal<char*>(para.toLatin1().data(), para.size());
                *val = it.value();
                retValue += it->copyValueFrom( &(*val));
            }
            else
            {
                retValue += ito::RetVal(ito::retError, 0, tr("Your device is not supporting this channel or the task is not initialized").toLatin1().data());
            }
        }
        else if (key == "aoChParams")
        { // (dev-channel,inConfig,minInLim,maxInLim)
            QStringList in = QString(val->getVal<char*>()).split(",");
            if (m_channels.contains(in[0]) && in.size() == 3 && m_taskMap.value("ao")->isInitialized()) 
            {
                niAnalogOutputChannel *ai = NULL;
                if (m_channels.value(in[0]) == NULL)
                { // channel does not exist yet
                    ai = new niAnalogOutputChannel();
                }
                else
                { // channel already exists
                    ai = dynamic_cast<niAnalogOutputChannel*>(m_channels.value(in[0]));
                }
                ai->setDevID(in[0].split('/')[0]);
                ai->setChID(in[0].split('/')[1]);
                ai->setMinOutputLim(in[1].toInt());
                ai->setMaxOutputLim(in[2].toInt());
                // Channel is finished and added to the corresponding input task
                retValue += ai->applyParameters(m_taskMap.value("ao"));
                // increase the corresponding counter
                m_channels.insert(in[0], ai);
                QString para = m_channels.getAllChParameters(niBaseChannel::chTypeAnalog, niBaseChannel::chIoOutput).join(";");
                m_params["aoChParams"].setVal<char*>(para.toLatin1().data(), para.size());
                *val = it.value();
                retValue += it->copyValueFrom( &(*val));
            }
            else
            {
                retValue += ito::RetVal(ito::retError, 0, tr("Your device is not supporting this channel or the task is not initialized").toLatin1().data());
            }
        }
        else if (key == "diChParams")
        { // (dev-channel,inConfig,minInLim,maxInLim)
            QStringList in = QString(val->getVal<char*>()).split(",");
            if (m_channels.contains(in[0]) && in.size() == 1 && m_taskMap.value("di")->isInitialized()) 
            {
                niDigitalInputChannel *ai = NULL;
                if (m_channels.value(in[0]) == NULL)
                { // channel does not exist yet
                    ai = new niDigitalInputChannel();
                }
                else
                { // channel already exists
                    ai = dynamic_cast<niDigitalInputChannel*>(m_channels.value(in[0]));
                }
                ai->setDevID(in[0].split('/')[0]);
                ai->setChID(in[0].split('/')[1]);
                // Channel is finished and added to the corresponding input task
                retValue += ai->applyParameters(m_taskMap.value("di"));
                // increase the corresponding counter
                m_channels.insert(in[0], ai);
                QString para = m_channels.getAllChParameters(niBaseChannel::chTypeDigital, niBaseChannel::chIoInput).join(";");
                m_params["diChParams"].setVal<char*>(para.toLatin1().data(), para.size());
                *val = it.value();
                retValue += it->copyValueFrom( &(*val));
            }
            else
            {
                retValue += ito::RetVal(ito::retError, 0, tr("Your device is not supporting this port or the task is not initialized").toLatin1().data());
            }
        }
        else if (key == "doChParams")
        { // (dev-channel,inConfig,minInLim,maxInLim)
            QStringList in = QString(val->getVal<char*>()).split(",");
            if (m_channels.contains(in[0]) && in.size() == 1 && m_taskMap.value("do")->isInitialized()) 
            {
                niDigitalOutputChannel *ai = NULL;
                if (m_channels.value(in[0]) == NULL)
                { // channel does not exist yet
                    ai = new niDigitalOutputChannel();
                }
                else
                { // channel already exists
                    ai = dynamic_cast<niDigitalOutputChannel*>(m_channels.value(in[0]));
                }
                ai->setDevID(in[0].split('/')[0]);
                ai->setChID(in[0].split('/')[1]);
                // Channel is finished and added to the corresponding input task
                retValue += ai->applyParameters(m_taskMap.value("do"));
                // increase the corresponding counter
                m_channels.insert(in[0], ai);
                QString para = m_channels.getAllChParameters(niBaseChannel::chTypeDigital, niBaseChannel::chIoOutput).join(";");
                m_params["doChParams"].setVal<char*>(para.toLatin1().data(), para.size());
                *val = it.value();
                retValue += it->copyValueFrom( &(*val));
            }
            else
            {
                retValue += ito::RetVal(ito::retError, 0, tr("Your device is not supporting this port or the task is not initialized").toLatin1().data());
            }
        }
        else if (key == "setValMode")
        {
            // 1 = Analog; 2 = Digital; 3 = Counter
            int mode = val->getVal<int>();
            if (mode == 1)
            {
                this->m_aOutIsAcquired = true;
            }
            else if (mode == 2)
            {
                this->m_dOutIsAcquired = true;
            }
            else if (mode == 3)
            {
                this->m_cOutIsAcquired = true;
            }
            else
            {
                retValue += ito::RetVal(ito::retError, 0, tr("The number you wanted to set doesn´t correspond to analog, digital or counter (1-3)").toLatin1().data());
            }

        }
        else if (key.right(10) == "TaskParams")
        {
            QString taskID = key.left(2);
            QStringList in = QString(val->getVal<char*>()).split(",");
            if (in.size() >= 3) // minimum parameter count ok
            {
                m_taskMap.value(taskID)->setRateHz(in[0].toInt());
                m_taskMap.value(taskID)->setSamplesToRW(in[1].toInt());
                m_taskMap.value(taskID)->setMode(in[2].toInt());
                retValue += it->copyValueFrom( &(*val) );
                if (in.size() > 3) // Trigger source defined
                {
                    m_taskMap.value(taskID)->setTriggerPort(in[3]);
                    if (in[4] == "rising")
                    {
                        m_taskMap.value(taskID)->setTriggerEdge(DAQmx_Val_Rising);
                    }
                    else if (in[4] == "falling")
                    {
                        m_taskMap.value(taskID)->setTriggerEdge(DAQmx_Val_Falling);
                    }
                }
            }
            else 
            {
                retValue += ito::RetVal(ito::retError, 0, tr("Wrong number of parameters (rate[Hz], samples[1], mode)").toLatin1().data());
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
        QStringList dummyReads;
        dummyReads << "channel" << "chAssociated" << "taskStatus";
        foreach(const QString &p, dummyReads)
        {
            QSharedPointer<ito::Param> val(new ito::Param(p.toLatin1().data()));
            getParam(val, NULL);
        }        
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
ito::RetVal niDAQmx::startDevice(ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);
    //
    //incGrabberStarted(); //increment a counter to see how many times startDevice has been called
    //
    ////todo:
    //// if this function has been called for the first time (grabberStartedCount() == 1),
    //// start the camera, allocate necessary buffers or do other work that is necessary
    //// to prepare the camera for image acquisitions.
    //
    //if (waitCond)
    //{
    //    waitCond->returnValue = retValue;
    //    waitCond->release();
    //}
    return retValue;
}
         
//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal niDAQmx::stopDevice(ItomSharedSemaphore *waitCond)
{
    //ItomSharedSemaphoreLocker locker(waitCond);
   ito::RetVal retValue(ito::retOk);

    //decGrabberStarted(); //decrements the counter (see startDevice)

    //if (grabberStartedCount() < 0)
    //{
    //    retValue += ito::RetVal(ito::retWarning, 0, tr("The grabber has already been stopped.").toLatin1().data());
    //    setGrabberStarted(0);
    //}
    //
    ////todo:
    //// if the counter (obtained by grabberStartedCount()) drops to zero again, stop the camera, free all allocated
    //// image buffers of the camera... (it is the opposite from all things that have been started, allocated... in startDevice)

    if (waitCond)
    {
        waitCond->returnValue = retValue;
        waitCond->release();
    }

    return ito::retOk;
}
         
//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal niDAQmx::acquire(const int trigger, ItomSharedSemaphore *waitCond)
{
    // The trigger is used here in another meaning. It s a bitmask and defines
    // which tasks are started! (all decimal)
    //  1 = Analog Input
    //  2 = Digital Input
    //  4 = Counter Input
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retval(ito::retOk);

    if (trigger & 1)
    {
        retval += m_taskMap.value("ai")->run();
        m_aInIsAcquired = true;
    }
    if (trigger & 2)
    {
        retval += m_taskMap.value("di")->run();
        m_dInIsAcquired = true;
    }
    if (trigger & 4)
    {
        retval += m_taskMap.value("ci")->run();
        m_cInIsAcquired = true;
    }

    if (!retval.containsError())
    {
        m_isgrabbing = true;
    }

    if (waitCond)
    {
        waitCond->returnValue = retval;
        waitCond->release();  
    }

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal niDAQmx::readAnalog(ito::DataObject *externalDataObject)
{
    ito::RetVal retValue(ito::retOk);

    ito::DataObject *dataObj = externalDataObject ? externalDataObject : &m_data;
    bool copyExternal = (externalDataObject != NULL);

    int channels = m_taskMap.value("ai")->getChCount();
    int samples = m_taskMap.value("ai")->getSamplesToRW();

    if (m_isgrabbing == false)
    {
        retValue += ito::RetVal(ito::retWarning, 0, tr("Tried to get picture without triggering exposure").toLatin1().data());
    }
    else
    {
        //step 1: create m_data (if not yet available)
        if (externalDataObject)
        {
            retValue += checkData(NULL, channels, samples); //update external object or m_data
            retValue += checkData(externalDataObject, channels, samples); //update external object
        }
        else
        {
            //not always necessary
            retValue += checkData(NULL, channels, samples); //update external object or m_data
        }

        int size = channels * samples;
        int32 retSize = -1;
        //int err = -1;
        //while (err != 0)
        //{
        //    err = DAQmxReadAnalogF64(*m_taskMap["ai"]->getTaskHandle(), -1, 1.0 /*sec*/, DAQmx_Val_GroupByChannel, (ito::float64*)m_data.rowPtr(0,0), size, &retSize, NULL);
        //    setAlive();
        //}

        int err = DAQmxReadAnalogF64(*m_taskMap["ai"]->getTaskHandle(), -1, DAQmx_Val_WaitInfinitely, DAQmx_Val_GroupByChannel, (ito::float64*)m_data.rowPtr(0,0), size, &retSize, NULL);
        double vScale = 1;
        double tScale = 1/(double)m_taskMap["ai"]->getRateHz();
        m_data.setAxisScale(1, tScale);
        m_data.setAxisUnit(1, "sec");
        m_data.setAxisDescription(1, "time");
        
        // TODO: Hier muss die Skala an den hoechsten Range angepasst werden und alle Reihen im Datenobject entsprechend ihres Ranges durchmultipliziert werden
        //m_channels m_taskMap.value("ai")->getChList()
        //m_channels.getAllChannelOfType(niBaseChannel::chTypeAnalog)[0]->
        m_data.setAxisScale(2, vScale);
        m_data.setAxisUnit(2, "volt");
        m_data.setAxisDescription(2, "voltage");


        if (copyExternal)
        {
            retValue += m_data.deepCopyPartial(*externalDataObject);
        }

        m_isgrabbing = false;
    }

    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
// not supported yet, because an external clock source is neccesary
ito::RetVal niDAQmx::readDigital(ito::DataObject *externalDataObject)
{
    // right now I only support 8 line Ports! Some devices have 16 or 32 lines per port! 
    // just use: DAQmxReadDigitalU16, DAQmxReadDigitalU32
    ito::RetVal retValue(ito::retOk);

    ito::DataObject *dataObj = externalDataObject ? externalDataObject : &m_data;
    bool copyExternal = (externalDataObject != NULL);

    int ports = m_taskMap.value("di")->getChCount();
    int samples = m_taskMap.value("di")->getSamplesToRW();

    if (m_isgrabbing == false)
    {
        retValue += ito::RetVal(ito::retWarning, 0, tr("Tried to get picture without triggering exposure").toLatin1().data());
    }
    else
    {
        //step 1: create m_data (if not yet available)
        if (externalDataObject)
        {
            retValue += checkData(NULL, ports*8, samples); //update external object or m_data
            retValue += checkData(externalDataObject, ports, samples); //update external object
        }
        else
        {
            //not always necessary
            retValue += checkData(NULL, ports, samples); //update external object or m_data
        }

        int size = ports * samples;
        int32 retSize = -1;
        int err = DAQmxReadDigitalU8(*m_taskMap.value("di")->getTaskHandle(), samples, DAQmx_Val_WaitInfinitely, DAQmx_Val_GroupByChannel, (ito::uint8*)m_data.rowPtr(0,0), size, &retSize, NULL); 
        m_data.setAxisDescription(0, "status");
        m_data.setAxisDescription(1, "lines");

        if (copyExternal)
        {
            retValue += m_data.deepCopyPartial(*externalDataObject);
        }

        m_isgrabbing = false;
    }

    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal niDAQmx::readCounter(ito::DataObject *externalDataObject)
{
    // TODO: Add Implementation
    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal niDAQmx::writeAnalog(const ito::DataObject *externalDataObject)
{
    ito::RetVal retValue(ito::retOk);
    int channels = m_taskMap.value("ao")->getChCount();
    int samples = m_taskMap.value("ao")->getSamplesToRW();
    int32 smplW = -1;
    /*if (m_isgrabbing == false)
    {
        retValue += ito::RetVal(ito::retWarning, 0, tr("Tried to set picture without triggering exposure").toLatin1().data());
    }*/
    int err = DAQmxWriteAnalogF64(*m_taskMap.value("ao")->getTaskHandle(), samples, true, 0, DAQmx_Val_GroupByChannel, (ito::float64*)externalDataObject->rowPtr(0,0), &smplW, NULL);

    if (err != 0)
    {
        retValue += ito::RetVal(ito::retError, 0, tr("Error occured").toLatin1().data());
    }
    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal niDAQmx::writeDigital(ito::DataObject *externalDataObject)
{
    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal niDAQmx::writeCounter(ito::DataObject *externalDataObject)
{
    return ito::retOk;
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
ito::RetVal niDAQmx::getVal(void *vpdObj, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);
    int error = -1;
    ito::DataObject *dObj = reinterpret_cast<ito::DataObject *>(vpdObj);
    //call retrieveData without argument. Retrieve data should then put the currently acquired image into the dataObject m_data of the camera.
    
    if (m_aInIsAcquired)
    {
        retValue += readAnalog();
        m_aInIsAcquired = false;
        // Die folgende zeile stoppt den task um ihn erneut starten zu koennen. Rsourcen bleiben erhalten. Vielleicht in extra funktion auslagern
        // error = DAQmxTaskControl(m_taskMap.value("ai")->getTaskHandle(),DAQmx_Val_Task_Reserve);
        retValue += m_taskMap.value("ai")->stop();
    }
    else if (m_dInIsAcquired)
    {
        retValue += ito::RetVal(ito::retError, 0, tr("The digital read mode is not supported yet, because an external clock source is neccesary").toLatin1().data());
        // retValue += readDigital();
        //m_dInIsAcquired = false;
    }
    else if (m_cInIsAcquired)
    {
        retValue += ito::RetVal(ito::retWarning, 0, tr("The counter read mode is not supported yet").toLatin1().data());
        retValue += readCounter();
        m_cInIsAcquired = false;
    }

    if (!retValue.containsError())
    {
        if (dObj)
        {
            (*dObj) = m_data; //copy reference to externally given object
        }
    }

    retValue += m_taskMap.value("ai")->resetTaskHandle();

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
ito::RetVal niDAQmx::copyVal(void *vpdObj, ItomSharedSemaphore *waitCond)
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
        if (m_aInIsAcquired)
        {
            retValue += readAnalog();
            m_aInIsAcquired = false;
        }
        else if (m_dInIsAcquired)
        {
            retValue += ito::RetVal(ito::retError, 0, tr("The digital read mode is not supported yet, because an external clock source is neccesary").toLatin1().data());
            //retValue += readDigital();
            //m_dInIsAcquired = false;
        }
        else if (m_cInIsAcquired)
        {
            retValue += readCounter();
            m_cInIsAcquired = false;
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
ito::RetVal niDAQmx::setVal(const char *data, const int length, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);
    
    const ito::DataObject *dObj = reinterpret_cast<const ito::DataObject*>(data);

    int error = 0;

    if (dObj->getDims() != 2 || dObj->getSize(0) != m_taskMap.value("ao")->getChCount() || dObj->getSize(1) != m_taskMap.value("ao")->getSamplesToRW())
    {
        retValue += ito::RetVal::format(ito::retWarning, 0, "Error occured: given Dataobject has wrong dimensions.",0);
    }

    if (!retValue.containsError())
    {
        //this method calls retrieveData with the passed dataObject as argument such that retrieveData is able to copy the image obtained
        //by the camera directly into the given, external dataObject
        if (m_aOutIsAcquired)
        {
            retValue += writeAnalog(dObj);
            m_taskMap.value("ao")->applyParameters();
            error = DAQmxStartTask(*m_taskMap.value("ao")->getTaskHandle());
            m_aOutIsAcquired = false;
        }
        else if (m_dOutIsAcquired)
        {
            retValue += writeDigital();
            m_taskMap.value("do")->applyParameters();
            error = DAQmxStartTask(*m_taskMap.value("do")->getTaskHandle());
            m_dOutIsAcquired = false;
        }
        else if (m_cOutIsAcquired)
        {
            retValue += writeCounter();
            m_taskMap.value("co")->applyParameters();
            error = DAQmxStartTask(*m_taskMap.value("co")->getTaskHandle());
            m_cOutIsAcquired = false;
        }
    }

    if (error > 0)
    {
        retValue += ito::RetVal::format(ito::retWarning, 0, "Warning occured while starting read task. \n Code: %i", error);
    }
    else if (error < 0)
    {
        retValue += ito::RetVal::format(ito::retError, 0, "Error occured while starting read task. \n Code: %i", error);
    }
    else
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
//! slot called if the dock widget of the plugin becomes (in)visible
/*!
    Overwrite this method if the plugin has a dock widget. If so, you can connect the parametersChanged signal of the plugin
    with the dock widget once its becomes visible such that no resources are used if the dock widget is not visible. Right after
    a re-connection emit parametersChanged(m_params) in order to send the current status of all plugin parameters to the dock widget.
*/
void niDAQmx::dockWidgetVisibilityChanged(bool visible)
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
const ito::RetVal niDAQmx::showConfDialog(void)
{
    return apiShowConfigurationDialog(this, new DialogNiDAQmx(this, (void*)this));
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal niDAQmx::checkData(ito::DataObject *externalDataObject, int channels, int samples)
{
    if (externalDataObject == NULL)
    {
        //check internal object m_data
        if (m_data.getDims() != 2 || m_data.getSize(0) != (unsigned int)channels || m_data.getSize(1) != (unsigned int)samples || m_data.getType() != ito::tFloat64)
        {
            m_data = ito::DataObject(channels, samples, ito::tFloat64);
        }
    }
    else
    {
        int dims = externalDataObject->getDims();
        if (externalDataObject->getDims() == 0)
        {
            *externalDataObject = ito::DataObject(channels, samples, ito::tFloat64);
        }
        else if (externalDataObject->calcNumMats () > 1)
        {
            return ito::RetVal(ito::retError, 0, tr("Error during check data, external dataObject invalid. Object has more than 1 plane. It must be of right size and type or a uninitilized image.").toLatin1().data());            
        }
        else if (externalDataObject->getSize(dims - 2) != (unsigned int)channels || externalDataObject->getSize(dims - 1) != (unsigned int)samples || externalDataObject->getType() != ito::tFloat64)
        {
            return ito::RetVal(ito::retError, 0, tr("Error during check data, external dataObject invalid. Object must be of right size and type or a uninitilized image.").toLatin1().data());
        }
    }

    return ito::retOk;
}