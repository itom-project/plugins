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
#include <iostream>

//----------------------------------------------------------------------------------------------------------------------------------
#if QT_VERSION < 0x050000
    Q_EXPORT_PLUGIN2(niDAQmxinterface, niDAQmxInterface) //the second parameter must correspond to the class-name of the interface class, the first parameter is arbitrary (usually the same with small letters only)
#endif

//#include "dockWidgetniDAQmx.h"

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
The installation needs the NI-DAQmx Library that can be downloaded from the NI website \n(http://www.ni.com/download/ni-daqmx-14.2/5046/en/).\n\n\
Basic plugin documentation is found in ";
    char last[] = ". \nOnline help is available through <plugin_ref>.exec('help').";
    strcpy (docstring, first);
    strcat (docstring, dirpath.c_str());
    strcat (docstring, pathSeparator);
    strcat (docstring, "doc");
    strcat (docstring, pathSeparator);
    strcat (docstring, "niDAQmx.rst");
    strcat (docstring, last);

    m_detaildescription = QObject::tr(docstring);

    m_author = "Martin Hoppe, ITO, University Stuttgart; Dan Nessett, Unaffiliated";
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
    paramVal = ito::Param("configForTesting", ito::ParamBase::String | ito::ParamBase::In, "", tr("Set Parameterset for testing").toLatin1().data());  
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
    paramVal = ito::Param("setValMode", ito::ParamBase::Int | ito::ParamBase::In, 1, 3, 0, tr("Defines whether the setVal data is meant for the analog, digital or counter task").toLatin1().data());
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
    registerExecFunc("help:taskStatus", pMand, pOpt, pOut, tr("Prints information taskStatus plugin method."));

    //end register Exec Functions

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

    // need to get rid of "spaces" in buffer

    QString s(buffer);
    s.replace(QString(" "), QString(""));
																																																																																																																																																								
    if (!s.split(",").contains(device, Qt::CaseSensitive))
    {
        retValue += ito::RetVal::format(ito::retError, 0, "The specified device does not exist in your system. The following devices were found: %s", buffer);
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

    setInitialized(true); //init method has been finished (independent of retval)
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
    //// - this function is considered to be the "inverse" of init.

    // stop and reset all tasks

    retValue += resetTask(QString("ai"));
    retValue += resetTask(QString("di"));
    retValue += resetTask(QString("ci"));
    retValue += resetTask(QString("ao"));
    retValue += resetTask(QString("do"));
    retValue += resetTask(QString("co"));

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
        else if (key == "configForTesting")
        {
            m_params[key].setVal<char*>(m_configForTesting.toLatin1().data(), m_configForTesting.size());
            *val = it.value();
        }
        // Channel Parameters
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
            QRegularExpression first_comma("^([^,]+),(.*)$");
            QString para = m_channels.getAllChParameters(niBaseChannel::chTypeAnalog, niBaseChannel::chIoInput).join(";");
            
            // Replace comma between Device and Channel ids with '/'
            para.replace(first_comma, "\\1/\\2");

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
            retValue += ito::RetVal(ito::retError, 0, tr("niDAQmx::getParam - ciChParams. Counter input is not implemented").toLatin1().data());
 //           QString para = m_channels.getAllChParameters(niBaseChannel::chTypeCounter, niBaseChannel::chIoInput).join(";");
 //           m_params["ciChParams"].setVal<char*>(para.toLatin1().data(), para.size());
 //           *val = it.value();
        }
        else if (key == "coChParams")
        {
            retValue += ito::RetVal(ito::retError, 0, tr("niDAQmx::getParam - coChParams. Counter output is not implemented").toLatin1().data());
 //           QString para = m_channels.getAllChParameters(niBaseChannel::chTypeCounter, niBaseChannel::chIoOutput).join(";");
 //           m_params["coChParams"].setVal<char*>(para.toLatin1().data(), para.size());
 //           *val = it.value();
        }
        // Task Paramters
        else if (key == "taskStatus")
        {
            QStringList res;
            foreach(niTask* t, m_taskMap)
            {            
                QStringList ch;
                ch.append(t->getName());
                if (t->getTaskParamsInitialized())
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

            if(ch == "ai")
            {
                if (m_taskMap.contains(ch))
                {
                    if (m_taskMap.value(ch)->taskParamsValid())
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
            else if(ch == "ao")
            {
                retValue += ito::RetVal(ito::retError, 0, tr("niDAQmx::getParam - aoTaskParams. aoTaskParams is not implemented").toLatin1().data());
            }
            else if(ch == "di")
            {
                retValue += ito::RetVal(ito::retError, 0, tr("niDAQmx::getParam - diTaskParams. diTaskParams is not implemented").toLatin1().data());
            }
            else if(ch == "do")
            {
                retValue += ito::RetVal(ito::retError, 0, tr("niDAQmx::getParam - doTaskParams. doTaskParams is not implemented").toLatin1().data());
            }
            else if(ch == "ci")
            {
                retValue += ito::RetVal(ito::retError, 0, tr("niDAQmx::getParam - ciTaskParams. ciTaskParams is not implemented").toLatin1().data());
            }
            else if(ch == "co")
            {
                retValue += ito::RetVal(ito::retError, 0, tr("niDAQmx::getParam - coTaskParams. coTaskParams is not implemented").toLatin1().data());
            }
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
        { // (dev-channel,inConfig, MinOutputVoltage, MaxOutputVoltage)
            QStringList in = QString(val->getVal<char*>()).split(",");
            if ( m_channels.contains(in[0]) && in.size() == 4 && (m_taskMap.value("ai")->getTaskParamsInitialized() || m_taskMap.value("ai")->getIgnoreTaskParamsInitialization()) ) 
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
                ai->setMinOutputLim(in[2].toInt());
                ai->setMaxOutputLim(in[3].toInt());
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
                retValue += ito::RetVal(ito::retError, 0, tr("niDAQmx::setParam - aiChParams. Your device does not support this channel or the task is not initialized").toLatin1().data());
            }
        }
        else if (key == "aoChParams")
        { // (dev-channel,minInLim,maxInLim)
            QStringList in = QString(val->getVal<char*>()).split(",");
            if (m_channels.contains(in[0]) && in.size() == 3 && m_taskMap.value("ao")->getTaskParamsInitialized()) 
            {
                niAnalogOutputChannel *ao = NULL;
                if (m_channels.value(in[0]) == NULL)
                { // channel does not exist yet
                    ao = new niAnalogOutputChannel();
                }
                else
                { // channel already exists
                    ao = dynamic_cast<niAnalogOutputChannel*>(m_channels.value(in[0]));
                }
                ao->setDevID(in[0].split('/')[0]);
                ao->setChID(in[0].split('/')[1]);
                ao->setMinOutputLim(in[1].toInt());
                ao->setMaxOutputLim(in[2].toInt());
                // Channel is finished and added to the corresponding input task
                retValue += ao->applyParameters(m_taskMap.value("ao"));
                // increase the corresponding counter
                m_channels.insert(in[0], ao);
                QString para = m_channels.getAllChParameters(niBaseChannel::chTypeAnalog, niBaseChannel::chIoOutput).join(";");
                m_params["aoChParams"].setVal<char*>(para.toLatin1().data(), para.size());
                *val = it.value();
                retValue += it->copyValueFrom( &(*val));
            }
            else
            {
                retValue += ito::RetVal(ito::retError, 0, tr("niDAQmx::setParam - aoChParams. Your device does not support this channel or the task is not initialized").toLatin1().data());
            }
        }
        else if (key == "diChParams")
        { // (dev-channel)
            QStringList in = QString(val->getVal<char*>()).split(",");
            if (m_channels.contains(in[0]) && in.size() == 1 && m_taskMap.value("di")->getTaskParamsInitialized()) 
            {
                niDigitalInputChannel *di = NULL;
                if (m_channels.value(in[0]) == NULL)
                { // channel does not exist yet
                    di = new niDigitalInputChannel();
                }
                else
                { // channel already exists
                    di = dynamic_cast<niDigitalInputChannel*>(m_channels.value(in[0]));
                }
                di->setDevID(in[0].split('/')[0]);
                di->setChID(in[0].split('/')[1]);
                // Channel is finished and added to the corresponding input task
                retValue += di->applyParameters(m_taskMap.value("di"));
                // increase the corresponding counter
                m_channels.insert(in[0], di);
                QString para = m_channels.getAllChParameters(niBaseChannel::chTypeDigital, niBaseChannel::chIoInput).join(";");
                m_params["diChParams"].setVal<char*>(para.toLatin1().data(), para.size());
                *val = it.value();
                retValue += it->copyValueFrom( &(*val));
            }
            else
            {
                retValue += ito::RetVal(ito::retError, 0, tr("niDAQmx::setParam - diChParams. Your device does not support this port or the task is not initialized").toLatin1().data());
            }
        }
        else if (key == "doChParams")
        { // (dev-channel)
            QStringList in = QString(val->getVal<char*>()).split(",");
            if (m_channels.contains(in[0]) && in.size() == 1 && m_taskMap.value("do")->getTaskParamsInitialized()) 
            {
                niDigitalOutputChannel *dout = NULL; // "do" is a keyword; have to use something else, i.e., dout
                if (m_channels.value(in[0]) == NULL)
                { // channel does not exist yet
                    dout = new niDigitalOutputChannel();
                }
                else
                { // channel already exists
                    dout = dynamic_cast<niDigitalOutputChannel*>(m_channels.value(in[0]));
                }
                dout->setDevID(in[0].split('/')[0]);
                dout->setChID(in[0].split('/')[1]);
                // Channel is finished and added to the corresponding input task
                retValue += dout->applyParameters(m_taskMap.value("do"));
                // increase the corresponding counter
                m_channels.insert(in[0], dout);
                QString para = m_channels.getAllChParameters(niBaseChannel::chTypeDigital, niBaseChannel::chIoOutput).join(";");
                m_params["doChParams"].setVal<char*>(para.toLatin1().data(), para.size());
                *val = it.value();
                retValue += it->copyValueFrom( &(*val));
            }
            else
            {
                retValue += ito::RetVal(ito::retError, 0, tr("niDAQmx::setParam - doChParams. Your device does not support this port or the task is not initialized").toLatin1().data());
            }
        }
        else if (key == "ciChParams")
        {
            retValue += ito::RetVal(ito::retError, 0, tr("niDAQmx::setParam - ciChParams. Counter input is not implemented").toLatin1().data());
        }
        else if (key == "coChParams")
        {
            retValue += ito::RetVal(ito::retError, 0, tr("niDAQmx::setParam - coChParams. Counter output is not implemented").toLatin1().data());
        }
        else if (key == "setValMode")
        {

        // Note: We will never get here if the mode value is not between 1-3. Such an error is caught by the call to apiValidateParam()
        // at the beginning of setParam()

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
        }
        else if (key.right(10) == "TaskParams")
        {
            QString taskID = key.left(2);
            QStringList in = QString(val->getVal<char*>()).split(",");

            if(taskID == "ai")
            {
                // aiTaskParams takes 3, 4 or 5 values; otherwise an error
                if( in.size() < 3 || in.size() > 5 )
                {
                    retValue += ito::RetVal(ito::retError, 0, tr("niDAQmx::setParam - TaskParams. Wrong number of parameters (rate[Hz], samples[1], mode)").toLatin1().data());
                }

                // Invalid mode identifier? (getPassThroughToPeripheralClasses should only be true when unittesting for error reporting functionality
                // In that case, we want the mode set eventhough it is in error, so that underlying functionality in the NI-PeripheralClasses can be tested)

                else if(in[2].toInt() > 2 && !m_taskMap.value("ai")->getPassThroughToPeripheralClasses())
                {
                    retValue += ito::RetVal(ito::retError, 0, tr("niDAQmx::setParam - TaskParams. Task Parmaters mode invalid").toLatin1().data());
                }
                // on demand mode is not yet supported. Catch it now before things get out of hand.
                else if(in[2].toInt() == 2)
                {
                    retValue += ito::RetVal(ito::retError, 0, tr("niDAQmx::setParam - TaskParams. On demand mode is not yet supported").toLatin1().data());
                }
                else
                {
                    m_taskMap.value(taskID)->setRateHz(in[0].toInt());
                    m_taskMap.value(taskID)->setSamplesToRW(in[1].toInt());
                    m_taskMap.value(taskID)->setMode(in[2].toInt());
                    retValue += it->copyValueFrom( &(*val) );
                    if (in.size() >= 4) // Trigger source defined
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

                    m_taskMap.value(taskID)->setTaskParamsInitialized();
                }
            }
            else if(taskID == "ao")
            {
                retValue += ito::RetVal(ito::retError, 0, tr("niDAQmx::setParam - aoTaskParams. aoTaskParams is not implemented").toLatin1().data());
            }
            else if(taskID == "di")
            {
                retValue += ito::RetVal(ito::retError, 0, tr("niDAQmx::setParam - diTaskParams. diTaskParams is not implemented").toLatin1().data());
            }
            else if(taskID == "do")
            {
                retValue += ito::RetVal(ito::retError, 0, tr("niDAQmx::setParam - doTaskParams. doTaskParams is not implemented").toLatin1().data());
            }
            else if(taskID == "ci")
            {
                retValue += ito::RetVal(ito::retError, 0, tr("niDAQmx::setParam - ciTaskParams. ciTaskParams is not implemented").toLatin1().data());
            }
            else if(taskID == "co")
            {
                retValue += ito::RetVal(ito::retError, 0, tr("niDAQmx::setParam - coTaskParams. coTaskParams is not implemented").toLatin1().data());
            }
        }
        else if (key == "configForTesting")
        {
            // Record the flags in m_configForTesting by concatenating those specified here with those already there.

            if(!m_configForTesting.isEmpty())
            {
                m_configForTesting.append(QString(","));
            }

            m_configForTesting.append(QString(val->getVal<char*>()));

            // Enable various flags that support testing the plugin. Should not be specified when the plugin is used operationally.

            QStringList in = QString(val->getVal<char*>()).split(",");
            QMap<QString, niTask*>::iterator taskMapIterator;
            QStringList::iterator flag;
            for(taskMapIterator = m_taskMap.begin(); taskMapIterator != m_taskMap.end(); taskMapIterator++)
            {
                for(flag = in.begin(); flag != in.end(); flag++)
                {
                    if(*flag == "ignoreTaskParamInit")
                    {
                        (*taskMapIterator)->setIgnoreTaskParamsInitialization();
                    }
                    if(*flag == "passThroughToPeripheralClasses")
                    {
                        (*taskMapIterator)->setPassThroughToPeripheralClasses();
                    }
                }
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
    ito::RetVal retValue;

    // startDevice is not used in this plugin.

    retValue += ito::RetVal(ito::retError, 0, tr("niDAQmx::startDevice. Warning: startDevice() is not used in niDAQmx. It is a NOOP").toLatin1().data());;

    if (waitCond)
    {
        waitCond->returnValue = retValue;
        waitCond->release();
    }

    return retValue;
}
         
//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal niDAQmx::stopDevice(ItomSharedSemaphore *waitCond)
{
    //ItomSharedSemaphoreLocker locker(waitCond);
   ito::RetVal retValue;

    // stopDevice is not used in this plugin.

    retValue += ito::RetVal(ito::retError, 0, tr("niDAQmx::stopDevice. Warning: stopDevice() is not used in niDAQmx It is a NOOP").toLatin1().data());;

    if (waitCond)
    {
        waitCond->returnValue = retValue;
        waitCond->release();
    }

    return retValue;
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

    if ( m_aInIsAcquired || m_dInIsAcquired || m_cInIsAcquired )
    {
        // acquire was called twice without an intervening call to getVal or copyVal.
        m_aInIsAcquired = m_dInIsAcquired = m_cInIsAcquired = false;
        retval += resetTask(QString("ai"));
        retval += resetTask(QString("di"));
        retval += resetTask(QString("ci"));
        retval += ito::RetVal(ito::retError, 0, tr("niDAQmx::acquire - acquire was called twice without an intervening call to getVal or copyVal").toLatin1().data());
    }
    else if (trigger & 1)
    {
        retval += m_taskMap.value("ai")->run();
        m_aInIsAcquired = true;
    }
    else if (trigger & 2)
    {
        retval += m_taskMap.value("di")->run();
        m_dInIsAcquired = true;
    }
    else if (trigger & 4)
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
ito::RetVal niDAQmx::readAnalog()
{
    ito::RetVal retValue(ito::retOk);

    int channels = m_taskMap.value("ai")->getChCount();
    int samples = m_taskMap.value("ai")->getSamplesToRW();

    if (m_isgrabbing == false)
    {
        retValue += ito::RetVal(ito::retError, 0, tr("niDAQmx::readAnalog - Tried to return data without first calling acquire").toLatin1().data());
    }
    else
    {
        //step 1: create and check m_data (if not yet available)
        retValue += checkData(NULL, channels, samples); //update external object or m_data

        if (!retValue.containsError())
        {
            if (m_taskMap.value("ai")->getMode() == 0)
            {
                int size = channels * samples;
                int32 retSize = -1;

                //direct grab into the internal buffer of m_data!
                int err = DAQmxReadAnalogF64(*m_taskMap["ai"]->getTaskHandle(), -1, DAQmx_Val_WaitInfinitely, DAQmx_Val_GroupByChannel, (ito::float64*)m_data.rowPtr(0, 0), size, &retSize, NULL);

                double tScale = 1 / (double)m_taskMap["ai"]->getRateHz();
                m_data.setAxisScale(1, tScale);
                m_data.setAxisUnit(1, "s");
                m_data.setAxisDescription(1, "time");

                m_data.setValueUnit("volt"); //marc: todo: is the unit 'volt' correct? If there is a need to scale the values to a certain unit, this has to be done in a for loop, since DataObjects don't have a valueScale or valueOffset property (only for axes)
                m_data.setValueDescription("voltage");
                m_isgrabbing = false;
            }
            else if (m_taskMap.value("ai")->getMode() == 1)
            {
                retValue += ito::RetVal(ito::retError, 0, tr("niDAQmx::readAnalog - Continuous mode is not yet supported").toLatin1().data());
            }
            else if (m_taskMap.value("ai")->getMode() == 2)
            {
                retValue += ito::RetVal(ito::retError, 0, tr("niDAQmx::readAnalog - On demand mode is not yet supported").toLatin1().data());
            }
            else
            {
                retValue += ito::RetVal(ito::retError, 0, tr("niDAQmx::readAnalog - internal error invalid mode passed to readAnalog").toLatin1().data());
            }
        }
    }

    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
// not supported yet, because an external clock source is neccesary, marc: todo: similar to readAnalog
ito::RetVal niDAQmx::readDigital()
{
    // right now I only support 8 line Ports! Some devices have 16 or 32 lines per port! 
    // just use: DAQmxReadDigitalU16, DAQmxReadDigitalU32
    ito::RetVal retValue(ito::retOk);

    int ports = m_taskMap.value("di")->getChCount();
    int samples = m_taskMap.value("di")->getSamplesToRW();

    if (m_isgrabbing == false)
    {
        retValue += ito::RetVal(ito::retError, 0, tr("niDAQmx::readDigital - Tried to return data without first calling acquire").toLatin1().data());
    }
    else
    {
        //step 1: create and check m_data (if not yet available)
        retValue += checkData(NULL, ports*8, samples); //update external object or m_data

        if (!retValue.containsError())
        {
            int size = ports * 8 * samples;
            int32 retSize = -1;

            //direct grab into the internal buffer of m_data!
            int err = DAQmxReadDigitalU8(*m_taskMap.value("di")->getTaskHandle(), samples, DAQmx_Val_WaitInfinitely, DAQmx_Val_GroupByChannel, (ito::uint8*)m_data.rowPtr(0,0), size, &retSize, NULL);

	    // If DAQmxReadDigitalU8 returned an error or warning, translate the numerical code into text and attach a prefix

	    retValue += checkError(err, "niDAQmx::readDigital - DAQmxReadDigitalU8 abnormal return");

            double tScale = 1 / (double)m_taskMap["di"]->getRateHz();
            m_data.setAxisScale(1, tScale);
            m_data.setAxisUnit(1, "s");
            m_data.setAxisDescription(1, "time");

            m_data.setValueUnit("ZeroOrOne"); //marc: todo: is the unit 'volt' correct? If there is a need to scale the values to a certain unit, this has to be done in a for loop, since DataObjects don't have a valueScale or valueOffset property (only for axes)
            m_data.setValueDescription("BinaryValue");
        }

        m_isgrabbing = false;
    }

    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
//marc: todo: similar to readAnalog
ito::RetVal niDAQmx::readCounter()
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

	// If DAQmxWriteAnalogF64 returned an error or warning, translate the numerical code into text and attach a prefix

	retValue += checkError(err, "niDAQmx::writeAnalog - DAQmxWriteAnalogF64 abnormal return");

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
//! Returns the grabbed niDAQmx device data as reference.
/*!
    This method returns a reference to the recently acquired niDAQmx data. Therefore this data must fit into the data structure of the 
    DataObject.
    
    This method returns a reference to the internal dataObject m_data of the niDAQmx device where the currently acquired data is copied to (either
    in the acquire method or in retrieve data). Please remember, that the reference may directly change if a set of data is acquired.

    \param [in,out] vpdObj is the pointer to a given dataObject (this pointer should be cast to ito::DataObject*). After the call, the dataObject is a reference to the internal m_data dataObject of the niDAQmx device.
    \param [in] waitCond is the semaphore (default: NULL), which is released if this method has been terminated
    \return retOk if everything is ok, retError occurs when the niDAQmx device has not been started or no data has been acquired by the method acquire.
    
    \sa retrieveImage, copyVal
*/
ito::RetVal niDAQmx::getVal(void *vpdObj, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);
    int error = -1;
    ito::DataObject *dObj = reinterpret_cast<ito::DataObject *>(vpdObj);

//    *dObj = this->m_data; //dObj is now a shallow copy of the internal object m_data.

    if (!retValue.containsError())
    {
        retValue += retrieveData(static_cast<int*>(nullptr), static_cast<int*>(nullptr));
    }

    *dObj = this->m_data; //dObj is now a shallow copy of the internal object m_data.

    if (waitCond) 
    {
        waitCond->returnValue = retValue;
        waitCond->release();
    }

    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
//! Returns the grabbed niDAQmx device data as a deep copy.
/*!
    This method copies the recently grabbed niDAQmx device data to the given DataObject. 
    
    The given dataObject must either have an empty size (then it is resized to the size and type of the niDAQmx device data) or its size or adjusted region of
    interest must exactly fit to the size of the niDAQmx device data. Then, the acquired data is copied inside of the given region of interest.

    \param [in,out] vpdObj is the pointer to a given dataObject (this pointer should be cast to ito::DataObject*) where the acquired data is deep copied to.
    \param [in] waitCond is the semaphore (default: NULL), which is released if this method has been terminated
    \return retOk if everything is ok, retError is niDAQmx device has not been started or no data has been acquired by the method acquire.
    
    \sa retrieveImage, getVal
*/
ito::RetVal niDAQmx::copyVal(void *vpdObj, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);
    ito::DataObject *externalDataObject = reinterpret_cast<ito::DataObject *>(vpdObj);
    int channels;
    int samples;

    if (!externalDataObject)
    {
        retValue += ito::RetVal(ito::retError, 0, tr("niDAQmx::copyVal - Empty object handle provided by caller").toLatin1().data());
    }

    retValue += retrieveData(&channels, &samples); //retrieve data from device and store it in m_data

    if (!retValue.containsError())
    {
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
ito::RetVal niDAQmx::retrieveData(int *channels, int *samples)
{
    ito::RetVal retValue(ito::retOk);

    if( !(m_aInIsAcquired || m_dInIsAcquired || m_cInIsAcquired) )
    {
        retValue += ito::RetVal(ito::retError, 0, tr("niDAQmx::retrieveData - cannot retrieve data because none was acquired.").toLatin1().data());
    }

    if (m_aInIsAcquired)
    {
        retValue += readAnalog(); //marc: read out analog values from started task and put the results into the internal m_data object
    }
    else if (m_dInIsAcquired)
    {
        //marc: todo
        retValue += ito::RetVal(ito::retError, 0, tr("The digital read mode is not supported yet, because an external clock source is neccesary").toLatin1().data());
        // retValue += readDigital();
    }
    else if (m_cInIsAcquired)
    {
        //marc: todo
        retValue += ito::RetVal(ito::retError, 0, tr("The counter read mode is not supported yet").toLatin1().data());
        // retValue += readCounter();
    }

    if (!retValue.containsError())
    {
        retValue += manageTasks();
    }

    if (!retValue.containsError() && channels != NULL && samples != NULL)
    {
        *channels = m_taskMap.value("ai")->getChCount();
        *samples = m_taskMap.value("ai")->getSamplesToRW();
    }

    return retValue;
}

//---------------------------------------------------------------------------------------------
/* manageTasks is used by retrieveData to clear the relevant ni tasks when
   in "finite" mode. When in "continuous" mode it simply returns without 
   changing any task state.
*/
ito::RetVal niDAQmx::manageTasks()
{
    ito::RetVal retValue(ito::retOk);

    if (m_aInIsAcquired)
    {
        if (m_taskMap.value("ai")->getMode() == 1)
        {
            retValue += resetTask(QString("ai"));
            m_aInIsAcquired =  false;
        }
    }
    if (m_dInIsAcquired)
    {
        if(m_taskMap.value("di")->getMode() == 1)
        {
            retValue += resetTask(QString("di"));
            m_aInIsAcquired =  false;
        }
    }
    if (m_cInIsAcquired)
    {
        if(m_taskMap.value("ci")->getMode() == 1)
        {
            retValue += resetTask(QString("ci"));
            m_aInIsAcquired =  false;
        }
    }

    return retValue;
}


//---------------------------------------------------------------------------------------------
/* resetTask is used by manageTasks and acquire to stop the identifed task and reset
   its task handle.
*/
ito::RetVal niDAQmx::resetTask(QString task)
{
    ito::RetVal retValue(ito::retOk);

        // no need to stop the task, since resetTaskHandle calls DAQmxClearTask, which calls DAQmxStopTask
        retValue += m_taskMap.value(task)->resetTaskHandle();

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
        retValue += ito::RetVal(ito::retError, 0, tr("niDAQmx::setVal - Error occured: given Dataobject has wrong dimensions.").toLatin1().data());
    }

    if (!retValue.containsError())
    {
        //this method calls retrieveData with the passed dataObject as argument such that retrieveData is able to copy the data obtained
        //by the niDAQmx device directly into the given, external dataObject
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
    

        // If DAQmxStartTask returned an error or warning, translate the numerical code into text and attach a prefix

        retValue = checkError(error, "niDAQmx::setVal - DAQmxStartTask abnormal return");
        if(!retValue.containsError())
//        else
        {
            m_isgrabbing = true;
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
            return ito::RetVal(ito::retError, 0, tr("niDAQmx::checkData - Error during check data, external dataObject invalid. Object has more than 1 plane. It must be of right size and type or a uninitilized image.").toLatin1().data());            
        }
        else if (externalDataObject->getSize(dims - 2) != (unsigned int)channels || externalDataObject->getSize(dims - 1) != (unsigned int)samples || externalDataObject->getType() != ito::tFloat64)
        {
            return ito::RetVal(ito::retError, 0, tr("niDAQmx::checkData - Error during check data, external dataObject invalid. Object must be of right size and type or a uninitilized image.").toLatin1().data());
        }
    }

    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal niDAQmx::execFunc(const QString helpCommand, QSharedPointer<QVector<ito::ParamBase> > paramsMand, QSharedPointer<QVector<ito::ParamBase> > paramsOpt, QSharedPointer<QVector<ito::ParamBase> > /*paramsOut*/, ItomSharedSemaphore *waitCond)
{
    ito::RetVal retValue = ito::retOk;

    QStringList parts = helpCommand.split(":");
    QString function = parts[0];
    QString empty = QString("");

    if (function == "help")
    {
        if(parts.size() == 1)
        {
             retValue += help(&empty);
        }
        else
        {
            retValue += help(&parts[1]);
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

ito::RetVal niDAQmx::help(QString * helpTopic)
{
    ito::RetVal retValue = ito::retOk;

    if (*helpTopic == "")
    {
        std::cout << "The niDAQmx plugin provides data acquistion and control using\n" << std::endl;
        std::cout << "National Instruments computer interface hardware.\n" << std::endl;
        std::cout << "The plugin supports the following methods: name, getParam, setParam,\n" << std::endl;
        std::cout << "startDevice, stopDevice, acquire, setValMode, getVal, copyVal, taskStatus,\n" << std::endl;
        std::cout << "channel, and chAssociated. startDevice and stopDevice are NOOPs in this plugin.\n\n" << std::endl;
        std::cout << "Each method is documented separately with online help commands.\n" << std::endl;
        std::cout << "For example, setParam method documentation is accessed using\n" << std::endl;
        std::cout << "the command: <plugin ref>.exec('help:setParam').\n\n" << std::endl;
        std::cout << "An example acquisition would start with plugin=dataIO('niDAQmx');\n" << std::endl;
        std::cout << "Followed by plugin.setParam('aiTaskParams','20000,100,0');\n" << std::endl;
        std::cout << "Then, plugin.setParam('aiChParams','Dev1/ai0,3,-10,10');\n" << std::endl;
        std::cout << "After setting these parameters, execute the command plugin.acquire(1);\n" << std::endl;
        std::cout << "(the number 1 indicates that the analog input task is the acquire target)\n" << std::endl;
        std::cout << "This call will immediately return and at a later time the following\n" << std::endl;
        std::cout << "commands retrieve the data: d=dataObject(); followed by plugin.getVal(d);\n" << std::endl;
        std::cout << "If getVal is called before the acquistion is finished, it will wait until\n" << std::endl;
        std::cout << "all data is acquired before returning.\n\n" << std::endl;
    }
    else if (*helpTopic == "name")
    {
        std::cout << "\nTODO: name description.\n\n" << std::endl;
    }
    else if (*helpTopic == "getParam")
    {
        std::cout << "\nTODO: getParam description.\n\n" << std::endl;
    }
    else if (*helpTopic == "setParam")
    {
        std::cout << "\nTODO: setParam description.\n\n" << std::endl;
    }
    else if (*helpTopic == "startDevice")
    {
        std::cout << "\nTODO: startDevice description.\n\n" << std::endl;
    }
    else if (*helpTopic == "stopDevice")
    {
        std::cout << "\nTODO: stopDevice description.\n\n" << std::endl;
    }
    else if (*helpTopic == "setValMode")
    {
        std::cout << "\nTODO: setValMode description.\n\n" << std::endl;
    }
    else if (*helpTopic == "acquire")
    {
        std::cout << "\nTODO: acquire description.\n\n" << std::endl;
    }
    else if (*helpTopic == "getVal")
    {
        std::cout << "\nTODO: getVal description.\n\n" << std::endl;
    }
    else if (*helpTopic == "copyVal")
    {
        std::cout << "\nTODO: copyVal description.\n\n" << std::endl;
    }
    else if (*helpTopic == "taskStatus")
    {
        std::cout << "\nTODO: taskStatus description.\n\n" << std::endl;
    }
    else if (*helpTopic == "channel")
    {
        std::cout << "\nTODO: channel description.\n\n" << std::endl;
    }
    else if (*helpTopic == "chAssociated")
    {
        std::cout << "\nTODO: chAssociated description.\n\n" << std::endl;
    }
    else if (*helpTopic == "TaskParams")
    {
        std::cout << "TaskParams are specified as follows:\n\n" << std::endl;
        std::cout << "xxTaskParams, where xx is one of 'ai', 'ao', 'di', 'do', 'ci', 'do',\n" << std::endl;
        std::cout << "these abbreviations corresponding respectively to 'analog input', 'analog output',\n" << std::endl;
        std::cout << "'digital input', 'digital output', 'counter input', and 'counter output'.\n" << std::endl;
        std::cout << "These are set using setParam() and read using getParam().For more information\n" << std::endl;
        std::cout << "on specific TaskParmeters, type <plugin ref>.exec('help:xxTaskParams');\n" << std::endl;
        std::cout << "for example, <plugin ref>.exec('help:aiTaskParams').\n\n" << std::endl;
    }
    else if (*helpTopic == "aiTaskParams")
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
    else if (*helpTopic == "ChParams")
    {
        std::cout << "ChParams are specified as follows:\n\n" << std::endl;
        std::cout << "xxChParams, where xx is one of 'ai', 'ao', 'di', 'do', 'ci', 'do',\n" << std::endl;
        std::cout << "these abbreviations corresponding respectively to 'analog input', 'analog output',\n" << std::endl;
        std::cout << "'digital input', 'digital output', 'counter input', and 'counter output'.\n" << std::endl;
        std::cout << "These are set using setParam() and read using getParam().For more information\n" << std::endl;
        std::cout << "on specific ChParmeters, type <plugin ref>.exec('help:xxChParams');\n" << std::endl;
        std::cout << "for example, <plugin ref>.exec('help:aiChParams').\n\n" << std::endl;
    }
    else if (*helpTopic == "aiChParams")
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
