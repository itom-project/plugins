/* ********************************************************************
    Plugin "OphirSerialPlugin" for itom software
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

#include "OphirSerialPlugin.h"
#include "pluginVersion.h"
#include "gitVersion.h"

#include <qstring.h>
#include <qstringlist.h>
#include <qplugin.h>
#include <QtCore/QtPlugin>
#include <qregexp.h>
#include <qwaitcondition.h>
#include <qmutex.h>
#include <QTime>

#include "common/helperCommon.h"
#include "common/apiFunctionsInc.h"
//#include "iostream"

#include "dockWidgetOphirSerialPlugin.h"

#define READTIMEOUT 1000

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal OphirSerialPluginInterface::getAddInInst(ito::AddInBase **addInInst)
{
    NEW_PLUGININSTANCE(OphirSerialPlugin)
    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal OphirSerialPluginInterface::closeThisInst(ito::AddInBase **addInInst)
{
   REMOVE_PLUGININSTANCE(OphirSerialPlugin)
   return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
OphirSerialPluginInterface::OphirSerialPluginInterface()
{
    m_type = ito::typeDataIO  | ito::typeRawIO;
    setObjectName("OphirSerialPlugin");

    m_description = QObject::tr("Plugin for Ophir Powermeters.");
    m_detaildescription = QObject::tr("The OphirSerialPlugin is an itom plugin which is used for Powermeters from Ophir. \n\
It uses a serialIO instance to communication via RS232.\n\
This plugin sets the necessary serialIO parameter automatically during initialization.\n\
\n\
Tested devices: VEGA");

    m_author = "J. Krauter, Trumpf Lasersystems For Semiconductor Manufacturing Gmbh";
    m_version = (PLUGIN_VERSION_MAJOR << 16) + (PLUGIN_VERSION_MINOR << 8) + PLUGIN_VERSION_PATCH;
    m_minItomVer = MINVERSION;
    m_maxItomVer = MAXVERSION;
    m_license = QObject::tr("licensed under LGPL");
    m_aboutThis = QObject::tr(GITVERSION);    
    
    ito::Param paramVal("serial", ito::ParamBase::HWRef | ito::ParamBase::In, NULL, tr("An opened serial port (the right communcation parameters will be set by this plugin).").toLatin1().data());
    paramVal.setMeta(new ito::HWMeta("SerialIO"), true);
    m_initParamsMand.append(paramVal);

    m_initParamsOpt.append(ito::Param("serialNo", ito::ParamBase::String, "", tr("Serial number of the device to be loaded, if empty, the first device that is detected will be opened").toLatin1().data()));
}

//----------------------------------------------------------------------------------------------------------------------------------


//----------------------------------------------------------------------------------------------------------------------------------
const ito::RetVal OphirSerialPlugin::showConfDialog(void)
{
    return apiShowConfigurationDialog(this, new DialogOphirSerialPlugin(this));
}

//----------------------------------------------------------------------------------------------------------------------------------
OphirSerialPlugin::OphirSerialPlugin() : AddInDataIO(), 
m_pSer(NULL), 
m_delayAfterSendCommandMS(0), 
m_dockWidget(NULL)
{
    ito::Param paramVal("name", ito::ParamBase::String | ito::ParamBase::Readonly | ito::ParamBase::NoAutosave, "OphirSerialPlugin", tr("Name of plugin").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    m_params.insert("comPort", ito::Param("comPort", ito::ParamBase::Int | ito::ParamBase::Readonly, 0, 65355, 0, tr("The current com-port ID of this specific device. -1 means undefined").toLatin1().data()));
    m_params.insert("battery", ito::Param("battery", ito::ParamBase::Int | ito::ParamBase::Readonly, 0, 1, 0, tr("1 if battery is OK, 0 if battery is low").toLatin1().data()));
    m_params.insert("timeout", ito::Param("timeout", ito::ParamBase::Int | ito::ParamBase::Readonly, 0, 100000, 1000, tr("Request timeout, default 1000 ms").toLatin1().data()));
    m_params.insert("serialNumber", ito::Param("serialNumber", ito::ParamBase::String | ito::ParamBase::Readonly, "", tr("Serial number of the device shown on display").toLatin1().data()));

    paramVal = ito::Param("deviceType", ito::ParamBase::String | ito::ParamBase::Readonly, "", tr("Device type (NOVA, VEGA, LASERSTAR-S (single channel), LASERSTAR-D (dual channel), Nova-II)").toLatin1().data());
    ito::StringMeta sm(ito::StringMeta::String, "deviceType");
    sm.addItem("NOVA");
    sm.addItem("VEGA");
    sm.addItem("LASERSTAR-S");
    sm.addItem("LASERSTAR-D channel A");
    sm.addItem("LASERSTAR-D channel B");
    sm.addItem("NOVA-II");
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param("headType", ito::ParamBase::String | ito::ParamBase::Readonly, "", tr("Head type (thermopile, BC20, temperature probe, photodiode, CIE head, RP head, pyroelectric, nanoJoule meter, no head connected").toLatin1().data()); 
    ito::StringMeta sm2(ito::StringMeta::String, "headType");
    sm2.addItem("BC20");
    sm2.addItem("beam track");
    sm2.addItem("RM9");
    sm2.addItem("axial sensor");
    sm2.addItem("PD300-CIE sensor");
    sm2.addItem("nanoJoule meter");
    sm2.addItem("pyroelectric");
    sm2.addItem("PD300RM");
    sm2.addItem("photodiode");
    sm2.addItem("thermopile");
    sm2.addItem("temperature probe");
    sm2.addItem("no sensor connected");
    paramVal.setMeta(&sm2, false);
    m_params.insert(paramVal.getName(), paramVal);
    
    if (hasGuiSupport())
    {
        //now create dock widget for this plugin
        DockWidgetOphirSerialPlugin *m_dockWidget = new DockWidgetOphirSerialPlugin(getID(), this);
        Qt::DockWidgetAreas areas = Qt::AllDockWidgetAreas;
        QDockWidget::DockWidgetFeatures features = QDockWidget::DockWidgetClosable | QDockWidget::DockWidgetFloatable | QDockWidget::DockWidgetMovable;
        createDockWidget(QString(m_params["name"].getVal<char *>()), features, areas, m_dockWidget);
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal OphirSerialPlugin::init(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retval = ito::retOk;
    QByteArray answer;
    QByteArray request;

    if (reinterpret_cast<ito::AddInBase *>((*paramsMand)[0].getVal<void *>())->getBasePlugin()->getType() & (ito::typeDataIO | ito::typeRawIO))
    {
        m_pSer = (ito::AddInDataIO *)(*paramsMand)[0].getVal<void *>();
        retval += m_pSer->setParam(QSharedPointer<ito::ParamBase>(new ito::ParamBase("baud", ito::ParamBase::Int, 9600)), NULL);
        retval += m_pSer->setParam(QSharedPointer<ito::ParamBase>(new ito::ParamBase("bits", ito::ParamBase::Int, 8)), NULL);
        retval += m_pSer->setParam(QSharedPointer<ito::ParamBase>(new ito::ParamBase("parity", ito::ParamBase::Double, 0.0)), NULL);
        retval += m_pSer->setParam(QSharedPointer<ito::ParamBase>(new ito::ParamBase("stopbits", ito::ParamBase::Int, 1)), NULL);
        retval += m_pSer->setParam(QSharedPointer<ito::ParamBase>(new ito::ParamBase("flow", ito::ParamBase::Int, 0)), NULL);
        retval += m_pSer->setParam(QSharedPointer<ito::ParamBase>(new ito::ParamBase("endline", ito::ParamBase::String, "\r\n")), NULL);
    }
    else
    {
        retval += ito::RetVal(ito::retError, 1, tr("Doesn't fit to interface DataIO!").toLatin1().data());
    }

    if (!retval.containsError())
    {
        QSharedPointer<QVector<ito::ParamBase> > emptyParamVec(new QVector<ito::ParamBase>());
        m_pSer->execFunc("clearInputBuffer", emptyParamVec, emptyParamVec, emptyParamVec);
        m_pSer->execFunc("clearOutputBuffer", emptyParamVec, emptyParamVec, emptyParamVec);

        QSharedPointer<ito::Param> param(new ito::Param("port"));
        retval += m_pSer->getParam(param, NULL);
        if (retval.containsError() || param->getVal<int>() < 1)
        {
            retval += ito::RetVal(ito::retError, 0, tr("Could not read port number from serial port or port number invalid").toLatin1().data());
        }
        else
        {
            m_params["comPort"].setVal<int>(param->getVal<int>());
        }
    }

    if (!retval.containsError()) // get information
    {
        QByteArray serialNoInput = paramsOpt->at(0).getVal<char*>();
        bool found = false;
        
        QByteArray headType = "";
        QByteArray serialNum = "";
        request = QByteArray("$HI");
        retval += SendQuestionWithAnswerString(request, answer, m_params["timeout"].getVal<int>());  //optical output check query

        QRegExp reg("(\\d+)"); // matches numbers

        QStringList list;
        int pos = 0;

        while ((pos = reg.indexIn(answer, pos)) != -1) {
            list << reg.cap(1);
            pos += reg.matchedLength();
        }
        QByteArray foundSerialNo = list.at(0).toLatin1();
        
        if (serialNoInput == "")
        {
            found = true;
        }
        else if (serialNoInput.contains(foundSerialNo) && serialNoInput.length() == foundSerialNo.length())
        {
            found = true;
        }
        else
        {
            retval += ito::RetVal(ito::retError, 0, tr("Given serial number %1 does not match the received number %2").arg(serialNoInput.data()).arg(foundSerialNo.data()).toLatin1().data());
        }

        if (!retval.containsError() && found)
        {
            m_params["serialNumber"].setVal<char*>(foundSerialNo.data());

            if (answer.contains("BC"))
            {
                headType = "BC20";
            }
            else if (answer.contains("BT"))
            {
                headType = "beam track";
            }
            else if (answer.contains("CR"))
            {
                headType = "RM9";
            }
            else if (answer.contains("CP") || answer.contains("PY"))
            {
                headType = "pyroelectric";
            }
            else if (answer.contains("FX"))
            {
                headType = "axial sensor";
            }
            else if (answer.contains("LX"))
            {
                headType = "PD300-CIE sensor";
            }
            else if (answer.contains("NJ"))
            {
                headType = "nanoJoule meter";
            }
            else if (answer.contains("RM"))
            {
                headType = "PD300RM";
            }
            else if (answer.contains("SI"))
            {
                headType = "photodiode";
            }
            else if (answer.contains("TH"))
            {
                headType = "thermopile";
            }
            else if (answer.contains("TP"))
            {
                headType = "temperature probe";
            }
            else if (answer.contains("XX"))
            {
                headType = "no sensor connected";
            }
            else
            {
                retval += ito::RetVal(ito::retError, 0, tr("return answer %1 for rquest $HT not found.").arg(answer.data()).toLatin1().data());
            }

            m_params["headType"].setVal<char*>(headType.data());
        }

    }

    if (!retval.containsError()) // instrument information
    {
        QByteArray type;
        request = QByteArray("$II");
        retval += SendQuestionWithAnswerString(request, answer, m_params["timeout"].getVal<int>());  //optical output check query
        
        if (!retval.containsError())
        {
            if (answer.contains("NOVA"))
            {
                type = "NOVA";
            }
            else if (answer.contains("VEGA"))
            {
                type = "VEGA";
            }
            else if (answer.contains("LS-A 54545"))
            {
                type = "LASERSTAR-S";
            }
            else if (answer.contains("LS-A 23452"))
            {
                type = "LASERSTAR-D channel A";
            }
            else if (answer.contains("LS-B 23453"))
            {
                type = "LASERSTAR-D channel B";
            }
            else
            {
                retval += ito::RetVal(ito::retError, 0, tr("return answer %1 for rquest $HT not found.").arg(answer.data()).toLatin1().data());
            }

            m_params["deviceType"].setVal<char*>(type.data());
        }
    }

    if (waitCond)
    {
        waitCond->returnValue = retval;
        waitCond->release();
    }

    setInitialized(true); //init method has been finished (independent on retval)
    return retval;
}


//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal OphirSerialPlugin::getParam(QSharedPointer<ito::Param> val, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);
    QString key = val->getName();
    QByteArray request;
    

    if (key == "")
    {
        retValue += ito::RetVal(ito::retError, 0, tr("name of requested parameter is empty.").toLatin1().data());
    }
    else if (key == "battery")
    {
        int answer;
        request = QByteArray("$BC");
        retValue += SendQuestionWithAnswerInt(request, answer, m_params["timeout"].getVal<int>());  //optical output check query

        if (!retValue.containsError())
        {
            val->setVal<int>(answer);
        }
    }
    else
    {
        QMap<QString, ito::Param>::const_iterator paramIt = m_params.constFind(key);
        if (paramIt != m_params.constEnd())
        {
            *val = paramIt.value();
        }
        else
        {
            retValue += ito::RetVal(ito::retError, 0, tr("parameter not found in m_params.").toLatin1().data());
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
ito::RetVal OphirSerialPlugin::setParam(QSharedPointer<ito::ParamBase> val, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);
    QString key = val->getName();

    QString paramName;
    bool hasIndex;
    int index;
    QString additionalTag;
    QString suffix;
    QMap<QString, ito::Param>::iterator it;

    retValue += apiParseParamName(key, paramName, hasIndex, index, additionalTag);


    //parse the given parameter-name (if you support indexed or suffix-based parameters)
    retValue += apiParseParamName(val->getName(), key, hasIndex, index, suffix);

    if (!retValue.containsError())
    {
        //gets the parameter key from m_params map (read-only is not allowed and leads to ito::retError).
        retValue += apiGetParamFromMapByKey(m_params, key, it, true);
    }

    if (!retValue.containsError())
    {
        retValue += apiValidateParam(*it, *val, false, true);
    }

    if (!retValue.containsError())
    {
        if (key == "")
        {

        }
        else if (key == "")
        {

        }
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
ito::RetVal OphirSerialPlugin::close(ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);

    if (waitCond)
    {
        waitCond->returnValue = retValue;
        waitCond->release();
    }

    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal OphirSerialPlugin::waitForDone(const int timeoutMS, const QVector<int> /*axis*/ /*if empty -> all axis*/, const int /*flags*/ /*for your use*/)
{
    ito::RetVal retVal(ito::retOk);
    QMutex waitMutex;
    QWaitCondition waitCondition;
    bool atTarget = false;
    int timeoutMS_ = timeoutMS;
    ito::RetVal ontRetVal;
    int ontIterations = 10;
    QSharedPointer<double> actPos = QSharedPointer<double>(new double);

    while(timeoutMS_ > 0 && !atTarget)
    {
        //short delay
        waitMutex.lock();
        if (timeoutMS > 1000)
        {
            waitCondition.wait(&waitMutex, 1000);
            timeoutMS_ -= 1000;
        }
        else
        {
            waitCondition.wait(&waitMutex, timeoutMS);
            timeoutMS_ = 0;
        }
        waitMutex.unlock();
        setAlive();
    }

    return retVal;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal OphirSerialPlugin::requestStatusAndPosition(bool sendCurrentPos, bool sendTargetPos)
{
    ito::RetVal retValue;
    retValue += ito::RetVal::format(ito::retError, 0, tr("function not defined").toLatin1().data());
    return retValue;
}

//---------------------------------------------------------------------------------------------------------------------------------- 
void OphirSerialPlugin::dockWidgetVisibilityChanged(bool visible)
{
    if (getDockWidget())
    {
        QWidget *w = getDockWidget()->widget(); //your toolbox instance
        if (visible)
        {
            QObject::connect(this, SIGNAL(parametersChanged(QMap<QString, ito::Param>)), w, \
                SLOT(parametersChanged(QMap<QString, ito::Param>)));
            emit parametersChanged(m_params); //send current parameters

            //actuators only
            QObject::connect(this, SIGNAL(actuatorStatusChanged(QVector<int>,QVector<double>)), w, \
                SLOT(actuatorStatusChanged(QVector<int>,QVector<double>)));
            QObject::connect(this, SIGNAL(targetChanged(QVector<double>)), w, \
                SLOT(targetChanged(QVector<double>)));
            requestStatusAndPosition(true,true); //send current status, positions and targets
        }
        else
        {
            QObject::disconnect(this, SIGNAL(parametersChanged(QMap<QString, ito::Param>)), w, \
                SLOT(parametersChanged(QMap<QString, ito::Param>)));

            //actuators only
            QObject::disconnect(this, SIGNAL(actuatorStatusChanged(QVector<int>,QVector<double>)), w, \
                SLOT(actuatorStatusChanged(QVector<int>,QVector<double>)));
            QObject::disconnect(this, SIGNAL(targetChanged(QVector<double>)), w, \
                SLOT(targetChanged(QVector<double>)));
        }
    }
}

//---------------------------------------------------------------------------------------------------------------------------------- 
ito::RetVal OphirSerialPlugin::SendQuestionWithAnswerInt(QByteArray questionCommand, int &answer, int timeoutMS)
{
    int readSigns;
    bool ok;
    QByteArray _answer;
    ito::RetVal retValue = SerialSendCommand(questionCommand);
    retValue += readString(_answer, readSigns, timeoutMS);

    if (_answer[0] == '*')
    {
        _answer.remove(0, 1);
    }
    answer = _answer.toInt(&ok);

    if (retValue.containsError() || !ok)
    {
        retValue += ito::RetVal(ito::retError, 0, tr("value could not be parsed to a double value").toLatin1().data());
    }

    return retValue;
}

//---------------------------------------------------------------------------------------------------------------------------------- 
ito::RetVal OphirSerialPlugin::SendQuestionWithAnswerString(QByteArray questionCommand, QByteArray &answer, int timeoutMS)
{
    int readSigns;
    ito::RetVal retValue = SerialSendCommand(questionCommand);
    retValue += readString(answer, readSigns, timeoutMS);

    if (answer[0] == '*')
    {
        answer.remove(0, 1);
    }

    if (retValue.errorCode() == READTIMEOUT)
    {
        retValue = ito::RetVal(ito::retError, READTIMEOUT, tr("timeout").toLatin1().data());
    }

    return retValue;
}

//---------------------------------------------------------------------------------------------------------------------------------- 
ito::RetVal OphirSerialPlugin::readString(QByteArray &result, int &len, int timeoutMS)
{
    ito::RetVal retValue = ito::retOk;
    bool done = false;
    QTime timer;
    QByteArray endline;
    int curFrom = 0;
    int pos = 0;
    int buflen = 100;
    QSharedPointer<int> curBufLen(new int);
    QSharedPointer<char> curBuf(new char[buflen]);
    result = "";

    QSharedPointer<ito::Param> param(new ito::Param("endline"));
    retValue += m_pSer->getParam(param, NULL);

    if (param->getType() == (ito::ParamBase::String & ito::paramTypeMask))
    {
        char* temp = param->getVal<char*>(); //borrowed reference
        int len = temp[0] == 0 ? 0 : (temp[1] == 0 ? 1 : (temp[2] == 0 ? 2 : 3));
        endline = QByteArray::fromRawData(temp, len);
        //
        //endline[0] = temp[0];
        //endline[1] = temp[1];
        //endline[2] = temp[2];
        //endline = endline.trimmed();
    }
    else
    {
        retValue += ito::RetVal(ito::retError, 0, tr("could not read endline parameter from serial port").toLatin1().data());
    }

    while (!done && !retValue.containsError())
    {
        *curBufLen = buflen;
        retValue += m_pSer->getVal(curBuf, curBufLen, NULL);

        if (!retValue.containsError())
        {
            result += QByteArray(curBuf.data(), *curBufLen);
            pos = result.indexOf(endline, curFrom);
            curFrom = qMax(0, result.length() - 3);

            if (pos >= 0) //found
            {
                done = true;
                result = result.left(pos);
            }
        }

        if (!done && timer.elapsed() > timeoutMS && timeoutMS >= 0)
        {
            retValue += ito::RetVal(ito::retError, READTIMEOUT, tr("timeout").toLatin1().data());
        }

        len = result.length();
    }

    return retValue;
}

//---------------------------------------------------------------------------------------------------------------------------------- 
ito::RetVal OphirSerialPlugin::SerialSendCommand(QByteArray command)
{
    ito::RetVal retVal = m_pSer->setVal(command.data(), command.length(), NULL);

    if (m_delayAfterSendCommandMS > 0)
    {
        QMutex mutex;
        mutex.lock();
        QWaitCondition waitCondition;
        waitCondition.wait(&mutex,m_delayAfterSendCommandMS);
        mutex.unlock();
    }

    return retVal;
}
