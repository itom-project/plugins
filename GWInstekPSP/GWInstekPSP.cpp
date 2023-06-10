/* ********************************************************************
    Plugin "GWInstekPSP" for itom software
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

#include "GWInstekPSP.h"

#ifdef WIN32
    #include <windows.h>
#else
    #include <unistd.h>
#endif

#include <qstring.h>
#include <qstringlist.h>
#include <QtCore/QtPlugin>

#include "pluginVersion.h"
#include "gitVersion.h"
#include <iostream>

//#include "common/helperCommon.h"


#define GWDELAY 10

//----------------------------------------------------------------------------------------------------------------------------------
/*!
    \detail creates new instance of GWInstekPSPInterface and returns the instance-pointer
    \param [in,out] addInInst is a double pointer of type ito::AddInBase. The newly created GWInstekPSPInterface-instance is stored in *addInInst
    \return retOk
    \sa GWInstekPSP
*/
ito::RetVal GWInstekPSPInterface::getAddInInst(ito::AddInBase **addInInst)
{
    NEW_PLUGININSTANCE(GWInstekPSP)
    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
/*!
    \detail Closes an instance of of GWInstekPSPInterface. This instance is given by parameter addInInst.
    \param [in] double pointer to the instance which should be deleted.
    \return retOk
    \sa GWInstekPSP
*/
ito::RetVal GWInstekPSPInterface::closeThisInst(ito::AddInBase **addInInst)
{
   REMOVE_PLUGININSTANCE(GWInstekPSP)
   return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
/*! \detail defines the plugin type (typeActuator) and sets the plugins object name. Theplugin is initialized (e.g. by a Python call)
    with mandatory or optional parameters (m_initParamsMand and m_initParamsOpt).
*/
GWInstekPSPInterface::GWInstekPSPInterface()
{
    m_type = ito::typeDataIO | ito::typeRawIO;
    setObjectName("GWInstekPSP");

    m_description = QObject::tr("Controller for power supplies PSP-405, PSP-603 and PSP-2010 of GWInstek");

    //for the docstring, please don't set any spaces at the beginning of the line.
/*    char docstring[] = \
"This itom-plugin allows communicating with power supplies PSP-405, PSP-603 and PSP-2010 (tested with PSP-405) of company GWInstek. \
Therefore an opened connected via the serial port (using the plugin 'SerialIO') is required. You need to give a valid handle to this \
instance when initializing this plugin.";
    m_detaildescription = QObject::tr(docstring);*/
    m_detaildescription = QObject::tr(
"This itom-plugin allows communicating with power supplies PSP-405, PSP-603 and PSP-2010 (tested with PSP-405) of company GWInstek. \
Therefore an opened connected via the serial port (using the plugin 'SerialIO') is required. You need to give a valid handle to this \
instance when initializing this plugin.");

    m_author = "H. Bieger, M. Gronle, ITO, University Stuttgart";
    m_version = (PLUGIN_VERSION_MAJOR << 16) + (PLUGIN_VERSION_MINOR << 8) + PLUGIN_VERSION_PATCH;
    m_minItomVer = MINVERSION;
    m_maxItomVer = MAXVERSION;
    m_license = QObject::tr("Licensed under LGPL");
    m_aboutThis = QObject::tr(GITVERSION);

    ito::Param paramVal("serial", ito::ParamBase::HWRef, NULL, tr("An initialized SerialIO").toLatin1().data());
    paramVal.setMeta( new ito::HWMeta("SerialIO"), true);
    m_initParamsMand.append(paramVal);

    paramVal = ito::Param("save", ito::ParamBase::Int, 0, 1, 1, tr("Save the present status to the EEPROM").toLatin1().data());
    m_initParamsOpt.append(paramVal);
    return;
}

//----------------------------------------------------------------------------------------------------------------------------------
/*!
    clears both vectors m_initParamsMand and m_initParamsOpt.
*/
GWInstekPSPInterface::~GWInstekPSPInterface()
{
}

//----------------------------------------------------------------------------------------------------------------------------------
// this makro registers the class GWInstekPSPInterface with the name GWInstekPSPInterface as plugin for the Qt-System (see Qt-DOC)


//----------------------------------------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------------------------------------
//!
/*!
    \detail This method must be executed in the main (GUI) thread and is usually called by the addIn-Manager.
    creates new instance of dialogGWInstekPSP, calls the method setVals of dialogGWInstekPSP, starts the execution loop and if the dialog
    is closed, reads the new parameter set and deletes the dialog.

    \return retOk
    \sa dialogGWInstekPSP
*/
const ito::RetVal GWInstekPSP::showConfDialog(void)
{

    dialogGWInstekPSP *confDialog = new dialogGWInstekPSP((void*)this);
    confDialog->setVals(&m_params);
    if (confDialog->exec())
    {
        confDialog->getVals(&m_params);
    }
    delete confDialog;

    return ito::retOk;
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
const ito::RetVal GWInstekPSP::SetParams()
{
    QByteArray status(m_status);
    m_params.find("status").value().setVal<char*>(m_status);
    m_params.find("voltage").value().setVal<double>((status.mid(1, 5).toDouble()));
    //m_params.find("voltage").value().setMax(status.mid(19, 2).toDouble());
    static_cast<ito::DoubleMeta*>( m_params["voltage"].getMeta() )->setMax( status.mid(19, 2).toDouble() );
    m_params.find("current").value().setVal<double>((status.mid(7, 5).toDouble()));
    m_params.find("load").value().setVal<double>((status.mid(13, 5).toDouble()));
    m_params.find("voltage_limit").value().setVal<double>((status.mid(19, 2).toDouble()));
    m_params.find("current_limit").value().setVal<double>((status.mid(22, 4).toDouble()));
    m_params.find("load_limit").value().setVal<double>((status.mid(27, 3).toDouble()));
    m_params.find("relay").value().setVal<int>((status.mid(31, 1).toInt()));
    m_params.find("temperature").value().setVal<int>((status.mid(32, 1).toInt()));
    m_params.find("wheel").value().setVal<int>((status.mid(33, 1).toInt()));
    m_params.find("wheel_lock").value().setVal<int>((status.mid(34, 1).toInt()));
    m_params.find("remote").value().setVal<int>((status.mid(35, 1).toInt()));
    m_params.find("lock").value().setVal<int>((status.mid(36, 1).toInt()));

    emit parametersChanged(m_params);
    return ito::retOk;
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
const ito::RetVal GWInstekPSP::ReadFromSerial(bool *state)
{
    QSharedPointer<int> len(new int);
    *len = 0;
    char buf[50];
    QSharedPointer<char> tempBuf;
    ito::RetVal retValue = ito::retOk;

    memset( (void*)buf, 0, 50 );
    int bufsize = sizeof(buf), totlen = 0, startpos = 0, endpos = 0, timeout = 500;

    QByteArray baBuffer = "";

    Sleep(GWDELAY);
    do
    {
        *len = bufsize - totlen;
        tempBuf = QSharedPointer<char>(&buf[totlen], GWInstekPSP::doNotDelSharedPtr); //trick to access part of buf using a shared pointer. the shared pointer is not allowed to delete the char-array, therefore the Deleter-method.
        retValue += m_pSer->getVal(tempBuf, len);
        totlen += *len;
        Sleep(2);
    }
    while ((totlen == 0 || buf[totlen - 1] != '\n') && (totlen < bufsize) && (retValue != ito::retError) && (0 < timeout--));

    if (timeout == -1)
    {
        return ito::RetVal(ito::retError, 0, tr("Undefined answer from serial port").toLatin1().data());
    }

    for (startpos = 0; (startpos < totlen) && (buf[startpos] != 'V'); startpos++); {}
    for (endpos = totlen - 1; (endpos > startpos) && ((buf[endpos] == '\r') || (buf[endpos] == '\n')); endpos--); {}
    *state = (endpos - startpos == 36);
    if (*state)
    {
        memcpy(m_status, &buf[startpos], 37);
        retValue += SetParams();
    }

    return retValue;
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
const ito::RetVal GWInstekPSP::WriteToSerial(const char *text, bool commandHasAnswer, bool getCurrentStatus /*= true*/)
{
    ito::RetVal retValue = ito::retOk;

    if (strcmp(text, "L") != 0) //if question is L (the only command that sends an answer, goto the if-case below)
    {

        retValue += m_pSer->setVal(text, (int)strlen(text));
        if (retValue == ito::retError)
        {
            return retValue;
        }

        if (commandHasAnswer)
        {
            bool state;
            retValue += ReadFromSerial(&state);
            if (retValue == ito::retError)
            {
                return retValue;
            }
        }
        else
        {
            Sleep(200);
        }
    }
    else
    {
        getCurrentStatus = true;
    }

    if (getCurrentStatus)
    {
        retValue += m_pSer->setVal("L", 1);
        if (retValue == ito::retError)
        {
            return retValue;
        }

        bool state;
        retValue += ReadFromSerial(&state);
        if (retValue == ito::retError)
        {
            return retValue;
        }
        if (!state)
        {
            return ito::RetVal(ito::retError, 0, tr("No answer from serial port").toLatin1().data());
        }
    }

    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
void GWInstekPSP::setParamVoltageFromWgt(double voltage)
{
    setParam( QSharedPointer<ito::ParamBase>(new ito::ParamBase("voltage", ito::ParamBase::Double, voltage)) , NULL );
}

//----------------------------------------------------------------------------------------------------------------------------------
GWInstekPSP::GWInstekPSP() : AddInDataIO(), m_pSer(NULL)
{
    qRegisterMetaType< QMap<QString, ito::Param> >("QMap<QString, ito::Param>");

    memset(&m_status, 0, 38);

    ito::Param paramVal("name", ito::ParamBase::String | ito::ParamBase::Readonly, "GWInstekPSP", NULL);
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param("status", ito::ParamBase::String | ito::ParamBase::Readonly, m_status, tr("Current status string of controller").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("voltage", ito::ParamBase::Double, 0.0, 40.0, 0.0, tr("Ouput voltage; the unit: V").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("current", ito::ParamBase::Double | ito::ParamBase::Readonly, 0.0, 99.0, 0.0, tr("Ouput current; the unit: A").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("load", ito::ParamBase::Double | ito::ParamBase::Readonly, 0.0, 250.0, 0.0, tr("Ouput load; the unit: W").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("voltage_limit", ito::ParamBase::Double, 0.0, 40.0, 0.0, tr("Ouput voltage limit; the unit: V").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("current_limit", ito::ParamBase::Double, 0.0, 5.0, 0.0, tr("Ouput current limit; the unit: A").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("load_limit", ito::ParamBase::Double, 0.0, 200.0, 0.0, tr("Ouput load limit; the unit: W").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("save", ito::ParamBase::Int, 0, 1, 1, tr("Save the present status to the EEPROM on exit").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("relay", ito::ParamBase::Int, 0, 1, 0, tr("Relay status 0: off, 1: on").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("temperature", ito::ParamBase::Int | ito::ParamBase::Readonly, 0, 1, 0, tr("Temperature status 0: normal, 1: overheat").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("wheel", ito::ParamBase::Int, 0, 1, 0, tr("Wheel knob 0: normal, 1: fine").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("wheel_lock", ito::ParamBase::Int | ito::ParamBase::Readonly, 0, 1, 0, tr("Wheel knob 0: lock, 1: unlock").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("remote", ito::ParamBase::Int | ito::ParamBase::Readonly, 0, 1, 0, tr("Remote status 0: normal, 1: remote").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("lock", ito::ParamBase::Int | ito::ParamBase::Readonly, 0, 1, 0, tr("Lock status 0: lock, 1: unlock").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);

    if (hasGuiSupport())
    {
        //now create dock widget for this plugin
        DockWidgetGWInstekPSP *GWInstekPSPWidget = new DockWidgetGWInstekPSP(m_params, getID());
        connect(this, SIGNAL(parametersChanged(QMap<QString, ito::Param>)), GWInstekPSPWidget, SLOT(valuesChanged(QMap<QString, ito::Param>)));
        connect(GWInstekPSPWidget, SIGNAL(setParamVoltage(double)), this, SLOT(setParamVoltageFromWgt(double)));

        Qt::DockWidgetAreas areas = Qt::AllDockWidgetAreas;
        QDockWidget::DockWidgetFeatures features = QDockWidget::DockWidgetClosable | QDockWidget::DockWidgetFloatable | QDockWidget::DockWidgetMovable;
        createDockWidget(QString(m_params["name"].getVal<char *>()), features, areas, GWInstekPSPWidget);
    }

   //register exec functions
    QVector<ito::Param> pMand;
    pMand << ito::Param("startVoltage", ito::ParamBase::Double | ito::ParamBase::In, 0.0, 50.0, 0.0, tr("start voltage in volt").toLatin1().data());
    pMand << ito::Param("endVoltage", ito::ParamBase::Double | ito::ParamBase::In, 0.0, 50.0, 10.0, tr("end voltage in volt").toLatin1().data());
    pMand << ito::Param("totalTime", ito::ParamBase::Int | ito::ParamBase::In, 0, 100000, 10000, tr("total ramp time in ms").toLatin1().data());
    pMand << ito::Param("steps", ito::ParamBase::Int | ito::ParamBase::In, 0, 10000, 10, tr("start voltage in volt").toLatin1().data());
    QVector<ito::Param> pOpt;
    pOpt << ito::Param("async", ito::ParamBase::Int | ito::ParamBase::In, 0, 1, 1, tr("synchronous (0) or asynchronous (1, default)").toLatin1().data());
    QVector<ito::Param> pOut;
    registerExecFunc("startRamp", pMand, pOpt, pOut, tr("todo"));
}

//----------------------------------------------------------------------------------------------------------------------------------
/*!
    \detail Class destructo which deletes the thread and clear the m_params
    \return retOk
*/
GWInstekPSP::~GWInstekPSP()
{
   m_params.clear();
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal GWInstekPSP::getParam(QSharedPointer<ito::Param> val, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);
    QString key = val->getName();

    if (key == "")
    {
        retValue += ito::RetVal(ito::retError, 0, tr("name of requested parameter is empty.").toLatin1().data());
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
ito::RetVal GWInstekPSP::setParam(QSharedPointer<ito::ParamBase> val, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);
    QString key = val->getName();
    char text[50];
    bool commandHasAnswer;

    if (key == "")
    {
        retValue += ito::RetVal(ito::retError, 0, tr("name of given parameter is empty.").toLatin1().data());
    }
    else
    {
        QMap<QString, ito::Param>::iterator paramIt = m_params.find(key);
        if (paramIt != m_params.end())
        {
            if (paramIt->getFlags() & ito::ParamBase::Readonly)    //check read-only
            {
                retValue += ito::RetVal(ito::retWarning, 0, tr("Parameter is read only, input ignored").toLatin1().data());
                goto end;
            }
            else if (val->isNumeric() && paramIt->isNumeric())
            {
                double curval = val->getVal<double>();
                if (curval > paramIt->getMax())
                {
                    retValue += ito::RetVal(ito::retError, 0, tr("New value is larger than parameter range, input ignored").toLatin1().data());
                    goto end;
                }
                else if (curval < paramIt->getMin())
                {
                    retValue += ito::RetVal(ito::retError, 0, tr("New value is smaller than parameter range, input ignored").toLatin1().data());
                    goto end;
                }
                else
                {
                    paramIt.value().setVal<double>(curval);
                }
            }
            else if (paramIt->getType() == val->getType())
            {
                retValue += paramIt.value().copyValueFrom( &(*val) );
            }
            else
            {
                retValue += ito::RetVal(ito::retError, 0, tr("Parameter type conflict").toLatin1().data());
                goto end;
            }

            if (key == "voltage")
            {
                commandHasAnswer = false;
                sprintf(text, "SV %05.2f", m_params["voltage"].getVal<double>());
            }
            else if (key == "voltage_limit")
            {
                commandHasAnswer = false;
                sprintf(text, "SU %02.0f", (m_params["voltage_limit"].getVal<double>()));
            }
            else if (key == "current_limit")
            {
                commandHasAnswer = false;
                sprintf(text, "SI %04.2f", (m_params["current_limit"].getVal<double>()));
            }
            else if (key == "load_limit")
            {
                commandHasAnswer = false;
                sprintf(text, "SP %03.0f", (m_params["load_limit"].getVal<double>()));
            }
            else if (key == "relay")
            {
                commandHasAnswer = false;
                if (m_params["relay"].getVal<int>())
                {
                    sprintf(text, "KOE");
                }
                else
                {
                    sprintf(text, "KOD");
                }
            }
            else if (key == "wheel")
            {
                commandHasAnswer = false;
                if (m_params["wheel"].getVal<int>())
                {
                    sprintf(text, "KF");
                }
                else
                {
                    sprintf(text, "KN");
                }
            }
            else
            {
                goto end;
            }
            retValue += WriteToSerial(text, commandHasAnswer);

            if (retValue != ito::retError)
            {
                retValue += SetParams();
            }
        }
        else
        {
            retValue += ito::RetVal(ito::retError, 0, tr("parameter not found in m_params.").toLatin1().data());
        }
    }

end:
    if (waitCond)
    {
        waitCond->returnValue = retValue;
        waitCond->release();
    }

    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
/*! \detail Init method which is called by the addInManager after the initiation of a new instance of DummyGrabber.
    This init method gets the mandatory and optional parameter vectors of type tParam and must copy these given parameters to the
    internal m_params-vector. Notice that this method is called after that this instance has been moved to its own (non-gui) thread.

    \param [in] paramsMand is a pointer to the vector of mandatory tParams.
    \param [in] paramsOpt is a pointer to the vector of optional tParams.
    \param [in] waitCond is the semaphore (default: NULL), which is released if this method has been terminated
    \todo check if (*paramsMand)[0] is a serial port
    \return retOk
*/
ito::RetVal GWInstekPSP::init(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue = ito::retOk;

    // Set serial port
    if (reinterpret_cast<ito::AddInBase *>((*paramsMand)[0].getVal<void *>())->getBasePlugin()->getType() & (ito::typeDataIO | ito::typeRawIO))
    {
        m_pSer = (ito::AddInDataIO *)(*paramsMand)[0].getVal<void *>();
        retValue += m_pSer->setParam(QSharedPointer<ito::ParamBase>(new ito::ParamBase("baud", ito::ParamBase::Int, 2400)), NULL);
        retValue += m_pSer->setParam(QSharedPointer<ito::ParamBase>(new ito::ParamBase("bits", ito::ParamBase::Int, 8)), NULL);
        retValue += m_pSer->setParam(QSharedPointer<ito::ParamBase>(new ito::ParamBase("parity", ito::ParamBase::Double, 0.0)), NULL);
        retValue += m_pSer->setParam(QSharedPointer<ito::ParamBase>(new ito::ParamBase("stopbits", ito::ParamBase::Int, 1)), NULL);
        retValue += m_pSer->setParam(QSharedPointer<ito::ParamBase>(new ito::ParamBase("flow", ito::ParamBase::Int, 16)), NULL);
        retValue += m_pSer->setParam(QSharedPointer<ito::ParamBase>(new ito::ParamBase("endline", ito::ParamBase::String, "\r")), NULL);
    }
    else
    {
        retValue += ito::RetVal(ito::retError, 1, tr("Doesn't fit to interface DataIO!").toLatin1().data());
    }

    if (!retValue.containsError())
    {
        retValue += m_params["save"].copyValueFrom( &((*paramsOpt)[0]) );
        retValue += WriteToSerial("L", true);
    }

    if (waitCond)
    {
        waitCond->returnValue = retValue;
        waitCond->release();
    }
    setInitialized(true); //init method has been finished (independent on retval)
    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
/*! \detail close method which is called before that this instance is deleted by the GWInstekPSPInterface
    notice that this method is called in the actual thread of this instance.

    \param [in] waitCond is the semaphore (default: NULL), which is released if this method has been terminated
    \return retOk
    \sa ItomSharedSemaphore
*/
ito::RetVal GWInstekPSP::close(ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue = ito::retOk;

    if (m_params["save"].getVal<int>() && m_pSer)
    {
        retValue += WriteToSerial("EEP", false);
    }

    if (waitCond)
    {
        waitCond->returnValue = retValue;
        waitCond->release();
    }
    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal GWInstekPSP::getVal(char * /*data*/, int * /*len*/, ItomSharedSemaphore *waitCond)
{
//    ito::DataObject *dObj = reinterpret_cast<ito::DataObject *>(vpdObj);
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retval;
//    retval = m_serport.sread(data, len, 0);

//    emit serialLog(QByteArray(data,*len), "", '<');

    if (waitCond)
    {
        waitCond->returnValue = retval;
        waitCond->release();
    }

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal GWInstekPSP::setVal(const char * /*data*/, const int /*datalength*/, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
//    const char *buf = (const char*)data;
//    char endline[3] = {0, 0, 0};
    ito::RetVal retval;

//    m_serport.getendline(endline);
//    emit serialLog(QByteArray(buf,datalength), QByteArray(endline, strlen(endline)), '>');

//    retval = m_serport.swrite(buf, datalength, m_params["singlechar"].getVal<int>());

    if (waitCond)
    {
        waitCond->returnValue = retval;
        waitCond->release();
    }
    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal GWInstekPSP::execFunc(const QString funcName, QSharedPointer<QVector<ito::ParamBase> > paramsMand, QSharedPointer<QVector<ito::ParamBase> > paramsOpt, QSharedPointer<QVector<ito::ParamBase> > paramsOut, ItomSharedSemaphore *waitCond /*= NULL*/)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue;

    if (funcName == "startRamp")
    {
        double startVoltage = paramsMand->at(0).getVal<double>();
        double endVoltage = paramsMand->at(1).getVal<double>();
        int totalTime = paramsMand->at(2).getVal<int>();
        int steps = paramsMand->at(3).getVal<int>();
        bool async = paramsOpt->at(0).getVal<int>() > 0;

        if (async)
        {
            if (waitCond)
            {
                waitCond->returnValue = retValue;
                waitCond->release();
                waitCond = NULL;
            }
        }

        QElapsedTimer timer;
        timer.start();
        char text[50];
        double yourVoltage;
        int timeStep = totalTime*1000/steps; //ms
        int i = 0;
        bool firstRun = true;

        while(1)
        {
            if (timer.elapsed() >= timeStep || firstRun)
            {
                firstRun = false;
                //qDebug() << "bin drin";
                setAlive(); //marks that this plugin is still executing something "good"
                yourVoltage = startVoltage + i*(endVoltage - startVoltage)/steps;
                i++;
                sprintf(text, "SV %05.2f", yourVoltage);
                timer.restart();
                retValue += WriteToSerial(text, false, false);

                if (yourVoltage >= endVoltage)
                {
                    break;
                }

                if (retValue.containsError())
                {
                    if (retValue.hasErrorMessage())
                    {
                        std::cerr << "error while setting ramp: " << retValue.errorMessage() << "\n" << std::endl;
                    }
                    else
                    {
                        std::cerr << "unknown error while setting ramp.\n" << std::endl;
                    }
                    break;
                }
            }
            //else
            //{
            //    qDebug() << "muss warten";
            //}
        }

        retValue += WriteToSerial("L", true); //get current values after the end of the ramp operation

        if (!async)
        {
            if (waitCond)
            {
                waitCond->returnValue = retValue;
                waitCond->release();
                waitCond = NULL;
            }
        }
    }
    else
    {
        if (waitCond)
        {
            waitCond->returnValue = retValue;
            waitCond->release();
            waitCond = NULL;
        }

    }

    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
