/* ********************************************************************
    Plugin "SuperlumBS" for itom software
    URL: http://www.uni-stuttgart.de/ito
    Copyright (C) 2018, Institut für Technische Optik (ITO),
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

#define ITOM_IMPORT_API
#define ITOM_IMPORT_PLOTAPI

#include "SuperlumBL.h"
#include "pluginVersion.h"
#include "gitVersion.h"

#include <qstring.h>
#include <qstringlist.h>
#include <qplugin.h>
#include <QtCore/QtPlugin>
#include <qregularexpression.h>
#include <qwaitcondition.h>
#include <qmutex.h>
#include <QElapsedTimer>

#ifdef WIN32
    #include <windows.h>
#endif

#include "common/helperCommon.h"
#include "common/apiFunctionsInc.h"
//#include "iostream"

#include "dockWidgetSuperlumBL.h"

#define READTIMEOUT 256

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal SuperlumBLInterface::getAddInInst(ito::AddInBase **addInInst)
{
    NEW_PLUGININSTANCE(SuperlumBL)
    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal SuperlumBLInterface::closeThisInst(ito::AddInBase **addInInst)
{
   REMOVE_PLUGININSTANCE(SuperlumBL)
   return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
SuperlumBLInterface::SuperlumBLInterface()
{
    m_type = ito::typeDataIO | ito::typeRawIO;
    setObjectName("SuperlumBL");

    m_description = QObject::tr("Plugin for Superlum S-series BroadLighter SLDs.");

/*    char docstring[] = \
"The SuperlumBL is an itom-plugin (loosely based on SuperlumBS-plugin), which can be used to communicate with a BroadLighter.\n\
Only S-840-B-I-20 is tested by now.\n\
The company website can be found under http://www.superlumdiodes.com \n\
This system needs a serial port, which differs depending on the controller type. \
The parameters of the serial port (besides port number) are set automatically during initialization. \n\
\n\
It is initialized by dataIO(\"SuperlumBL\", SerialIO, deviceName).";
    m_detaildescription = QObject::tr(docstring);*/
    m_detaildescription = QObject::tr("The SuperlumBL is an itom-plugin (loosely based on SuperlumBS-plugin), which can be used to communicate with a BroadLighter.\n\
Only S-840-B-I-20 is tested by now.\n\
The company website can be found under http://www.superlumdiodes.com \n\
This system needs a serial port, which differs depending on the controller type. \
The parameters of the serial port (besides port number) are set automatically during initialization. \n\
\n\
It is initialized by dataIO(\"SuperlumBL\", SerialIO, deviceName).");

    m_author = PLUGIN_AUTHOR;
    m_version = PLUGIN_VERSION;
    m_minItomVer = PLUGIN_MIN_ITOM_VERSION;
    m_maxItomVer = PLUGIN_MAX_ITOM_VERSION;
    m_license = QObject::tr(PLUGIN_LICENCE);
    m_aboutThis = QObject::tr(GITVERSION);

    ito::Param paramVal("serial", ito::ParamBase::HWRef | ito::ParamBase::In, NULL, tr("An opened serial port (the right communication parameters will be set by this Superlum BroadSweeper).").toLatin1().data());
    paramVal.setMeta(new ito::HWMeta("SerialIO"), true);
    m_initParamsMand.append(paramVal);

    paramVal = ito::Param("deviceName", ito::ParamBase::String | ito::ParamBase::In, "S-840-B-I-20", tr("Device name of the Superlum BroadLighter. Only S-840-B-I-20 is implemented and tested.").toLatin1().data());
    ito::StringMeta *deviceMeta = new ito::StringMeta(ito::StringMeta::String);
    deviceMeta->addItem("S-840-B-I-20");
    paramVal.setMeta(deviceMeta, true);
    m_initParamsMand.append(paramVal);
}

//----------------------------------------------------------------------------------------------------------------------------------


//----------------------------------------------------------------------------------------------------------------------------------
const ito::RetVal SuperlumBL::showConfDialog(void)
{
    return apiShowConfigurationDialog(this, new DialogSuperlumBL(this));
}

//----------------------------------------------------------------------------------------------------------------------------------
SuperlumBL::SuperlumBL() : AddInDataIO(), m_pSer(NULL), m_delayAfterSendCommandMS(0), m_dockWidget(NULL)
{
    ito::Param paramVal("name", ito::ParamBase::String | ito::ParamBase::Readonly | ito::ParamBase::NoAutosave, "Superlum BroadLighter", tr("Name of plugin.").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("comPort", ito::ParamBase::Int | ito::ParamBase::Readonly, 0, 65355, 0, tr("The current com-port ID of this specific device. -1 means undefined.").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("serial_number", ito::ParamBase::String | ito::ParamBase::Readonly, "unknown", tr("Serial number of device.").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("local", ito::ParamBase::Int, 0, 1, 1, tr("(0) local or (1) remote mode.").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("optical_output", ito::ParamBase::Int, 0, 1, 0, tr("(0) optical output is disabled, (1) optical output is enabled.").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("power_mode", ito::ParamBase::Int, 0, 1, 0, tr("(0) LOW Power mode, (1) HIGH Power mode.").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);

    if (hasGuiSupport())
    {
        //now create dock widget for this plugin
        DockWidgetSuperlumBL *m_dockWidget = new DockWidgetSuperlumBL(getID(), this);
        Qt::DockWidgetAreas areas = Qt::AllDockWidgetAreas;
        QDockWidget::DockWidgetFeatures features = QDockWidget::DockWidgetClosable | QDockWidget::DockWidgetFloatable | QDockWidget::DockWidgetMovable;
        createDockWidget(QString(m_params["name"].getVal<char *>()), features, areas, m_dockWidget);
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal SuperlumBL::getParam(QSharedPointer<ito::Param> val, ItomSharedSemaphore *waitCond)
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
ito::RetVal SuperlumBL::setParam(QSharedPointer<ito::ParamBase> val, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);
    QString key = val->getName();

    QString paramName;
    bool hasIndex;
    bool outputOpt;
    bool powermod;
    int index;
    QString additionalTag;
    QVector<QPair<int, QByteArray> > lastError;
    QByteArray answer;
    QByteArray request;
    QMap<QString, ito::Param>::iterator it;

    retValue += apiParseParamName(key, paramName, hasIndex, index, additionalTag);

    if (!retValue.containsError())
    {
        retValue += apiGetParamFromMapByKey(m_params, key, it, true);
    }

    if (!retValue.containsError())
    {
#if defined(ITOM_ADDININTERFACE_VERSION) && ITOM_ADDININTERFACE_VERSION < 0x010300
        //old style api, round the incoming double value to the allowed step size.
        //in a new itom api, this is automatically done by a new api function.
        if (val->getType() == ito::ParamBase::Double || val->getType() == ito::ParamBase::Int)
        {
            double value = val->getVal<double>();
            if (it->getType() == ito::ParamBase::Double)
            {
                ito::DoubleMeta *meta = (ito::DoubleMeta*)it->getMeta();
                if (meta)
                {
                    double step = meta->getStepSize();
                    if (step != 0.0)
                    {
                        int multiple = qRound((value - meta->getMin()) / step);
                        value = meta->getMin() + multiple * step;
                        value = qBound(meta->getMin(), value, meta->getMax());
                        val->setVal<double>(value);
                    }
                }
            }
        }
        retValue += apiValidateParam(*it, *val, false, true);
#else
        retValue += apiValidateAndCastParam(*it, *val, false, true, true);
#endif
    }

    if (paramName.isEmpty())
    {
        retValue += ito::RetVal(ito::retError, 0, tr("name of given parameter is empty.").toLatin1().data());
    }
    else if (!paramName.isEmpty())
    {
        QMap<QString, ito::Param>::iterator paramIt = m_params.find(paramName);
        if (paramIt != m_params.end())
        {

            if (paramIt->getFlags() & ito::ParamBase::Readonly)
            {
                retValue += ito::RetVal(ito::retWarning, 0, tr("Parameter is read only, input ignored").toLatin1().data());
            }

            else if (val->isNumeric() && paramIt->isNumeric())
            {
                double curval = val->getVal<double>();
                if (curval > paramIt->getMax())
                {
                    retValue += ito::RetVal(ito::retError, 0, tr("New value is larger than parameter range, input ignored").toLatin1().data());
                }
                else if (curval < paramIt->getMin())
                {
                    retValue += ito::RetVal(ito::retError, 0, tr("New value is smaller than parameter range, input ignored").toLatin1().data());
                }

                //__________________________________________________________________________________________________________ Local/ Remote Mode
                else if (paramName == "local")
                {
                    //set remote operation of device
                    if (val->getVal<int>() == 0)
                    {
                        request = QByteArray("S11");
                        retValue += SendQuestionWithAnswerString(request, answer, 500);
                        if (answer.contains("A11") && !retValue.containsError())
                        {
                            m_params["local"].setVal<int>(0); // local mode
                        }
                        else
                        {
                            retValue += ito::RetVal::format(ito::retError, 0, tr("invalid answer '%s' for sending  '%s'").toLatin1().data(), answer.data(), request.data());
                        }
                    }
                    else if (val->getVal<int>() == 1)
                    {
                        request = QByteArray("S12");
                        retValue += SendQuestionWithAnswerString(request, answer, 500);
                        if (answer.contains("A12") && !retValue.containsError())
                        {
                            m_params["local"].setVal<int>(1); // remote mode
                        }
                        else
                        {
                            retValue += ito::RetVal::format(ito::retError, 0, tr("invalid answer '%s' for sending  '%s'").toLatin1().data(), answer.data(), request.data());
                        }
                    }
                    else
                    {
                        retValue += ito::RetVal::format(ito::retError, 0, tr("invalid answer '%s' for sending  '%s'").toLatin1().data(), answer.data(), request.data());
                    }
                }

                //__________________________________________________________________________________________________________ Optical Output
                else if (paramName == "optical_output")
                {
                    request = QByteArray("S20");
                    retValue += SendQuestionWithAnswerString(request, answer, 500);  //get optical output status
                    if (answer.contains("A2") && !retValue.containsError())
                    {
                        QRegularExpression regExp("^A2(\\d{2,2})");
                        QRegularExpressionMatch match = regExp.match(answer);
                        if (match.hasMatch() && !retValue.containsError())
                        {
                            if (((match.captured(1).toInt()) & 2) == 2)
                            {
                                outputOpt = true;//optical output enabled
                            }
                            else
                            {
                                outputOpt = false;//optical output disabled
                            }
                        }
                        else
                        {
                            retValue += ito::RetVal::format(ito::retError, 0, tr("invalid answer '%s'.").toLatin1().data(), answer.data());
                        }

                        if (!retValue.containsError() && outputOpt && (val->getVal<int>() == 0)) //disable optical output
                        {
                            request = QByteArray("S21");
                            retValue += SendQuestionWithAnswerString(request, answer, 500);
                            QRegularExpression regExp("^A2(\\d{2,2})");
                            QRegularExpressionMatch match = regExp.match(answer);
                            if (match.hasMatch() && !retValue.containsError() &&
                                (m_deviceType ==
                                 S_840_B_I_20)) // raises error, if 1st an 2nd try fail, but may not
                                                // be necessary for disabling.
                            {
                                if (((match.captured(1).toInt()) & 2) == 2)
                                {
                                    Sleep(500);
                                    retValue += SendQuestionWithAnswerString(request, answer, 500); //2nd try
                                    match = regExp.match(answer);
                                    if (match.hasMatch() && !retValue.containsError())
                                    {
                                        if (((match.captured(1).toInt()) & 2) == 2) // still on!
                                        {
                                            m_params["optical_output"].setVal<int>(1);
                                            retValue += ito::RetVal::format(ito::retError, 0, tr("Could not disable optical output. Answer was '%s'.").toLatin1().data(), answer.data());
                                        }
                                        else
                                        {
                                            m_params["optical_output"].setVal<int>(0); // 2nd try worked
                                        }
                                    }
                                }
                                else
                                {
                                    m_params["optical_output"].setVal<int>(0); // 1st try worked
                                }
                            }
                            else
                            {
                                retValue += ito::RetVal::format(ito::retError,0,"invalid answer '%s' for sending  '%s'", answer.data(), request.data());
                            }
                        }
                        else if (!retValue.containsError() && !outputOpt && (val->getVal<int>() == 1)) //enable optical output
                        {
                            request = QByteArray("S21");
                            retValue += SendQuestionWithAnswerString(request, answer, 500);
                            QRegularExpression regExp("^A2(\\d{2,2})");
                            QRegularExpressionMatch match = regExp.match(answer);
                            if (match.hasMatch() &&
                                !retValue.containsError()) //&& (m_deviceType == S_840_B_I_20))//
                                                           //raises error, if 1st an 2nd try fail.
                            {
                                if (((match.captured(1).toInt()) & 2) == 2)
                                {
                                    m_params["optical_output"].setVal<int>(1); // 1st try worked
                                }
                                else
                                {
                                    Sleep(500);
                                    retValue += SendQuestionWithAnswerString(request, answer, 500); //2nd try
                                    match = regExp.match(answer);
                                    if (match.hasMatch() && !retValue.containsError())
                                    {
                                        if (((match.captured(1).toInt()) & 2) == 2) // on!
                                        {
                                            m_params["optical_output"].setVal<int>(1); // 2nd try worked
                                        }
                                        else
                                        {
                                            m_params["optical_output"].setVal<int>(0);
                                            retValue += ito::RetVal::format(ito::retError, 0, tr("Could not enable optical output. Answer was '%s'.").toLatin1().data(), answer.data());
                                        }
                                    }
                                }
                            }
                            else
                            {
                                retValue += ito::RetVal::format(ito::retError, 0, tr("invalid answer '%s' for sending  '%s'").toLatin1().data(), answer.data(), request.data());
                            }
                        }
                        else if (!retValue.containsError() && !outputOpt && (val->getVal<int>() == 0)) //already disabled
                        {

                        }
                        else if (!retValue.containsError() && outputOpt && (val->getVal<int>() == 1)) //already enabled
                        {

                        }
                        else
                        {
                            retValue += ito::RetVal::format(ito::retError, 0, tr("invalid answer '%s'.").toLatin1().data(), answer.data());
                        }
                    }
                    else
                    {
                        retValue += ito::RetVal::format(ito::retError, 0, tr("invalid answer '%s' for sending  '%s'").toLatin1().data(), answer.data(), request.data());
                    }
                }

                //__________________________________________________________________________________________________________ Power Mode
                else if (paramName == "power_mode")
                {
                    request = QByteArray("S20");
                    retValue += SendQuestionWithAnswerString(request, answer, 500);  //get optical output status
                    QRegularExpression regExp("^A2(\\d{2,2})");
                    QRegularExpressionMatch match = regExp.match(answer);
                    if (match.hasMatch() && !retValue.containsError())
                    {
                        if (((match.captured(1).toInt()) & 2) == 2)
                        {
                            outputOpt = true;
                        }
                        else
                        {
                            outputOpt = false;
                        }
                        if (((match.captured(1).toInt()) & 16) == 16)
                        {
                            powermod = true; //means high
                        }
                        else
                        {
                            powermod = false; //means low
                        }
                    }
                    else
                    {
                        retValue += ito::RetVal::format(ito::retError, 0, tr("invalid answer '%s' for sending  '%s'").toLatin1().data(), answer.data(), request.data());
                    }
                    if (!powermod && (val->getVal<int>() == 1))
                    {
                        if (outputOpt)
                        {
                            request = QByteArray("S21");
                            retValue += SendQuestionWithAnswerString(request, answer, 500); // not safe
                            Sleep(500);
                            request = QByteArray("S41");
                            retValue += SendQuestionWithAnswerString(request, answer, 500); // not safe
                            Sleep(500);
                            request = QByteArray("S21");
                            retValue += SendQuestionWithAnswerString(request, answer, 500); // not safe
                        }
                        else
                        {
                            request = QByteArray("S41");
                            retValue += SendQuestionWithAnswerString(request, answer, 500); // not safe
                        }
                        m_params["power_mode"].setVal<int>(1);
                    }
                    if (powermod && (val->getVal<int>() == 0))
                    {
                        if (outputOpt)
                        {
                            request = QByteArray("S21");
                            retValue += SendQuestionWithAnswerString(request, answer, 500); // not safe
                            Sleep(500);
                            request = QByteArray("S41");
                            retValue += SendQuestionWithAnswerString(request, answer, 500); // not safe
                            Sleep(500);
                            request = QByteArray("S21");
                            retValue += SendQuestionWithAnswerString(request, answer, 500); // not safe
                        }
                        else
                        {
                            request = QByteArray("S41");
                            retValue += SendQuestionWithAnswerString(request, answer, 500); // not safe
                        }
                        m_params["power_mode"].setVal<int>(0);
                    }
                }
            }
            //__________________________________________________________________________________________________________
            else if (paramIt->getType() == val->getType())
            {
                retValue += paramIt.value().copyValueFrom(&(*val));
            }
            else
            {
                retValue += ito::RetVal(ito::retError, 0, tr("Given parameter and m_param do not have the same type").toLatin1().data());
            }
        }
    }

    else
    {
        retValue += ito::RetVal(ito::retError, 0, tr("parameter not found in m_params.").toLatin1().data());
    }

    emit parametersChanged(m_params); //send changed parameters to any connected dialogs or dock-widgets
    if (waitCond)
    {
        waitCond->returnValue = retValue;
        waitCond->release();
    }

    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal SuperlumBL::init(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retval = ito::retOk;
    QByteArray answer;

    QByteArray deviceName = paramsMand->at(1).getVal<const char*>();

    if (deviceName == "S-840-B-I-20")
    {
        m_deviceType = S_840_B_I_20;
        m_identifier = QString("Broadlighter S-840-B-I-20 (%1)").arg(getID());
    }
    else
    {
        retval += ito::RetVal::format(ito::retError, 0, tr("Device name '%s' not supported").toLatin1().data(), deviceName.data());
    }

    if (reinterpret_cast<ito::AddInBase *>((*paramsMand)[0].getVal<void *>())->getBasePlugin()->getType() & (ito::typeDataIO | ito::typeRawIO))
    {
        m_pSer = (ito::AddInDataIO *)(*paramsMand)[0].getVal<void *>();
         retval += IdentifyAndInitializeSystem();
    }
    else
    {
        retval += ito::RetVal(ito::retError, 1, tr("Doesn't fit to interface DataIO!").toLatin1().data());
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
ito::RetVal SuperlumBL::close(ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);
    QByteArray answer;
    QByteArray request;

    request = QByteArray("S20");
    retValue += SendQuestionWithAnswerString(request, answer, 500);  //ask, if optical output is enabled
    QRegularExpression regExp("^A2(\\d{2,2})");
    QRegularExpressionMatch match = regExp.match(answer);
    if (match.hasMatch() && !retValue.containsError())
    {
        if (((match.captured(1).toInt()) & 2) == 2)
        {
            retValue += SendQuestionWithAnswerString("S21", answer, 500);  //disable optical output
        }
    }
    else
    {
        retValue += ito::RetVal::format(ito::retError, 0, tr("invalid answer '%s' for sending  '%s'").toLatin1().data(), answer.data(), request.data());
    }

    request = QByteArray("S11");
    retValue += SendQuestionWithAnswerString(request, answer, 500); //set local mode
    if (retValue.containsError())
    {
        retValue += ito::RetVal::format(ito::retError, 0, tr("invalid answer '%s' for sending  '%s'").toLatin1().data(), answer.data(), request.data());
    }

    if (waitCond)
    {
        waitCond->returnValue = retValue;
        waitCond->release();
    }

    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
void SuperlumBL::dockWidgetVisibilityChanged(bool visible)
{
    if (getDockWidget())
    {
        QWidget *w = getDockWidget()->widget(); //your toolbox instance
        if (visible)
        {
            QObject::connect(this, SIGNAL(parametersChanged(QMap<QString, ito::Param>)), w, \
                SLOT(parametersChanged(QMap<QString, ito::Param>)));
            emit parametersChanged(m_params); //send current parameters
        }
        else
        {
            QObject::disconnect(this, SIGNAL(parametersChanged(QMap<QString, ito::Param>)), w, \
                SLOT(parametersChanged(QMap<QString, ito::Param>)));
        }
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal SuperlumBL::SendQuestionWithAnswerString(QByteArray questionCommand, QByteArray &answer, int timeoutMS)
{
    int readSigns;
    ito::RetVal retValue = SerialSendCommand(questionCommand);
    retValue += readString(questionCommand, answer, readSigns, timeoutMS);

    if (retValue.errorCode() == READTIMEOUT)
    {
        retValue = ito::RetVal(ito::retError, READTIMEOUT, tr("timeout").toLatin1().data());
    }

    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal SuperlumBL::readString(QByteArray &questionCommand, QByteArray &result, int &len, int timeoutMS)
{
    ito::RetVal retValue = ito::retOk;
    QElapsedTimer timer;
    QByteArray endline;
    QByteArray answer;
    bool done = false;
    int buflen = 100;
    QSharedPointer<int> curBufLen(new int);
    QSharedPointer<char> curBuf(new char[buflen]);
    result = "";
    int curFrom = 0;
    int pos = 0;

    if (m_deviceType == S_840_B_I_20)
    {
        endline = "\r\n"; //for sending a single \r is sufficient, but answers end with \r\n
    }
    else //ask serial port for endline character
    {
        QSharedPointer<ito::Param> param(new ito::Param("endline"));
        retValue += m_pSer->getParam(param, NULL);

        if (param->getType() == (ito::ParamBase::String & ito::paramTypeMask))
        {
            char* temp = param->getVal<char*>(); //borrowed reference
            int len = temp[0] == 0 ? 0 : (temp[1] == 0 ? 1 : (temp[2] == 0 ? 2 : 3));
            endline = QByteArray::fromRawData(temp,len);
        }
        else
        {
            retValue += ito::RetVal(ito::retError, 0, tr("could not read endline parameter from serial port").toLatin1().data());
        }
    }

    if (!retValue.containsError())
    {
        len = 0;
        timer.start();

        while(!done && !retValue.containsError())
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

            //qDebug() << "readString. done: "<<done<<", tempResult: "<<result;

            if (!done && timer.elapsed() > timeoutMS && timeoutMS >= 0)
            {
                retValue += ito::RetVal(ito::retError, READTIMEOUT, tr("timeout").toLatin1().data());
                return retValue;
            }
        }

        len = result.length();
    }

    if (!retValue.containsError() && result.contains("AE"))// general error!)
    {
        retValue += ito::RetVal(ito::retError, 0, tr(m_params["serial_number"].getVal<char*>(), "general error!").toLatin1().data());
        return retValue;
    }

    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal SuperlumBL::SerialSendCommand(QByteArray command)
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

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal SuperlumBL::IdentifyAndInitializeSystem()
{
    ito::RetVal retval = ito::retOk;
    QByteArray answer;
    QByteArray request;

    //default serial settings ... still valid for Broadlighter
    retval += m_pSer->setParam(QSharedPointer<ito::ParamBase>(new ito::ParamBase("baud", ito::ParamBase::Int, 57600)), NULL);
    retval += m_pSer->setParam(QSharedPointer<ito::ParamBase>(new ito::ParamBase("bits", ito::ParamBase::Int, 8)), NULL);
    retval += m_pSer->setParam(QSharedPointer<ito::ParamBase>(new ito::ParamBase("parity", ito::ParamBase::Double, 0.0)), NULL);
    retval += m_pSer->setParam(QSharedPointer<ito::ParamBase>(new ito::ParamBase("stopbits", ito::ParamBase::Int, 1)), NULL);
    retval += m_pSer->setParam(QSharedPointer<ito::ParamBase>(new ito::ParamBase("flow", ito::ParamBase::Int, 0)), NULL);
    retval += m_pSer->setParam(QSharedPointer<ito::ParamBase>(new ito::ParamBase("endline", ito::ParamBase::String, "\r\n")), NULL);

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

    //__________________________________________________________________________________________________________ Set serial number
    if (!retval.containsError())
    {
        request = QByteArray("S0");
        retval += SendQuestionWithAnswerString(request, answer, 500);
        if (!retval.containsError())
        {
            //Superlum BroadSweeper and BroadLighter identification information
            //A0: response code
            //1-byte integer: type of device 0..9.
            //1-byte integer: number of channels 1..4.
            //1-byte integer: firmware version 0..9.
            //5-byte data: serial number of device

            //ITO Superlum BroadLighter identification information
            //A0: response code
            //1: this integer means that the type of the instrument is 1.
            //1: this integer means that it is a single-channel device.
            //0: this integer means that the firmware version is 0.
            //00086: serial number

            m_params["serial_number"].setVal<char*>(answer.data());
        }
        else
        {
            retval += ito::RetVal::format(ito::retError, 0, tr("invalid answer '%s' for sending  '%s'").toLatin1().data(), answer.data(), request.data());
        }
    }

    //__________________________________________________________________________________________________________ set device in remote control
    if (!retval.containsError())
    {
        request = QByteArray("S12"); // ask for remote mode
        retval += SendQuestionWithAnswerString(request, answer, 500);
        if (!retval.containsError() && answer.contains("A11"))
        {
            m_params["local"].setVal<int>(0); // local mode
        }
        else if (!retval.containsError() && answer.contains("A12"))
        {
            m_params["local"].setVal<int>(1); // remote mode
        }
        else
        {
            retval += ito::RetVal::format(ito::retError, 0, tr("invalid answer '%s' for sending  '%s'").toLatin1().data(), answer.data(), request.data());
        }
    }

    //__________________________________________________________________________________________________________ check optical output, power mode and SLD error (regularly at startup of device)
    if (!retval.containsError())
    {
        request = QByteArray("S20");
        retval += SendQuestionWithAnswerString(request, answer, 500);
        // check optical output
        QRegularExpression regExp("^A2(\\d{2,2})");
        QRegularExpressionMatch match = regExp.match(answer);
        if (match.hasMatch() && !retval.containsError())
        {
            if (((match.captured(1).toInt()) & 2) == 2)
            {
                m_params["optical_output"].setVal<int>(1);
            }
            else
            {
                m_params["optical_output"].setVal<int>(0);
            }
            if (((match.captured(1).toInt()) & 16) == 16)
            {
                m_params["power_mode"].setVal<int>(1);
            }
            else
            {
                m_params["power_mode"].setVal<int>(0);
            }
            /*if (((regExp.cap(1).toInt()) & 8) == 8)
            {
                m_params["sld_error"].setVal<int>(1);
            }
            else
            {
                m_params["sld_error"].setVal<int>(0);
            }    */
        }
        else
        {
            retval += ito::RetVal::format(ito::retError, 0, tr("invalid answer '%s' for sending  '%s'").toLatin1().data(), answer.data(), request.data());
        }
    }
    return retval;
}
