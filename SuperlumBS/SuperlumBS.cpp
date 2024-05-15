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

#include "SuperlumBS.h"
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

#include "common/helperCommon.h"
#include "common/apiFunctionsInc.h"
//#include "iostream"

#include "dockWidgetSuperlumBS.h"

#define READTIMEOUT 256

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal SuperlumBSInterface::getAddInInst(ito::AddInBase **addInInst)
{
    NEW_PLUGININSTANCE(SuperlumBS)
    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal SuperlumBSInterface::closeThisInst(ito::AddInBase **addInInst)
{
   REMOVE_PLUGININSTANCE(SuperlumBS)
   return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
SuperlumBSInterface::SuperlumBSInterface()
{
    m_type = ito::typeDataIO  | ito::typeRawIO;
    setObjectName("SuperlumBS");

    m_description = QObject::tr("Plugin for Superlum BraodSweeper BS-840-1-HP, BS-840-2-HP, BS-1060-1-HP, BS-1060-2-HP.");

/*    char docstring[] = \
"The SuperlumBS is an itom-plugin, which can be used to communicate with a Superlum BroadSweeper.\n\
Different BroadSweeper types (BS-840-1-HP, BS-840-2-HP, BS-1060-1-HP, BS-1060-2-HP) are implemented.\n\
Only BS-840-1-HP is tested.\n\
The company website can be found under http://www.superlumdiodes.com \n\
This system needs a serial port, which differs depending on the controller type. \
The parameters of the serial port (besides port number) are set automatically during initialization. \n\
\n\
It is initialized by actuator(\"SuperlumBS\", SerialIO).";
    m_detaildescription = QObject::tr(docstring);*/
    m_detaildescription = QObject::tr("The SuperlumBS is an itom-plugin, which can be used to communicate with a Superlum BroadSweeper.\n\
Different BroadSweeper types (BS-840-1-HP, BS-840-2-HP, BS-1060-1-HP, BS-1060-2-HP) are implemented.\n\
Only BS-840-1-HP is tested.\n\
The company website can be found under http://www.superlumdiodes.com \n\
This system needs a serial port, which differs depending on the controller type. \
The parameters of the serial port (besides port number) are set automatically during initialization. \n\
\n\
It is initialized by dataIO(\"SuperlumBS\", SerialIO).");

    m_author = PLUGIN_AUTHOR;
    m_version = PLUGIN_VERSION;
    m_minItomVer = PLUGIN_MIN_ITOM_VERSION;
    m_maxItomVer = PLUGIN_MAX_ITOM_VERSION;
    m_license = QObject::tr(PLUGIN_LICENCE);
    m_aboutThis = QObject::tr(GITVERSION);

    ito::Param paramVal("serial", ito::ParamBase::HWRef | ito::ParamBase::In, NULL, tr("An opened serial port (the right communication parameters will be set by this Superlum BroadSweeper).").toLatin1().data());
    paramVal.setMeta(new ito::HWMeta("SerialIO"), true);
    m_initParamsMand.append(paramVal);

    paramVal = ito::Param("deviceName", ito::ParamBase::String | ito::ParamBase::In, "BS-840-1-HP", tr("Device name of the Superlum BroadSweeper [BS-840-1-HP].").toLatin1().data());
    ito::StringMeta *deviceMeta = new ito::StringMeta(ito::StringMeta::String);
    deviceMeta->addItem("BS-840-1-HP");
    paramVal.setMeta(deviceMeta, true);
    m_initParamsOpt.append(paramVal);
}

//----------------------------------------------------------------------------------------------------------------------------------
QStringList regexHelper(const char* reg, const QByteArray& charToInspect)
{
    QRegularExpression regExp(reg);
    QRegularExpressionMatch match = regExp.match(charToInspect);
    QStringList result = match.capturedTexts();
    return result;
}

//----------------------------------------------------------------------------------------------------------------------------------
const ito::RetVal SuperlumBS::showConfDialog(void)
{
    return apiShowConfigurationDialog(this, new DialogSuperlumBS(this));
}

//----------------------------------------------------------------------------------------------------------------------------------
SuperlumBS::SuperlumBS() : AddInDataIO(), m_pSer(NULL), m_delayAfterSendCommandMS(0), m_dockWidget(NULL)
{
    ito::Param paramVal("name", ito::ParamBase::String | ito::ParamBase::Readonly | ito::ParamBase::NoAutosave, "Superlum BroadSweeper", tr("Name of plugin.").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("comPort", ito::ParamBase::Int | ito::ParamBase::Readonly, 0, 65355, 0, tr("The current com-port ID of this specific device. -1 means undefined.").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("serial_number", ito::ParamBase::String | ito::ParamBase::Readonly, "unknown", tr("Serial number of device.").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("remote_interlock", ito::ParamBase::Int | ito::ParamBase::Readonly, 0, 1, 0, tr("Remote Interlock is in open (0) or is closed (1).").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("master_key", ito::ParamBase::Int | ito::ParamBase::Readonly, 0, 1, 1, tr("Master Key is in position O (0) or position I (1).").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("full_tuning_range_LOW_end", ito::ParamBase::Double | ito::ParamBase::Readonly, 820.00, 870.00, 870.00, tr("FULL spectral tuning range of sweeping in AUTOmatic OR EXTernal sweep mode in LOW power mode.").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("full_tuning_range_LOW_start", ito::ParamBase::Double | ito::ParamBase::Readonly, 820.00, 870.00, 870.00, tr("FULL spectral tuning range of sweeping in AUTOmatic OR EXTernal sweep mode in HIGH power mode.").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("full_tuning_range_HIGH_end", ito::ParamBase::Double | ito::ParamBase::Readonly, 820.00, 870.00, 870.00, tr("FULL spectral tuning range of sweeping in AUTOmatic OR EXTernal sweep mode in LOW power mode.").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("full_tuning_range_HIGH_start", ito::ParamBase::Double | ito::ParamBase::Readonly, 820.00, 870.00, 870.00, tr("FULL spectral tuning range of sweeping in AUTOmatic OR EXTernal sweep mode in HIGH power mode.").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("operation_mode", ito::ParamBase::Int, 1, 4, 1, tr("(1) MANual, (2) AUTOmatic, (3) EXTernal, (4) MODulation.").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("local", ito::ParamBase::Int, 0, 1, 1, tr("(0) local or (1) remote mode.").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("operation_booster", ito::ParamBase::Int | ito::ParamBase::Readonly, -1, 1, 0, tr("(-1) booster module is not installed, (0) optical output of booster is disabled, (1) optical output of booster is enabled.").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("power_mode", ito::ParamBase::Int, 0, 1, 0, tr("(0) LOW Power mode, (1) HIGH Power mode.").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("modulation_frequency", ito::ParamBase::Double, 0.1, 1000.0, (0.1, 0.2, 0.5, 1.0, 2.0, 5.0, 10.0, 20.0, 50.0, 100.0, 200.0, 500.0, 1000.0), tr("Modulation frequency in Two-Wavelength MODulation mode.").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("wavelength_first", ito::ParamBase::Double, 820.00, 870.00, 870.00, tr("first wavelength in Two-Wavelength MODulation mode.").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("wavelength_second", ito::ParamBase::Double, 820.00, 870.00, 820.00, tr("second wavelength in Two-Wavelength MODulation mode.").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("sweep_speed", ito::ParamBase::Int, 2, 10000, 10, tr("sweep speed in AUTOmatic or EXTernal mode between 2 nm/s - 10000 nm/s. Increment: 1nm/s.").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("modification_end_wavelength", ito::ParamBase::Double, 820.00, 870.00, 820.00, tr("end modification wavelength in AUTOmatic or EXTernal sweep mode.").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("modification_start_wavelength", ito::ParamBase::Double, 820.00, 870.00, 870.00, tr("start modification wavelength in AUTOmatic or EXTernal sweep mode.").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("wavelength", ito::ParamBase::Double, 820.00, 870.00, 845.00, tr("operation wavelength [nm] in MANual Mode. Increment: 0.05 nm.").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("optical_output", ito::ParamBase::Int, 0, 1, 0, tr("(0) optical output is disabled, (1) optical output is enabled.").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);

    if (hasGuiSupport())
    {
        //now create dock widget for this plugin
        DockWidgetSuperlumBS *m_dockWidget = new DockWidgetSuperlumBS(getID(), this);
        Qt::DockWidgetAreas areas = Qt::AllDockWidgetAreas;
        QDockWidget::DockWidgetFeatures features = QDockWidget::DockWidgetClosable | QDockWidget::DockWidgetFloatable | QDockWidget::DockWidgetMovable;
        createDockWidget(QString(m_params["name"].getVal<char *>()), features, areas, m_dockWidget);
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal SuperlumBS::getParam(QSharedPointer<ito::Param> val, ItomSharedSemaphore *waitCond)
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
ito::RetVal SuperlumBS::setParam(QSharedPointer<ito::ParamBase> val, ItomSharedSemaphore *waitCond)
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

                //__________________________________________________________________________________________________________ Operation Mode
                else if (paramName == "operation_mode")
                {
                    // Optical output needs to be disabled before changing operation mode
                    request = QByteArray("S20");
                    retValue += SendQuestionWithAnswerString(request, answer, 500);  //optical output check query
                    QRegularExpression regEx("^A2(\\d{3,3})(\\d{2,2})");
                    QRegularExpressionMatch match = regEx.match(answer);
                    if (match.hasMatch() && !retValue.containsError())
                    {
                        request = QByteArray("S6") + QByteArray::number(val->getVal<int>());
                        bool optical;
                        switch (match.captured(1).toInt())
                        {
                            case 97:
                            case 101:
                            case 113:
                            case 117:
                                optical = false; // optical output is disabled
                                break;

                            case 99:
                            case 103:
                            case 115:
                            case 119:
                                optical = true; // optical output is enabled
                                break;

                            default:
                                retValue += ito::RetVal::format(ito::retError, 0, tr("invalid answer '%s'.").toLatin1().data(), answer.data());
                                break;
                        }

                        if (optical)
                        {
                            retValue += ito::RetVal(ito::retError, 0, tr("Optical output of device is enabled!").toLatin1().data());
                        }
                        else if (!optical)
                        {
                            retValue += SendQuestionWithAnswerString(request, answer, 500);  //set operation mode of device
                            QStringList regexStringList2 = regexHelper("^A6(1|2|3|4)$", answer);
                            if (!regexStringList2.isEmpty() && !retValue.containsError())
                            {
                                m_params["operation_mode"].setVal<int>(regexStringList2[1].toInt());
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
                    else
                    {
                        retValue += ito::RetVal::format(ito::retError, 0, tr("invalid answer '%s' for sending  '%s'").toLatin1().data(), answer.data(), request.data());
                    }
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
                    QStringList regexStringList = regexHelper("^A2(\\d{3,3})(\\d{2,2})", answer);
                    if (!regexStringList.isEmpty() && !retValue.containsError())
                    {
                        switch (regexStringList[1].toInt())
                        {
                            case 97:
                            case 101:
                            case 113:
                            case 117:
                                outputOpt = false;//optical output disabled
                                break;
                            case 99:
                            case 103:
                            case 115:
                            case 119:

                                outputOpt = true;//optical output enabled
                                break;

                            default:
                                retValue += ito::RetVal::format(ito::retError, 0, tr("invalid answer '%s'.").toLatin1().data(), answer.data());
                                break;
                        }

                        if (!retValue.containsError() && outputOpt && (val->getVal<int>() == 0)) //disabled optical output
                        {
                            request = QByteArray("S21");
                            retValue += SendQuestionWithAnswerString(request, answer, 500);
                            if (answer.contains("A2") && !retValue.containsError())
                            {
                                m_params["optical_output"].setVal<int>(0);
                            }
                            else
                            {
                                retValue += ito::RetVal::format(ito::retError, 0, tr("invalid answer '%s' for sending  '%s'").toLatin1().data(), answer.data(), request.data());
                            }
                        }
                        else if (!retValue.containsError() && !outputOpt && (val->getVal<int>() == 1)) //enable optical output
                        {
                            request = QByteArray("S21");
                            retValue += SendQuestionWithAnswerString(request, answer, 500);
                            if (answer.contains("A2") && !retValue.containsError())
                            {
                                m_params["optical_output"].setVal<int>(1);
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
                    QStringList regexStringList = regexHelper("^A2(\\d{3,3})(\\d{2,2})", answer);
                    if (!regexStringList.isEmpty() && !retValue.containsError())
                    {
                        switch (regexStringList[1].toInt())
                        {
                            case 99: //optical output enabled
                            case 103:
                            case 115:
                            case 119:
                                retValue += ito::RetVal::format(ito::retError, 0, tr("Optical Output is ENABLED!").toLatin1().data());
                                break;

                            case 97: //optical output disabled
                            case 101:
                            case 113:
                            case 117:
                                break;

                            default:
                                retValue += ito::RetVal::format(ito::retError, 0, tr("invalid answer '%s'.").toLatin1().data(), answer.data());
                                break;
                        }
                    }
                    else
                    {
                        retValue += ito::RetVal::format(ito::retError, 0, tr("invalid answer '%s' for sending  '%s'").toLatin1().data(), answer.data(), request.data());
                    }
                    QStringList regexStringList2 = regexHelper("^A2(\\d{3,3})(\\d{2,2})", answer);
                    if (!regexStringList2.isEmpty() && !retValue.containsError())
                    {
                        switch (regexStringList2[1].toInt())
                        {
                            case 97: //low power mode
                            case 99:
                            case 101:
                            case 103:
                                powermod = false;
                                break;

                            case 113: //high power mode
                            case 115:
                            case 117:
                            case 119:
                                powermod = true;
                                break;

                            default:
                                retValue += ito::RetVal::format(ito::retError, 0, tr("invalid answer '%s'.").toLatin1().data(), answer.data());
                                break;
                        }
                    }
                       else
                    {
                        retValue += ito::RetVal::format(ito::retError, 0, tr("invalid answer '%s'.").toLatin1().data(), answer.data());
                    }

                    if (!retValue.containsError() && powermod && (val->getVal<int>() == 0)) //power mode to LOW
                    {
                        request = QByteArray("S41");
                        retValue += SendQuestionWithAnswerString(request, answer, 500);
                        if (retValue.containsError())
                        {
                            retValue += ito::RetVal::format(ito::retError, 0, tr("invalid answer '%s' for sending  '%s'").toLatin1().data(), answer.data(), request.data());
                        }
                        else
                        {
                            m_params["power_mode"].setVal<int>(0);
                            static_cast<ito::DoubleMeta*>(m_params["modification_end_wavelength"].getMeta())->setMin(m_params["full_tuning_range_LOW_end"].getVal<double>());
                            static_cast<ito::DoubleMeta*>(m_params["modification_end_wavelength"].getMeta())->setMax(m_params["full_tuning_range_LOW_start"].getVal<double>());
                            static_cast<ito::DoubleMeta*>(m_params["modification_start_wavelength"].getMeta())->setMin(m_params["full_tuning_range_LOW_end"].getVal<double>());
                            static_cast<ito::DoubleMeta*>(m_params["modification_start_wavelength"].getMeta())->setMax(m_params["full_tuning_range_LOW_start"].getVal<double>());
                            static_cast<ito::DoubleMeta*>(m_params["wavelength"].getMeta())->setMin(m_params["full_tuning_range_LOW_end"].getVal<double>());
                            static_cast<ito::DoubleMeta*>(m_params["wavelength"].getMeta())->setMax(m_params["full_tuning_range_LOW_start"].getVal<double>());
                            static_cast<ito::DoubleMeta*>(m_params["wavelength_first"].getMeta())->setMin(m_params["full_tuning_range_LOW_end"].getVal<double>());
                            static_cast<ito::DoubleMeta*>(m_params["wavelength_first"].getMeta())->setMax(m_params["full_tuning_range_LOW_start"].getVal<double>());
                            static_cast<ito::DoubleMeta*>(m_params["wavelength_second"].getMeta())->setMin(m_params["full_tuning_range_LOW_end"].getVal<double>());
                            static_cast<ito::DoubleMeta*>(m_params["wavelength_second"].getMeta())->setMax(m_params["full_tuning_range_LOW_start"].getVal<double>());
                        }
                    }
                    else if (!retValue.containsError() && !powermod && (val->getVal<int>() == 1)) //power mode to HIGH
                    {
                        request = QByteArray("S41");
                        retValue += SendQuestionWithAnswerString(request, answer, 500);
                        if (retValue.containsError())
                        {
                            retValue += ito::RetVal::format(ito::retError, 0, tr("invalid answer '%s' for sending  '%s'").toLatin1().data(), answer.data(), request.data());
                        }
                        else
                        {
                            m_params["power_mode"].setVal<int>(1);
                            static_cast<ito::DoubleMeta*>(m_params["modification_end_wavelength"].getMeta())->setMin(m_params["full_tuning_range_HIGH_end"].getVal<double>());
                            static_cast<ito::DoubleMeta*>(m_params["modification_end_wavelength"].getMeta())->setMax(m_params["full_tuning_range_HIGH_start"].getVal<double>());
                            static_cast<ito::DoubleMeta*>(m_params["modification_start_wavelength"].getMeta())->setMin(m_params["full_tuning_range_HIGH_end"].getVal<double>());
                            static_cast<ito::DoubleMeta*>(m_params["modification_start_wavelength"].getMeta())->setMax(m_params["full_tuning_range_HIGH_start"].getVal<double>());
                            static_cast<ito::DoubleMeta*>(m_params["wavelength"].getMeta())->setMin(m_params["full_tuning_range_HIGH_end"].getVal<double>());
                            static_cast<ito::DoubleMeta*>(m_params["wavelength"].getMeta())->setMax(m_params["full_tuning_range_HIGH_start"].getVal<double>());
                            static_cast<ito::DoubleMeta*>(m_params["wavelength_first"].getMeta())->setMin(m_params["full_tuning_range_HIGH_end"].getVal<double>());
                            static_cast<ito::DoubleMeta*>(m_params["wavelength_first"].getMeta())->setMax(m_params["full_tuning_range_HIGH_start"].getVal<double>());
                            static_cast<ito::DoubleMeta*>(m_params["wavelength_second"].getMeta())->setMin(m_params["full_tuning_range_HIGH_end"].getVal<double>());
                            static_cast<ito::DoubleMeta*>(m_params["wavelength_second"].getMeta())->setMax(m_params["full_tuning_range_HIGH_start"].getVal<double>());
                        }
                    }
                    else if (!retValue.containsError() && !powermod && (val->getVal<int>() == 0)) // not changed
                    {
                    }
                    else if (!retValue.containsError() && powermod && (val->getVal<int>() == 1)) // not changed
                    {
                    }
                       else
                    {
                        retValue += ito::RetVal::format(ito::retError,0,"invalid answer '%s' for sending  '%s'", answer.data(), request.data());
                    }
                }

                //__________________________________________________________________________________________________________ Wavelength
                else if (paramName == "wavelength")
                {
                    request = QByteArray("S81") + (QByteArray::number(20 * val->getVal<double>() - 14000));
                    retValue += SendQuestionWithAnswerString(request, answer, 500);  //set wavelength of optical output in Manual sweep mode
                    QStringList regexStringList = regexHelper("^A81(\\d{4,4})", answer);
                    if (!regexStringList.isEmpty() && !retValue.containsError())
                    {
                        m_params["wavelength"].setVal<double>(
                            0.05 * regexStringList[1].toInt() + 700);
                    }
                    else
                    {
                        retValue += ito::RetVal::format(ito::retError, 0, tr("invalid answer '%s' for sending  '%s'").toLatin1().data(), answer.data(), request.data());
                    }
                }

                //__________________________________________________________________________________________________________ Sweep Speed
                else if (paramName == "sweep_speed")
                {
                    if (val->getVal<double>() < 10)
                    {
                        request = QByteArray("S88") + (QByteArray::number(val->getVal<double>())); //set sweep speed of device (1-byte code for 2 - 9 nm/s)
                        retValue += SendQuestionWithAnswerString(request, answer, 500);
                        QStringList regexStringList = regexHelper("^A88(\\d{1,1})", answer);

                        if (regexStringList.length() > 0 && !retValue.containsError())
                        {
                            m_params["sweep_speed"].setVal<int>(regexStringList[1].toInt());
                        }
                        else
                        {
                            retValue += ito::RetVal::format(ito::retError, 0, tr("invalid answer '%s' for sending  '%s'").toLatin1().data(), answer.data(), request.data());
                        }
                    }
                    else if (val->getVal<double>() >= 10)
                    {
                        QByteArray speed("0000");
                        speed.replace(4-QByteArray::number(0.1 * val->getVal<double>()).length(), QByteArray::number(0.1 * val->getVal<double>()).length(), QByteArray::number(0.1 * val->getVal<double>()));
                        request = QByteArray("S84").append(speed.data()); //set sweep speed of device (4-byte code for 10 - 10000 nm/s)
                        retValue += SendQuestionWithAnswerString(request, answer, 500);
                        QStringList regexStringList = regexHelper("^A84(\\d{4,4})", answer);

                        if (regexStringList.length() > 0 && !retValue.containsError())
                        {
                            m_params["sweep_speed"].setVal<int>(10 * regexStringList[1].toInt());
                        }
                        else
                        {
                            retValue += ito::RetVal::format(ito::retError, 0, tr("invalid answer '%s' for sending  '%s'").toLatin1().data(), answer.data(), request.data());
                        }
                    }
                    else
                    {
                        retValue += ito::RetVal::format(ito::retError, 0, tr("invalid sweep speed value '%s'").toLatin1().data(), request.data());
                    }
                }

                //__________________________________________________________________________________________________________ Modification Wavelength
                else if (paramName == "modification_end_wavelength")
                {
                    request = QByteArray("S83") + (QByteArray::number(20 * val->getVal<double>() - 14000));
                    retValue += SendQuestionWithAnswerString(request, answer, 500);  //set end wavelength in Automatic or external sweep mode
                    QStringList regexStringList = regexHelper("^A83(\\d{4,4})", answer);
                    if (!regexStringList.isEmpty() && !retValue.containsError())
                    {
                        m_params["modification_end_wavelength"].setVal<double>(
                            0.05 * regexStringList[1].toInt() + 700);
                    }
                    else
                    {
                        retValue += ito::RetVal::format(ito::retError, 0, tr("invalid answer '%s' for sending  '%s'").toLatin1().data(), answer.data(), request.data());
                    }
                }

                //__________________________________________________________________________________________________________ Modification Wavelength
                else if (paramName == "modification_start_wavelength")
                {
                    request = QByteArray("S82") + (QByteArray::number(20 * val->getVal<double>() - 14000));
                    retValue += SendQuestionWithAnswerString(request, answer, 500);  //set start wavelength in Automatic or external sweep mode
                    QStringList regexStringList = regexHelper("^A82(\\d{4,4})", answer);
                    if (!regexStringList.isEmpty() && !retValue.containsError())
                    {
                        m_params["modification_start_wavelength"].setVal<double>(
                            0.05 * regexStringList[1].toInt() + 700);
                    }
                    else
                    {
                        retValue += ito::RetVal::format(ito::retError, 0, tr("invalid answer '%s' for sending  '%s'").toLatin1().data(), answer.data(), request.data());
                    }
                }

                //__________________________________________________________________________________________________________ First Wavelength
                else if (paramName == "wavelength_first")
                {
                    request = QByteArray("S85") + (QByteArray::number(20 * val->getVal<double>() - 14000));
                    retValue += SendQuestionWithAnswerString(request, answer, 500);  //set first wavelength in TWO-WAVELENGTH MODULATION mode
                    QStringList regexStringList = regexHelper("^A85(\\d{4,4})", answer);
                    if (!regexStringList.isEmpty() && !retValue.containsError())
                    {
                        m_params["wavelength_first"].setVal<double>(
                            0.05 * regexStringList[1].toInt() + 700);
                    }
                    else
                    {
                        retValue += ito::RetVal::format(ito::retError, 0, tr("invalid answer '%s' for sending  '%s'").toLatin1().data(), answer.data(), request.data());
                    }
                }

                //__________________________________________________________________________________________________________ Second Wavelength
                else if (paramName == "wavelength_second")
                {
                    request = QByteArray("S86") + (QByteArray::number(20 * val->getVal<double>() - 14000));
                    retValue += SendQuestionWithAnswerString(request, answer, 500);  //set second wavelength in TWO-WAVELENGTH MODULATION mode
                    QStringList regexStringList = regexHelper("^A86(\\d{4,4})", answer);
                    if (!regexStringList.isEmpty() && !retValue.containsError())
                    {
                        m_params["wavelength_second"].setVal<double>(
                            0.05 * regexStringList[1].toInt() + 700);
                    }
                    else
                    {
                        retValue += ito::RetVal::format(ito::retError, 0, tr("invalid answer '%s' for sending  '%s'").toLatin1().data(), answer.data(), request.data());
                    }
                }

                //__________________________________________________________________________________________________________ Modulation frequency
                else if (paramName == "modulation_frequency")
                {
                    int mod_f = val->getVal<double>() * 10;
                    switch (mod_f) //set modulation frequency of TWO-WAVELENGTH MODULATION mode
                    {
                        case 1:
                            retValue += SendQuestionWithAnswerString("S8701", answer, 500);
                            break;
                        case 2:
                            retValue += SendQuestionWithAnswerString("S8702", answer, 500);
                            break;
                        case 5:
                            retValue += SendQuestionWithAnswerString("S8703", answer, 500);
                            break;
                        case 10:
                            retValue += SendQuestionWithAnswerString("S8704", answer, 500);
                            break;
                        case 20:
                            retValue += SendQuestionWithAnswerString("S8705", answer, 500);
                            break;
                        case 50:
                            retValue += SendQuestionWithAnswerString("S8706", answer, 500);
                            break;
                        case 100:
                            retValue += SendQuestionWithAnswerString("S8707", answer, 500);
                            break;
                        case 200:
                            retValue += SendQuestionWithAnswerString("S8708", answer, 500);
                            break;
                        case 500:
                            retValue += SendQuestionWithAnswerString("S8709", answer, 500);
                            break;
                        case 1000:
                            retValue += SendQuestionWithAnswerString("S8710", answer, 500);
                            break;
                        case 2000:
                            retValue += SendQuestionWithAnswerString("S8711", answer, 500);
                            break;
                        case 5000:
                            retValue += SendQuestionWithAnswerString("S8712", answer, 500);
                            break;
                        case 10000:
                            retValue += SendQuestionWithAnswerString("S8713", answer, 500);
                            break;

                        default:
                            retValue += ito::RetVal::format(ito::retError, 0, tr("'%s' is not a possible modulation frequency.").toLatin1().data(), answer.data());
                            break;
                    }

                    request = QByteArray("S77");
                    retValue += SendQuestionWithAnswerString(request, answer, 500);  //operational parameters check query | modulation frequency in TWO-WAVELENGTH-MODULATION mode
                    QStringList regexStringList = regexHelper("^A77(\\d{2,2})", answer);
                    if (!regexStringList.isEmpty() && !retValue.containsError())
                    {
                        switch (regexStringList[1].toInt())
                        {
                            case 1:
                                m_params["modulation_frequency"].setVal<double>(0.1);
                                break;
                            case 2:
                                m_params["modulation_frequency"].setVal<double>(0.2);
                                break;
                            case 3:
                                m_params["modulation_frequency"].setVal<double>(0.5);
                                break;
                            case 4:
                                m_params["modulation_frequency"].setVal<double>(1.0);
                                break;
                            case 5:
                                m_params["modulation_frequency"].setVal<double>(2.0);
                                break;
                            case 6:
                                m_params["modulation_frequency"].setVal<double>(5.0);
                                break;
                            case 7:
                                m_params["modulation_frequency"].setVal<double>(10.0);
                                break;
                            case 8:
                                m_params["modulation_frequency"].setVal<double>(20.0);
                                break;
                            case 9:
                                m_params["modulation_frequency"].setVal<double>(50.0);
                                break;
                            case 10:
                                m_params["modulation_frequency"].setVal<double>(100.0);
                                break;
                            case 11:
                                m_params["modulation_frequency"].setVal<double>(200.0);
                                break;
                            case 12:
                                m_params["modulation_frequency"].setVal<double>(500.0);
                                break;
                            case 13:
                                m_params["modulation_frequency"].setVal<double>(1000.0);
                                break;

                            default:
                                retValue += ito::RetVal::format(ito::retError, 0, tr("invalid answer '%s'.").toLatin1().data(), answer.data());
                                break;
                        }
                    }
                    else
                    {
                        retValue += ito::RetVal::format(ito::retError, 0, tr("invalid answer '%s' for sending  '%s'").toLatin1().data(), answer.data(), request.data());
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
ito::RetVal SuperlumBS::init(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retval = ito::retOk;
    QByteArray answer;

    QString deviceName = paramsOpt->at(0).getVal<char*>();

    if (deviceName == "BS-840-1-HP")
    {
        m_deviceType = BS_840_1_HP;
        m_identifier = QString("BS-840-HP-1 (%1)").arg(getID());
    }
    else
    {
        retval += ito::RetVal::format(ito::retError, 0, tr("Device name '%s' not supported").toLatin1().data(), deviceName.toLatin1().data());
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
ito::RetVal SuperlumBS::close(ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);
    QByteArray answer;
    QByteArray request;

    request = QByteArray("S20");
    retValue += SendQuestionWithAnswerString(request, answer, 500);  //ask, if optical output is enabled
    QStringList regexStringList = regexHelper("^A2(\\d{3,3})(\\d{2,2})", answer);
    if (!regexStringList.isEmpty() && !retValue.containsError())
    {
        switch (regexStringList[1].toInt())
        {
            case 99:
            case 103:
            case 115:
            case 119:
                retValue += SendQuestionWithAnswerString("S21", answer, 500);  //disable optical output
                break;

            case 97:                                                            // optical output already disabled
            case 101:
            case 113:
            case 117:

                break;
            default:
                retValue += ito::RetVal::format(ito::retError, 0, tr("invalid answer '%s'.").toLatin1().data(), answer.data());
                break;
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
ito::RetVal SuperlumBS::calib(const int axis, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue;
    retValue += ito::RetVal::format(ito::retError, 0, tr("function not defined").toLatin1().data());

    if (waitCond)
    {
        waitCond->returnValue = retValue;
        waitCond->release();
    }

    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal SuperlumBS::calib(const QVector<int> /*axis*/, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue;
    retValue += ito::RetVal::format(ito::retError, 0, tr("function not defined").toLatin1().data());

    if (waitCond)
    {
        waitCond->returnValue = retValue;
        waitCond->release();
    }

    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal SuperlumBS::setOrigin(const int axis, ItomSharedSemaphore * waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue;
    retValue += ito::RetVal::format(ito::retError, 0, tr("function not defined").toLatin1().data());

    if (waitCond)
    {
        waitCond->returnValue = retValue;
        waitCond->release();
    }

    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal SuperlumBS::setOrigin(QVector<int> axis, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue;
    retValue += ito::RetVal::format(ito::retError, 0, tr("function not defined").toLatin1().data());

    if (waitCond)
    {
        waitCond->returnValue = retValue;
        waitCond->release();
    }

    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal SuperlumBS::getStatus(QSharedPointer<QVector<int> > status, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue;
    retValue += ito::RetVal::format(ito::retError, 0, tr("function not defined").toLatin1().data());

    if (waitCond)
    {
        waitCond->returnValue = retValue;
        waitCond->release();
    }

    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal SuperlumBS::getPos(const int axis, QSharedPointer<double> pos, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue;
    retValue += ito::RetVal::format(ito::retError, 0, tr("function not defined").toLatin1().data());

    if (waitCond)
    {
        waitCond->returnValue = retValue;
        waitCond->release();
    }

    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal SuperlumBS::getPos(const QVector<int> axis, QSharedPointer<QVector<double> > pos, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue;
    retValue += ito::RetVal::format(ito::retError, 0, tr("function not defined").toLatin1().data());

    if (waitCond)
    {
        waitCond->returnValue = retValue;
        waitCond->release();
    }

    return retValue;
}


//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal SuperlumBS::setPosAbs(const int axis, const double pos, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue;
    retValue += ito::RetVal::format(ito::retError, 0, tr("function not defined").toLatin1().data());

    if (waitCond)
    {
        waitCond->returnValue = retValue;
        waitCond->release();
    }

    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal SuperlumBS::setPosAbs(const QVector<int> axis, QVector<double> pos, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue;
    retValue += ito::RetVal::format(ito::retError, 0, tr("function not defined").toLatin1().data());

    if (waitCond)
    {
        waitCond->returnValue = retValue;
        waitCond->release();
    }

    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal SuperlumBS::setPosRel(const int axis, const double pos, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue;
    retValue += ito::RetVal::format(ito::retError, 0, tr("function not defined").toLatin1().data());
    if (waitCond)
    {
        waitCond->returnValue = retValue;
        waitCond->release();
    }

    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal SuperlumBS::setPosRel(const QVector<int> axis, QVector<double> pos, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue;
    retValue += ito::RetVal::format(ito::retError, 0, tr("function not defined").toLatin1().data());
    if (waitCond)
    {
        waitCond->returnValue = retValue;
        waitCond->release();
    }

    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal SuperlumBS::waitForDone(const int timeoutMS, const QVector<int> /*axis*/ /*if empty -> all axis*/, const int /*flags*/ /*for your use*/)
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
ito::RetVal SuperlumBS::requestStatusAndPosition(bool sendCurrentPos, bool sendTargetPos)
{
    ito::RetVal retValue;
    retValue += ito::RetVal::format(ito::retError, 0, tr("function not defined").toLatin1().data());
    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
void SuperlumBS::dockWidgetVisibilityChanged(bool visible)
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
ito::RetVal SuperlumBS::SendQuestionWithAnswerString(QByteArray questionCommand, QByteArray &answer, int timeoutMS)
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
ito::RetVal SuperlumBS::readString(QByteArray &questionCommand, QByteArray &result, int &len, int timeoutMS)
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

    if (m_deviceType == BS_840_1_HP)
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

    if (!retValue.containsError())
    {
        if (result.contains("AE"))// general error!
        {
            retValue += ito::RetVal(ito::retError,0, tr(m_params["serial_number"].getVal<char*>(), "general error!").toLatin1().data());
            return retValue;
        }
        else if (result.contains("AL"))// device is in local mode and can not be controlled by remote
        {
            m_params["local"].setVal<int>(0);
            retValue += ito::RetVal(ito::retError,0, tr(m_params["serial_number"].getVal<char*>(), ("Devices is in local mode! Remote needs to be activated!")).toLatin1().data());
            return retValue;
        }
        else if (result.contains("A2033"))// Alarm Remote Interlock connection!
        {
                retValue += ito::RetVal(ito::retError, 0, tr("Alarm, Remote Interlock is opened!!! Close the Remote Interlock.").toLatin1().data());
                m_params["remote_interlock"].setVal<int>(0);
        }
        else if (result.contains("A2065"))// Master Key is in position O!
        {
                retValue += ito::RetVal(ito::retError, 0, tr("Master Key is in position O!!! Turn the Master Key to I.").toLatin1().data());
                m_params["master_key"].setVal<int>(0);
        }
        else //If Remote Interlock and Master Key OK
        {
            m_params["remote_interlock"].setVal<int>(1);
            m_params["master_key"].setVal<int>(1);
        }
    }
    else
    {
        retValue += ito::RetVal::format(ito::retError, 0, tr("invalid answer '%s'.").toLatin1().data(), answer.data());
    }

    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal SuperlumBS::SerialSendCommand(QByteArray command)
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
ito::RetVal SuperlumBS::IdentifyAndInitializeSystem()
{
    ito::RetVal retval = ito::retOk;
    QByteArray answer;
    QByteArray request;

    if (m_deviceType == BS_840_1_HP)
    {
        //set serial settings for BS-840-1-HP
        retval += m_pSer->setParam(QSharedPointer<ito::ParamBase>(new ito::ParamBase("baud", ito::ParamBase::Int, 57600)), NULL);
        retval += m_pSer->setParam(QSharedPointer<ito::ParamBase>(new ito::ParamBase("bits", ito::ParamBase::Int, 8)), NULL);
        retval += m_pSer->setParam(QSharedPointer<ito::ParamBase>(new ito::ParamBase("parity", ito::ParamBase::Double, 0.0)), NULL);
        retval += m_pSer->setParam(QSharedPointer<ito::ParamBase>(new ito::ParamBase("stopbits", ito::ParamBase::Int, 1)), NULL);
        retval += m_pSer->setParam(QSharedPointer<ito::ParamBase>(new ito::ParamBase("flow", ito::ParamBase::Int, 0)), NULL);
        retval += m_pSer->setParam(QSharedPointer<ito::ParamBase>(new ito::ParamBase("endline", ito::ParamBase::String, "\r\n")), NULL);
    }
    else
    {
        //default serial settings
        retval += m_pSer->setParam(QSharedPointer<ito::ParamBase>(new ito::ParamBase("baud", ito::ParamBase::Int, 57600)), NULL);
        retval += m_pSer->setParam(QSharedPointer<ito::ParamBase>(new ito::ParamBase("bits", ito::ParamBase::Int, 8)), NULL);
        retval += m_pSer->setParam(QSharedPointer<ito::ParamBase>(new ito::ParamBase("parity", ito::ParamBase::Double, 0.0)), NULL);
        retval += m_pSer->setParam(QSharedPointer<ito::ParamBase>(new ito::ParamBase("stopbits", ito::ParamBase::Int, 1)), NULL);
        retval += m_pSer->setParam(QSharedPointer<ito::ParamBase>(new ito::ParamBase("flow", ito::ParamBase::Int, 0)), NULL);
        retval += m_pSer->setParam(QSharedPointer<ito::ParamBase>(new ito::ParamBase("endline", ito::ParamBase::String, "\r\n")), NULL);
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

    //__________________________________________________________________________________________________________ Set serial number
    if (!retval.containsError())
    {
        request = QByteArray("S0");
        retval += SendQuestionWithAnswerString(request, answer, 500);
        if (!retval.containsError())
        {
            //ITO Superlum BroadSweeper identification information
            //A0: response code
            //6: this integer means that the type of the instrument is 6.
            //2: this integer means that it is a double-channel device.
            //8: this integer means that the firmware version is 4.
            //0579: serial number
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
        request = QByteArray("S12");
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

    //__________________________________________________________________________________________________________ set FULL tuning range end wavelength LOW power
    if (!retval.containsError())
    {
        //end wavelength of full tuning range in LOW power mode
        request = QByteArray("S51");
        retval += SendQuestionWithAnswerString(request, answer, 500);//end wavelength for LOW power mode
        QStringList regexStringList = regexHelper("^A51(\\d{4,4})", answer);
        if (!regexStringList.isEmpty() && !retval.containsError())
        {
            double value = 0.05 * regexStringList[1].toDouble() + 700;
            m_params["full_tuning_range_LOW_end"].setVal<double>(value);
            static_cast<ito::DoubleMeta*>(m_params["full_tuning_range_LOW_end"].getMeta())->setMin(value);
            static_cast<ito::DoubleMeta*>(m_params["full_tuning_range_LOW_start"].getMeta())->setMin(value);
            static_cast<ito::DoubleMeta*>(m_params["modification_end_wavelength"].getMeta())->setMin(value);
            static_cast<ito::DoubleMeta*>(m_params["modification_start_wavelength"].getMeta())->setMin(value);
            static_cast<ito::DoubleMeta*>(m_params["wavelength"].getMeta())->setMin(value);
            static_cast<ito::DoubleMeta*>(m_params["wavelength_first"].getMeta())->setMin(value);
            static_cast<ito::DoubleMeta*>(m_params["wavelength_second"].getMeta())->setMin(value);
        }
        else
        {
            retval += ito::RetVal::format(ito::retError, 0, tr("invalid answer '%s' for sending  '%s'").toLatin1().data(), answer.data(), request.data());
        }
    }

    //__________________________________________________________________________________________________________ set FULL tuning range start wavelength LOW power
    if (!retval.containsError())
    {
        //end wavelength of full tuning range in LOW power mode
        request = QByteArray("S52");
        retval += SendQuestionWithAnswerString(request, answer, 500);//end wavelength for LOW power mode
        QStringList regexStringList = regexHelper("^A52(\\d{4,4})", answer);
        if (!regexStringList.isEmpty() && !retval.containsError())
        {
            double value = 0.05 * regexStringList[1].toDouble() + 700;
            m_params["full_tuning_range_LOW_start"].setVal<double>(value);

            static_cast<ito::DoubleMeta*>(m_params["full_tuning_range_LOW_end"].getMeta())->setMax(value);
            static_cast<ito::DoubleMeta*>(m_params["full_tuning_range_LOW_start"].getMeta())->setMax(value);
            static_cast<ito::DoubleMeta*>(m_params["modification_end_wavelength"].getMeta())->setMax(value);
            static_cast<ito::DoubleMeta*>(m_params["modification_start_wavelength"].getMeta())->setMax(value);
            static_cast<ito::DoubleMeta*>(m_params["wavelength"].getMeta())->setMax(value);
            static_cast<ito::DoubleMeta*>(m_params["wavelength_first"].getMeta())->setMax(value);
            static_cast<ito::DoubleMeta*>(m_params["wavelength_second"].getMeta())->setMax(value);
        }
        else
        {
            retval += ito::RetVal::format(ito::retError, 0, tr("invalid answer '%s' for sending  '%s'").toLatin1().data(), answer.data(), request.data());
        }
    }

    //__________________________________________________________________________________________________________ set FULL tuning range end wavelength HIGH power
    if (!retval.containsError())
    {
        //end wavelength of full tuning range in LOW power mode
        request = QByteArray("S53");
        retval += SendQuestionWithAnswerString(request, answer, 500);//end wavelength for LOW power mode
        QStringList regexStringList = regexHelper("^A53(\\d{4,4})", answer);
        if (!regexStringList.isEmpty() && !retval.containsError())
        {
            double value = 0.05 * regexStringList[1].toDouble() + 700;
            m_params["full_tuning_range_HIGH_end"].setVal<double>(value);
            static_cast<ito::DoubleMeta*>(m_params["full_tuning_range_HIGH_end"].getMeta())->setMin(value);
            static_cast<ito::DoubleMeta*>(m_params["full_tuning_range_HIGH_start"].getMeta())->setMin(value);
            static_cast<ito::DoubleMeta*>(m_params["modification_end_wavelength"].getMeta())->setMin(value);
            static_cast<ito::DoubleMeta*>(m_params["modification_start_wavelength"].getMeta())->setMin(value);
            static_cast<ito::DoubleMeta*>(m_params["wavelength"].getMeta())->setMin(value);
            static_cast<ito::DoubleMeta*>(m_params["wavelength_first"].getMeta())->setMin(value);
            static_cast<ito::DoubleMeta*>(m_params["wavelength_second"].getMeta())->setMin(value);
        }
        else
        {
            retval += ito::RetVal::format(ito::retError, 0, tr("invalid answer '%s' for sending  '%s'").toLatin1().data(), answer.data(), request.data());
        }
    }

    //__________________________________________________________________________________________________________ set FULL tuning range start wavelength LOW power
    if (!retval.containsError())
    {
        //end wavelength of full tuning range in LOW power mode
        request = QByteArray("S54");
        retval += SendQuestionWithAnswerString("S54", answer, 500);//end wavelength for LOW power mode
        QStringList regexStringList = regexHelper("^A54(\\d{4,4})", answer);
        if (!regexStringList.isEmpty() && !retval.containsError())
        {
            double value = 0.05 * regexStringList[1].toDouble() + 700;
            m_params["full_tuning_range_HIGH_start"].setVal<double>(value);

            static_cast<ito::DoubleMeta*>(m_params["full_tuning_range_HIGH_start"].getMeta())->setMax(value);
            static_cast<ito::DoubleMeta*>(m_params["full_tuning_range_HIGH_end"].getMeta())->setMax(value);
            static_cast<ito::DoubleMeta*>(m_params["modification_end_wavelength"].getMeta())->setMax(value);
            static_cast<ito::DoubleMeta*>(m_params["modification_start_wavelength"].getMeta())->setMax(value);
            static_cast<ito::DoubleMeta*>(m_params["wavelength"].getMeta())->setMax(value);
            static_cast<ito::DoubleMeta*>(m_params["wavelength_first"].getMeta())->setMax(value);
            static_cast<ito::DoubleMeta*>(m_params["wavelength_second"].getMeta())->setMax(value);
        }
        else
        {
            retval += ito::RetVal::format(ito::retError, 0, tr("invalid answer '%s' for sending  '%s'").toLatin1().data(), answer.data(), request.data());
        }
    }

    //__________________________________________________________________________________________________________ check optical output and booster
    if (!retval.containsError())
    {
        request = QByteArray("S20");
        retval += SendQuestionWithAnswerString(request, answer, 500);
        // check optical output
        QStringList regexStringList = regexHelper("^A2(\\d{3,3})(\\d{2,2})", answer);
        if (!regexStringList.isEmpty() && !retval.containsError())
        {
            switch (regexStringList[1].toInt())
            {
                case 97:
                case 101:
                case 113:
                case 117:
                    m_params["optical_output"].setVal<int>(0);
                    break;

                case 99:
                case 103:
                case 115:
                case 119:
                    m_params["optical_output"].setVal<int>(1);
                    break;

                default:
                    retval += ito::RetVal::format(ito::retError, 0, tr("invalid answer '%s'.").toLatin1().data(), answer.data());
                    break;
            }
        }
        else
        {
            retval += ito::RetVal::format(ito::retError, 0, tr("invalid answer '%s' for sending  '%s'").toLatin1().data(), answer.data(), request.data());
        }

        // check booster
        QStringList regexStringList2 = regexHelper("^A2(\\d{3,3})(\\d{2,2})", answer);
        if (!regexStringList2.isEmpty() && !retval.containsError())
        {
            switch (regexStringList2[1].toInt())
            {
                case 0:
                    m_params["operation_booster"].setVal<int>(-1);
                    break;

                case 2:
                case 6:
                    m_params["operation_booster"].setVal<int>(0);
                    break;

                case 3:
                case 7:
                    m_params["operation_booster"].setVal<int>(1);
                    break;

                default:
                    retval += ito::RetVal::format(ito::retError, 0, tr("invalid answer '%s'.").toLatin1().data(), answer.data());
                    break;
            }
        }
        else
        {
            retval += ito::RetVal::format(ito::retError, 0, tr("invalid answer '%s' for sending  '%s'").toLatin1().data(), answer.data(), request.data());
        }
    }

    //__________________________________________________________________________________________________________ optical power check query
    if (!retval.containsError())
    {
        request = QByteArray("S40");
        retval += SendQuestionWithAnswerString(request, answer, 500);
        QStringList regexStringList = regexHelper("^A4(\\d{3,3})(\\d{2,2})", answer);
        if (!regexStringList.isEmpty() && !retval.containsError())
        {
            switch (regexStringList[1].toInt())
            {
                case 97:
                case 101:
                case 99:
                case 103:
                    m_params["power_mode"].setVal<int>(0);
                    static_cast<ito::DoubleMeta*>(m_params["modification_end_wavelength"].getMeta())->setMin(m_params["full_tuning_range_LOW_end"].getVal<double>());
                    static_cast<ito::DoubleMeta*>(m_params["modification_end_wavelength"].getMeta())->setMax(m_params["full_tuning_range_LOW_start"].getVal<double>());
                    static_cast<ito::DoubleMeta*>(m_params["modification_start_wavelength"].getMeta())->setMin(m_params["full_tuning_range_LOW_end"].getVal<double>());
                    static_cast<ito::DoubleMeta*>(m_params["modification_start_wavelength"].getMeta())->setMax(m_params["full_tuning_range_LOW_start"].getVal<double>());
                    static_cast<ito::DoubleMeta*>(m_params["wavelength"].getMeta())->setMin(m_params["full_tuning_range_LOW_end"].getVal<double>());
                    static_cast<ito::DoubleMeta*>(m_params["wavelength"].getMeta())->setMax(m_params["full_tuning_range_LOW_start"].getVal<double>());
                    static_cast<ito::DoubleMeta*>(m_params["wavelength_first"].getMeta())->setMin(m_params["full_tuning_range_LOW_end"].getVal<double>());
                    static_cast<ito::DoubleMeta*>(m_params["wavelength_first"].getMeta())->setMax(m_params["full_tuning_range_LOW_start"].getVal<double>());
                    static_cast<ito::DoubleMeta*>(m_params["wavelength_second"].getMeta())->setMin(m_params["full_tuning_range_LOW_end"].getVal<double>());
                    static_cast<ito::DoubleMeta*>(m_params["wavelength_second"].getMeta())->setMax(m_params["full_tuning_range_LOW_start"].getVal<double>());
                    break;

                case 113:
                case 117:
                case 115:
                case 119:
                    m_params["power_mode"].setVal<int>(1);
                    static_cast<ito::DoubleMeta*>(m_params["modification_end_wavelength"].getMeta())->setMin(m_params["full_tuning_range_HIGH_end"].getVal<double>());
                    static_cast<ito::DoubleMeta*>(m_params["modification_end_wavelength"].getMeta())->setMax(m_params["full_tuning_range_HIGH_start"].getVal<double>());
                    static_cast<ito::DoubleMeta*>(m_params["modification_start_wavelength"].getMeta())->setMin(m_params["full_tuning_range_HIGH_end"].getVal<double>());
                    static_cast<ito::DoubleMeta*>(m_params["modification_start_wavelength"].getMeta())->setMax(m_params["full_tuning_range_HIGH_start"].getVal<double>());
                    static_cast<ito::DoubleMeta*>(m_params["wavelength"].getMeta())->setMin(m_params["full_tuning_range_HIGH_end"].getVal<double>());
                    static_cast<ito::DoubleMeta*>(m_params["wavelength"].getMeta())->setMax(m_params["full_tuning_range_HIGH_start"].getVal<double>());
                    static_cast<ito::DoubleMeta*>(m_params["wavelength_first"].getMeta())->setMin(m_params["full_tuning_range_HIGH_end"].getVal<double>());
                    static_cast<ito::DoubleMeta*>(m_params["wavelength_first"].getMeta())->setMax(m_params["full_tuning_range_HIGH_start"].getVal<double>());
                    static_cast<ito::DoubleMeta*>(m_params["wavelength_second"].getMeta())->setMin(m_params["full_tuning_range_HIGH_end"].getVal<double>());
                    static_cast<ito::DoubleMeta*>(m_params["wavelength_second"].getMeta())->setMax(m_params["full_tuning_range_HIGH_start"].getVal<double>());
                    break;

                default:
                    retval += ito::RetVal::format(ito::retError, 0, tr("invalid answer '%s'.").toLatin1().data(), answer.data());
                    break;
            }
        }
        else
        {
            retval += ito::RetVal::format(ito::retError, 0, tr("invalid answer '%s' for sending  '%s'").toLatin1().data(), answer.data(), request.data());
        }
    }

    //__________________________________________________________________________________________________________ operation mode check query
    if (!retval.containsError())
    {
        request = QByteArray("S60");
        retval += SendQuestionWithAnswerString(request, answer, 500);
        QStringList regexStringList = regexHelper("^A6(1|2|3|4)$", answer);
        if (!regexStringList.isEmpty() && !retval.containsError())
        {
            m_params["operation_mode"].setVal<int>(regexStringList[1].toInt());
        }
        else
        {
            retval += ito::RetVal::format(ito::retError, 0, tr("invalid answer '%s' for sending  '%s'").toLatin1().data(), answer.data(), request.data());
        }
    }

    //__________________________________________________________________________________________________________ check wavelength
    if (!retval.containsError())
    {
        request = QByteArray("S71");
        retval += SendQuestionWithAnswerString(request, answer, 500);  //operational parameters check query
        QStringList regexStringList = regexHelper("^A71(\\d{4,4})", answer);
        if (!regexStringList.isEmpty() && !retval.containsError())
        {
            m_params["wavelength"].setVal<double>(0.05 * regexStringList[1].toInt() + 700);
        }
        else
        {
            retval += ito::RetVal::format(ito::retError, 0, tr("invalid answer '%s' for sending  '%s'").toLatin1().data(), answer.data(), request.data());
        }
    }

    //__________________________________________________________________________________________________________ check start wavelength of Sweep
    if (!retval.containsError())
    {
        request = QByteArray("S72");
        retval += SendQuestionWithAnswerString(request, answer, 500);
        QStringList regexStringList = regexHelper("^A72(\\d{4,4})", answer);
        if (!regexStringList.isEmpty() && !retval.containsError())
        {
            m_params["modification_start_wavelength"].setVal<double>(
                0.05 * regexStringList[1].toInt() + 700);
        }
        else
        {
            retval += ito::RetVal::format(ito::retError, 0, tr("invalid answer '%s' for sending  '%s'").toLatin1().data(), answer.data(), request.data());
        }
    }

    //__________________________________________________________________________________________________________ check stop wavelength of Sweep
    if (!retval.containsError())
    {
        request = QByteArray("S73");
        retval += SendQuestionWithAnswerString(request, answer, 500);
        QStringList regexStringList = regexHelper("^A73(\\d{4,4})", answer);
        if (!regexStringList.isEmpty() && !retval.containsError())
        {
            m_params["modification_end_wavelength"].setVal<double>(
                0.05 * regexStringList[1].toInt() + 700);
        }
        else
        {
            retval += ito::RetVal::format(ito::retError, 0, tr("invalid answer '%s' for sending  '%s'").toLatin1().data(), answer.data(), request.data());
        }
    }

    //__________________________________________________________________________________________________________ check sweep speed
    if (!retval.containsError())
    {
        request = QByteArray("S74");
        retval += SendQuestionWithAnswerString(request, answer, 500);  //operational parameters check query | sweep speed from 10 - 10000 nm/s (4-byte code)
        QStringList regexStringList = regexHelper("^A74(\\d{4,4})", answer);
        if (!regexStringList.isEmpty() && !retval.containsError())
        {
            if (regexStringList[1].toInt() == 0)
            {
                request = QByteArray("S78");
                retval += SendQuestionWithAnswerString(request, answer, 500);  //operational parameters check query | sweep speed from 2 - 9 nm/s (1-byte code)
                QStringList regexStringList2 = regexHelper("^A78(\\d{1,1})", answer);
                if (!regexStringList2.isEmpty() && !retval.containsError())
                {
                    m_params["sweep_speed"].setVal<int>(regexStringList2[1].toInt());
                }
                else
                {
                    retval += ito::RetVal::format(ito::retError, 0, tr("invalid answer '%s' for sending  '%s'").toLatin1().data(), answer.data(), request.data());
                }
            }
            else
            {
                m_params["sweep_speed"].setVal<int>(10 * regexStringList[1].toInt());
            }
        }
        else
        {
            retval += ito::RetVal::format(ito::retError, 0, tr("invalid answer '%s' for sending  '%s'").toLatin1().data(), answer.data(), request.data());
        }
    }

    //__________________________________________________________________________________________________________ check Modulation first wavelength
    if (!retval.containsError())
    {
        request = QByteArray("S75");
        retval += SendQuestionWithAnswerString(request, answer, 500);  //operational parameters check query | first wavelength of TWO-WAVELENGTH-MODULATION
        QStringList regexStringList = regexHelper("^A75(\\d{4,4})", answer);
        if (!regexStringList.isEmpty() && !retval.containsError())
        {
            m_params["wavelength_first"].setVal<double>(0.05 * regexStringList[1].toInt() + 700);
        }
        else
        {
            retval += ito::RetVal::format(ito::retError, 0, tr("invalid answer '%s' for sending  '%s'").toLatin1().data(), answer.data(), request.data());
        }
    }

    //__________________________________________________________________________________________________________ check Modulation second wavelength
    if (!retval.containsError())
    {
        request = QByteArray("S76");
        retval += SendQuestionWithAnswerString(request, answer, 500);  //operational parameters check query | second wavelength of TWO-WAVELENGTH-MODULATION mode
        QStringList regexStringList = regexHelper("^A76(\\d{4,4})", answer);
        if (!regexStringList.isEmpty() && !retval.containsError())
        {
            m_params["wavelength_second"].setVal<double>(0.05 * regexStringList[1].toInt() + 700);
        }
        else
        {
            retval += ito::RetVal::format(ito::retError, 0, tr("invalid answer '%s' for sending  '%s'").toLatin1().data(), answer.data(), request.data());
        }
    }

    //__________________________________________________________________________________________________________ check Modulation frequency
    if (!retval.containsError())
    {
        request = QByteArray("S77");
        retval += SendQuestionWithAnswerString(request, answer, 500);  //operational parameters check query | modulation frequency in TWO-WAVELENGTH-MODULATION mode
        QStringList regexStringList = regexHelper("^A77(\\d{4,4})", answer);
        if (!regexStringList.isEmpty() && !retval.containsError())
        {
            switch (regexStringList[1].toInt())
            {
                case 1:
                    m_params["modulation_frequency"].setVal<double>(0.1);
                    break;
                case 2:
                    m_params["modulation_frequency"].setVal<double>(0.2);
                    break;
                case 3:
                    m_params["modulation_frequency"].setVal<double>(0.5);
                    break;
                case 4:
                    m_params["modulation_frequency"].setVal<double>(1.0);
                    break;
                case 5:
                    m_params["modulation_frequency"].setVal<double>(2.0);
                    break;
                case 6:
                    m_params["modulation_frequency"].setVal<double>(5.0);
                    break;
                case 7:
                    m_params["modulation_frequency"].setVal<double>(10.0);
                    break;
                case 8:
                    m_params["modulation_frequency"].setVal<double>(20.0);
                    break;
                case 9:
                    m_params["modulation_frequency"].setVal<double>(50.0);
                    break;
                case 10:
                    m_params["modulation_frequency"].setVal<double>(100.0);
                    break;
                case 11:
                    m_params["modulation_frequency"].setVal<double>(200.0);
                    break;
                case 12:
                    m_params["modulation_frequency"].setVal<double>(500.0);
                    break;
                case 13:
                    m_params["modulation_frequency"].setVal<double>(1000.0);
                    break;

                default:
                    retval += ito::RetVal::format(ito::retError, 0, tr("invalid answer '%s'.").toLatin1().data(), answer.data());
                    break;
            }
        }
        else
        {
            retval += ito::RetVal::format(ito::retError, 0, tr("invalid answer '%s' for sending  '%s'").toLatin1().data(), answer.data(), request.data());
        }
    }

    return retval;
}
