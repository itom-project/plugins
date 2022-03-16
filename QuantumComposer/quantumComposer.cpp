/* ********************************************************************
    Plugin "PIPiezoControl" for itom software
    URL: http://www.uni-stuttgart.de/ito
    Copyright (C) 2022, Institut fuer Technische Optik (ITO),
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

#include "quantumComposer.h"
#include "common/helperCommon.h"
#include "gitVersion.h"
#include "pluginVersion.h"

#include <qmessagebox.h>
#include <qplugin.h>
#include <qstring.h>
#include <qstringlist.h>
#include <qwaitcondition.h>

//----------------------------------------------------------------------------------------------------------------------------------
//! Constructor of Interface Class.
/*!
    \todo add necessary information about your plugin here.
*/
QuantumComposerInterface::QuantumComposerInterface()
{
    m_type = ito::typeDataIO | ito::typeRawIO; // any grabber is a dataIO device AND its subtype
                                               // grabber (bitmask -> therefore the OR-combination).
    setObjectName("QuantumComposer");

    m_description = QObject::tr("QuantumComposer");

    // for the docstring, please don't set any spaces at the beginning of the line.
    char docstring[] = "";
    m_detaildescription = QObject::tr(
        "QuantumComposer is an itom-plugin to communicate with the pulse generator 9520 series. \n\
\n\
This plugin has been developed for the 9520 series via a RS232 interface. So you first have to create an instance of the SerialIO plugin \n\
which is a mandatory input argument of the QuantumComposer plugin. \n\
The plugin sets the right RS232 parameter during initialization. \n\
\n\
The default parameters are: \n\
* Baud Rate: 38400 (default for USB), 115200 (default for RS232)\n\
* Data Bits: 8 \n\
* Parity: None \n\
* Stop Bits: 1\n\
* endline: \\r\\n");

    m_author = PLUGIN_AUTHOR;
    m_version = PLUGIN_VERSION;
    m_minItomVer = MINVERSION;
    m_maxItomVer = MAXVERSION;
    m_license = QObject::tr("licensed under LGPL");
    m_aboutThis = QObject::tr(GITVERSION);

    ito::Param paramVal(
        "serialIOInstance",
        ito::ParamBase::HWRef | ito::ParamBase::In,
        nullptr,
        tr("An opened serial port.").toLatin1().data());
    paramVal.setMeta(new ito::HWMeta("SerialIO"), true);
    m_initParamsMand.append(paramVal);

    paramVal = ito::Param(
        "connection",
        ito::ParamBase::String,
        "USB",
        "Type of connection ('USB', 'RS232'). The Baud Rate for the USB connection will be set "
        "to 38400 and for RS232 to 115200.");
    ito::StringMeta* sm = new ito::StringMeta(ito::StringMeta::String, "USB");
    sm->addItem("RS232");
    paramVal.setMeta(sm, true);
    m_initParamsOpt.append(paramVal);
}

//----------------------------------------------------------------------------------------------------------------------------------
QuantumComposerInterface::~QuantumComposerInterface()
{
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal QuantumComposerInterface::getAddInInst(ito::AddInBase** addInInst)
{
    NEW_PLUGININSTANCE(QuantumComposer) // the argument of the macro is the classname of the plugin
    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal QuantumComposerInterface::closeThisInst(ito::AddInBase** addInInst)
{
    REMOVE_PLUGININSTANCE(
        QuantumComposer) // the argument of the macro is the classname of the plugin
    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
#if QT_VERSION < 0x050000
Q_EXPORT_PLUGIN2(
    quantumcomposerinterface,
    QuantumComposerInterface) // the second parameter must correspond to the class-name of the
                              // interface class, the first parameter is arbitrary (usually the same
                              // with small letters only)
#endif

//----------------------------------------------------------------------------------------------------------------------------------
//! Constructor of plugin.
/*!
    \todo add internal parameters of the plugin to the map m_params. It is allowed to append or
   remove entries from m_params in this constructor or later in the init method
*/
QuantumComposer::QuantumComposer() :
    AddInDataIO(), m_pSer(nullptr), m_delayAfterSendCommandMS(100), m_requestTimeOutMS(500)
{
    ito::Param paramVal = ito::Param(
        "name",
        ito::ParamBase::String | ito::ParamBase::Readonly,
        "QuantumComposer",
        tr("Plugin name.").toLatin1().data());
    paramVal.setMeta(new ito::StringMeta(ito::StringMeta::String, "General"), true);
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param(
        "manufacturer",
        ito::ParamBase::String | ito::ParamBase::Readonly,
        "unknown",
        tr("Manufacturer identification.").toLatin1().data());
    paramVal.setMeta(new ito::StringMeta(ito::StringMeta::String, "Device parameter"), true);
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param(
        "model",
        ito::ParamBase::String | ito::ParamBase::Readonly,
        "unknown",
        tr("Model identification.").toLatin1().data());
    paramVal.setMeta(new ito::StringMeta(ito::StringMeta::String, "Device parameter"), true);
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param(
        "serialNumber",
        ito::ParamBase::String | ito::ParamBase::Readonly,
        "unknown",
        tr("Serial number.").toLatin1().data());
    paramVal.setMeta(new ito::StringMeta(ito::StringMeta::String, "Device parameter"), true);
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param(
        "version",
        ito::ParamBase::String | ito::ParamBase::Readonly,
        "unknown",
        tr("Version number.").toLatin1().data());
    paramVal.setMeta(new ito::StringMeta(ito::StringMeta::String, "Device parameter"), true);
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param(
        "requestTimeout",
        ito::ParamBase::Int,
        m_requestTimeOutMS,
        new ito::IntMeta(0, std::numeric_limits<int>::max(), 1, "SerialIO parameter"),
        tr("Request timeout for the SerialIO interface.").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param(
        "mode",
        ito::ParamBase::String,
        "",
        tr("Mode of the system output. (NORM: normal, SING: single shot, BURST: burst, DCYC: duty "
           "cycle).")
            .toLatin1()
            .data());
    ito::StringMeta* sm = new ito::StringMeta(ito::StringMeta::String, "NORM", "Device parameter");
    sm->addItem("NORM");
    sm->addItem("SING");
    sm->addItem("BURST");
    sm->addItem("DCYC");
    paramVal.setMeta(sm, true);
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param(
        "state",
        ito::ParamBase::Int,
        0,
        new ito::IntMeta(0, 1, 1, "Device parameter"),
        tr("Enables (1), disables (0) the output for all channels. Command is the same as pressing "
           "the RUN/STOP button.")
            .toLatin1()
            .data());
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param(
        "burstCounter",
        ito::ParamBase::Int,
        1,
        new ito::IntMeta(1, 9999999, 1, "Device parameter"),
        tr("Number of pulses to generate in the burst mode.").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param(
        "pulseCounter",
        ito::ParamBase::Int,
        1,
        new ito::IntMeta(1, 9999999, 1, "Device parameter"),
        tr("Number of pulses to inhibit output during the off cycle of the Duty Cycle mode.")
            .toLatin1()
            .data());
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param(
        "offCounter",
        ito::ParamBase::Int,
        1,
        new ito::IntMeta(1, 9999999, 1, "Device parameter"),
        tr("Number of pulses to inhibit output during the off cycle of the Duty Cycle mode.")
            .toLatin1()
            .data());
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param(
        "gateMode",
        ito::ParamBase::String,
        "",
        tr("Global gate mode of the system output. (DIS: diabled, PULS: pulse inhibit, OUTP: "
           "output inhibit, CHAN: channel)."
           "cycle).")
            .toLatin1()
            .data());
    sm = new ito::StringMeta(ito::StringMeta::String, "DIS", "Device parameter");
    sm->addItem("DIS");
    sm->addItem("PULS");
    sm->addItem("OUTP");
    sm->addItem("CHAN");
    paramVal.setMeta(sm, true);
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param(
        "gateLogic",
        ito::ParamBase::String,
        "",
        tr("Gate logic level (LOW, HIGH).").toLatin1().data());
    sm = new ito::StringMeta(ito::StringMeta::String, "LOW", "Device parameter");
    sm->addItem("HIGH");
    paramVal.setMeta(sm, true);
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param(
        "gateLevel",
        ito::ParamBase::Double,
        0.20,
        new ito::DoubleMeta(0.20, 15.0, 0.01, "Device parameter"),
        tr("Gate threshold in units of V with a range of 0.20V to 15.0V.").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param(
        "triggerMode",
        ito::ParamBase::String,
        "",
        tr("Trigger mode (DIS: disabled, TRIG: triggered, enabled).").toLatin1().data());
    sm = new ito::StringMeta(ito::StringMeta::String, "DIS", "Device parameter");
    sm->addItem("TRIG");
    paramVal.setMeta(sm, true);
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param(
        "triggerEdge",
        ito::ParamBase::String,
        "",
        tr("Trigger edge to use as the trigger signal (RIS: rising, FALL: falling).")
            .toLatin1()
            .data());
    sm = new ito::StringMeta(ito::StringMeta::String, "RIS", "Device parameter");
    sm->addItem("FALL");
    paramVal.setMeta(sm, true);
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param(
        "triggerLevel",
        ito::ParamBase::Double,
        0.20,
        new ito::DoubleMeta(0.20, 15.0, 0.01, "Device parameter"),
        tr("Trigger threshold in units of V with a range of 0.20V to 15.0V.").toLatin1().data());

    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param(
        "icLock",
        ito::ParamBase::String,
        "",
        tr("Source for the internal rate generator. System clock or external source ranging from "
           "10MHz to 100MHz (SYS, EXT10, EXT20, EXT25, EXT40, EXT50, EXT80, EXT100).")
            .toLatin1()
            .data());
    sm = new ito::StringMeta(ito::StringMeta::String, "SYS", "Device parameter");
    sm->addItem("EXT10");
    sm->addItem("EXT20");
    sm->addItem("EXT25");
    sm->addItem("EXT40");
    sm->addItem("EXT50");
    sm->addItem("EXT80");
    sm->addItem("EXT100");
    paramVal.setMeta(sm, true);
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param(
        "ocLock",
        ito::ParamBase::String,
        "",
        tr("External clock output. T0 pulse or 50% duty cycle TTL output from 10MHz to 100MHz (T0, "
           "10, 11, 12, 14, 16, 20, 25, 33, 50, 100).")
            .toLatin1()
            .data());
    sm = new ito::StringMeta(ito::StringMeta::String, "T0", "Device parameter");
    sm->addItem("10");
    sm->addItem("11");
    sm->addItem("12");
    sm->addItem("14");
    sm->addItem("16");
    sm->addItem("20");
    sm->addItem("25");
    sm->addItem("33");
    sm->addItem("50");
    sm->addItem("100");
    paramVal.setMeta(sm, true);
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param(
        "period",
        ito::ParamBase::Double,
        100e-9,
        new ito::DoubleMeta(6e-8, 5000.0, 1e-8, "Device parameter"),
        tr("T0 period in units of seconds (100ns - 5000s).").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param(
        "counterState",
        ito::ParamBase::Int,
        0,
        new ito::IntMeta(0, 1, 1, "Device parameter"),
        tr("Enables (1), disables(0) the counter function.").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param(
        "counterCounts",
        ito::ParamBase::Int | ito::ParamBase::Readonly,
        0,
        new ito::IntMeta(0, std::numeric_limits<int>::max(), 1, "Device parameter"),
        tr("Number of counts.").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);

    // EXEC functions
    // register exec functions to set channels output states
    QVector<ito::Param> pMand;
    QVector<ito::Param> pOpt;
    QVector<ito::Param> pOut;
    ito::int32 channels[8] = {1, 2, 3, 4, 5, 6, 7, 8};
    paramVal = ito::Param(
        "channelIndexList",
        ito::ParamBase::IntArray | ito::ParamBase::In,
        8,
        channels,
        new ito::IntArrayMeta(1, 8, 1, 1, 8, 1, "Channel parameter"),
        tr("List of channel indices which output should be enabled/disabled (ChA = 1, ChB = 2, "
           "...).")
            .toLatin1()
            .data());
    pMand.append(paramVal);
    ito::int32 states[8] = {1, 2, 3, 4, 5, 6, 7, 8};
    paramVal = ito::Param(
        "statesList",
        ito::ParamBase::IntArray | ito::ParamBase::In,
        8,
        states,
        new ito::IntArrayMeta(0, 1, 1, 1, 8, 1, "Channel parameter"),
        tr("List of states to enalbe/disable channels listed in the parameter channelIndexList. "
           "List must have the same length as the parameter channelIndexList.")
            .toLatin1()
            .data());
    pMand.append(paramVal);

    registerExecFunc(
        "setChannelOutputState",
        pMand,
        pOpt,
        pOut,
        tr("Enables/Disables the output state of the given channels.").toLatin1().data());
    pMand.clear();
    pOpt.clear();
    pOut.clear();

    // register exec functions to set channels widths
    paramVal = ito::Param(
        "channelIndexList",
        ito::ParamBase::IntArray | ito::ParamBase::In,
        8,
        channels,
        new ito::IntArrayMeta(1, 8, 1, 1, 8, 1, "Channel parameter"),
        tr("List of channel indices which width should be set (ChA = 1, ChB = 2, "
           "...).")
            .toLatin1()
            .data());
    pMand.append(paramVal);

    ito::float64 widths[8] = {
        0.00000000200,
        0.00000000200,
        0.00000000200,
        0.00000000200,
        0.00000000200,
        0.00000000200,
        0.00000000200,
        0.00000000200
    };
    paramVal = ito::Param(
        "widthsList",
        ito::ParamBase::DoubleArray | ito::ParamBase::In,
        8,
        widths,
        new ito::DoubleArrayMeta(
            0.00000000200, 999.99999999975, 0.00000000025, 1, 8, 1, "Channel parameter"),
        tr("List of widths to set to the channels listed in the parameter channelIndexList. List "
           "must have the same length as the parameter channelIndexList.")
            .toLatin1()
            .data());
    pMand.append(paramVal);
    registerExecFunc(
        "setChannelWidths",
        pMand,
        pOpt,
        pOut,
        tr("Set the pulse width of the given channels.").toLatin1().data());
    pMand.clear();
    pOpt.clear();
    pOut.clear();

    // register exec functions to set channels delays
    paramVal = ito::Param(
        "channelIndexList",
        ito::ParamBase::IntArray | ito::ParamBase::In,
        8,
        channels,
        new ito::IntArrayMeta(1, 8, 1, 1, 8, 1, "Channel parameter"),
        tr("List of channel indices which width should be set (ChA = 1, ChB = 2, "
           "...).")
            .toLatin1()
            .data());
    pMand.append(paramVal);

    ito::float64 delays[8] = {
        -999.99999999975,
        -999.99999999975,
        -999.99999999975,
        -999.99999999975,
        -999.99999999975,
        -999.99999999975,
        -999.99999999975,
        -999.99999999975
    };
    paramVal = ito::Param(
        "delaysList",
        ito::ParamBase::DoubleArray | ito::ParamBase::In,
        8,
        delays,
        new ito::DoubleArrayMeta(
            -999.99999999975, 999.99999999975, 0.00000000025, 1, 8, 1, "Channel parameter"),
        tr("List of delays to set to the channels listed in the parameter channelIndexList. List "
           "must have the same length as the parameter channelIndexList.")
            .toLatin1()
            .data());
    pMand.append(paramVal);
    registerExecFunc(
        "setChannelDelays",
        pMand,
        pOpt,
        pOut,
        tr("Set the pulse delays of the given channels.").toLatin1().data());
    pMand.clear();
    pOpt.clear();
    pOut.clear();

    // register exec functions to set channels sync
    paramVal = ito::Param(
        "channelIndexList",
        ito::ParamBase::IntArray | ito::ParamBase::In,
        8,
        channels,
        new ito::IntArrayMeta(1, 8, 1, 1, 8, 1, "Channel parameter"),
        tr("List of channel indices which width should be set (ChA = 1, ChB = 2, "
           "...).")
            .toLatin1()
            .data());
    pMand.append(paramVal);

    paramVal = ito::Param(
        "syncsList",
        ito::ParamBase::IntArray | ito::ParamBase::In,
        8,
        channels,
        new ito::IntArrayMeta(1, 8, 1, 1, 8, 1, "Channel parameter"),
        tr("List of channels to sync with the channels listed in the parameter channelIndexList. List "
           "must have the same length as the parameter channelIndexList.")
            .toLatin1()
            .data());
    pMand.append(paramVal);
    registerExecFunc(
        "setChannelSyncs",
        pMand,
        pOpt,
        pOut,
        tr("Set the sync channels of the given channels.").toLatin1().data());
    pMand.clear();
    pOpt.clear();
    pOut.clear();
}

//----------------------------------------------------------------------------------------------------------------------------------
QuantumComposer::~QuantumComposer()
{
}

//----------------------------------------------------------------------------------------------------------------------------------
//! initialization of plugin
/*!
    \sa close
*/
ito::RetVal QuantumComposer::init(
    QVector<ito::ParamBase>* paramsMand,
    QVector<ito::ParamBase>* paramsOpt,
    ItomSharedSemaphore* waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);

    QByteArray answerStr;
    double answerDouble;
    int answerInt;

    if (reinterpret_cast<ito::AddInBase*>((*paramsMand)[0].getVal<void*>())
            ->getBasePlugin()
            ->getType() &
        (ito::typeDataIO | ito::typeRawIO))
    {
        m_pSer = (ito::AddInDataIO*)(*paramsMand)[0].getVal<void*>();
    }
    else
    {
        retValue += ito::RetVal(
            ito::retError,
            1,
            tr("Input parameter is not a dataIO instance of ther SerialIO Plugin!")
                .toLatin1()
                .data());
    }

    if (!retValue.containsError())
    {
        QByteArray connectionType = paramsOpt->at(0).getVal<char*>();
        int baud;
        if (connectionType == "USB")
        {
            baud = 38400;
            retValue += SendCommand(":SYST:COMM:SER:USB 38400");
        }
        else if (connectionType == "RS232")
        {
            baud = 115200;
            retValue += SendCommand(":SYST:COMM:SER:USB 115200");
        }

        retValue += m_pSer->setParam(
            QSharedPointer<ito::ParamBase>(new ito::ParamBase("baud", ito::ParamBase::Int, baud)),
            nullptr);
        retValue += m_pSer->setParam(
            QSharedPointer<ito::ParamBase>(new ito::ParamBase("bits", ito::ParamBase::Int, 8)),
            nullptr);
        retValue += m_pSer->setParam(
            QSharedPointer<ito::ParamBase>(
                new ito::ParamBase("parity", ito::ParamBase::Double, 0.0)),
            nullptr);
        retValue += m_pSer->setParam(
            QSharedPointer<ito::ParamBase>(new ito::ParamBase("stopbits", ito::ParamBase::Int, 1)),
            nullptr);
        retValue += m_pSer->setParam(
            QSharedPointer<ito::ParamBase>(new ito::ParamBase("flow", ito::ParamBase::Int, 0)),
            nullptr);
        retValue += m_pSer->setParam(
            QSharedPointer<ito::ParamBase>(
                new ito::ParamBase("endline", ito::ParamBase::String, "\r\n")),
            nullptr);

        QSharedPointer<QVector<ito::ParamBase>> _dummy;
        m_pSer->execFunc("clearInputBuffer", _dummy, _dummy, _dummy, nullptr);
        m_pSer->execFunc("clearOutputBuffer", _dummy, _dummy, _dummy, nullptr);

        retValue += SendQuestionWithAnswerString("*IDN?", answerStr, 1000);

        if (!retValue.containsError())
        {
            QByteArrayList idn =
                answerStr.split(','); // split identification answer in lines and by comma
            if (idn.length() == 4)
            {
                m_params["manufacturer"].setVal<char*>(idn[0].data());
                m_params["model"].setVal<char*>(idn[1].data());
                m_params["serialNumber"].setVal<char*>(idn[2].data());
                m_params["version"].setVal<char*>(idn[3].data());
            }
            else
            {
                retValue += ito::RetVal(
                    ito::retError,
                    1,
                    tr("Answer of the identification request is not valid!").toLatin1().data());
            }
        }

        if (!retValue.containsError())
        {
            retValue +=
                SendQuestionWithAnswerString(":PULSE0:MODE?", answerStr, m_requestTimeOutMS);
            m_params["mode"].setVal<char*>(answerStr.data());

            retValue +=
                SendQuestionWithAnswerInteger(":PULSE0:STAT?", answerInt, m_requestTimeOutMS);
            m_params["state"].setVal<int>(answerInt);

            retValue +=
                SendQuestionWithAnswerInteger(":PULSE0:BCO?", answerInt, m_requestTimeOutMS);
            m_params["burstCounter"].setVal<int>(answerInt);

            retValue +=
                SendQuestionWithAnswerInteger(":PULSE0:PCO?", answerInt, m_requestTimeOutMS);
            m_params["pulseCounter"].setVal<int>(answerInt);

            retValue +=
                SendQuestionWithAnswerInteger(":PULSE0:OCO?", answerInt, m_requestTimeOutMS);
            m_params["pulseCounter"].setVal<int>(answerInt);

            retValue +=
                SendQuestionWithAnswerString(":PULSE0:GAT:MOD?", answerStr, m_requestTimeOutMS);
            m_params["gateMode"].setVal<char*>(answerStr.data());

            retValue +=
                SendQuestionWithAnswerDouble(":PULSE0:GAT:LEV?", answerDouble, m_requestTimeOutMS);
            m_params["gateLevel"].setVal<double>(answerDouble);

            retValue +=
                SendQuestionWithAnswerString(":PULSE0:GAT:LOG?", answerStr, m_requestTimeOutMS);
            m_params["gateLogic"].setVal<char*>(answerStr.data());

            retValue +=
                SendQuestionWithAnswerString(":PULSE0:TRIG:MOD?", answerStr, m_requestTimeOutMS);
            m_params["triggerMode"].setVal<char*>(answerStr.data());

            retValue +=
                SendQuestionWithAnswerString(":PULSE0:TRIG:EDG?", answerStr, m_requestTimeOutMS);
            m_params["triggerEdge"].setVal<char*>(answerStr.data());

            retValue += SendQuestionWithAnswerString(":PULSE0:ICL?", answerStr, m_requestTimeOutMS);
            m_params["icLock"].setVal<char*>(answerStr.data());

            retValue += SendQuestionWithAnswerString(":PULSE0:OCL?", answerStr, m_requestTimeOutMS);
            m_params["ocLock"].setVal<char*>(answerStr.data());

            retValue +=
                SendQuestionWithAnswerDouble(":PULSE0:TRIG:LEV?", answerDouble, m_requestTimeOutMS);
            m_params["triggerLevel"].setVal<double>(answerDouble);

            retValue +=
                SendQuestionWithAnswerDouble(":PULSE0:PER?", answerDouble, m_requestTimeOutMS);
            m_params["period"].setVal<double>(answerDouble);

            retValue +=
                SendQuestionWithAnswerInteger(":PULSE0:COUN:STAT?", answerInt, m_requestTimeOutMS);
            m_params["counterState"].setVal<int>(answerInt);

            retValue +=
                SendQuestionWithAnswerInteger(":PULSE0:COUN:COUN?", answerInt, m_requestTimeOutMS);
            m_params["counterCounts"].setVal<int>(answerInt);
        }


        emit parametersChanged(m_params);
    }

    if (waitCond)
    {
        waitCond->returnValue = retValue;
        waitCond->release();
    }

    setInitialized(true); // init method has been finished (independent on retval)
    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
//! shutdown of plugin
/*!
    \sa init
*/
ito::RetVal QuantumComposer::close(ItomSharedSemaphore* waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);

    // todo:
    //  - disconnect the device if not yet done
    //  - this funtion is considered to be the "inverse" of init.

    if (waitCond)
    {
        waitCond->returnValue = retValue;
        waitCond->release();
    }

    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal QuantumComposer::getParam(QSharedPointer<ito::Param> val, ItomSharedSemaphore* waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue;
    QString key;
    bool hasIndex = false;
    int index;
    QString suffix;
    QMap<QString, ito::Param>::iterator it;

    // parse the given parameter-name (if you support indexed or suffix-based parameters)
    retValue += apiParseParamName(val->getName(), key, hasIndex, index, suffix);

    if (retValue == ito::retOk)
    {
        // gets the parameter key from m_params map (read-only is allowed, since we only want to get
        // the value).
        retValue += apiGetParamFromMapByKey(m_params, key, it, false);
    }

    if (!retValue.containsError())
    {
        QSharedPointer<QVector<ito::ParamBase>> _dummy;
        m_pSer->execFunc("clearInputBuffer", _dummy, _dummy, _dummy, nullptr);
        m_pSer->execFunc("clearOutputBuffer", _dummy, _dummy, _dummy, nullptr);

        int answerInt;
        if (key == "counterCounts")
        {
            retValue +=
                SendQuestionWithAnswerInteger(":PULSE0:COUN:COUN?", answerInt, m_requestTimeOutMS);
            val->setVal<int>(answerInt);
        }
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
ito::RetVal QuantumComposer::setParam(
    QSharedPointer<ito::ParamBase> val, ItomSharedSemaphore* waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);
    QString key;
    bool hasIndex;
    int index;
    QString suffix;
    QMap<QString, ito::Param>::iterator it;

    // parse the given parameter-name (if you support indexed or suffix-based parameters)
    retValue += apiParseParamName(val->getName(), key, hasIndex, index, suffix);

    if (!retValue.containsError())
    {
        // gets the parameter key from m_params map (read-only is not allowed and leads to
        // ito::retError).
        retValue += apiGetParamFromMapByKey(m_params, key, it, true);
    }

    if (!retValue.containsError())
    {
        // here the new parameter is checked whether its type corresponds or can be cast into the
        //  value in m_params and whether the new type fits to the requirements of any possible
        //  meta structure.
        retValue += apiValidateParam(*it, *val, false, true);
    }

    if (!retValue.containsError())
    {
        if (key == "mode")
        {
            retValue += SendCommand(
                QString(":PULSE0:MODE %1").arg(val->getVal<char*>()).toStdString().c_str());
            retValue += it->copyValueFrom(&(*val));
        }
        else if (key == "state")
        {
            retValue += SendCommand(
                QString(":PULSE0:STAT %1").arg(val->getVal<int>()).toStdString().c_str());
            retValue += it->copyValueFrom(&(*val));
        }
        else if (key == "burstCounter")
        {
            retValue += SendCommand(
                QString(":PULSE0:BCO %1").arg(val->getVal<int>()).toStdString().c_str());
            retValue += it->copyValueFrom(&(*val));
        }
        else if (key == "pulseCounter")
        {
            retValue += SendCommand(
                QString(":PULSE0:PCO %1").arg(val->getVal<int>()).toStdString().c_str());
            retValue += it->copyValueFrom(&(*val));
        }
        else if (key == "offCounter")
        {
            retValue += SendCommand(
                QString(":PULSE0:OCO %1").arg(val->getVal<int>()).toStdString().c_str());
            retValue += it->copyValueFrom(&(*val));
        }
        else if (key == "gateMode")
        {
            retValue += SendCommand(
                QString(":PULSE0:GAT:MOD %1").arg(val->getVal<char*>()).toStdString().c_str());
            retValue += it->copyValueFrom(&(*val));
        }
        else if (key == "gateLogic")
        {
            retValue += SendCommand(
                QString(":PULSE0:GAT:LOG %1").arg(val->getVal<char*>()).toStdString().c_str());
            retValue += it->copyValueFrom(&(*val));
        }
        else if (key == "gateLevel")
        {
            retValue += SendCommand(QString(":PULSE0:GAT:LEV %1")
                                        .arg(QString::number(val->getVal<double>()))
                                        .toLatin1());
            retValue += it->copyValueFrom(&(*val));
        }
        else if (key == "triggerMode")
        {
            retValue += SendCommand(
                QString(":PULSE0:TRIG:MOD %1").arg(val->getVal<char*>()).toStdString().c_str());
            retValue += it->copyValueFrom(&(*val));
        }
        else if (key == "triggerEdge")
        {
            retValue += SendCommand(
                QString(":PULSE0:TRIG:EDG %1").arg(val->getVal<char*>()).toStdString().c_str());
            retValue += it->copyValueFrom(&(*val));
        }
        else if (key == "triggerLevel")
        {
            retValue += SendCommand(QString(":PULSE0:TRIG:LEV %1")
                                        .arg(QString::number(val->getVal<double>()))
                                        .toLatin1());
            retValue += it->copyValueFrom(&(*val));
        }
        else if (key == "requestTimeOut")
        {
            m_requestTimeOutMS = val->getVal<int>();
            m_params["requestTimeOut"].setVal<int>(m_requestTimeOutMS);
            retValue += it->copyValueFrom(&(*val));
        }
        else if (key == "icLock")
        {
            retValue += SendCommand(
                QString(":PULSE0:ICL %1").arg(val->getVal<char*>()).toStdString().c_str());
            retValue += it->copyValueFrom(&(*val));
        }
        else if (key == "ocLock")
        {
            retValue += SendCommand(
                QString(":PULSE0:OCL %1").arg(val->getVal<char*>()).toStdString().c_str());
            retValue += it->copyValueFrom(&(*val));
        }
        else if (key == "period")
        {
            retValue += SendCommand(
                QString(":PULSE0:PER %1")
                    .arg(QString::number(val->getVal<double>(), 'f', 8).replace(".", ","))
                    .toLatin1());
            retValue += it->copyValueFrom(&(*val));
        }
        else if (key == "counterState")
        {
            retValue += SendCommand(
                QString(":PULSE0:COUNT:STAT %1").arg(val->getVal<int>()).toStdString().c_str());
            retValue += it->copyValueFrom(&(*val));
        }
        else
        {
            // all parameters that don't need further checks can simply be assigned
            // to the value in m_params (the rest is already checked above)
            retValue += it->copyValueFrom(&(*val));
        }
    }

    if (!retValue.containsError())
    {
        emit parametersChanged(
            m_params); // send changed parameters to any connected dialogs or dock-widgets
    }

    if (waitCond)
    {
        waitCond->returnValue = retValue;
        waitCond->release();
    }

    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal QuantumComposer::SendCommand(const QByteArray& command)
{
    ito::RetVal retVal;
    retVal += m_pSer->setVal(command.data(), command.length(), nullptr);
    if (m_delayAfterSendCommandMS > 0)
    {
        QMutex mutex;
        mutex.lock();
        QWaitCondition waitCondition;
        waitCondition.wait(&mutex, m_delayAfterSendCommandMS);
        mutex.unlock();
    }

    return retVal;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal QuantumComposer::ReadString(QByteArray& result, int& len, int timeoutMS)
{
    ito::RetVal retValue = ito::retOk;
    QElapsedTimer timer;

    bool done = false;
    int curFrom = 0;
    int pos = 0;

    int buflen = 100;
    QSharedPointer<int> curBufLen(new int);
    QSharedPointer<char> curBuf(new char[buflen]);
    result = "";

    QByteArray endline;

    QSharedPointer<ito::Param> param(new ito::Param("endline"));
    retValue += m_pSer->getParam(param, nullptr);

    if (param->getType() == (ito::ParamBase::String & ito::paramTypeMask))
    {
        char* temp = param->getVal<char*>(); // borrowed reference
        int len = temp[0] == 0 ? 0 : (temp[1] == 0 ? 1 : (temp[2] == 0 ? 2 : 3));
        endline = QByteArray::fromRawData(temp, len);
    }
    else
    {
        retValue += ito::RetVal(
            ito::retError,
            0,
            tr("could not read endline parameter from serial port").toLatin1().data());
    }

    if (!retValue.containsError())
    {
        len = 0;
        timer.start();
        _sleep(m_delayAfterSendCommandMS); // The amount of time required to receive, process, and
                                           // repond to a command is
        // approximately 10ms.

        while (!done && !retValue.containsError())
        {
            *curBufLen = buflen;
            retValue += m_pSer->getVal(curBuf, curBufLen, nullptr);

            if (!retValue.containsError())
            {
                result += QByteArray(curBuf.data(), *curBufLen);
                pos = result.indexOf(endline, curFrom);
                curFrom = qMax(0, result.length() - 3);

                if (pos >= 0) // found
                {
                    done = true;
                    result = result.mid(pos + endline.length(), curFrom);
                }

                if (!done && timer.elapsed() > timeoutMS && timeoutMS >= 0)
                {
                    retValue += ito::RetVal(
                        ito::retError,
                        timeoutMS,
                        tr("timeout during read string from SerialIO").toLatin1().data());
                }
            }
        }
        result = result.trimmed();
        len = result.length();
    }

    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal QuantumComposer::SendQuestionWithAnswerString(
    const QByteArray& questionCommand, QByteArray& answer, const int timeoutMS)
{
    int readSigns;
    ito::RetVal retValue = SendCommand(questionCommand);
    retValue += ReadString(answer, readSigns, timeoutMS);
    return retValue;
}


//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal QuantumComposer::SendQuestionWithAnswerDouble(
    const QByteArray& questionCommand, double& answer, const int timeoutMS)
{
    int readSigns;
    QByteArray _answer;
    bool ok;
    ito::RetVal retValue = SendCommand(questionCommand);
    retValue += ReadString(_answer, readSigns, timeoutMS);
    _answer = _answer.replace(",", ".");
    answer = _answer.toDouble(&ok);

    if (!ok)
    {
        retValue += ito::RetVal(
            ito::retError,
            0,
            tr("Error during SendQuestionWithAnswerDouble, converting %1 to double value.")
                .arg(_answer.constData())
                .toLatin1()
                .data());
    }
    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal QuantumComposer::SendQuestionWithAnswerInteger(
    const QByteArray& questionCommand, int& answer, const int timeoutMS)
{
    int readSigns;
    QByteArray _answer;
    bool ok;
    ito::RetVal retValue = SendCommand(questionCommand);
    retValue += ReadString(_answer, readSigns, timeoutMS);

    if (_answer.contains("?"))
    {
        _answer.replace("?", "");
    }
    answer = _answer.toInt(&ok);

    if (!ok)
    {
        retValue += ito::RetVal(
            ito::retError,
            0,
            tr("Error during SendQuestionWithAnswerInteger, converting %1 to double value.")
                .arg(_answer.constData())
                .toLatin1()
                .data());
    }
    return retValue;
}

//--------------------------------------------------------------------------------------------------------------------------
ito::RetVal QuantumComposer::execFunc(
    const QString funcName,
    QSharedPointer<QVector<ito::ParamBase>> paramsMand,
    QSharedPointer<QVector<ito::ParamBase>> paramsOpt,
    QSharedPointer<QVector<ito::ParamBase>> paramsOut,
    ItomSharedSemaphore* waitCond)
{
    ito::RetVal retValue = ito::retOk;


    if (funcName == "setChannelOutputState")
    {
        ito::ParamBase* channelList = nullptr;
        ito::ParamBase* stateList = nullptr;

        channelList = ito::getParamByName(&(*paramsMand), "channelIndexList", &retValue);
        stateList = ito::getParamByName(&(*paramsMand), "statesList", &retValue);

        if (!retValue.containsError())
        {
            retValue += QuantumComposer::setChannelOutputState(*channelList, *stateList);
        }
    }
    else if (funcName == "setChannelWidths")
    {
        ito::ParamBase* channelList = nullptr;
        ito::ParamBase* widthsList = nullptr;

        channelList = ito::getParamByName(&(*paramsMand), "channelIndexList", &retValue);
        widthsList = ito::getParamByName(&(*paramsMand), "widthsList", &retValue);

        if (!retValue.containsError())
        {
            retValue += QuantumComposer::setChannelWidths(*channelList, *widthsList);
        }
    }
       else if (funcName == "setChannelDelays")
    {
        ito::ParamBase* channelList = nullptr;
        ito::ParamBase* delaysList = nullptr;

        channelList = ito::getParamByName(&(*paramsMand), "channelIndexList", &retValue);
        delaysList = ito::getParamByName(&(*paramsMand), "delaysList", &retValue);

        if (!retValue.containsError())
        {
            retValue += QuantumComposer::setChannelDelays(*channelList, *delaysList);
        }
    }
    else if (funcName == "setChannelSyncs")
    {
        ito::ParamBase* channelList = nullptr;
        ito::ParamBase* syncList = nullptr;

        channelList = ito::getParamByName(&(*paramsMand), "channelIndexList", &retValue);
        syncList = ito::getParamByName(&(*paramsMand), "syncsList", &retValue);

        if (!retValue.containsError())
        {
            retValue += QuantumComposer::setChannelSyncs(*channelList, *syncList);
        }
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

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal QuantumComposer::setChannelOutputState(
    ito::ParamBase& channelIndices, ito::ParamBase& statesList)
{
    ito::RetVal retValue(ito::retOk);

    const int* channels = channelIndices.getVal<int*>();
    const int* states = statesList.getVal<int*>();

    if (channelIndices.getLen() != statesList.getLen())
    {
        retValue += ito::RetVal(
            ito::retError,
            0,
            tr("The lengths of the channel list (%1) and states list (%2) must be the same.")
                .arg(channelIndices.getLen())
                .arg(statesList.getLen())
                .toLatin1()
                .data());
    }
    else
    {
        for (int ch = 0; ch < channelIndices.getLen(); ch++)
        {
            retValue += SendCommand(QString(":PULSE%1:STAT %2")
                                        .arg(channels[ch])
                                        .arg(states[ch])
                                        .toStdString()
                                        .c_str());
        }
    }

    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal QuantumComposer::setChannelWidths(
    ito::ParamBase& channelIndices, ito::ParamBase& widthsList)
{
    ito::RetVal retValue(ito::retOk);

    const int* channels = channelIndices.getVal<int*>();
    const double* widths = widthsList.getVal<double*>();

    if (channelIndices.getLen() != widthsList.getLen())
    {
        retValue += ito::RetVal(
            ito::retError,
            0,
            tr("The lengths of the channel list (%1) and widths list (%2) must be the same.")
                .arg(channelIndices.getLen())
                .arg(widthsList.getLen())
                .toLatin1()
                .data());
    }
    else
    {
        for (int ch = 0; ch < channelIndices.getLen(); ch++)
        {
            retValue += SendCommand(QString(":PULSE%1:WIDT %2")
                                        .arg(channels[ch])
                                        .arg(QString::number(widths[ch], 'f', 11))
                                        .toStdString()
                                        .c_str());
        }
    }

    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal QuantumComposer::setChannelDelays(
    ito::ParamBase& channelIndices, ito::ParamBase& delaysList)
{
    ito::RetVal retValue(ito::retOk);

    const int* channels = channelIndices.getVal<int*>();
    const double* delays = delaysList.getVal<double*>();

    if (channelIndices.getLen() != delaysList.getLen())
    {
        retValue += ito::RetVal(
            ito::retError,
            0,
            tr("The lengths of the channel list (%1) and widths list (%2) must be the same.")
                .arg(channelIndices.getLen())
                .arg(delaysList.getLen())
                .toLatin1()
                .data());
    }
    else
    {
        for (int ch = 0; ch < channelIndices.getLen(); ch++)
        {
            retValue += SendCommand(QString(":PULSE%1:DEL %2")
                                        .arg(channels[ch])
                                        .arg(QString::number(delays[ch], 'f', 11))
                                        .toStdString()
                                        .c_str());
        }
    }

    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal QuantumComposer::setChannelSyncs(
    ito::ParamBase& channelIndices, ito::ParamBase& syncsList)
{
    ito::RetVal retValue(ito::retOk);

    const int* channels = channelIndices.getVal<int*>();
    const int* syncs = syncsList.getVal<int*>();

    if (channelIndices.getLen() != syncsList.getLen())
    {
        retValue += ito::RetVal(
            ito::retError,
            0,
            tr("The lengths of the channel list (%1) and widths list (%2) must be the same.")
                .arg(channelIndices.getLen())
                .arg(syncsList.getLen())
                .toLatin1()
                .data());
    }
    else
    {
        for (int ch = 0; ch < channelIndices.getLen(); ch++)
        {
            QString channelName;
            switch (syncs[ch])
            {
            case 0:
                channelName = "To";
                break;
            case 1:
                channelName = "CHA";
                break;
            case 2:
                channelName = "CHB";
                break;
            case 3:
                channelName = "CHC";
                break;
            case 4:
                channelName = "CHD";
                break;
            case 5:
                channelName = "CHE";
                break;
            case 6:
                channelName = "CHF";
                break;
            case 7:
                channelName = "CHG";
                break;
            case 8:
                channelName = "CHH";
                break;
            default:
                channelName = "To";
                break;
            }
            retValue += SendCommand(QString(":PULSE%1:SYNC %2")
                                        .arg(channels[ch])
                                        .arg(channelName)
                                        .toStdString()
                                        .c_str());
        }
    }

    return retValue;
}