/*# ********************************************************************
    Plugin "QuantumComposer" for itom software
    URL: http://www.uni-stuttgart.de/ito
    Copyright (C) 2022, Institut für Technische Optik (ITO),
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

#include "quantumComposer.h"
#include "common/helperCommon.h"
#include "gitVersion.h"
#include "pluginVersion.h"

#include "dialogQuantumComposer.h"
#include "dockWidgetQuantumComposer.h"

#include <qelapsedtimer.h>
#include <qmessagebox.h>
#include <qplugin.h>
#include <qstring.h>
#include <qstringlist.h>
#include <qwaitcondition.h>

//----------------------------------------------------------------------------------------------------------------------------------
QuantumComposerInterface::QuantumComposerInterface()
{
    m_type = ito::typeDataIO | ito::typeRawIO;

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
\n\
========== ======================================================\n\
Baud Rate  38400 (default for USB), 115200 (default for RS232)\n\
Data Bits  8\n\
Parity     None\n\
Stop bits  1\n\
endline    \\r\\n\n\
========== ======================================================\n\
\n\
.. warning::\n\
\n\
    Disable **Echo** of the system settings!");

    m_author = PLUGIN_AUTHOR;
    m_version = PLUGIN_VERSION;
    m_minItomVer = PLUGIN_MIN_ITOM_VERSION;
    m_maxItomVer = PLUGIN_MAX_ITOM_VERSION;
    m_license = QObject::tr(PLUGIN_LICENCE);
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
    NEW_PLUGININSTANCE(QuantumComposer)
    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal QuantumComposerInterface::closeThisInst(ito::AddInBase** addInInst)
{
    REMOVE_PLUGININSTANCE(QuantumComposer)
    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
#if QT_VERSION < 0x050000
Q_EXPORT_PLUGIN2(quantumcomposerinterface, QuantumComposerInterface)
#endif

//----------------------------------------------------------------------------------------------------------------------------------
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
        tr("Request timeout in ms for the SerialIO interface.").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param(
        "mode",
        ito::ParamBase::String,
        "",
        tr("Mode of the system output. (NORM: normal, SING: single shot, BURS: burst, DCYC: duty "
           "cycle).")
            .toLatin1()
            .data());
    ito::StringMeta* sm = new ito::StringMeta(ito::StringMeta::String, "NORM", "Device parameter");
    sm->addItem("SING");
    sm->addItem("BURS");
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
        tr("Global gate mode of the system output. (DIS: disabled, PULS: pulse inhibit, OUTP: "
           "output inhibit, CHAN: channel)."
           "cycle).")
            .toLatin1()
            .data());
    sm = new ito::StringMeta(ito::StringMeta::String, "DIS", "Device parameter");
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
    ito::Param channelVal = ito::Param(
        "channelIndexList",
        ito::ParamBase::IntArray | ito::ParamBase::In,
        8,
        channels,
        new ito::IntArrayMeta(1, 8, 1, 1, 8, 1, "Channel parameter"),
        tr("List of channel indices which output should be enabled/disabled (ChA = 1, ChB = 2, "
           "...).")
            .toLatin1()
            .data());
    pMand.append(channelVal);
    ito::int32 states[8] = {
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
    };
    paramVal = ito::Param(
        "statesList",
        ito::ParamBase::IntArray | ito::ParamBase::In,
        8,
        states,
        new ito::IntArrayMeta(0, 1, 1, 1, 8, 1, "Channel parameter"),
        tr("List of states to enable/disable channels listed in the parameter channelIndexList. "
           "List must have the same length as the parameter channelIndexList.")
            .toLatin1()
            .data());
    pMand.append(paramVal);
    m_params.insert(paramVal.getName(), paramVal);

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
    pMand.append(channelVal);

    ito::float64 widths[8] = {
        0.00000000200,
        0.00000000200,
        0.00000000200,
        0.00000000200,
        0.00000000200,
        0.00000000200,
        0.00000000200,
        0.00000000200};
    paramVal = ito::Param(
        "widthsList",
        ito::ParamBase::DoubleArray | ito::ParamBase::In,
        8,
        widths,
        new ito::DoubleArrayMeta(0.00000000200, 999.99999999975, 0.0, 1, 8, 1, "Channel parameter"),
        tr("List of widths to set to the channels listed in the parameter channelIndexList. List "
           "must have the same length as the parameter channelIndexList.")
            .toLatin1()
            .data());
    pMand.append(paramVal);
    m_params.insert(paramVal.getName(), paramVal);

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
    pMand.append(channelVal);

    ito::float64 delays[8] = {
        -999.99999999975,
        -999.99999999975,
        -999.99999999975,
        -999.99999999975,
        -999.99999999975,
        -999.99999999975,
        -999.99999999975,
        -999.99999999975};
    paramVal = ito::Param(
        "delaysList",
        ito::ParamBase::DoubleArray | ito::ParamBase::In,
        8,
        delays,
        new ito::DoubleArrayMeta(
            -999.99999999975, 999.99999999975, 0.0, 1, 8, 1, "Channel parameter"),
        tr("List of delays to set to the channels listed in the parameter channelIndexList. List "
           "must have the same length as the parameter channelIndexList.")
            .toLatin1()
            .data());
    pMand.append(paramVal);
    m_params.insert(paramVal.getName(), paramVal);

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
    pMand.append(channelVal);

    paramVal = ito::Param(
        "syncsList",
        ito::ParamBase::IntArray | ito::ParamBase::In,
        8,
        channels,
        new ito::IntArrayMeta(1, 8, 1, 1, 8, 1, "Channel parameter"),
        tr("List of channels to sync with the channels listed in the parameter channelIndexList. "
           "List "
           "must have the same length as the parameter channelIndexList.")
            .toLatin1()
            .data());
    pMand.append(paramVal);
    m_params.insert(paramVal.getName(), paramVal);

    registerExecFunc(
        "setChannelSyncs",
        pMand,
        pOpt,
        pOut,
        tr("Set the sync channels of the given channels.").toLatin1().data());
    pMand.clear();
    pOpt.clear();
    pOut.clear();

    // register exec functions to set channels sync
    pMand.append(channelVal);

    paramVal = ito::Param(
        "muxsList",
        ito::ParamBase::IntArray | ito::ParamBase::In,
        8,
        channels,
        new ito::IntArrayMeta(0, 255, 1, 1, 8, 1, "Channel parameter"),
        tr("List of timers which are enabled as output for the given channel. "
           "List "
           "must have the same length as the parameter channelIndexList.")
            .toLatin1()
            .data());
    pMand.append(paramVal);
    m_params.insert(paramVal.getName(), paramVal);

    registerExecFunc(
        "setChannelMuxs",
        pMand,
        pOpt,
        pOut,
        tr("Set which timers are enabled as output for the given channels.").toLatin1().data());
    pMand.clear();
    pOpt.clear();
    pOut.clear();

    // register exec functions to set channels sync
    pMand.append(channelVal);

    paramVal = ito::Param(
        "polaritiesList",
        ito::ParamBase::StringList,
        nullptr,
        tr("List of polarities which are set to the output for the given channels (NORM = normal, "
           "COMP = complement, INV = inverted).")
            .toLatin1()
            .data());

    ito::ByteArray strList[] = {
        ito::ByteArray("NORM"), ito::ByteArray("COMP"), ito::ByteArray("INV")};
    paramVal.setVal<ito::ByteArray*>(strList, 3);


    sm = new ito::StringListMeta(ito::StringListMeta::String, 1, 8, 1, "Channel parameter");
    sm->addItem("NORM");
    sm->addItem("COMP");
    sm->addItem("INV");
    paramVal.setMeta(sm, true);
    pMand.append(paramVal);
    m_params.insert(paramVal.getName(), paramVal);

    registerExecFunc(
        "setChannelPolarities",
        pMand,
        pOpt,
        pOut,
        tr("Set the polarity of the pulse for the given channels.").toLatin1().data());
    pMand.clear();
    pOpt.clear();
    pOut.clear();

    // register exec functions to set channels sync
    pMand.append(channelVal);

    paramVal = ito::Param(
        "outputModesList",
        ito::ParamBase::StringList,
        nullptr,
        tr("List of output modes which are set to the output for the given channels (TTL = "
           "TTL/CMOS, "
           "ADJ = adjustable).")
            .toLatin1()
            .data());

    ito::ByteArray modList[] = {ito::ByteArray("TTL"), ito::ByteArray("ADJ")};
    paramVal.setVal<ito::ByteArray*>(modList, 2);

    sm = new ito::StringListMeta(ito::StringListMeta::String, 1, 8, 1, "Channel parameter");
    sm->addItem("TTL");
    sm->addItem("ADJ");
    paramVal.setMeta(sm, true);
    pMand.append(paramVal);
    m_params.insert(paramVal.getName(), paramVal);
    registerExecFunc(
        "setChannelOutputModes",
        pMand,
        pOpt,
        pOut,
        tr("Set the output amplitude mode of the given channels.").toLatin1().data());
    pMand.clear();
    pOpt.clear();
    pOut.clear();

    // register exec functions to set channels sync
    pMand.append(channelVal);
    paramVal = ito::Param(
        "amplitudesList",
        ito::ParamBase::DoubleArray | ito::ParamBase::In,
        8,
        delays,
        new ito::DoubleArrayMeta(2.0, 20.0, 0.01, 1, 8, 1, "Channel parameter"),
        tr("List of amplitude levels to set to the channels listed in the parameter "
           "channelIndexList. List "
           "must have the same length as the parameter channelIndexList.")
            .toLatin1()
            .data());
    pMand.append(paramVal);
    m_params.insert(paramVal.getName(), paramVal);
    registerExecFunc(
        "setChannelAdjustableAmplitude",
        pMand,
        pOpt,
        pOut,
        tr("Set the adjustable amplitude of channel output level of the given channels.")
            .toLatin1()
            .data());
    pMand.clear();
    pOpt.clear();
    pOut.clear();

    // register exec functions to set channels sync
    pMand.append(channelVal);

    paramVal = ito::Param(
        "channelModesList",
        ito::ParamBase::StringList,
        nullptr,
        tr("List of channel modes which are set to the output for the given channels (NORM = "
           "normal, SING = single shot, BURS = burst, DCYC = duty cycle).")
            .toLatin1()
            .data());

    ito::ByteArray chModeList[] = {
        ito::ByteArray("NORM"),
        ito::ByteArray("SING"),
        ito::ByteArray("BURS"),
        ito::ByteArray("DCYC")};
    paramVal.setVal<ito::ByteArray*>(chModeList, 4);

    sm = new ito::StringListMeta(ito::StringListMeta::String, 1, 8, 1, "Channel parameter");
    sm->addItem("NORM");
    sm->addItem("SING");
    sm->addItem("BURS");
    sm->addItem("DCYC");
    paramVal.setMeta(sm, true);
    pMand.append(paramVal);
    m_params.insert(paramVal.getName(), paramVal);

    registerExecFunc(
        "setChannelModes",
        pMand,
        pOpt,
        pOut,
        tr("Set the channel mode of the given channels.").toLatin1().data());
    pMand.clear();
    pOpt.clear();
    pOut.clear();

    // register exec functions to set channels sync
    pMand.append(channelVal);

    paramVal = ito::Param(
        "channelBurstCounterList",
        ito::ParamBase::IntArray | ito::ParamBase::In,
        8,
        channels,
        new ito::IntArrayMeta(1, 9999999, 1, 1, 8, 1, "Channel parameter"),
        tr("List of burst counter values for the given channels (1 - 9999999). "
           "List "
           "must have the same length as the parameter channelIndexList.")
            .toLatin1()
            .data());
    pMand.append(paramVal);
    m_params.insert(paramVal.getName(), paramVal);
    registerExecFunc(
        "setChannelBurstCounter",
        pMand,
        pOpt,
        pOut,
        tr("Set the channel burst counter for the burst mode of the given channels.")
            .toLatin1()
            .data());
    pMand.clear();
    pOpt.clear();
    pOut.clear();

    // register exec functions to set channels sync
    pMand.append(channelVal);

    paramVal = ito::Param(
        "channelPulseCounterList",
        ito::ParamBase::IntArray | ito::ParamBase::In,
        8,
        channels,
        new ito::IntArrayMeta(1, 9999999, 1, 1, 8, 1, "Channel parameter"),
        tr("List of pulse counter values to generate during the ON cycle of the duty cycle mode "
           "for the given channels (1 - 9999999). "
           "List "
           "must have the same length as the parameter channelIndexList.")
            .toLatin1()
            .data());
    pMand.append(paramVal);
    m_params.insert(paramVal.getName(), paramVal);

    registerExecFunc(
        "setChannelPulseCounter",
        pMand,
        pOpt,
        pOut,
        tr("Set the channel pulse counter during the ON cycles for the duty cycle modes of the "
           "given channels.")
            .toLatin1()
            .data());
    pMand.clear();
    pOpt.clear();
    pOut.clear();

    // register exec functions to set channels sync
    pMand.append(channelVal);

    paramVal = ito::Param(
        "channelOffCounterList",
        ito::ParamBase::IntArray | ito::ParamBase::In,
        8,
        channels,
        new ito::IntArrayMeta(1, 9999999, 1, 1, 8, 1, "Channel parameter"),
        tr("List of pulse counter values to inhibit during the OFF cycle of the duty cycle mode "
           "for the given channels (1 - 9999999). "
           "List "
           "must have the same length as the parameter channelIndexList.")
            .toLatin1()
            .data());
    pMand.append(paramVal);
    m_params.insert(paramVal.getName(), paramVal);

    registerExecFunc(
        "setChannelOffCounter",
        pMand,
        pOpt,
        pOut,
        tr("Set the channel pulse counter during the OFF cycles for the duty cycle modes of the "
           "given channels.")
            .toLatin1()
            .data());
    pMand.clear();
    pOpt.clear();
    pOut.clear();

    // register exec functions to set channels sync
    pMand.append(channelVal);

    paramVal = ito::Param(
        "channelPulseWaitCounterList",
        ito::ParamBase::IntArray | ito::ParamBase::In,
        8,
        channels,
        new ito::IntArrayMeta(0, 9999999, 1, 1, 8, 1, "Channel parameter"),
        tr("List of pulse counter values to wait until enabling output of the duty cycle mode "
           "for the given channels (0 - 9999999). "
           "List "
           "must have the same length as the parameter channelIndexList.")
            .toLatin1()
            .data());
    pMand.append(paramVal);
    m_params.insert(paramVal.getName(), paramVal);

    registerExecFunc(
        "setChannelWaitCounter",
        pMand,
        pOpt,
        pOut,
        tr("Set the channel pulse counter to wait until enabling output for the duty cycle modes "
           "of the "
           "given channels.")
            .toLatin1()
            .data());
    pMand.clear();
    pOpt.clear();
    pOut.clear();

    // register exec functions to set channels sync
    pMand.append(channelVal);

    paramVal = ito::Param(
        "channelGateModeList",
        ito::ParamBase::StringList,
        nullptr,
        tr("List of channel gate modes (DIS = disable, PULS = pulse inhibit, OUTP = output "
           "inhibit).")
            .toLatin1()
            .data());

    ito::ByteArray gatesList[] = {
        ito::ByteArray("DIS"), ito::ByteArray("PULS"), ito::ByteArray("OUTP")};
    paramVal.setVal<ito::ByteArray*>(gatesList, 3);

    sm = new ito::StringListMeta(ito::StringListMeta::String, 1, 8, 1, "Channel parameter");
    sm->addItem("DIS");
    sm->addItem("PULS");
    sm->addItem("OUTP");
    paramVal.setMeta(sm, true);
    pMand.append(paramVal);
    m_params.insert(paramVal.getName(), paramVal);

    registerExecFunc(
        "setChannelGatesModes",
        pMand,
        pOpt,
        pOut,
        tr("Set the channel gates mode of the given channels.").toLatin1().data());
    pMand.clear();
    pOpt.clear();
    pOut.clear();

    // register exec functions to set channels sync
    pMand.append(channelVal);

    paramVal = ito::Param(
        "channelGateLogicList",
        ito::ParamBase::StringList,
        nullptr,
        tr("List of channel gate logic level (LOW, HIGH).").toLatin1().data());

    ito::ByteArray levelList[] = {ito::ByteArray("LOW"), ito::ByteArray("HIGH")};
    paramVal.setVal<ito::ByteArray*>(levelList, 2);

    sm = new ito::StringListMeta(ito::StringListMeta::String, 1, 8, 1, "Channel parameter");
    sm->addItem("LOW");
    sm->addItem("HIGH");
    paramVal.setMeta(sm, true);
    pMand.append(paramVal);
    m_params.insert(paramVal.getName(), paramVal);

    registerExecFunc(
        "setChannelGatesLogicLevel",
        pMand,
        pOpt,
        pOut,
        tr("Set the channel gates logic level of the given channels.").toLatin1().data());
    pMand.clear();
    pOpt.clear();
    pOut.clear();

    if (hasGuiSupport())
    {
        // now create dock widget for this plugin
        DockWidgetQuantumComposer* dw = new DockWidgetQuantumComposer(this);
        Qt::DockWidgetAreas areas = Qt::AllDockWidgetAreas;
        QDockWidget::DockWidgetFeatures features = QDockWidget::DockWidgetClosable |
            QDockWidget::DockWidgetFloatable | QDockWidget::DockWidgetMovable;
        createDockWidget(QString(m_params["name"].getVal<char*>()), features, areas, dw);
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
QuantumComposer::~QuantumComposer()
{
}

//----------------------------------------------------------------------------------------------------------------------------------
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
            tr("Input parameter is not a dataIO instance of the SerialIO Plugin!")
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

        retValue += SendQuestionWithAnswerString("*IDN?", answerStr, m_requestTimeOutMS);
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
            m_params["offCounter"].setVal<int>(answerInt);

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

    setIdentifier(QString::number(getID()));

    if (waitCond)
    {
        waitCond->returnValue = retValue;
        waitCond->release();
    }

    setInitialized(true); // init method has been finished (independent on retval)

    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal QuantumComposer::close(ItomSharedSemaphore* waitCond)
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
        else if (key == "statesList")
        {
            int* values = new int[m_numChannels];
            for (int ch = 1; ch <= m_numChannels; ch++)
            {
                retValue += SendQuestionWithAnswerInteger(
                    QString(":PULSE%1:STAT?").arg(ch).toStdString().c_str(),
                    values[ch - 1],
                    m_requestTimeOutMS);
            }
            it->setVal<int*>(values, m_numChannels);
            DELETE_AND_SET_NULL_ARRAY(values);
        }
        else if (key == "widthsList")
        {
            double* values = new double[m_numChannels];
            for (int ch = 1; ch <= m_numChannels; ch++)
            {
                retValue += SendQuestionWithAnswerDouble(
                    QString(":PULSE%1:WIDT?").arg(ch).toStdString().c_str(),
                    values[ch - 1],
                    m_requestTimeOutMS);
            }
            it->setVal<double*>(values, m_numChannels);
            DELETE_AND_SET_NULL_ARRAY(values);
        }
        else if (key == "delaysList")
        {
            double* values = new double[m_numChannels];
            for (int ch = 1; ch <= m_numChannels; ch++)
            {
                retValue += SendQuestionWithAnswerDouble(
                    QString(":PULSE%1:DEL?").arg(ch).toStdString().c_str(),
                    values[ch - 1],
                    m_requestTimeOutMS);
            }
            it->setVal<double*>(values, m_numChannels);
            DELETE_AND_SET_NULL_ARRAY(values);
        }
        else if (key == "syncsList")
        {
            int* values = new int[m_numChannels];
            QByteArray answerStr;
            for (int ch = 1; ch <= m_numChannels; ch++)
            {
                retValue += SendQuestionWithAnswerString(
                    QString(":PULSE%1:SYNC?").arg(ch).toStdString().c_str(),
                    answerStr,
                    m_requestTimeOutMS);
                if (answerStr == "T0")
                {
                    values[ch - 1] = 0;
                }
                else if (answerStr == "CHA")
                {
                    values[ch - 1] = 1;
                }
                else if (answerStr == "CHB")
                {
                    values[ch - 1] = 2;
                }
                else if (answerStr == "CHC")
                {
                    values[ch - 1] = 3;
                }
                else if (answerStr == "CHD")
                {
                    values[ch - 1] = 4;
                }
                else if (answerStr == "CHE")
                {
                    values[ch - 1] = 5;
                }
                else if (answerStr == "CHF")
                {
                    values[ch - 1] = 6;
                }
                else if (answerStr == "CHG")
                {
                    values[ch - 1] = 7;
                }
                else if (answerStr == "CHH")
                {
                    values[ch - 1] = 8;
                }
            }
            it->setVal<int*>(values, m_numChannels);
            DELETE_AND_SET_NULL_ARRAY(values);
        }
        else if (key == "muxsList")
        {
            int* values = new int[m_numChannels];
            for (int ch = 1; ch <= m_numChannels; ch++)
            {
                retValue += SendQuestionWithAnswerInteger(
                    QString(":PULSE%1:MUX?").arg(ch).toStdString().c_str(),
                    values[ch - 1],
                    m_requestTimeOutMS);
            }
            it->setVal<int*>(values, m_numChannels);
            DELETE_AND_SET_NULL_ARRAY(values);
        }
        else if (key == "polaritiesList")
        {
            ito::ByteArray* values = new ito::ByteArray[m_numChannels];
            QByteArray answer;
            for (int ch = 1; ch <= m_numChannels; ch++)
            {
                retValue += SendQuestionWithAnswerString(
                    QString(":PULSE%1:POL?").arg(ch).toStdString().c_str(),
                    answer,
                    m_requestTimeOutMS);
                values[ch - 1] = answer.toStdString().c_str();
            }

            it->setVal<ito::ByteArray*>(values, m_numChannels);
            DELETE_AND_SET_NULL_ARRAY(values);
        }
        else if (key == "outputModesList")
        {
            ito::ByteArray* values = new ito::ByteArray[m_numChannels];
            QByteArray answer;
            for (int ch = 1; ch <= m_numChannels; ch++)
            {
                retValue += SendQuestionWithAnswerString(
                    QString(":PULSE%1:OUTP:MOD?").arg(ch).toStdString().c_str(),
                    answer,
                    m_requestTimeOutMS);
                values[ch - 1] = answer.toStdString().c_str();
            }

            it->setVal<ito::ByteArray*>(values, m_numChannels);
            DELETE_AND_SET_NULL_ARRAY(values);
        }
        else if (key == "amplitudesList")
        {
            double* values = new double[m_numChannels];
            for (int ch = 1; ch <= m_numChannels; ch++)
            {
                retValue += SendQuestionWithAnswerDouble(
                    QString(":PULSE%1:OUTP:AMPL?").arg(ch).toStdString().c_str(),
                    values[ch - 1],
                    m_requestTimeOutMS);
            }
            it->setVal<double*>(values, m_numChannels);
            DELETE_AND_SET_NULL_ARRAY(values);
        }
        else if (key == "channelModesList")
        {
            ito::ByteArray* values = new ito::ByteArray[m_numChannels];
            QByteArray answer;
            for (int ch = 1; ch <= m_numChannels; ch++)
            {
                retValue += SendQuestionWithAnswerString(
                    QString(":PULSE%1:CMOD?").arg(ch).toStdString().c_str(),
                    answer,
                    m_requestTimeOutMS);
                values[ch - 1] = answer.toStdString().c_str();
            }

            it->setVal<ito::ByteArray*>(values, m_numChannels);
            DELETE_AND_SET_NULL_ARRAY(values);
        }
        else if (key == "channelBurstCounterList")
        {
            int* values = new int[m_numChannels];
            for (int ch = 1; ch <= m_numChannels; ch++)
            {
                retValue += SendQuestionWithAnswerInteger(
                    QString(":PULSE%1:BCO?").arg(ch).toStdString().c_str(),
                    values[ch - 1],
                    m_requestTimeOutMS);
            }
            it->setVal<int*>(values, m_numChannels);
            DELETE_AND_SET_NULL_ARRAY(values);
        }
        else if (key == "channelPulseCounterList")
        {
            int* values = new int[m_numChannels];
            for (int ch = 1; ch <= m_numChannels; ch++)
            {
                retValue += SendQuestionWithAnswerInteger(
                    QString(":PULSE%1:PCO?").arg(ch).toStdString().c_str(),
                    values[ch - 1],
                    m_requestTimeOutMS);
            }
            it->setVal<int*>(values, m_numChannels);
            DELETE_AND_SET_NULL_ARRAY(values);
        }
        else if (key == "channelOffCounterList")
        {
            int* values = new int[m_numChannels];
            for (int ch = 1; ch <= m_numChannels; ch++)
            {
                retValue += SendQuestionWithAnswerInteger(
                    QString(":PULSE%1:OCO?").arg(ch).toStdString().c_str(),
                    values[ch - 1],
                    m_requestTimeOutMS);
            }
            it->setVal<int*>(values, m_numChannels);
            DELETE_AND_SET_NULL_ARRAY(values);
        }
        else if (key == "channelPulseWaitCounterList")
        {
            int* values = new int[m_numChannels];
            for (int ch = 1; ch <= m_numChannels; ch++)
            {
                retValue += SendQuestionWithAnswerInteger(
                    QString(":PULSE%1:WCO?").arg(ch).toStdString().c_str(),
                    values[ch - 1],
                    m_requestTimeOutMS);
            }
            it->setVal<int*>(values, m_numChannels);
            DELETE_AND_SET_NULL_ARRAY(values);
        }
        else if (key == "channelGateModeList")
        {
            ito::ByteArray* values = new ito::ByteArray[m_numChannels];
            QByteArray answer;
            for (int ch = 1; ch <= m_numChannels; ch++)
            {
                retValue += SendQuestionWithAnswerString(
                    QString(":PULSE%1:CGAT?").arg(ch).toStdString().c_str(),
                    answer,
                    m_requestTimeOutMS);
                values[ch - 1] = answer.toStdString().c_str();
            }

            it->setVal<ito::ByteArray*>(values, m_numChannels);
            DELETE_AND_SET_NULL_ARRAY(values);
        }
        else if (key == "channelGateLogicList")
        {
            ito::ByteArray* values = new ito::ByteArray[m_numChannels];
            QByteArray answer;
            for (int ch = 1; ch <= m_numChannels; ch++)
            {
                retValue += SendQuestionWithAnswerString(
                    QString(":PULSE%1:CLOG?").arg(ch).toStdString().c_str(),
                    answer,
                    m_requestTimeOutMS);
                values[ch - 1] = answer.toStdString().c_str();
            }

            it->setVal<ito::ByteArray*>(values, m_numChannels);
            DELETE_AND_SET_NULL_ARRAY(values);
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
        retValue += apiGetParamFromMapByKey(m_params, key, it, true);
    }

    if (!retValue.containsError())
    {
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
            retValue += it->copyValueFrom(&(*val));
        }
    }

    if (!retValue.containsError())
    {
        emit parametersChanged(m_params);
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
        _sleep(m_delayAfterSendCommandMS);

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
                    result = result.left(pos);
                }
            }

            if (!done && timer.elapsed() > timeoutMS && timeoutMS >= 0)
            {
                retValue += ito::RetVal(
                    ito::retError,
                    m_delayAfterSendCommandMS,
                    tr("timeout during read string.").toLatin1().data());
            }
        }

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
    else if (funcName == "setChannelMuxs")
    {
        ito::ParamBase* channelList = nullptr;
        ito::ParamBase* muxsList = nullptr;

        channelList = ito::getParamByName(&(*paramsMand), "channelIndexList", &retValue);
        muxsList = ito::getParamByName(&(*paramsMand), "muxsList", &retValue);

        if (!retValue.containsError())
        {
            retValue += QuantumComposer::setChannelMuxs(*channelList, *muxsList);
        }
    }
    else if (funcName == "setChannelPolarities")
    {
        ito::ParamBase* channelList = nullptr;
        ito::ParamBase* polsList = nullptr;

        channelList = ito::getParamByName(&(*paramsMand), "channelIndexList", &retValue);
        polsList = ito::getParamByName(&(*paramsMand), "polaritiesList", &retValue);

        if (!retValue.containsError())
        {
            retValue += QuantumComposer::setChannelPolarities(*channelList, *polsList);
        }
    }
    else if (funcName == "setChannelOutputModes")
    {
        ito::ParamBase* channelList = nullptr;
        ito::ParamBase* modesList = nullptr;

        channelList = ito::getParamByName(&(*paramsMand), "channelIndexList", &retValue);
        modesList = ito::getParamByName(&(*paramsMand), "outputModesList", &retValue);

        if (!retValue.containsError())
        {
            retValue += QuantumComposer::setChannelOutputModes(*channelList, *modesList);
        }
    }
    else if (funcName == "setChannelAdjustableAmplitude")
    {
        ito::ParamBase* channelList = nullptr;
        ito::ParamBase* ampList = nullptr;

        channelList = ito::getParamByName(&(*paramsMand), "channelIndexList", &retValue);
        ampList = ito::getParamByName(&(*paramsMand), "amplitudesList", &retValue);

        if (!retValue.containsError())
        {
            retValue += QuantumComposer::setChannelAdjustableAmplitude(*channelList, *ampList);
        }
    }
    else if (funcName == "setChannelModes")
    {
        ito::ParamBase* channelList = nullptr;
        ito::ParamBase* modeList = nullptr;

        channelList = ito::getParamByName(&(*paramsMand), "channelIndexList", &retValue);
        modeList = ito::getParamByName(&(*paramsMand), "channelModesList", &retValue);

        if (!retValue.containsError())
        {
            retValue += QuantumComposer::setChannelModes(*channelList, *modeList);
        }
    }
    else if (funcName == "setChannelBurstCounter")
    {
        ito::ParamBase* channelList = nullptr;
        ito::ParamBase* burstCounters = nullptr;

        channelList = ito::getParamByName(&(*paramsMand), "channelIndexList", &retValue);
        burstCounters = ito::getParamByName(&(*paramsMand), "channelBurstCounterList", &retValue);

        if (!retValue.containsError())
        {
            retValue += QuantumComposer::setChannelBurstCounter(*channelList, *burstCounters);
        }
    }
    else if (funcName == "setChannelPulseCounter")
    {
        ito::ParamBase* channelList = nullptr;
        ito::ParamBase* pulseCounters = nullptr;

        channelList = ito::getParamByName(&(*paramsMand), "channelIndexList", &retValue);
        pulseCounters = ito::getParamByName(&(*paramsMand), "channelPulseCounterList", &retValue);

        if (!retValue.containsError())
        {
            retValue += QuantumComposer::setChannelPulseCounter(*channelList, *pulseCounters);
        }
    }
    else if (funcName == "setChannelOffCounter")
    {
        ito::ParamBase* channelList = nullptr;
        ito::ParamBase* pulseCounters = nullptr;

        channelList = ito::getParamByName(&(*paramsMand), "channelIndexList", &retValue);
        pulseCounters = ito::getParamByName(&(*paramsMand), "channelOffCounterList", &retValue);

        if (!retValue.containsError())
        {
            retValue += QuantumComposer::setChannelOffCounter(*channelList, *pulseCounters);
        }
    }
    else if (funcName == "setChannelWaitCounter")
    {
        ito::ParamBase* channelList = nullptr;
        ito::ParamBase* pulseCounters = nullptr;

        channelList = ito::getParamByName(&(*paramsMand), "channelIndexList", &retValue);
        pulseCounters =
            ito::getParamByName(&(*paramsMand), "channelPulseWaitCounterList", &retValue);

        if (!retValue.containsError())
        {
            retValue += QuantumComposer::setChannelWaitCounter(*channelList, *pulseCounters);
        }
    }
    else if (funcName == "setChannelGatesModes")
    {
        ito::ParamBase* channelList = nullptr;
        ito::ParamBase* gates = nullptr;

        channelList = ito::getParamByName(&(*paramsMand), "channelIndexList", &retValue);
        gates = ito::getParamByName(&(*paramsMand), "channelGateModeList", &retValue);

        if (!retValue.containsError())
        {
            retValue += QuantumComposer::setChannelGatesModes(*channelList, *gates);
        }
    }

    else if (funcName == "setChannelGatesLogicLevel")
    {
        ito::ParamBase* channelList = nullptr;
        ito::ParamBase* gates = nullptr;

        channelList = ito::getParamByName(&(*paramsMand), "channelIndexList", &retValue);
        gates = ito::getParamByName(&(*paramsMand), "channelGateLogicList", &retValue);

        if (!retValue.containsError())
        {
            retValue += QuantumComposer::setChannelGatesLogicLevel(*channelList, *gates);
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
        int* paramStates = m_params["statesList"].getVal<int*>();
        for (int ch = 0; ch < channelIndices.getLen(); ch++)
        {
            retValue += SendCommand(QString(":PULSE%1:STAT %2")
                                        .arg(channels[ch])
                                        .arg(states[ch])
                                        .toStdString()
                                        .c_str());
            paramStates[ch] = states[ch];
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
                channelName = "T0";
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


//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal QuantumComposer::setChannelMuxs(
    ito::ParamBase& channelIndices, ito::ParamBase& muxsList)
{
    ito::RetVal retValue(ito::retOk);

    const int* channels = channelIndices.getVal<int*>();
    const int* muxs = muxsList.getVal<int*>();

    if (channelIndices.getLen() != muxsList.getLen())
    {
        retValue += ito::RetVal(
            ito::retError,
            0,
            tr("The lengths of the channel list (%1) and widths list (%2) must be the same.")
                .arg(channelIndices.getLen())
                .arg(muxsList.getLen())
                .toLatin1()
                .data());
    }
    else
    {
        for (int ch = 0; ch < channelIndices.getLen(); ch++)
        {
            retValue += SendCommand(QString(":PULSE%1:MUX %2")
                                        .arg(channels[ch])
                                        .arg(QString::number(muxs[ch]))
                                        .toStdString()
                                        .c_str());
        }
    }

    return retValue;
}


//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal QuantumComposer::setChannelPolarities(
    ito::ParamBase& channelIndices, ito::ParamBase& polsList)
{
    ito::RetVal retValue(ito::retOk);

    const int* channels = channelIndices.getVal<int*>();
    const char* pols = polsList.getVal<char*>();

    if (channelIndices.getLen() != polsList.getLen())
    {
        retValue += ito::RetVal(
            ito::retError,
            0,
            tr("The lengths of the channel list (%1) and widths list (%2) must be the same.")
                .arg(channelIndices.getLen())
                .arg(polsList.getLen())
                .toLatin1()
                .data());
    }
    else
    {
        for (int ch = 0; ch < channelIndices.getLen(); ch++)
        {
            retValue += SendCommand(
                QString(":PULSE%1:POL %2").arg(channels[ch]).arg(pols[ch]).toStdString().c_str());
        }
    }

    return retValue;
}


//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal QuantumComposer::setChannelOutputModes(
    ito::ParamBase& channelIndices, ito::ParamBase& modesList)
{
    ito::RetVal retValue(ito::retOk);

    const int* channels = channelIndices.getVal<int*>();
    const ito::ByteArray* modes = modesList.getVal<const ito::ByteArray*>();

    if (channelIndices.getLen() != modesList.getLen())
    {
        retValue += ito::RetVal(
            ito::retError,
            0,
            tr("The lengths of the channel list (%1) and widths list (%2) must be the same.")
                .arg(channelIndices.getLen())
                .arg(modesList.getLen())
                .toLatin1()
                .data());
    }
    else
    {
        for (int ch = 0; ch < channelIndices.getLen(); ch++)
        {
            retValue += SendCommand(QString(":PULSE%1:OUTP:MOD %2")
                                        .arg(channels[ch])
                                        .arg(modes[ch].data())
                                        .toStdString()
                                        .c_str());
        }
    }

    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal QuantumComposer::setChannelAdjustableAmplitude(
    ito::ParamBase& channelIndices, ito::ParamBase& amplitudesList)
{
    ito::RetVal retValue(ito::retOk);

    const int* channels = channelIndices.getVal<int*>();
    const double* amplitudes = amplitudesList.getVal<double*>();

    if (channelIndices.getLen() != amplitudesList.getLen())
    {
        retValue += ito::RetVal(
            ito::retError,
            0,
            tr("The lengths of the channel list (%1) and widths list (%2) must be the same.")
                .arg(channelIndices.getLen())
                .arg(amplitudesList.getLen())
                .toLatin1()
                .data());
    }
    else
    {
        for (int ch = 0; ch < channelIndices.getLen(); ch++)
        {
            retValue += SendCommand(QString(":PULSE%1:OUTP:AMPL %2")
                                        .arg(channels[ch])
                                        .arg(QString::number(amplitudes[ch], 'f', 2))
                                        .toStdString()
                                        .c_str());
        }
    }

    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal QuantumComposer::setChannelModes(
    ito::ParamBase& channelIndices, ito::ParamBase& modesList)
{
    ito::RetVal retValue(ito::retOk);

    const int* channels = channelIndices.getVal<int*>();
    const ito::ByteArray* modes = modesList.getVal<const ito::ByteArray*>();

    if (channelIndices.getLen() != modesList.getLen())
    {
        retValue += ito::RetVal(
            ito::retError,
            0,
            tr("The lengths of the channel list (%1) and widths list (%2) must be the same.")
                .arg(channelIndices.getLen())
                .arg(modesList.getLen())
                .toLatin1()
                .data());
    }
    else
    {
        for (int ch = 0; ch < channelIndices.getLen(); ch++)
        {
            retValue += SendCommand(QString(":PULSE%1:CMOD %2")
                                        .arg(channels[ch])
                                        .arg(modes[ch].data())
                                        .toStdString()
                                        .c_str());
        }
    }

    return retValue;
}


//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal QuantumComposer::setChannelBurstCounter(
    ito::ParamBase& channelIndices, ito::ParamBase& burstCounterList)
{
    ito::RetVal retValue(ito::retOk);

    const int* channels = channelIndices.getVal<int*>();
    const int* bursts = burstCounterList.getVal<int*>();

    if (channelIndices.getLen() != burstCounterList.getLen())
    {
        retValue += ito::RetVal(
            ito::retError,
            0,
            tr("The lengths of the channel list (%1) and widths list (%2) must be the same.")
                .arg(channelIndices.getLen())
                .arg(burstCounterList.getLen())
                .toLatin1()
                .data());
    }
    else
    {
        for (int ch = 0; ch < channelIndices.getLen(); ch++)
        {
            retValue += SendCommand(QString(":PULSE%1:BCO %2")
                                        .arg(channels[ch])
                                        .arg(QString::number(bursts[ch]))
                                        .toStdString()
                                        .c_str());
        }
    }

    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal QuantumComposer::setChannelPulseCounter(
    ito::ParamBase& channelIndices, ito::ParamBase& pulseCounterList)
{
    ito::RetVal retValue(ito::retOk);

    const int* channels = channelIndices.getVal<int*>();
    const int* pulses = pulseCounterList.getVal<int*>();

    if (channelIndices.getLen() != pulseCounterList.getLen())
    {
        retValue += ito::RetVal(
            ito::retError,
            0,
            tr("The lengths of the channel list (%1) and widths list (%2) must be the same.")
                .arg(channelIndices.getLen())
                .arg(pulseCounterList.getLen())
                .toLatin1()
                .data());
    }
    else
    {
        for (int ch = 0; ch < channelIndices.getLen(); ch++)
        {
            retValue += SendCommand(QString(":PULSE%1:PCO %2")
                                        .arg(channels[ch])
                                        .arg(QString::number(pulses[ch]))
                                        .toStdString()
                                        .c_str());
        }
    }

    return retValue;
}


//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal QuantumComposer::setChannelOffCounter(
    ito::ParamBase& channelIndices, ito::ParamBase& pulseCounterList)
{
    ito::RetVal retValue(ito::retOk);

    const int* channels = channelIndices.getVal<int*>();
    const int* pulses = pulseCounterList.getVal<int*>();

    if (channelIndices.getLen() != pulseCounterList.getLen())
    {
        retValue += ito::RetVal(
            ito::retError,
            0,
            tr("The lengths of the channel list (%1) and widths list (%2) must be the same.")
                .arg(channelIndices.getLen())
                .arg(pulseCounterList.getLen())
                .toLatin1()
                .data());
    }
    else
    {
        for (int ch = 0; ch < channelIndices.getLen(); ch++)
        {
            retValue += SendCommand(QString(":PULSE%1:OCO %2")
                                        .arg(channels[ch])
                                        .arg(QString::number(pulses[ch]))
                                        .toStdString()
                                        .c_str());
        }
    }

    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal QuantumComposer::setChannelWaitCounter(
    ito::ParamBase& channelIndices, ito::ParamBase& pulseCounterList)
{
    ito::RetVal retValue(ito::retOk);

    const int* channels = channelIndices.getVal<int*>();
    const int* pulses = pulseCounterList.getVal<int*>();

    if (channelIndices.getLen() != pulseCounterList.getLen())
    {
        retValue += ito::RetVal(
            ito::retError,
            0,
            tr("The lengths of the channel list (%1) and widths list (%2) must be the same.")
                .arg(channelIndices.getLen())
                .arg(pulseCounterList.getLen())
                .toLatin1()
                .data());
    }
    else
    {
        for (int ch = 0; ch < channelIndices.getLen(); ch++)
        {
            retValue += SendCommand(QString(":PULSE%1:WCO %2")
                                        .arg(channels[ch])
                                        .arg(QString::number(pulses[ch]))
                                        .toStdString()
                                        .c_str());
        }
    }

    return retValue;
}


//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal QuantumComposer::setChannelGatesModes(
    ito::ParamBase& channelIndices, ito::ParamBase& modesList)
{
    ito::RetVal retValue(ito::retOk);

    const int* channels = channelIndices.getVal<int*>();
    const ito::ByteArray* gates = modesList.getVal<const ito::ByteArray*>();

    if (channelIndices.getLen() != modesList.getLen())
    {
        retValue += ito::RetVal(
            ito::retError,
            0,
            tr("The lengths of the channel list (%1) and widths list (%2) must be the same.")
                .arg(channelIndices.getLen())
                .arg(modesList.getLen())
                .toLatin1()
                .data());
    }
    else
    {
        for (int ch = 0; ch < channelIndices.getLen(); ch++)
        {
            retValue += SendCommand(QString(":PULSE%1:CGAT %2")
                                        .arg(channels[ch])
                                        .arg(gates[ch].data())
                                        .toStdString()
                                        .c_str());
        }
    }

    return retValue;
}


//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal QuantumComposer::setChannelGatesLogicLevel(
    ito::ParamBase& channelIndices, ito::ParamBase& levelList)
{
    ito::RetVal retValue(ito::retOk);

    const int* channels = channelIndices.getVal<int*>();
    const ito::ByteArray* gates = levelList.getVal<const ito::ByteArray*>();

    if (channelIndices.getLen() != levelList.getLen())
    {
        retValue += ito::RetVal(
            ito::retError,
            0,
            tr("The lengths of the channel list (%1) and widths list (%2) must be the same.")
                .arg(channelIndices.getLen())
                .arg(levelList.getLen())
                .toLatin1()
                .data());
    }
    else
    {
        for (int ch = 0; ch < channelIndices.getLen(); ch++)
        {
            retValue += SendCommand(QString(":PULSE%1:CLOG %2")
                                        .arg(channels[ch])
                                        .arg(gates[ch].data())
                                        .toStdString()
                                        .c_str());
        }
    }

    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
void QuantumComposer::dockWidgetVisibilityChanged(bool visible)
{
    if (getDockWidget())
    {
        QWidget* widget = getDockWidget()->widget();
        if (visible)
        {
            connect(
                this,
                SIGNAL(parametersChanged(QMap<QString, ito::Param>)),
                widget,
                SLOT(parametersChanged(QMap<QString, ito::Param>)));
            emit parametersChanged(m_params);
        }
        else
        {
            disconnect(
                this,
                SIGNAL(parametersChanged(QMap<QString, ito::Param>)),
                widget,
                SLOT(parametersChanged(QMap<QString, ito::Param>)));
        }
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
const ito::RetVal QuantumComposer::showConfDialog(void)
{
    return apiShowConfigurationDialog(this, new DialogQuantumComposer(this));
}
