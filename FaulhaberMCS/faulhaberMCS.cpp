/* ********************************************************************
    Plugin "FaulhaberMCS" for itom software
    URL: http://www.uni-stuttgart.de/ito
    Copyright (C) 2024, Institut für Technische Optik (ITO),
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

#define NOMINMAX

#include "faulhaberMCS.h"
#include "common/helperCommon.h"
#include "gitVersion.h"
#include "pluginVersion.h"

#include <qplugin.h>
#include <qstring.h>
#include <qstringlist.h>
#include <qwaitcondition.h>
#include <qelapsedtimer.h>
#include <qdatetime.h>

#include "dockWidgetFaulhaberMCS.h"

QMap<ito::uint8, QList<ito::uint8>> openedNodes;

//----------------------------------------------------------------------------------------------------------------------------------
FaulhaberMCSInterface::FaulhaberMCSInterface()
{
    m_type = ito::typeActuator;
    setObjectName("FaulhaberMCS");

    m_description = QObject::tr("FaulhaberMCS");

    m_detaildescription =
        QObject::tr("This plugin is an actuator plugin to control servo motors from Faulhaber.\n\
\n\
It was implemented for RS232 communication and tested with:\n\
\n\
* Serie MCS 3242: https://www.faulhaber.com/de/produkte/serie/mcs-3242bx4-et/\n\
\n\
Homing options:\n\n\
- 'setOrigin' function of the plugin uses the homing method '37' to set the current position to 0.\n\n\
- 'homing' execFunction can be used for the other homing methods which needs more input parameters.\n\n\
");

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
        tr("An opened serial port of 'SerialIO' plugin instance.").toUtf8().data());
    paramVal.setMeta(new ito::HWMeta("SerialIO"), true);
    m_initParamsMand.append(paramVal);

    paramVal = ito::Param(
        "nodeID",
        ito::ParamBase::Int | ito::ParamBase::In,
        1,
        new ito::IntMeta(1, std::numeric_limits<ito::uint8>::max(), 1, "Communication"),
        tr("Node ID of device. Register '0x2400.03'.").toLatin1().data());
    m_initParamsMand.append(paramVal);
}

//----------------------------------------------------------------------------------------------------------------------------------
FaulhaberMCSInterface::~FaulhaberMCSInterface()
{
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal FaulhaberMCSInterface::getAddInInst(ito::AddInBase** addInInst)
{
    NEW_PLUGININSTANCE(FaulhaberMCS)
    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal FaulhaberMCSInterface::closeThisInst(ito::AddInBase** addInInst)
{
    REMOVE_PLUGININSTANCE(FaulhaberMCS)
    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
FaulhaberMCS::FaulhaberMCS() :
    AddInActuator(), m_delayAfterSendCommandMS(50), m_async(0), m_numOfAxes(1), m_node(1),
    m_statusWord(0x0000), m_requestTimeOutMS(5000), m_waitForDoneTimeout(60000),
    m_waitForMCSTimeout(3000), m_nodeAppended(false), m_serialBufferSize(100)
{
    m_serialBuffer = QSharedPointer<char>(new char[m_serialBufferSize], [](char* ptr) {
        delete[] ptr; // Custom deleter to release the array properly
    });
    m_serialBufferLength = QSharedPointer<int>(new int(m_serialBufferSize));
    *m_serialBufferLength = m_serialBufferSize;
    // Clear the buffer initially
    std::memset(m_serialBuffer.data(), '\0', m_serialBufferSize);

    ito::Param paramVal(
        "name",
        ito::ParamBase::String | ito::ParamBase::Readonly,
        tr("FaulhaberMCS").toLatin1().data(),
        nullptr);
    m_params.insert(paramVal.getName(), paramVal);

    //------------------------------- category general device parameter
    //---------------------------//
    paramVal = ito::Param(
        "serialNumber",
        ito::ParamBase::String | ito::ParamBase::Readonly,
        "",
        tr("Serial number of device. Register '%1'.")
            .arg(convertHexToString(nodeID_register))
            .toUtf8()
            .data());
    paramVal.setMeta(new ito::StringMeta(ito::StringMeta::String, "", "General"), true);
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param(
        "deviceName",
        ito::ParamBase::String | ito::ParamBase::Readonly,
        "",
        tr("Name of device. Register '%1'.")
            .arg(convertHexToString(deviceName_register))
            .toUtf8()
            .data());
    paramVal.setMeta(new ito::StringMeta(ito::StringMeta::String, "", "General"), true);
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param(
        "vendorID",
        ito::ParamBase::String | ito::ParamBase::Readonly,
        "",
        tr("Vendor ID of device. Register '%1'.")
            .arg(convertHexToString(vendorID_register))
            .toUtf8()
            .data());
    paramVal.setMeta(new ito::StringMeta(ito::StringMeta::String, "", "General"), true);
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param(
        "productCode",
        ito::ParamBase::String | ito::ParamBase::Readonly,
        "",
        tr("Product code number. Register '%1'.")
            .arg(convertHexToString(productCode_register))
            .toUtf8()
            .data());
    paramVal.setMeta(new ito::StringMeta(ito::StringMeta::String, "", "General"), true);
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param(
        "revisionNumber",
        ito::ParamBase::String | ito::ParamBase::Readonly,
        "",
        tr("Revision number. Register '%1'")
            .arg(convertHexToString(revisionNumber_register))
            .toUtf8()
            .data());
    paramVal.setMeta(new ito::StringMeta(ito::StringMeta::String, "", "General"), true);
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param(
        "firmware",
        ito::ParamBase::String | ito::ParamBase::Readonly,
        "",
        tr("Firmware version. Register '%1'")
            .arg(convertHexToString(firmwareVersion_register))
            .toUtf8()
            .data());
    paramVal.setMeta(new ito::StringMeta(ito::StringMeta::String, "", "General"), true);
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param(
        "operationMode",
        ito::ParamBase::Int,
        -4,
        10,
        1,
        tr("Operation Mode. -4: Analog Torque Control Mode, -3: Analog Veclocity Control Mode, -2: Analog Position Control Mode, -1: Voltage mode, 0: Controller not "
           "activated, 1: Profile Position Mode (default), 3: Profile Velocity Mode, 6: Homing, 8: Cyclic Synchronous Position Mode, 9: Cyclic Synchronouse Velocity Mode, 10: Cyclic Synchronous Torque Mode. Register '%1'.")
            .arg(convertHexToString(operationMode_register))
            .toUtf8()
            .data());
    paramVal.setMeta(new ito::IntMeta(-4, 10, 1, "General"));
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param(
        "nominalVoltage",
        ito::ParamBase::Int,
        0,
        tr("Nominal voltage of device. Register '%1'.")
            .arg(convertHexToString(nominalVoltage_register))
            .toUtf8()
            .data());
    paramVal.setMeta(new ito::IntMeta(0, std::numeric_limits<ito::int16>::max(), 1, "General"));
    m_params.insert(paramVal.getName(), paramVal);

    //------------------------------- category communication ---------------------------//
    paramVal = ito::Param(
        "netMode",
        ito::ParamBase::Int,
        0,
        tr("RS232 net mode. Register '%1'.")
            .arg(convertHexToString(netMode_register))
            .toUtf8()
            .data());
    paramVal.setMeta(new ito::IntMeta(0, 1, 1, "Communication"));
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param(
        "nodeID",
        ito::ParamBase::Int,
        0,
        tr("Node number. Register '%1'.")
            .arg(convertHexToString(nodeID_register)).toUtf8()
            .data());
    paramVal.setMeta(new ito::IntMeta(
        std::numeric_limits<ito::uint8>::min(),
        std::numeric_limits<ito::uint8>::max(),
        1,
        "Communication"));
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param(
        "deviceID",
        ito::ParamBase::Int,
        0,
        tr("Explicit device ID. Register '%1'")
            .arg(convertHexToString(deviceID_register))
            .toUtf8()
            .data());
    paramVal.setMeta(new ito::IntMeta(
        std::numeric_limits<ito::uint16>::min(),
        std::numeric_limits<ito::uint16>::max(),
        1,
        "Communication"));
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param(
        "ignoreCRC",
        ito::ParamBase::Int,
        0,
        1,
        0,
        tr("Ignore CRC checksum. Default is '0'.").toUtf8().data());
    paramVal.setMeta(new ito::IntMeta(0, 1, 1, "Communication"));
    m_params.insert(paramVal.getName(), paramVal);

    //------------------------------- category temperatures ---------------------------//
    paramVal = ito::Param(
        "temperatureCPU",
        ito::ParamBase::Int | ito::ParamBase::Readonly,
        0,
        tr("CPU temperature in [°C]. Register '%1'.")
            .arg(convertHexToString(CPUTemperature_register))
            .toUtf8()
            .data());
    paramVal.setMeta(new ito::IntMeta(0, std::numeric_limits<ito::int16>::max(), 1, "Temperature"));
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param(
        "temperaturePowerStage",
        ito::ParamBase::Int | ito::ParamBase::Readonly,
        0,
        tr("Power stage temperature in [°C]. Register '%1'.")
            .arg(convertHexToString(powerStageTemperature_register))
            .toUtf8()
            .data());
    paramVal.setMeta(new ito::IntMeta(0, std::numeric_limits<ito::int16>::max(), 1, "Temperature"));
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param(
        "temperatureWinding",
        ito::ParamBase::Int | ito::ParamBase::Readonly,
        0,
        tr("Winding temperature in [°C]. Register '%1'.")
            .arg(convertHexToString(windingTemperature_register))
            .toUtf8()
            .data());
    paramVal.setMeta(new ito::IntMeta(0, std::numeric_limits<ito::int16>::max(), 1, "Temperature"));
    m_params.insert(paramVal.getName(), paramVal);

    //------------------------------- category movement ---------------------------//
    paramVal = ito::Param(
        "async",
        ito::ParamBase::Int | ito::ParamBase::Readonly,
        0,
        1,
        m_async,
        tr("Asynchronous move (1), synchronous (0) [default]. Only synchronous operation is "
           "implemented.")
            .toUtf8()
            .data());
    paramVal.setMeta(new ito::IntMeta(0, 1, 1, "Movement"));
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param(
        "moveTimeout",
        ito::ParamBase::Int,
        0,
        std::numeric_limits<int>::max(),
        60000,
        tr("Timeout for movement in ms.").toUtf8().data());
    paramVal.setMeta(new ito::IntMeta(0, std::numeric_limits<int>::max(), 1, "Movement"));
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param(
        "operation",
        ito::ParamBase::Int,
        0,
        1,
        0,
        tr("Enable (1) or Disable (0) operation.").toUtf8().data());
    paramVal.setMeta(new ito::IntMeta(0, 1, 1, "Movement"));
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param(
        "power",
        ito::ParamBase::Int,
        0,
        1,
        0,
        tr("Enable (1) or Disable (0) device power.").toUtf8().data());
    paramVal.setMeta(new ito::IntMeta(0, 1, 1, "Movement"));
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param(
        "homed",
        ito::ParamBase::Int | ito::ParamBase::Readonly,
        0,
        1,
        0,
        tr("homed (1) or not homed (0).").toUtf8().data());
    paramVal.setMeta(new ito::IntMeta(0, 1, 1, "Movement"));
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param(
        "torque",
        ito::ParamBase::Int | ito::ParamBase::Readonly,
        0,
        tr("Actual value of the torque in relative scaling. Register '%1'.")
            .arg(convertHexToString(torqueActualValue_register))
            .toUtf8()
            .data());
    paramVal.setMeta(new ito::IntMeta(
        std::numeric_limits<ito::int16>::min(),
        std::numeric_limits<ito::int16>::max(),
        1,
        "Movement"));
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param(
        "current",
        ito::ParamBase::Int | ito::ParamBase::Readonly,
        0,
        tr("Actual value of the current in relative scaling. Register '%1'.")
            .arg(convertHexToString(currentActualValue_register))
            .toUtf8()
            .data());
    paramVal.setMeta(new ito::IntMeta(
        std::numeric_limits<ito::int16>::min(),
        std::numeric_limits<ito::int16>::max(),
        1,
        "Movement"));
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param(
        "loadInertia",
        ito::ParamBase::Double,
        0,
        tr("Load inertia in [gcm²]. Register '%1'.")
            .arg(convertHexToString(loadInertia_register))
            .toUtf8()
            .data());
    paramVal.setMeta(new ito::DoubleMeta(0.0, std::numeric_limits<ito::uint32>::max(), 0.1, "Movement"));
    m_params.insert(paramVal.getName(), paramVal);

    //------------------------------- category control ---------------------------//
    paramVal = ito::Param(
        "torqueGainControl",
        ito::ParamBase::Int,
        0,
        tr("Torque control gain parameter [mOm]. Register '%1'.")
            .arg(convertHexToString(torqueGainControl_register))
            .toUtf8()
            .data());
    paramVal.setMeta(new ito::IntMeta(
        std::numeric_limits<ito::uint32>::min(),
        std::numeric_limits<ito::uint32>::max(),
        1,
        "Control"));
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param(
        "torqueIntegralTimeControl",
        ito::ParamBase::Int,
        0,
        tr("Torque control integral time control parameter [µs]. Register '%1'.")
            .arg(convertHexToString(torqueGainControl_register))
            .toUtf8()
            .data());
    paramVal.setMeta(new ito::IntMeta(150, 2600, 1, "Control"));
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param(
        "fluxGainControl",
        ito::ParamBase::Int,
        0,
        tr("Flux control gain parameter [mOm]. Register '%1'.")
            .arg(convertHexToString(torqueGainControl_register))
            .toUtf8()
            .data());
    paramVal.setMeta(new ito::IntMeta(
        std::numeric_limits<ito::uint32>::min(),
        std::numeric_limits<ito::uint32>::max(),
        1,
        "Control"));
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param(
        "fluxIntegralTimeControl",
        ito::ParamBase::Int,
        0,
        tr("Flux control integral time control parameter [µs]. Register '%1'.")
            .arg(convertHexToString(torqueGainControl_register))
            .toUtf8()
            .data());
    paramVal.setMeta(new ito::IntMeta(150, 2600, 1, "Control"));
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param(
        "velocityGainControl",
        ito::ParamBase::Int,
        0,
        tr("Velocity gain control parameter [As 1e-6]. Register '%1'.")
            .arg(convertHexToString(torqueGainControl_register))
            .toUtf8()
            .data());
    paramVal.setMeta(new ito::IntMeta(
        std::numeric_limits<ito::uint32>::min(),
        std::numeric_limits<ito::uint32>::max(),
        1,
        "Control"));
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param(
        "velocityIntegralTimeControl",
        ito::ParamBase::Int,
        0,
        tr("Velocity integral time control parameter [µs]. Register '%1'.")
            .arg(convertHexToString(velocityIntegralTimeControl_register))
            .toUtf8()
            .data());
    paramVal.setMeta(new ito::IntMeta(
        std::numeric_limits<ito::uint16>::min(),
        std::numeric_limits<ito::uint16>::max(),
        1,
        "Control"));
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param(
        "velocityDeviationThresholdControl",
        ito::ParamBase::Int,
        0,
        tr("Velocity deviation threshold control parameter. Register '%1'.")
            .arg(convertHexToString(velocityDeviationThreshold_register))
            .toUtf8()
            .data());
    paramVal.setMeta(new ito::IntMeta(
        std::numeric_limits<ito::uint16>::min(),
        std::numeric_limits<ito::uint16>::max(),
        1,
        "Control"));
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param(
        "velocityDeviationTimeControl",
        ito::ParamBase::Int,
        0,
        tr("Velocity deviation time control parameter. Register '%1'.")
            .arg(convertHexToString(velocityDeviationTime_register))
            .toUtf8()
            .data());
    paramVal.setMeta(new ito::IntMeta(
        std::numeric_limits<ito::uint16>::min(),
        std::numeric_limits<ito::uint16>::max(),
        1,
        "Control"));
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param(
        "velocityWarningThresholdControl",
        ito::ParamBase::Int,
        0,
        tr("Velocity warning threshold control parameter. Register '%1'.")
            .arg(convertHexToString(velocityWarningThreshold_register))
            .toUtf8()
            .data());
    paramVal.setMeta(new ito::IntMeta(
        std::numeric_limits<ito::uint16>::min(),
        std::numeric_limits<ito::uint16>::max(),
        1,
        "Control"));
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param(
        "velocityIntegralPartOption",
        ito::ParamBase::Int,
        0,
        tr("Velocity integral part option. Configuration of the speed control loop. '0': integral "
           "component active, '1': stopped integral component in the position windoed (in PP "
           "mode), '2': integral component deactivated. Register '%1'.")
            .arg(convertHexToString(velocityIntegralPartOption))
            .toUtf8()
            .data());
    paramVal.setMeta(new ito::IntMeta(0, 2, 1, "Control"));
    m_params.insert(paramVal.getName(), paramVal);

    //------------------------------- category Statusword ---------------------------//
    paramVal = ito::Param(
        "readyToSwitchOn",
        ito::ParamBase::Int | ito::ParamBase::Readonly,
        0,
        1,
        0,
        tr("1: Ready to switch ON, 0: Not ready to switch ON (Bit 0).").toUtf8().data());
    paramVal.setMeta(new ito::IntMeta(0, 1, 1, "Status"));
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param(
        "switchedOn",
        ito::ParamBase::Int | ito::ParamBase::Readonly,
        0,
        1,
        0,
        tr("1: Drive is in the 'Switched ON' state, 0: No voltage present (Bit 1).")
            .toUtf8()
            .data());
    paramVal.setMeta(new ito::IntMeta(0, 1, 1, "Status"));
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param(
        "operationEnabled",
        ito::ParamBase::Int | ito::ParamBase::Readonly,
        0,
        1,
        0,
        tr("1: Operation enabled, 0: Operation disabled (Bit 2).").toUtf8().data());
    paramVal.setMeta(new ito::IntMeta(0, 1, 1, "Status"));
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param(
        "fault",
        ito::ParamBase::Int | ito::ParamBase::Readonly,
        0,
        1,
        0,
        tr("1: Error present, 0: No error present (Bit 3).").toUtf8().data());
    paramVal.setMeta(new ito::IntMeta(0, 1, 1, "Status"));
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param(
        "voltageEnabled",
        ito::ParamBase::Int | ito::ParamBase::Readonly,
        0,
        1,
        0,
        tr("1: Power supply enabled, 0: Power supply disabled (Bit 4).").toUtf8().data());
    paramVal.setMeta(new ito::IntMeta(0, 1, 1, "Status"));
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param(
        "quickStop",
        ito::ParamBase::Int | ito::ParamBase::Readonly,
        0,
        1,
        0,
        tr("1: Quick stop enabled, Quick stop disabled (Bit 5).").toUtf8().data());
    paramVal.setMeta(new ito::IntMeta(0, 1, 1, "Status"));
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param(
        "switchOnDisabled",
        ito::ParamBase::Int | ito::ParamBase::Readonly,
        0,
        1,
        0,
        tr("1: Switch on disabled, 0: Switch on enabled (Bit 6).").toUtf8().data());
    paramVal.setMeta(new ito::IntMeta(0, 1, 1, "Status"));
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param(
        "warning",
        ito::ParamBase::Int | ito::ParamBase::Readonly,
        0,
        1,
        0,
        tr("1: One of the monitored temperatures has exceeded at least the warning threshold, 0: "
           "No raised temperatures (Bit 7).")
            .toUtf8()
            .data());
    paramVal.setMeta(new ito::IntMeta(0, 1, 1, "Status"));
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param(
        "targetReached",
        ito::ParamBase::Int | ito::ParamBase::Readonly,
        0,
        1,
        0,
        tr("1: Target has reached, 0: is moving (Bit 10).").toUtf8().data());
    paramVal.setMeta(new ito::IntMeta(0, 1, 1, "Status"));
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param(
        "internalLimitActive",
        ito::ParamBase::Int | ito::ParamBase::Readonly,
        0,
        1,
        0,
        tr("1: Internal range limit (e.g. limit switch reached), 0: Internal range limit not "
           "reached (Bit 11).")
            .toUtf8()
            .data());
    paramVal.setMeta(new ito::IntMeta(0, 1, 1, "Status"));
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param(
        "setPointAcknowledged",
        ito::ParamBase::Int | ito::ParamBase::Readonly,
        0,
        1,
        0,
        tr("1: New set-point has been loaded, 0: Previous set-point being changed or already "
           "reached (Bit 12).")
            .toUtf8()
            .data());
    paramVal.setMeta(new ito::IntMeta(0, 1, 1, "Status"));
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param(
        "followingError",
        ito::ParamBase::Int | ito::ParamBase::Readonly,
        0,
        1,
        0,
        tr("1: Permissible range for the following error exceeded, 0: The actual position follows "
           "the instructions without a following error (Bit 13).")
            .toUtf8()
            .data());
    paramVal.setMeta(new ito::IntMeta(0, 1, 1, "Status"));
    m_params.insert(paramVal.getName(), paramVal);


    //------------------------------- category Motion control ---------------------------//
    paramVal = ito::Param(
        "maxMotorSpeed",
        ito::ParamBase::Int,
        0,
        tr("Max motor speed in 1/min. Register '%1'.")
            .arg(convertHexToString(maxMotorSpeed_register))
            .toUtf8()
            .data());
    paramVal.setMeta(new ito::IntMeta(1, 32767, 1, "Motion control"));
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param(
        "profileVelocity",
        ito::ParamBase::Int,
        0,
        tr("Profile velocity in 1/min. Register '%1'.")
            .arg(convertHexToString(profileVelocity_register))
            .toUtf8()
            .data());
    paramVal.setMeta(new ito::IntMeta(1, 32767, 1, "Motion control"));
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param(
        "acceleration",
        ito::ParamBase::Int,
        0,
        tr("Acceleration in 1/s². Register '%1'.")
            .arg(convertHexToString(acceleration_register))
            .toUtf8()
            .data());
    paramVal.setMeta(new ito::IntMeta(1, 30000, 1, "Motion control"));
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param(
        "deceleration",
        ito::ParamBase::Int,
        0,
        tr("Deceleration in 1/s². Register '%1'.")
            .arg(convertHexToString(deceleration_register))
            .toUtf8()
            .data());
    paramVal.setMeta(new ito::IntMeta(1, 30000, 1, "Motion control"));
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param(
        "quickStopDeceleration",
        ito::ParamBase::Int,
        0,
        tr("Quickstop deceleration in 1/s². Register '%1'.")
            .arg(convertHexToString(quickStopDeceleration_register))
            .toUtf8()
            .data());
    paramVal.setMeta(new ito::IntMeta(1, 32750, 1, "Motion control"));
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param(
        "maxTorqueLimit",
        ito::ParamBase::Int | ito::ParamBase::Readonly,
        0,
        tr("Maximum torque limit in relative scaling. 1000 = motor rated torque. Register '%1'.")
            .arg(convertHexToString(maxTorqueLimit_register))
            .toUtf8()
            .data());
    paramVal.setMeta(new ito::IntMeta(1, 30000, 1, "Motion control"));
    m_params.insert(paramVal.getName(), paramVal);

    int torqueLimits[] = {1000, 1000};
    paramVal = ito::Param(
        "torqueLimits",
        ito::ParamBase::IntArray | ito::ParamBase::In,
        2,
        torqueLimits,
        new ito::IntArrayMeta(0, 6000, 1, "Motion control"),
        tr("Negative/ positive torque limit values in relative scaling. 1000 = motor rated torque. "
           "Register negative limit '%1', positive limit '%2'.")
            .arg(convertHexToString(negativeTorqueLimit_register))
            .arg(convertHexToString(positiveTorqueLimit_register))
            .toUtf8()
            .data());
    m_params.insert(paramVal.getName(), paramVal);

    int softwareLimits[] = {
        std::numeric_limits<ito::int32>::min(), std::numeric_limits<ito::int32>::max()};
    paramVal = ito::Param(
        "positionLimits",
        ito::ParamBase::IntArray | ito::ParamBase::In,
        2,
        torqueLimits,
        new ito::IntArrayMeta(
            std::numeric_limits<ito::int32>::min(),
            std::numeric_limits<ito::int32>::max(),
            1,
            "Motion control"),
        tr("Lower/ upper limit of the position range in userdefined uints. "
           "Register lower limit '%1', upper limit '%2'.")
            .arg(convertHexToString(positionLowerLimit_register))
            .arg(convertHexToString(positionUpperLimit_register))
            .toUtf8()
            .data());
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param(
        "motionProfile",
        ito::ParamBase::Int,
        0,
        tr("Motion profile type (0: Linear ramp, 1: Sin2 ramp). Register '%1'.")
            .arg(convertHexToString(motionProfileType_register))
            .toUtf8()
            .data());
    paramVal.setMeta(new ito::IntMeta(0, 1, 1, "Motion control"));
    m_params.insert(paramVal.getName(), paramVal);

    //------------------------------------------------- EXEC FUNCTIONS
    QVector<ito::Param> pMand = QVector<ito::Param>();
    QVector<ito::Param> pOpt = QVector<ito::Param>();
    QVector<ito::Param> pOut = QVector<ito::Param>();

    paramVal = ito::Param(
        "method",
        ito::ParamBase::Int | ito::ParamBase::In,
        -4,
        37,
        0,
        tr("Homing method. Methods 1…34: A limit switch or an additional reference switch is used "
           "as reference. Method 37: The position is set to 0 without reference run. Methods "
           "–1…–4: A mechanical limit stop is set as reference. Register '%1'.")
            .arg(convertHexToString(homingMode_register))
            .toUtf8()
            .data());
    pMand.append(paramVal);

    paramVal = ito::Param(
        "offset",
        ito::ParamBase::Int | ito::ParamBase::In,
        std::numeric_limits<ito::int32>::min(),
        std::numeric_limits<ito::int32>::max(),
        0,
        tr("Offset of the zero position relative to the position of the reference switch in "
           "userdefined units. Register '%1'.")
            .arg(convertHexToString(homingOffset_register))
            .toUtf8()
            .data());
    pOpt.append(paramVal);

    paramVal = ito::Param(
        "switchSeekVelocity",
        ito::ParamBase::Int | ito::ParamBase::In,
        1,
        32767,
        400,
        tr("Speed during search for switch. Register '%1'.")
            .arg(convertHexToString(homingSeekVelocity_register))
            .toUtf8()
            .data());
    pOpt.append(paramVal);

    paramVal = ito::Param(
        "homingSpeed",
        ito::ParamBase::Int | ito::ParamBase::In,
        1,
        32767,
        400,
        tr("Speed during search for zero. Register '%1'.")
            .arg(convertHexToString(homingSpeed_register))
            .toUtf8()
            .data());
    pOpt.append(paramVal);

    paramVal = ito::Param(
        "acceleration",
        ito::ParamBase::Int | ito::ParamBase::In,
        1,
        30000,
        50,
        tr("Speed during search for zero. Register '%1'.")
            .arg(convertHexToString(homingAcceleration_register))
            .toUtf8()
            .data());
    pOpt.append(paramVal);

    paramVal = ito::Param(
        "limitCheckDelayTime",
        ito::ParamBase::Int | ito::ParamBase::In,
        0,
        32750,
        10,
        tr("Delay time until blockage detection [ms]. Register '%1'.")
            .arg(convertHexToString(homingLimitCheckDelayTime_register))
            .toUtf8()
            .data());
    pOpt.append(paramVal);

    int homingTorqueLimits[] = {1000, 1000};
    paramVal = ito::Param(
        "torqueLimits",
        ito::ParamBase::IntArray | ito::ParamBase::In,
        2,
        homingTorqueLimits,
        new ito::IntArrayMeta(0, 6000, 1, "Torque control"),
        tr("Upper/ lower limit values for the reference run in 1/1000 of the rated motor torque. "
           "Register negative limit '%1', positive limit '%2'.")
            .arg(convertHexToString(homingNegativeTorqueLimit_register))
            .arg(convertHexToString(homingPositiveTorqueLimit_register))
            .toUtf8()
            .data());
    pOpt.append(paramVal);

    paramVal = ito::Param(
        "timeout",
        ito::ParamBase::Int,
        0,
        60000,
        10000,
        tr("Timeout for homing in ms.").toUtf8().data());
    pOpt.append(paramVal);

    registerExecFunc(
        "homing",
        pMand,
        pOpt,
        pOut,
        tr("In most of the cases before position control is to be used, the drive must perform a "
           "reference run to align the position used by the drive to the mechanic setup.")
            .toUtf8()
            .data());
    pMand.clear();
    pOpt.clear();
    pOut.clear();

    // initialize the current position vector, the status vector and the target position vector
    m_currentPos.fill(0.0, m_numOfAxes);
    m_currentStatus.fill(0, m_numOfAxes);
    m_targetPos.fill(0.0, m_numOfAxes);

    //------------------------------------------------- DOCK WIDGET
    DockWidgetFaulhaberMCS* dw = new DockWidgetFaulhaberMCS(getID(), this);

    Qt::DockWidgetAreas areas = Qt::AllDockWidgetAreas;
    QDockWidget::DockWidgetFeatures features = QDockWidget::DockWidgetClosable |
        QDockWidget::DockWidgetFloatable | QDockWidget::DockWidgetMovable;
    createDockWidget(QString(m_params["name"].getVal<char*>()), features, areas, dw);
}

//----------------------------------------------------------------------------------------------------------------------------------
FaulhaberMCS::~FaulhaberMCS()
{
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal FaulhaberMCS::init(
    QVector<ito::ParamBase>* paramsMand,
    QVector<ito::ParamBase>* paramsOpt,
    ItomSharedSemaphore* waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);

    if (reinterpret_cast<ito::AddInBase*>((*paramsMand)[0].getVal<void*>())
            ->getBasePlugin()
            ->getType() &
        (ito::typeDataIO | ito::typeRawIO))
    {
        // Us given SerialIO instance
        m_pSerialIO = (ito::AddInDataIO*)(*paramsMand)[0].getVal<void*>();

        QSharedPointer<ito::Param> val(new ito::Param("port"));
        retValue += m_pSerialIO->getParam(val, NULL);

        if (!retValue.containsError())
        {
            m_port = val->getVal<int>();
        }
        m_node = (ito::uint8)paramsMand->at(1).getVal<int>();
        if (openedNodes[m_port].contains(m_node))
        {
            retValue += ito::RetVal(
                ito::retError,
                0,
                tr("An instance of node number '%1' is already open.")
                    .arg(m_node)
                    .toLatin1()
                    .data());
        }
        else
        {
            ito::uint8 node;
            retValue += getNodeID(node);
            if (!retValue.containsError())
            {
                if (node != m_node)
                {
                    retValue = ito::RetVal(
                        ito::retError,
                        0,
                        tr("The node number of the device is '%1' and not '%2'.")
                            .arg(node)
                            .arg(m_node)
                            .toLatin1()
                            .data());
                }
                else
                {
                    openedNodes[m_port].append(m_node);
                    m_nodeAppended = true;
                    m_params["nodeID"].setVal<int>(m_node);
                }
            }
            else
            {
                retValue = ito::RetVal(
                    ito::retError,
                    999,
                    tr("No device found for serialIO port '%1' and node '%2'.")
                        .arg(m_port)
                        .arg(m_node)
                        .toLatin1()
                        .data());
            }
        }
    }
    else
    {
        retValue += ito::RetVal(
            ito::retError,
            0,
            tr("Input parameter is not a dataIO instance of the SerialIO Plugin!")
                .toLatin1()
                .data());
    }

    if (!retValue.containsError())
    {
        QSharedPointer<QVector<ito::ParamBase>> _dummy;
        m_pSerialIO->execFunc("clearInputBuffer", _dummy, _dummy, _dummy, nullptr);
        m_pSerialIO->execFunc("clearOutputBuffer", _dummy, _dummy, _dummy, nullptr);
        m_serialBuffer = QSharedPointer<char>(new char[m_serialBufferSize]);
        m_serialBufferLength = QSharedPointer<int>(new int(m_serialBufferSize));

        *m_serialBufferLength = m_serialBufferSize;
        std::memset(m_serialBuffer.data(), '\0', m_serialBufferSize);
        retValue += setCommunicationSettings(TRANSMIT_EMCY_VIA_RS232); // Transmit EMCYs via RS232
    }

    // ENABLE
    if (!retValue.containsError())
    {
        retValue += startupSequence();
    }

    if (!retValue.containsError())
    {
        QString answerString;
        retValue += getSerialNumber(answerString);
        if (!retValue.containsError())
#
        {
            m_params["serialNumber"].setVal<char*>(answerString.toLatin1().data());
        }
    }

    if (!retValue.containsError())
    {
        QString answerString;
        retValue += getDeviceName(answerString);
        if (!retValue.containsError())
        {
            m_params["deviceName"].setVal<char*>(answerString.toLatin1().data());
        }
    }

    if (!retValue.containsError())
    {
        ito::uint8 mode;
        retValue += getNetMode(mode);
        if (!retValue.containsError())
        {
            m_params["netMode"].setVal<int>(static_cast<int>(mode));
        }
    }

    if (!retValue.containsError())
    {
        QString answerString;
        retValue += getVendorID(answerString);
        if (!retValue.containsError())
        {
            m_params["vendorID"].setVal<char*>(answerString.toLatin1().data());
        }
    }

    if (!retValue.containsError())
    {
        QString answerString;
        retValue += getProductCode(answerString);
        if (!retValue.containsError())
        {
            m_params["productCode"].setVal<char*>(answerString.toLatin1().data());
        }
    }

    if (!retValue.containsError())
    {
        QString answerString;
        retValue += getRevisionNumber(answerString);
        if (!retValue.containsError())
        {
            m_params["revisionNumber"].setVal<char*>(answerString.toLatin1().data());
        }
    }

    if (!retValue.containsError())
    {
        QString answerString;
        retValue += getFirmware(answerString);
        if (!retValue.containsError())
        {
            m_params["firmware"].setVal<char*>(answerString.toLatin1().data());
        }
    }

    if (!retValue.containsError())
    {
        ito::int16 temp;
        retValue += getCPUTemperature(temp);
        if (!retValue.containsError())
        {
            m_params["temperatureCPU"].setVal<int>(temp);
        }
    }

    if (!retValue.containsError())
    {
        ito::int16 temp;
        retValue += getPowerStageTemperature(temp);
        if (!retValue.containsError())
        {
            m_params["temperaturePowerStage"].setVal<int>(temp);
        }
    }

    if (!retValue.containsError())
    {
        ito::int16 temp;
        retValue += getWindingTemperature(temp);
        if (!retValue.containsError())
        {
            m_params["temperatureWinding"].setVal<int>(temp);
        }
    }

    if (!retValue.containsError())
    {
        ito::uint32 load;
        retValue += getLoadInertia(load);
        if (!retValue.containsError())
        {
            m_params["loadInertia"].setVal<double>(std::round(load / 1000.0 * 10) / 10);
        }
    }

    if (!retValue.containsError())
    {
        ito::int8 mode;
        retValue +=
            setOperationMode(static_cast<ito::int8>(m_params["operationMode"].getVal<int>()));
        if (!retValue.containsError())
        {
            retValue += getOperationMode(mode);
            if (!retValue.containsError())
            {
                m_params["operationMode"].setVal<int>(mode);
            }
        }
    }

    if (!retValue.containsError())
    {
        ito::uint32 speed;
        retValue += getMaxMotorSpeed(speed);
        if (!retValue.containsError())
        {
            m_params["maxMotorSpeed"].setVal<int>(speed);
        }
    }

    if (!retValue.containsError())
    {
        ito::uint32 acceleration;
        retValue += getAcceleration(acceleration);
        if (!retValue.containsError())
        {
            m_params["acceleration"].setVal<int>(acceleration);
        }
    }

    if (!retValue.containsError())
    {
        ito::uint32 deceleration;
        retValue += getDeceleration(deceleration);
        if (!retValue.containsError())
        {
            m_params["deceleration"].setVal<int>(deceleration);
        }
    }

    if (!retValue.containsError())
    {
        ito::uint32 speed;
        retValue += getProfileVelocity(speed);
        if (!retValue.containsError())
        {
            m_params["profileVelocity"].setVal<int>(speed);
        }
    }

    if (!retValue.containsError())
    {
        ito::uint16 id;
        retValue += getExplicitDeviceID(id);
        if (!retValue.containsError())
        {
            m_params["deviceID"].setVal<int>(id);
        }
    }

    if (!retValue.containsError())
    {
        ito::uint32 quick;
        retValue += getQuickStopDeceleration(quick);
        if (!retValue.containsError())
        {
            m_params["quickStopDeceleration"].setVal<int>(quick);
        }
    }

    if (!retValue.containsError())
    {
        ito::uint16 limit;
        retValue += getMaxTorqueLimit(limit);
        if (!retValue.containsError())
        {
            m_params["maxTorqueLimit"].setVal<int>(limit);
        }
    }

    if (!retValue.containsError())
    {
        ito::uint16 nlimit, pLimit;
        retValue += getNegativeTorqueLimit(nlimit);
        retValue += getPositveTorqueLimit(pLimit);

        int limits[2] = {static_cast<int>(nlimit), static_cast<int>(pLimit)};
        if (!retValue.containsError())
        {
            m_params["torqueLimits"].setVal<int*>(limits, 2);
        }
    }

    if (!retValue.containsError())
    {
        ito::int32 nlimit, pLimit;
        retValue += getPositionLowerLimit(nlimit);
        retValue += getPositionUpperLimit(pLimit);

        int limits[2] = {static_cast<int>(nlimit), static_cast<int>(pLimit)};
        if (!retValue.containsError())
        {
            m_params["positionLimits"].setVal<int*>(limits, 2);
        }
    }

    if (!retValue.containsError())
    {
        ito::uint32 torque;
        retValue += getTorqueGainControl(torque);
        if (!retValue.containsError())
        {
            m_params["torqueGainControl"].setVal<int>(torque);
        }
    }

    if (!retValue.containsError())
    {
        ito::uint16 torque;
        retValue += getTorqueIntegralTimeControl(torque);
        if (!retValue.containsError())
        {
            m_params["torqueIntegralTimeControl"].setVal<int>(torque);
        }
    }

    if (!retValue.containsError())
    {
        ito::uint32 flux;
        retValue += getFluxGainControl(flux);
        if (!retValue.containsError())
        {
            m_params["fluxGainControl"].setVal<int>(flux);
        }
    }

    if (!retValue.containsError())
    {
        ito::uint16 flux;
        retValue += getTorqueIntegralTimeControl(flux);
        if (!retValue.containsError())
        {
            m_params["fluxIntegralTimeControl"].setVal<int>(flux);
        }
    }

    if (!retValue.containsError())
    {
        ito::uint32 gain;
        retValue += getFluxGainControl(gain);
        if (!retValue.containsError())
        {
            m_params["velocityGainControl"].setVal<int>(gain);
        }
    }

    if (!retValue.containsError())
    {
        ito::uint16 time;
        retValue += getVelocityIntegralTimeControl(time);
        if (!retValue.containsError())
        {
            m_params["velocityIntegralTimeControl"].setVal<int>(time);
        }
    }

    if (!retValue.containsError())
    {
        ito::uint16 threshold;
        retValue += getVelocityDeviationThreshold(threshold);
        if (!retValue.containsError())
        {
            m_params["velocityDeviationThresholdControl"].setVal<int>(threshold);
        }
    }

    if (!retValue.containsError())
    {
        ito::uint16 velocitytime;
        retValue += getVelocityDeviationTime(velocitytime);
        if (!retValue.containsError())
        {
            m_params["velocityDeviationTimeControl"].setVal<int>(velocitytime);
        }
    }

    if (!retValue.containsError())
    {
        ito::uint16 warning;
        retValue += getVelocityWarningThreshold(warning);
        if (!retValue.containsError())
        {
            m_params["velocityWarningThresholdControl"].setVal<int>(warning);
        }
    }

    if (!retValue.containsError())
    {
        ito::uint8 option;
        retValue += getVelocityIntegralPartOption(option);
        if (!retValue.containsError())
        {
            m_params["velocityIntegralPartOption"].setVal<int>(option);
        }
    }

    if (!retValue.containsError())
    {
        ito::int16 current;
        retValue += getCurrent(current);
        if (!retValue.containsError())
        {
            m_params["current"].setVal<int>(current);
        }
    }

    if (!retValue.containsError())
    {
        retValue += updateStatus();

        ito::int32 pos = 0;
        ito::int32 target = 0;

        for (int i = 0; i < m_numOfAxes; i++)
        {
            switch (m_params["operationMode"].getVal<int>())
            {
            case -1:
                ito::int16 voltage;
                retValue += getVoltageMCS(voltage);
                pos = static_cast<int>(voltage);
                target = pos;
                break;
            case 1: // position mode
                retValue += getPosMCS(pos);
                retValue += getTargetPosMCS(target);
                break;
            case 3: // velocity mode
                retValue += getVelocityMCS(pos);
                retValue += getTargetVelocityMCS(target);
                break;
            case 10: // cyclic synch torque mode
                ito::int16 torque;
                retValue += getTorqueMCS(torque);
                pos = static_cast<int>(torque);

                retValue += getTargetTorqueMCS(torque);
                target = static_cast<int>(torque);
                break;
            }
            m_currentPos[i] = static_cast<double>(pos);
            m_targetPos[i] = static_cast<double>(target);
            m_currentStatus[i] =
                ito::actuatorAtTarget | ito::actuatorEnabled | ito::actuatorAvailable;
        }

        retValue += updateStatus();
        sendStatusUpdate(false);
        emit parametersChanged(m_params);
    }

    if (waitCond)
    {
        waitCond->returnValue = retValue;
        waitCond->release();
    }

    setInitialized(true);
    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal FaulhaberMCS::close(ItomSharedSemaphore* waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);

    if (m_nodeAppended)
    {
        openedNodes[m_port].removeOne(m_node);
        if (openedNodes.isEmpty())
        {
            retValue += shutDownSequence();
            m_serialBuffer.clear();
            m_serialBufferLength.clear();
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
ito::RetVal FaulhaberMCS::getParam(QSharedPointer<ito::Param> val, ItomSharedSemaphore* waitCond)
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
        int answerInteger = 0;
        if (key == "operationMode")
        {
            ito::int8 mode;
            retValue += getOperationMode(mode);
            if (!retValue.containsError())
            {
                retValue += it->setVal<int>(static_cast<int>(mode));
            }
        }
        else if (key == "temperatureCPU")
        {
            ito::int16 temp;
            retValue += getCPUTemperature(temp);
            if (!retValue.containsError())
            {
                retValue += it->setVal<int>(static_cast<int>(temp));
            }
        }
        else if (key == "temperaturePowerStage")
        {
            ito::int16 temp;
            retValue += getPowerStageTemperature(temp);
            if (!retValue.containsError())
            {
                retValue += it->setVal<int>(static_cast<int>(temp));
            }
        }
        else if (key == "temperatureWinding")
        {
            ito::int16 temp;
            retValue += getWindingTemperature(temp);
            if (!retValue.containsError())
            {
                retValue += it->setVal<int>(static_cast<int>(temp));
            }
        }
        else if (key == "loadInertia")
        {
            ito::uint32 load;
            retValue += getLoadInertia(load);
            if (!retValue.containsError())
            {
                retValue +=
                    it->setVal<double>(static_cast<double>(std::round(load / 1000.0 * 10) / 10));
            }
        }
        else if (key == "maxMotorSpeed")
        {
            ito::uint32 speed;
            retValue += getMaxMotorSpeed(speed);
            if (!retValue.containsError())
            {
                retValue += it->setVal<int>(static_cast<int>(speed));
            }
        }
        else if (key == "profileVelocity")
        {
            ito::uint32 speed;
            retValue += getProfileVelocity(speed);
            if (!retValue.containsError())
            {
                retValue += it->setVal<int>(static_cast<int>(speed));
            }
        }
        else if (key == "acceleration")
        {
            ito::uint32 acceleration;
            retValue += getAcceleration(acceleration);
            if (!retValue.containsError())
            {
                retValue += it->setVal<int>(static_cast<int>(acceleration));
            }
        }
        else if (key == "deceleration")
        {
            ito::uint32 deceleartion;
            retValue += getDeceleration(deceleartion);
            if (!retValue.containsError())
            {
                retValue += it->setVal<int>(static_cast<int>(deceleartion));
            }
        }
        else if (key == "quickStopDeceleration")
        {
            ito::uint32 quick;
            retValue += getQuickStopDeceleration(quick);
            if (!retValue.containsError())
            {
                retValue += it->setVal<int>(static_cast<int>(quick));
            }
        }
        else if (key == "moveTimeout")
        {
            it->setVal<int>(m_waitForDoneTimeout);
        }
        else if (key == "maxTorqueLimit")
        {
            ito::uint16 limit;
            retValue += getMaxTorqueLimit(limit);
            if (!retValue.containsError())
            {
                retValue += it->setVal<int>(static_cast<int>(limit));
            }
        }
        else if (key == "netMode")
        {
            ito::uint8 mode;
            retValue += getNetMode(mode);
            if (!retValue.containsError())
            {
                retValue += it->setVal<int>(static_cast<int>(mode));
            }
        }
        else if (key == "nodeID")
        {
            ito::uint8 node;
            retValue += getNodeID(node);
            if (!retValue.containsError())
            {
                retValue += it->setVal<int>(static_cast<int>(node));
            }
        }
        else if (key == "ignoreCRC")
        {
            ito::uint32 communicationSettings;
            retValue += getCommunicationSettings(communicationSettings);
            if (!retValue.containsError())
            {
                retValue +=
                    it->setVal<int>(static_cast<int>(isBitSet(communicationSettings, IGNORE_CRC)));
            }
        }
        else if (key == "deviceID")
        {
            ito::uint16 device;
            retValue += getExplicitDeviceID(device);
            if (!retValue.containsError())
            {
                retValue += it->setVal<int>(static_cast<int>(device));
            }
        }
        else if (key == "torque")
        {
            ito::int16 torque;
            retValue += getTorque(torque);
            if (!retValue.containsError())
            {
                retValue += it->setVal<int>(static_cast<int>(torque));
            }
        }
        else if (key == "current")
        {
            ito::int16 current;
            retValue += getCurrent(current);
            if (!retValue.containsError())
            {
                retValue += it->setVal<int>(static_cast<int>(current));
            }
        }
        else if (key == "torqueLimits")
        {
            ito::uint16 nlimit, pLimit;
            retValue += getNegativeTorqueLimit(nlimit);
            retValue += getPositveTorqueLimit(pLimit);

            int limits[2] = {static_cast<int>(nlimit), static_cast<int>(pLimit)};
            if (!retValue.containsError())
            {
                retValue += it->setVal<int*>(limits, 2);
            }
        }
        else if (key == "positionLimits")
        {
            ito::int32 nlimit, pLimit;
            retValue += getPositionLowerLimit(nlimit);
            retValue += getPositionUpperLimit(pLimit);

            int limits[2] = {static_cast<int>(nlimit), static_cast<int>(pLimit)};
            if (!retValue.containsError())
            {
                retValue += it->setVal<int*>(limits, 2);
            }
        }
        else if (key == "torqueGainControl")
        {
            ito::uint32 torque;
            retValue += getTorqueGainControl(torque);
            if (!retValue.containsError())
            {
                retValue += it->setVal<int>(static_cast<int>(torque));
            }
        }
        else if (key == "torqueIntegralTimeControl")
        {
            ito::uint16 torque;
            retValue += getTorqueIntegralTimeControl(torque);
            if (!retValue.containsError())
            {
                retValue += it->setVal<int>(static_cast<int>(torque));
            }
        }
        else if (key == "fluxGainControl")
        {
            ito::uint32 gain;
            retValue += getFluxGainControl(gain);
            if (!retValue.containsError())
            {
                retValue += it->setVal<int>(static_cast<int>(gain));
            }
        }
        else if (key == "fluxIntegralTimeControl")
        {
            ito::uint16 flux;
            retValue += getFluxIntegralTimeControl(flux);
            if (!retValue.containsError())
            {
                retValue += it->setVal<int>(static_cast<int>(flux));
            }
        }
        else if (key == "velocityGainControl")
        {
            ito::uint32 gain;
            retValue += getVelocityGainControl(gain);
            if (!retValue.containsError())
            {
                retValue += it->setVal<int>(static_cast<int>(gain));
            }
        }
        else if (key == "velocityIntegralTimeControl")
        {
            ito::uint16 time;
            retValue += getVelocityIntegralTimeControl(time);
            if (!retValue.containsError())
            {
                retValue += it->setVal<int>(static_cast<int>(time));
            }
        }
        else if (key == "velocityDeviationThresholdControl")
        {
            ito::uint16 thres;
            retValue += getVelocityDeviationThreshold(thres);
            if (!retValue.containsError())
            {
                retValue += it->setVal<int>(static_cast<int>(thres));
            }
        }
        else if (key == "velocityIntegralPartOption")
        {
            ito::uint8 option;
            retValue += getVelocityIntegralPartOption(option);
            if (!retValue.containsError())
            {
                retValue += it->setVal<int>(static_cast<int>(option));
            }
        }
        else if (key == "nominalVoltage")
        {
            ito::uint16 voltage;
            retValue += getNominalVoltage(voltage);

        }
        else if (key == "motionProfile")
        {
            ito::int16 mode;
            retValue += getMotionProfileType(mode);
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
ito::RetVal FaulhaberMCS::setParam(
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

    if (isMotorMoving()) // this if-case is for actuators only.
    {
        retValue += ito::RetVal(
            ito::retError,
            0,
            tr("any axis is moving. Parameters cannot be set.").toLatin1().data());
    }

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
        if (key == "operationMode")
        {
            retValue += setOperationMode(val->getVal<int>());
        }
        else if (key == "operation")
        {
            int operation = val->getVal<int>();
            if (operation == 0)
            {
                shutDown();
            }
            else if (operation == 1)
            {
                enableOperation();
            }
            else
            {
                retValue += ito::RetVal(
                    ito::retError,
                    0,
                    tr("Value (%1) of parameter 'operation' must be 0 or 1.")
                        .arg(operation)
                        .toLatin1()
                        .data());
            }
            _sleep(100);
        }
        else if (key == "power")
        {
            int power = val->getVal<int>();
            if (power == 0)
            {
                disableVoltage();
            }
            else if (power == 1)
            {
                enableOperation();
            }
            else
            {
                retValue += ito::RetVal(
                    ito::retError,
                    0,
                    tr("Value (%1) of parameter 'power' must be 0 or 1.")
                        .arg(power)
                        .toLatin1()
                        .data());
            }
            retValue += updateStatus();
        }
        else if (key == "maxMotorSpeed")
        {
            retValue += setMaxMotorSpeed(static_cast<ito::uint32>(val->getVal<int>()));
        }
        else if (key == "profileVelocity")
        {
            retValue += setProfileVelocity(static_cast<ito::uint32>(val->getVal<int>()));
        }
        else if (key == "acceleration")
        {
            retValue += setAcceleration(static_cast<ito::uint32>(val->getVal<int>()));
        }
        else if (key == "deceleration")
        {
            retValue += setDeceleration(static_cast<ito::uint32>(val->getVal<int>()));
        }
        else if (key == "quickStopDeceleration")
        {
            retValue += setQuickStopDeceleration(static_cast<ito::uint32>(val->getVal<int>()));
        }
        else if (key == "loadInertia")
        {
            retValue += setLoadInertia(static_cast<ito::uint32>(val->getVal<double>()*1000));
        }
        else if (key == "moveTimeout")
        {
            m_waitForDoneTimeout = val->getVal<int>();
            retValue += it->copyValueFrom(&(*val));
        }
        else if (key == "netMode")
        {
            retValue += setNetMode(static_cast<ito::uint8>(val->getVal<int>()));
        }
        else if (key == "ignoreCRC")
        {
            ito::uint32 communicationSettings;
            retValue += getCommunicationSettings(communicationSettings);
            if (val->getVal<int>() == 1)
            {
                communicationSettings |= IGNORE_CRC;
            }
            else
            {
                communicationSettings &= ~IGNORE_CRC;
            }
            retValue += setCommunicationSettings(communicationSettings);
        }
        else if (key == "nodeID")
        {
            ito::uint8 node = static_cast<ito::uint8>(val->getVal<int>());
            retValue += setNodeID(node);
            if (!retValue.containsError())
            {
                openedNodes[m_port].replace(openedNodes[m_port].indexOf(m_node), node);
                m_node = node;
            }
        }
        else if (key == "deviceID")
        {
            retValue += setExplicitDeviceID(static_cast<ito::uint16>(val->getVal<int>()));
        }
        else if (key == "torqueLimits")
        {
            int* limits = val->getVal<int*>();
            retValue += setNegativeTorqueLimit(static_cast<ito::uint16>(limits[0]));
            retValue += setPositiveTorqueLimit(static_cast<ito::uint16>(limits[1]));
        }
        else if (key == "positionLimits")
        {
            int* limits = val->getVal<int*>();
            retValue += setPositionLowerLimit(static_cast<ito::int32>(limits[0]));
            retValue += setPositionUpperLimit(static_cast<ito::int32>(limits[1]));
        }
        else if (key == "torqueGainControl")
        {
            retValue += setTorqueGainControl(static_cast<ito::uint32>(val->getVal<int>()));
        }
        else if (key == "torqueIntegralTimeControl")
        {
            retValue += setTorqueIntegralTimeControl(static_cast<ito::uint16>(val->getVal<int>()));
        }
        else if (key == "fluxGainControl")
        {
            retValue += setFluxGainControl(static_cast<ito::uint32>(val->getVal<int>()));
        }
        else if (key == "fluxIntegralTimeControl")
        {
            retValue += setFluxIntegralTimeControl(static_cast<ito::uint16>(val->getVal<int>()));
        }
        else if (key == "velocityGainControl")
        {
            retValue += setVelocityGainControl(static_cast<ito::uint32>(val->getVal<int>()));
        }
        else if (key == "velocityIntegralTimeControl")
        {
            retValue +=
                setVelocityIntegralTimeControl(static_cast<ito::uint16>(val->getVal<int>()));
        }
        else if (key == "velocityDeviationThresholdControl")
        {
            retValue += setVelocityDeviationThreshold(static_cast<ito::uint16>(val->getVal<int>()));
        }
        else if (key == "velocityDeviationTimeControl")
        {
            retValue += setVelocityDeviationTime(static_cast<ito::uint16>(val->getVal<int>()));
        }
        else if (key == "velocityWarningThresholdControl")
        {
            retValue += setVelocityWarningThreshold(static_cast<ito::uint16>(val->getVal<int>()));
        }
        else if (key == "velocityIntegralPartOption")
        {
            retValue += setVelocityIntegralPartOption(static_cast<ito::uint8>(val->getVal<int>()));
        }
        else if (key == "nominalVoltage")
        {
            retValue += setNominalVoltage(static_cast<ito::uint16>(val->getVal<int>()));
        }
        else if (key == "motionProfile")
        {
            retValue += setMotionProfileType(static_cast<ito::int16>(val->getVal<int>()));
        }

        if (!retValue.containsError())
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
ito::RetVal FaulhaberMCS::calib(const int axis, ItomSharedSemaphore* waitCond)
{
    return calib(QVector<int>(1, axis), waitCond);
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal FaulhaberMCS::calib(const QVector<int> axis, ItomSharedSemaphore* waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue = ito::retOk;

    retValue +=
        ito::RetVal(ito::retError, 0, tr("'Calib' function is not implemented.").toLatin1().data());
    if (waitCond)
    {
        waitCond->returnValue = retValue;
        waitCond->release();
    }
    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal FaulhaberMCS::readRegister(
    const ito::uint16& index, const ito::uint8& subindex, QByteArray& response)
{
    ito::RetVal retValue = ito::retOk;

    std::vector<ito::uint8> command = {
        m_node,
        m_GET,
        static_cast<ito::uint8>(index & 0xFF),
        static_cast<ito::uint8>(index >> 8),
        subindex};
    std::vector<ito::uint8> fullCommand = {static_cast<ito::uint8>(command.size() + 2)};
    fullCommand.insert(fullCommand.end(), command.begin(), command.end());

    QByteArray CRC(reinterpret_cast<const char*>(fullCommand.data()), fullCommand.size());

    fullCommand.push_back(calculateChecksum(CRC));
    fullCommand.insert(fullCommand.begin(), m_S);
    fullCommand.push_back(m_E);

    QByteArray data(reinterpret_cast<char*>(fullCommand.data()), fullCommand.size());

    return sendCommandAndGetResponse(data, response);
}

//----------------------------------------------------------------------------------------------------------------------------------
void FaulhaberMCS::setControlWord(const ito::uint16 word)
{
    setRegister<ito::uint16>(controlWord_register.index, controlWord_register.subindex, word, 2);
}

//----------------------------------------------------------------------------------------------------------------------------------
void FaulhaberMCS::shutDown()
{
    setControlWord(shutDown_register);
}

//----------------------------------------------------------------------------------------------------------------------------------
void FaulhaberMCS::enableOperation()
{
    setControlWord(enableOperation_register);
}

//----------------------------------------------------------------------------------------------------------------------------------
void FaulhaberMCS::disable()
{
    setControlWord(disable_register);
}

//----------------------------------------------------------------------------------------------------------------------------------
void FaulhaberMCS::disableVoltage()
{
    setControlWord(disableVoltage_register);
}

//----------------------------------------------------------------------------------------------------------------------------------
void FaulhaberMCS::quickStop()
{
    setControlWord(quickStop_register);
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal FaulhaberMCS::homingCurrentPosToZero(const int& axis)
{
    ito::RetVal retValue = ito::retOk;
    QElapsedTimer timer;
    timer.start();
    QMutex waitMutex;
    QWaitCondition waitCondition;
    retValue += setOperationMode(OperationMode::Homing);

    if (retValue.containsError())
    {
        retValue += ito::RetVal(
            ito::retError, 0, tr("Could not set operation mode to Homing (6).").toLatin1().data());
    }

    if (!retValue.containsError())
    {
        retValue += setHomingMode(static_cast<ito::int8>(37));

        // Start homing
        setControlWord(0x000F);
        setControlWord(0x001F);

        while (!retValue.containsWarningOrError())
        {
            waitMutex.lock();
            waitCondition.wait(&waitMutex, m_delayAfterSendCommandMS);
            waitMutex.unlock();
            setAlive();

            retValue += updateStatus();

            if (m_statusWord[12]) // homing successful
            {
                break;
            }
            else if (m_statusWord[13]) // error during
                                       // homing
            {
                retValue += ito::RetVal(
                    ito::retError, 0, tr("Error occurs during homing").toLatin1().data());
            }

            // Timeout during movement
            if (timer.hasExpired(m_waitForMCSTimeout))
            {
                retValue += ito::RetVal(
                    ito::retError,
                    9999,
                    QString("Timeout occurred during homing").toLatin1().data());
                break;
            }
        }

        ito::int32 currentPos;
        ito::int32 targetPos;
        retValue += getPosMCS(currentPos);
        m_currentPos[axis] = static_cast<double>(currentPos);

        retValue += getTargetPosMCS(targetPos);
        m_targetPos[axis] = static_cast<double>(targetPos);

        if (retValue.containsError())
            return retValue;
    }

    retValue += setOperationMode(OperationMode::ProfilePositionMode);
    retValue += updateStatus();
    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal FaulhaberMCS::performHoming(
    const ito::int8& method,
    const ito::int32& offset,
    const ito::uint32& switchSeekVelocity,
    const ito::uint32& homingSpeed,
    const ito::uint32& acceleration,
    const ito::uint16& limitCheckDelayTime,
    const ito::uint16 *torqueLimits,
    const ito::uint16& timeoutTime)
{
    ito::RetVal retValue(ito::retOk);

    if (isMotorMoving())
    {
        retValue +=
            ito::RetVal(ito::retError, 0, tr("Any motor axis is already moving").toLatin1().data());
    }
    else
    {
        resetInterrupt();
        bool homingComplete = false;
        int setPoint;
        int target;
        bool timeout = false;
        QElapsedTimer timer;
        QMutex waitMutex;
        QWaitCondition waitCondition;

        ito::int8 currentOperation;
        retValue += getOperationMode(currentOperation);

        retValue += setOperationMode(OperationMode::Homing); // change to homing mode

        // set parameters
        retValue += setHomingTorqueLimits(torqueLimits);
        retValue += setHomingOffset(offset);
        retValue += setHomingMode(method);
        retValue += setHomingSeekVelocity(switchSeekVelocity);
        retValue += setHomingSpeed(homingSpeed);
        retValue += setHomingAcceleration(acceleration);
        retValue += setHomingLimitCheckDelayTime(limitCheckDelayTime);

        setControlWord(0x000F); // homing operation start
        setControlWord(0x001F);

        for (int i = 0; i < m_numOfAxes; i++)
        {
            setStatus(
                m_currentStatus[i], ito::actuatorMoving, ito::actSwitchesMask | ito::actStatusMask);
        }
        sendStatusUpdate();

        timer.start();
        while (!homingComplete && !timeout)
        {
            if (isInterrupted())
            {
                retValue += setOperationMode(
                    currentOperation); // changing into position mode stops homing operation
                for (int i = 0; i < m_numOfAxes; i++)
                {
                    replaceStatus(
                        m_currentStatus[i], ito::actuatorMoving, ito::actuatorInterrupted);
                }
                sendStatusUpdate();
                retValue += ito::RetVal(
                    ito::retError, 0, tr("Interrupt occurred during homing.").toLatin1().data());
                return retValue;
            }

            retValue += updateStatus();
            setPoint = m_params["setPointAcknowledged"].getVal<int>();
            target = m_params["targetReached"].getVal<int>();
            if (setPoint && target)
            {
                homingComplete = true;
                m_params["homed"].setVal<int>(1);
                break;
            }
            // short delay of 10ms
            waitMutex.lock();
            waitCondition.wait(&waitMutex, m_delayAfterSendCommandMS);
            waitMutex.unlock();
            setAlive();

            if (timer.hasExpired(timeoutTime)) // timeout during movement
            {
                timeout = true;
                retValue += ito::RetVal(
                    ito::retError,
                    9999,
                    "Timeout occurred during homing. If necessary increase the parameter "
                    "'timeoutTime'.");
                for (int i = 0; i < m_numOfAxes; i++)
                {
                    replaceStatus(m_currentStatus[i], ito::actuatorMoving, ito::actuatorTimeout);
                }
                sendStatusUpdate(true);

                quickStop();
                for (int i = 0; i < m_numOfAxes; i++)
                {
                    replaceStatus(
                        m_currentStatus[i], ito::actuatorMoving, ito::actuatorInterrupted);
                }
                retValue += startupSequence();
                sendStatusUpdate();
            }
        }


        ito::int32 pos;
        for (int i = 0; i < m_numOfAxes; i++)
        {
            retValue += getPosMCS(pos);
            m_currentPos[i] = static_cast<double>(pos);
            m_targetPos[i] = static_cast<double>(0);

            replaceStatus(m_currentStatus[i], ito::actuatorMoving, ito::actuatorAtTarget);
        }

        retValue += setOperationMode(currentOperation);
    }

    sendStatusUpdate();

    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal FaulhaberMCS::setOrigin(const int axis, ItomSharedSemaphore* waitCond)
{
    return setOrigin(QVector<int>(1, axis), waitCond);
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal FaulhaberMCS::setOrigin(QVector<int> axis, ItomSharedSemaphore* waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);

    if (isMotorMoving())
    {
        retValue +=
            ito::RetVal(ito::retError, 0, tr("Any motor axis is already moving").toLatin1().data());

        if (waitCond)
        {
            waitCond->release();
            waitCond->returnValue = retValue;
        }
    }
    else
    {
        foreach (const int& i, axis)
        {
            retValue += homingCurrentPosToZero(i);
            setStatus(
                m_currentStatus[i],
                ito::actuatorAtTarget,
                ito::actSwitchesMask | ito::actStatusMask);
            m_params["homed"].setVal<int>(1);
        }

        if (waitCond)
        {
            waitCond->returnValue = retValue;
            waitCond->release();
        }

        sendStatusUpdate();
        sendTargetUpdate();
    }

    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal FaulhaberMCS::getStatus(
    QSharedPointer<QVector<int>> status, ItomSharedSemaphore* waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);

    retValue += updateStatus();
    *status = m_currentStatus;

    if (waitCond)
    {
        waitCond->returnValue = retValue;
        waitCond->release();
    }
    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal FaulhaberMCS::getPos(
    const int axis, QSharedPointer<double> pos, ItomSharedSemaphore* waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    QSharedPointer<QVector<double>> pos2(new QVector<double>(1, 0.0));
    ito::RetVal retValue =
        getPos(QVector<int>(1, axis), pos2, nullptr); // forward to multi-axes version
    *pos = (*pos2)[0];

    if (waitCond)
    {
        waitCond->returnValue = retValue;
        waitCond->release();
    }

    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal FaulhaberMCS::getPos(
    QVector<int> axis, QSharedPointer<QVector<double>> pos, ItomSharedSemaphore* waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);

    foreach (const int i, axis)
    {
        if (i >= 0 && i < m_numOfAxes)
        {
            ito::int32 intPos;
            retValue += getPosMCS(intPos);
            if (!retValue.containsError())
            {
                m_currentPos[i] =
                    static_cast<double>(intPos); // set m_currentPos[i] to the obtained position
                (*pos)[i] = m_currentPos[i];
            }
        }
        else
        {
            retValue += ito::RetVal::format(
                ito::retError, 1, tr("axis %i not available").toLatin1().data(), i);
        }
    }

    sendStatusUpdate();

    if (waitCond)
    {
        waitCond->returnValue = retValue;
        waitCond->release();
    }
    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal FaulhaberMCS::setPosAbs(const int axis, const double pos, ItomSharedSemaphore* waitCond)
{
    return setPosAbs(QVector<int>(1, axis), QVector<double>(1, pos), waitCond);
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal FaulhaberMCS::setPosAbs(
    QVector<int> axis, QVector<double> pos, ItomSharedSemaphore* waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);

    // check if pos in integer32 value range
    foreach (const auto i, axis)
    {
        if (pos[i] > std::numeric_limits<ito::int32>::max() ||
            pos[i] < std::numeric_limits<ito::int32>::min())
        {
            retValue += ito::RetVal(
                ito::retError,
                0,
                tr("Target position %1 is out of range").arg(pos[i]).toLatin1().data());
            waitCond->returnValue = retValue;
            waitCond->release();
            return retValue;
        }
    }

    if (isMotorMoving())
    {
        retValue += ito::RetVal(
            ito::retError,
            0,
            tr("motor is running. Additional actions are not possible.").toLatin1().data());
    }
    else
    {
        foreach (const int i, axis)
        {
            if (i < 0 || i >= m_numOfAxes)
            {
                retValue += ito::RetVal::format(
                    ito::retError, 1, tr("axis %i not available").toLatin1().data(), i);
            }
            else
            {
                m_targetPos[i] = pos[i];
            }
        }

        if (retValue.containsError())
        {
            if (waitCond)
            {
                waitCond->returnValue = retValue;
                waitCond->release();
            }
        }
        else
        {
            setStatus(axis, ito::actuatorMoving, ito::actSwitchesMask | ito::actStatusMask);
            sendStatusUpdate();

            foreach (const int i, axis)
            {
                ito::int32 newVal = static_cast<ito::int32>(pos[i]);
                ito::int16 voltage;
                switch (m_params["operationMode"].getVal<int>())
                {
                case -1:
                    voltage = static_cast<ito::int16>(newVal);
                    retValue += setVoltageMCS(voltage);
                    break;
                case 1: // position mode
                    retValue += setPosAbsMCS(newVal);
                    break;
                case 3: // velocity mode
                    retValue += setVelocityMCS(newVal);
                    break;
                case 10: // cyclic synchronous torque control
                    retValue += setTorqueMCS(newVal);
                    break;
                }

                m_targetPos[i] = static_cast<ito::float64>(newVal);
            }

            sendTargetUpdate();

            if (m_async && waitCond) // async disabled
            {
                waitCond->returnValue = retValue;
                waitCond->release();
            }

            retValue += waitForDone(m_waitForDoneTimeout, axis); // drops into timeout

            replaceStatus(axis, ito::actuatorMoving, ito::actuatorAtTarget);
            sendStatusUpdate();

            if (!m_async && waitCond)
            {
                waitCond->returnValue = retValue;
                waitCond->release();
            }
        }
    }


    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal FaulhaberMCS::setPosRel(const int axis, const double pos, ItomSharedSemaphore* waitCond)
{
    return setPosRel(QVector<int>(1, axis), QVector<double>(1, pos), waitCond);
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal FaulhaberMCS::setPosRel(
    QVector<int> axis, QVector<double> pos, ItomSharedSemaphore* waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);

    // check if pos in integer32 value range
    foreach (const auto i, axis)
    {
        if (pos[i] > std::numeric_limits<ito::int32>::max() ||
            pos[i] < std::numeric_limits<ito::int32>::min())
        {
            retValue += ito::RetVal(
                ito::retError,
                0,
                tr("Relative position %1 is out of range").arg(pos[i]).toLatin1().data());
            waitCond->returnValue = retValue;
            waitCond->release();
            return retValue;
        }
    }

    if (isMotorMoving())
    {
        retValue += ito::RetVal(
            ito::retError,
            0,
            tr("motor is running. Additional actions are not possible.").toLatin1().data());
    }
    else
    {
        foreach (const int i, axis)
        {
            if (i < 0 || i >= m_numOfAxes)
            {
                retValue += ito::RetVal::format(
                    ito::retError, 1, tr("axis %i not available").toLatin1().data(), i);
            }
            else
            {
                m_targetPos[i] = m_currentPos[i] + pos[i];
            }
        }

        if (m_params["operationMode"].getVal<int>() != 1)
        {
            retValue += ito::RetVal(
                ito::retError,
                0,
                tr("Operation mode must be set to 'Position Mode' to set relative position.")
                    .toLatin1()
                    .data());
        }

        if (retValue.containsError())
        {
            if (waitCond)
            {
                waitCond->returnValue = retValue;
                waitCond->release();
            }
        }
        else
        {
            setStatus(axis, ito::actuatorMoving, ito::actSwitchesMask | ito::actStatusMask);
            sendStatusUpdate();

            foreach (const int i, axis)
            {
                ito::int32 newPos = static_cast<ito::int32>(pos[i]);
                retValue += setPosRelMCS(newPos);
                m_currentPos[i] = static_cast<ito::float64>(newPos);
            }

            sendTargetUpdate();

            if (m_async && waitCond) // async disabled
            {
                waitCond->returnValue = retValue;
                waitCond->release();
            }

            retValue += waitForDone(m_waitForDoneTimeout, axis); // drops into timeout

            replaceStatus(axis, ito::actuatorMoving, ito::actuatorAtTarget);
            sendStatusUpdate();

            if (!m_async && waitCond)
            {
                waitCond->returnValue = retValue;
                waitCond->release();
            }
        }
    }

    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal FaulhaberMCS::execFunc(
    const QString funcName,
    QSharedPointer<QVector<ito::ParamBase>> paramsMand,
    QSharedPointer<QVector<ito::ParamBase>> paramsOpt,
    QSharedPointer<QVector<ito::ParamBase>> paramsOut,
    ItomSharedSemaphore* waitCond)
{
    ito::RetVal retValue = ito::retOk;

    if (funcName == "homing")
    {
        retValue += updateStatus();

        if (!m_params["operationEnabled"].getVal<int>())
        {
            retValue += ito::RetVal(
                ito::retError,
                0,
                tr("Operation is not enabled. Please enable operation first.").toLatin1().data());
        }

        if (!retValue.containsError())
        {
            ito::int8 method = static_cast<ito::int8>((*paramsMand)[0].getVal<int>());
            ito::int32 offset = static_cast<ito::int32>((*paramsOpt)[0].getVal<int>());
            ito::uint32 switchSeekVelocity =
                static_cast<ito::uint32>((*paramsOpt)[1].getVal<int>());
            ito::uint32 homingSpeed = static_cast<ito::uint32>((*paramsOpt)[2].getVal<int>());
            ito::uint32 acceleration = static_cast<ito::uint32>((*paramsOpt)[3].getVal<int>());
            ito::uint16 limitCheckDelayTime =
                static_cast<ito::uint16>((*paramsOpt)[4].getVal<int>());

            ito::uint16 negativeLimit = static_cast<ito::uint16>((*paramsOpt)[5].getVal<int*>()[0]);
            ito::uint16 positiveLimit = static_cast<ito::uint16>((*paramsOpt)[5].getVal<int*>()[1]);
            ito::uint16 timeout = static_cast<ito::uint16>((*paramsOpt)[6].getVal<int>());
            ito::uint16 torqueLimits[] = {negativeLimit, positiveLimit};

            retValue += performHoming(
                method,
                offset,
                switchSeekVelocity,
                homingSpeed,
                acceleration,
                limitCheckDelayTime,
                torqueLimits,
                timeout);
        }
    }

    if (waitCond)
    {
        waitCond->returnValue = retValue;
        waitCond->release();
        waitCond->deleteSemaphore();
    }

    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal FaulhaberMCS::getSerialNumber(QString& serialNum)
{
    ito::uint32 c;
    ito::RetVal retVal = readRegisterWithParsedResponse<ito::uint32>(
        serialNumber_register.index, serialNumber_register.subindex, c);
    serialNum = QString::number(c);
    return retVal;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal FaulhaberMCS::getDeviceName(QString& name)
{
    return readRegisterWithParsedResponse<QString>(
        deviceName_register.index, deviceName_register.subindex, name);
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal FaulhaberMCS::getVendorID(QString& id)
{
    ito::uint32 c;
    ito::RetVal retVal = readRegisterWithParsedResponse<ito::uint32>(
        vendorID_register.index, vendorID_register.subindex, c);
    id = QString::number(c);
    return retVal;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal FaulhaberMCS::getProductCode(QString& code)
{
    ito::uint32 c;
    ito::RetVal retVal = readRegisterWithParsedResponse<ito::uint32>(
        productCode_register.index, productCode_register.subindex, c);
    code = QString::number(c);
    return retVal;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal FaulhaberMCS::getRevisionNumber(QString& num)
{
    ito::uint32 c;
    ito::RetVal retVal = readRegisterWithParsedResponse<ito::uint32>(
        revisionNumber_register.index, revisionNumber_register.subindex, c);
    num = QString::number(c);
    return retVal;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal FaulhaberMCS::getFirmware(QString& version)
{
    return readRegisterWithParsedResponse<QString>(
        firmwareVersion_register.index, firmwareVersion_register.subindex, version);
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal FaulhaberMCS::getCPUTemperature(ito::int16& temp)
{
    return readRegisterWithParsedResponse<ito::int16>(
        CPUTemperature_register.index, CPUTemperature_register.subindex, temp);
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal FaulhaberMCS::getPowerStageTemperature(ito::int16& temp)
{
    return readRegisterWithParsedResponse<ito::int16>(
        powerStageTemperature_register.index, powerStageTemperature_register.subindex, temp);
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal FaulhaberMCS::getWindingTemperature(ito::int16& temp)
{
    return readRegisterWithParsedResponse<ito::int16>(
        windingTemperature_register.index, windingTemperature_register.subindex, temp);
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal FaulhaberMCS::getLoadInertia(ito::uint32& inertia)
{
    return readRegisterWithParsedResponse<ito::uint32>(
        loadInertia_register.index, loadInertia_register.subindex, inertia);
}

ito::RetVal FaulhaberMCS::setLoadInertia(const ito::uint32& inertia)
{
    return setRegister<ito::uint32>(
        loadInertia_register.index, loadInertia_register.subindex, inertia, sizeof(inertia));
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal FaulhaberMCS::getNodeID(ito::uint8& id)
{
    return readRegisterWithParsedResponse<ito::uint8>(
        nodeID_register.index, nodeID_register.subindex, id);
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal FaulhaberMCS::setNodeID(const ito::uint8& id)
{
    return setRegister<ito::uint8>(nodeID_register.index, nodeID_register.subindex, id, sizeof(id));
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal FaulhaberMCS::getExplicitDeviceID(ito::uint16& id)
{
    return readRegisterWithParsedResponse<ito::uint16>(
        deviceID_register.index, deviceID_register.subindex, id);
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal FaulhaberMCS::setExplicitDeviceID(const ito::uint16& id)
{
    return setRegister<ito::uint16>(
        deviceID_register.index, deviceID_register.subindex, id, sizeof(id));
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal FaulhaberMCS::getPosMCS(ito::int32& pos)
{
    return readRegisterWithParsedResponse<ito::int32>(
        positionActualValue_register.index, positionActualValue_register.subindex, pos);
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal FaulhaberMCS::getTargetPosMCS(ito::int32& pos)
{
    return readRegisterWithParsedResponse<ito::int32>(
        positionTargetValue_register.index, positionTargetValue_register.subindex, pos);
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal FaulhaberMCS::getMaxMotorSpeed(ito::uint32& speed)
{
    return readRegisterWithParsedResponse<ito::uint32>(
        maxMotorSpeed_register.index, maxMotorSpeed_register.subindex, speed);
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal FaulhaberMCS::setMaxMotorSpeed(const ito::uint32& speed)
{
    return setRegister<ito::uint32>(
        maxMotorSpeed_register.index, maxMotorSpeed_register.subindex, speed, sizeof(speed));
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal FaulhaberMCS::getAcceleration(ito::uint32& acceleration)
{
    return readRegisterWithParsedResponse<ito::uint32>(
        acceleration_register.index, acceleration_register.subindex, acceleration);
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal FaulhaberMCS::setAcceleration(const ito::uint32& acceleration)
{
    return setRegister<ito::uint32>(
        acceleration_register.index,
        acceleration_register.subindex,
        acceleration,
        sizeof(acceleration));
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal FaulhaberMCS::getDeceleration(ito::uint32& deceleration)
{
    return readRegisterWithParsedResponse<ito::uint32>(
        deceleration_register.index, deceleration_register.subindex, deceleration);
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal FaulhaberMCS::setDeceleration(const ito::uint32& deceleration)
{
    return setRegister<ito::uint32>(
        deceleration_register.index,
        deceleration_register.subindex,
        deceleration,
        sizeof(deceleration));
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal FaulhaberMCS::getQuickStopDeceleration(ito::uint32& deceleration)
{
    return readRegisterWithParsedResponse<ito::uint32>(
        quickStopDeceleration_register.index,
        quickStopDeceleration_register.subindex,
        deceleration);
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal FaulhaberMCS::setQuickStopDeceleration(const ito::uint32& deceleration)
{
    return setRegister<ito::uint32>(
        quickStopDeceleration_register.index,
        quickStopDeceleration_register.subindex,
        deceleration,
        sizeof(deceleration));
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal FaulhaberMCS::getProfileVelocity(ito::uint32& speed)
{
    return readRegisterWithParsedResponse<ito::uint32>(
        profileVelocity_register.index, profileVelocity_register.subindex, speed);
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal FaulhaberMCS::setProfileVelocity(const ito::uint32& speed)
{
    ito::RetVal retVal = ito::retOk;
    ito::uint32 maxSpeed = static_cast<ito::uint32>(m_params["maxMotorSpeed"].getVal<int>());
    if (speed > maxSpeed)
    {
        retVal += setRegister<ito::uint32>(
            profileVelocity_register.index,
            profileVelocity_register.subindex,
            maxSpeed,
            sizeof(maxSpeed));
        retVal += ito::RetVal(
            ito::retWarning,
            0,
            tr("Speed is higher than maxMotorSpeed. Speed is set to maxMotorSpeed of value "
               "'%1'.")
                .arg(maxSpeed)
                .toLatin1()
                .data());
    }
    else
    {
        retVal += setRegister<ito::uint32>(
            profileVelocity_register.index,
            profileVelocity_register.subindex,
            speed,
            sizeof(speed));
    }
    return retVal;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal FaulhaberMCS::getOperationMode(ito::int8& mode)
{
    return readRegisterWithParsedResponse<ito::int8>(
        operationMode_register.index, operationMode_register.subindex, mode);
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal FaulhaberMCS::setOperationMode(const ito::int8& mode)
{
    return setRegister<ito::int8>(
        operationMode_register.index, operationMode_register.subindex, mode, sizeof(mode));
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal FaulhaberMCS::getMaxTorqueLimit(ito::uint16& limit)
{
    return readRegisterWithParsedResponse<ito::uint16>(
        maxTorqueLimit_register.index, maxTorqueLimit_register.subindex, limit);
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal FaulhaberMCS::setMaxTorqueLimit(const ito::uint16 limit)
{
    return setRegister<ito::uint16>(
        maxTorqueLimit_register.index, maxTorqueLimit_register.subindex, limit, sizeof(limit));
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal FaulhaberMCS::getTorqueMCS(ito::int16& torque)
{
    return readRegisterWithParsedResponse<ito::int16>(
        torqueActualValue_register.index, torqueActualValue_register.subindex, torque);
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal FaulhaberMCS::setTorqueMCS(const ito::int16 torque)
{
    return setRegister<ito::int16>(
        torqueTargetValue_register.index,
        torqueTargetValue_register.subindex,
        torque,
        sizeof(torque));
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal FaulhaberMCS::getTargetTorqueMCS(ito::int16& torque)
{
    return readRegisterWithParsedResponse<ito::int16>(
        torqueTargetValue_register.index, torqueTargetValue_register.subindex, torque);
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal FaulhaberMCS::getVoltageMCS(ito::int16& current)
{
    return readRegisterWithParsedResponse<ito::int16>(
        voltageValue_register.index, voltageValue_register.subindex, current);
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal FaulhaberMCS::setVoltageMCS(ito::int16& current)
{
    return setRegister<ito::int16>(
        voltageValue_register.index, voltageValue_register.subindex, current, sizeof(current));
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal FaulhaberMCS::getMotionProfileType(ito::int16& type)
{
    return readRegisterWithParsedResponse<ito::int16>(
        motionProfileType_register.index, motionProfileType_register.subindex, type);
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal FaulhaberMCS::setMotionProfileType(const ito::int16& type)
{
    return setRegister<ito::int16>(
        motionProfileType_register.index,
        motionProfileType_register.subindex,
        type, sizeof(type));
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal FaulhaberMCS::getNetMode(ito::uint8& mode)
{
    return readRegisterWithParsedResponse<ito::uint8>(
        netMode_register.index, netMode_register.subindex, mode);
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal FaulhaberMCS::setNetMode(const ito::uint8& mode)
{
    return setRegister<ito::uint8>(
        netMode_register.index, netMode_register.subindex, mode, sizeof(mode));
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal FaulhaberMCS::getTorque(ito::int16& torque)
{
    return readRegisterWithParsedResponse<ito::int16>(
        torqueActualValue_register.index, torqueActualValue_register.subindex, torque);
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal FaulhaberMCS::getCurrent(ito::int16& current)
{
    return readRegisterWithParsedResponse<ito::int16>(
        currentActualValue_register.index, currentActualValue_register.subindex, current);
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal FaulhaberMCS::getNegativeTorqueLimit(ito::uint16& limit)
{
    return readRegisterWithParsedResponse<ito::uint16>(
        negativeTorqueLimit_register.index, negativeTorqueLimit_register.subindex, limit);
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal FaulhaberMCS::setNegativeTorqueLimit(const ito::uint16 limit)
{
    return setRegister<ito::uint16>(
        negativeTorqueLimit_register.index,
        negativeTorqueLimit_register.subindex,
        limit,
        sizeof(limit));
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal FaulhaberMCS::getPositveTorqueLimit(ito::uint16& limit)
{
    return readRegisterWithParsedResponse<ito::uint16>(
        positiveTorqueLimit_register.index, positiveTorqueLimit_register.subindex, limit);
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal FaulhaberMCS::setPositiveTorqueLimit(const ito::uint16 limit)
{
    return setRegister<ito::uint16>(
        positiveTorqueLimit_register.index,
        positiveTorqueLimit_register.subindex,
        limit,
        sizeof(limit));
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal FaulhaberMCS::getPositionLowerLimit(ito::int32& limit)
{
    return readRegisterWithParsedResponse<ito::int32>(
        positionLowerLimit_register.index, positionLowerLimit_register.subindex, limit);
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal FaulhaberMCS::setPositionLowerLimit(const ito::int32 limit)
{
    return setRegister<ito::int32>(
        positionLowerLimit_register.index,
        positionLowerLimit_register.subindex,
        limit,
        sizeof(limit));
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal FaulhaberMCS::getPositionUpperLimit(ito::int32& limit)
{
    return readRegisterWithParsedResponse<ito::int32>(
        positionUpperLimit_register.index, positionUpperLimit_register.subindex, limit);
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal FaulhaberMCS::setPositionUpperLimit(const ito::int32 limit)
{
    return setRegister<ito::int32>(
        positionUpperLimit_register.index,
        positionUpperLimit_register.subindex,
        limit,
        sizeof(limit));
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal FaulhaberMCS::getNominalVoltage(ito::uint16& voltage)
{
    return readRegisterWithParsedResponse<ito::uint16>(
        nominalVoltage_register.index, nominalVoltage_register.subindex, voltage);
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal FaulhaberMCS::setNominalVoltage(const ito::uint16 voltage)
{
    return setRegister<ito::uint16>(
        nominalVoltage_register.index, nominalVoltage_register.subindex, voltage, sizeof(voltage));
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal FaulhaberMCS::getTorqueGainControl(ito::uint32& gain)
{
    return readRegisterWithParsedResponse<ito::uint32>(
        torqueGainControl_register.index, torqueGainControl_register.subindex, gain);
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal FaulhaberMCS::setTorqueGainControl(const ito::uint32 gain)
{
    return setRegister<ito::uint32>(
        torqueGainControl_register.index, torqueGainControl_register.subindex, gain, sizeof(gain));
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal FaulhaberMCS::getTorqueIntegralTimeControl(ito::uint16& time)
{
    return readRegisterWithParsedResponse<ito::uint16>(
        torqueIntegralTimeControl_register.index,
        torqueIntegralTimeControl_register.subindex,
        time);
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal FaulhaberMCS::setTorqueIntegralTimeControl(const ito::uint16 time)
{
    return setRegister<ito::uint16>(
        torqueIntegralTimeControl_register.index,
        torqueIntegralTimeControl_register.subindex,
        time,
        sizeof(time));
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal FaulhaberMCS::getFluxGainControl(ito::uint32& gain)
{
    return readRegisterWithParsedResponse<ito::uint32>(
        fluxGainControl_register.index, fluxGainControl_register.subindex, gain);
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal FaulhaberMCS::setFluxGainControl(const ito::uint32 gain)
{
    return setRegister<ito::uint32>(
        fluxGainControl_register.index, fluxGainControl_register.subindex, gain, sizeof(gain));
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal FaulhaberMCS::getFluxIntegralTimeControl(ito::uint16& time)
{
    return readRegisterWithParsedResponse<ito::uint16>(
        fluxIntegralTimeControl_register.index, fluxIntegralTimeControl_register.subindex, time);
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal FaulhaberMCS::setFluxIntegralTimeControl(const ito::uint16 time)
{
    return setRegister<ito::uint16>(
        fluxIntegralTimeControl_register.index,
        fluxIntegralTimeControl_register.subindex,
        time,
        sizeof(time));
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal FaulhaberMCS::getVelocityGainControl(ito::uint32& gain)
{
    return readRegisterWithParsedResponse<ito::uint32>(
        velocityGainControl_register.index, velocityGainControl_register.subindex, gain);
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal FaulhaberMCS::setVelocityGainControl(const ito::uint32 gain)
{
    return setRegister<ito::uint32>(
        velocityGainControl_register.index,
        velocityGainControl_register.subindex,
        gain,
        sizeof(gain));
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal FaulhaberMCS::getVelocityIntegralTimeControl(ito::uint16& time)
{
    return readRegisterWithParsedResponse<ito::uint16>(
        velocityIntegralTimeControl_register.index,
        velocityIntegralTimeControl_register.subindex,
        time);
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal FaulhaberMCS::setVelocityIntegralTimeControl(const ito::uint16 time)
{
    return setRegister<ito::uint16>(
        velocityIntegralTimeControl_register.index,
        velocityIntegralTimeControl_register.subindex,
        time,
        sizeof(time));
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal FaulhaberMCS::getVelocityDeviationThreshold(ito::uint16& time)
{
    return readRegisterWithParsedResponse<ito::uint16>(
        velocityDeviationThreshold_register.index,
        velocityDeviationThreshold_register.subindex,
        time);
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal FaulhaberMCS::setVelocityDeviationThreshold(const ito::uint16 time)
{
    return setRegister<ito::uint16>(
        velocityDeviationThreshold_register.index,
        velocityDeviationThreshold_register.subindex,
        time,
        sizeof(time));
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal FaulhaberMCS::getVelocityDeviationTime(ito::uint16& time)
{
    return readRegisterWithParsedResponse<ito::uint16>(
        velocityDeviationTime_register.index, velocityDeviationTime_register.subindex, time);
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal FaulhaberMCS::setVelocityDeviationTime(const ito::uint16 time)
{
    return setRegister<ito::uint16>(
        velocityDeviationTime_register.index,
        velocityDeviationTime_register.subindex,
        time,
        sizeof(time));
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal FaulhaberMCS::getVelocityWarningThreshold(ito::uint16& thres)
{
    return readRegisterWithParsedResponse<ito::uint16>(
        velocityWarningThreshold_register.index, velocityWarningThreshold_register.subindex, thres);
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal FaulhaberMCS::setVelocityWarningThreshold(const ito::uint16 thres)
{
    return setRegister<ito::uint16>(
        velocityWarningThreshold_register.index,
        velocityWarningThreshold_register.subindex,
        thres,
        sizeof(thres));
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal FaulhaberMCS::getVelocityIntegralPartOption(ito::uint8& option)
{
    return readRegisterWithParsedResponse<ito::uint8>(
        velocityIntegralPartOption.index, velocityIntegralPartOption.subindex, option);
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal FaulhaberMCS::setVelocityIntegralPartOption(const ito::uint8 option)
{
    return setRegister<ito::uint8>(
        velocityIntegralPartOption.index,
        velocityIntegralPartOption.subindex,
        option,
        sizeof(option));
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal FaulhaberMCS::setHomingOffset(const ito::int32& offset)
{
    return setRegister<ito::int32>(
        homingOffset_register.index, homingOffset_register.subindex, offset, sizeof(offset));
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal FaulhaberMCS::setHomingSpeed(const ito::uint32& speed)
{
    return setRegister<ito::uint32>(
        homingSpeed_register.index, homingSpeed_register.subindex, speed, sizeof(speed));
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal FaulhaberMCS::setHomingSeekVelocity(const ito::uint32& seek)
{
    return setRegister<ito::uint32>(
        homingSeekVelocity_register.index,
        homingSeekVelocity_register.subindex,
        seek,
        sizeof(seek));
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal FaulhaberMCS::setHomingAcceleration(const ito::uint32& acceleration)
{
    return setRegister<ito::uint32>(
        homingAcceleration_register.index,
        homingAcceleration_register.subindex,
        acceleration,
        sizeof(acceleration));
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal FaulhaberMCS::setHomingLimitCheckDelayTime(const ito::uint16& time)
{
    return setRegister<ito::uint16>(
        homingLimitCheckDelayTime_register.index,
        homingLimitCheckDelayTime_register.subindex,
        time,
        sizeof(time));
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal FaulhaberMCS::setHomingTorqueLimits(const ito::uint16 limits[])
{
    ito::RetVal retVal = ito::retOk;
    retVal += setRegister<ito::uint16>(
        homingNegativeTorqueLimit_register.index,
        homingNegativeTorqueLimit_register.subindex,
        limits[0],
        sizeof(limits[0]));
    retVal += setRegister<ito::uint16>(
        homingPositiveTorqueLimit_register.index,
        homingPositiveTorqueLimit_register.subindex,
        limits[1],
        sizeof(limits[1]));
    return retVal;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal FaulhaberMCS::updateStatusMCS()
{
    ito::RetVal retVal = readRegisterWithParsedResponse<ito::uint16>(
        statusWord_register.index, statusWord_register.subindex, m_statusWordValue);
    m_statusWord = static_cast<std::bitset<16>>(m_statusWordValue);

    if (!retVal.containsError())
    {
        QMutex waitMutex;
        QWaitCondition waitCondition;
        qint64 startTime = QDateTime::currentMSecsSinceEpoch();
        qint64 elapsedTime = 0;
        while ((m_statusWord == 0) && (elapsedTime < m_waitForMCSTimeout))
        {
            //  short delay
            waitMutex.lock();
            waitCondition.wait(&waitMutex, m_delayAfterSendCommandMS);
            waitMutex.unlock();
            setAlive();

            retVal += readRegisterWithParsedResponse<ito::uint16>(
                statusWord_register.index, statusWord_register.subindex, m_statusWordValue);
            m_statusWord = static_cast<std::bitset<16>>(m_statusWordValue);

            if (retVal.containsError())
            {
                retVal += ito::RetVal(
                    ito::retError,
                    0,
                    tr("Error occurred during reading statusword with value %1")
                        .arg(m_statusWordValue)
                        .toLatin1()
                        .data());
                break;
            }
            elapsedTime = QDateTime::currentMSecsSinceEpoch() - startTime;
        }

        if (!retVal.containsError())
        {
            updateStatusBits();
            emit parametersChanged(m_params);
        }
        sendStatusUpdate();
    }

    return retVal;
}

//----------------------------------------------------------------------------------------------------------------------------------
void FaulhaberMCS::updateStatusBits()
{
    m_params["readyToSwitchOn"].setVal<int>((m_statusWord[0] ? true : false));
    m_params["switchedOn"].setVal<int>((m_statusWord[1] ? true : false));
    m_params["operationEnabled"].setVal<int>((m_statusWord[2] ? true : false));
    m_params["fault"].setVal<int>((m_statusWord[3] ? true : false));
    m_params["voltageEnabled"].setVal<int>((m_statusWord[4] ? true : false));
    m_params["quickStop"].setVal<int>((m_statusWord[5] ? true : false));
    m_params["switchOnDisabled"].setVal<int>((m_statusWord[6] ? true : false));
    m_params["warning"].setVal<int>((m_statusWord[7] ? true : false));
    m_params["targetReached"].setVal<int>((m_statusWord[10] ? true : false));
    m_params["internalLimitActive"].setVal<int>((m_statusWord[11] ? true : false));
    m_params["setPointAcknowledged"].setVal<int>((m_statusWord[12] ? true : false));
    m_params["followingError"].setVal<int>((m_statusWord[13] ? true : false));
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal FaulhaberMCS::setCommunicationSettings(const ito::uint32& settings)
{
    return setRegister<ito::int8>(
        communicationSettings_register.index,
        communicationSettings_register.subindex,
        settings,
        sizeof(settings));
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal FaulhaberMCS::getCommunicationSettings(ito::uint32& settings)
{
    return readRegisterWithParsedResponse<ito::uint32>(
        communicationSettings_register.index, communicationSettings_register.subindex, settings);
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal FaulhaberMCS::getError()
{
    ito::RetVal retVal = ito::retOk;
    ito::uint16 error;
    retVal += readRegisterWithParsedResponse<ito::uint16>(
        error_register.index, error_register.subindex, error);
    return interpretEMCYError(error);
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal FaulhaberMCS::setPosAbsMCS(const ito::int32& pos)
{
    ito::RetVal retVal = ito::retOk;
    setRegister<ito::int32>(
        positionAbsolutValue_register.index,
        positionAbsolutValue_register.subindex,
        pos,
        sizeof(pos));
    setControlWord(0x000f);
    setControlWord(0x003F);
    return updateStatus();
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal FaulhaberMCS::setPosRelMCS(const ito::int32& pos)
{
    ito::RetVal retVal = ito::retOk;
    setRegister<ito::int32>(
        positionRelativeValue_register.index,
        positionRelativeValue_register.subindex,
        pos,
        sizeof(pos));
    setControlWord(0x000f);
    setControlWord(0x007F);
    return updateStatus();
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal FaulhaberMCS::getVelocityMCS(ito::int32& pos)
{
    return readRegisterWithParsedResponse<ito::int32>(
        velocityActualValue_register.index, velocityActualValue_register.subindex, pos);
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal FaulhaberMCS::setVelocityMCS(const ito::int32& pos)
{
    ito::RetVal retVal = ito::retOk;
    setRegister<ito::int32>(
        velocityTargetValue_register.index,
        velocityTargetValue_register.subindex,
        pos,
        sizeof(pos));
    setControlWord(0x000f);
    setControlWord(0x003F);
    return updateStatus();
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal FaulhaberMCS::getTargetVelocityMCS(ito::int32& pos)
{
    return readRegisterWithParsedResponse<ito::int32>(
        velocityTargetValue_register.index, velocityTargetValue_register.subindex, pos);
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal FaulhaberMCS::setHomingMode(const ito::int8& mode)
{
    return setRegister<ito::int8>(
        homingMode_register.index, homingMode_register.subindex, mode, sizeof(mode));
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal FaulhaberMCS::startupSequence()
{
    // Initial check of the status word
    ito::RetVal retValue = updateStatus();
    int enableStatus = 0; // Reset the local step counter

    if (retValue.containsError())
    {
        return retValue;
    }

    const ito::uint16 statusMask = 0x6F;
    ito::uint16 status = m_statusWordValue & statusMask;

    // Check for being in stopped mode
    if (m_params["quickStop"].getVal<int>())
    {
        enableOperation(); // Enable Operation
        enableStatus = 1;
    }
    else if (m_params["operationEnabled"].getVal<int>()) // Drive is already
                                                         // enabled
    {
        enableStatus = 2;
    }
    else if (m_params["switchOnDisabled"].getVal<int>()) // Otherwise, it's safe to
                                                         // disable first
    {
        // We need to send a shutdown first
        disableVoltage(); // Controlword = CiACmdDisableVoltage
    }

    QElapsedTimer timer;
    bool timeout = false;
    QMutex waitMutex;
    QWaitCondition waitCondition;

    // Loop until the drive is enabled
    timer.start();
    while (enableStatus != 2 && !timeout)
    {
        retValue += updateStatus();
        if (retValue.containsError())
        {
            return retValue;
        }

        status = m_statusWordValue & statusMask; // Cyclically check the status word

        if (enableStatus == 0)
        {
            if (status == 0x40)
            {
                // Send the enable signature
                shutDown(); // CiACmdShutdown
                enableOperation(); // CiACmdEnableOperation
                enableStatus = 1;
            }
        }
        else if (enableStatus == 1)
        {
            // Wait for enabled
            if (m_params["operationEnabled"].getVal<int>())
            {
                enableStatus = 2;
            }
        }

        // short delay
        waitMutex.lock();
        waitCondition.wait(&waitMutex, 10);
        waitMutex.unlock();

        if (timer.hasExpired(m_waitForDoneTimeout)) // timeout during movement
        {
            timeout = true;
            retValue += ito::RetVal(ito::retError, 9999, "Timeout occurred during initialization.");
        }
    }
    m_params["operation"].setVal<int>(1);
    m_params["power"].setVal<int>(1);

    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal FaulhaberMCS::shutDownSequence()
{
    ito::RetVal retValue = updateStatus();

    int disableState = 0; // Reset the local step counter

    const ito::uint16 statusMask = 0x6F;
    ito::uint16 status = m_statusWordValue & statusMask;

    if (m_params["operationEnabled"].getVal<int>())
    {
        // Send a shutdown command first to stop the motor
        disable();
        disableState = 1;
    }
    else
    {
        // Otherwise, the disable voltage is the next command
        // Out of quick-stop or switched on.
        disableState = 2;
    }

    QElapsedTimer timer;
    QMutex waitMutex;
    QWaitCondition waitCondition;
    bool timeout = false;

    // Loop until the disable operation is complete
    timer.start();
    while (disableState != 4 && !timeout)
    {
        retValue += updateStatus();
        if (retValue.containsError())
        {
            return retValue;
        }

        status = m_statusWordValue & statusMask; // Cyclically check the status

        if (disableState == 1)
        {
            if (status == 0x23)
            {
                // Only now it's safe to send the disable voltage command
                disableState = 2;
            }
        }
        else if (disableState == 2)
        {
            // Send the disable voltage command
            disableVoltage();
            disableState = 3;
        }
        else if (disableState == 3)
        {
            // Wait for final state
            if (status == 0x40)
            {
                disableState = 4;
            }
        }

        // Optional: Include a small delay to prevent busy looping
        waitMutex.lock();
        waitCondition.wait(&waitMutex, 10);
        waitMutex.unlock();

        if (timer.hasExpired(m_waitForDoneTimeout)) // timeout during movement
        {
            timeout = true;
            retValue += ito::RetVal(ito::retError, 9999, "Timeout occurred during closing.");
        }
    }
    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal FaulhaberMCS::interpretEMCYError(const ito::uint16& errorCode)
{
    ito::RetVal retVal;

    // Use QMap instead of std::unordered_map for error details
    QMap<ito::uint16, ErrorInfo> errorMap = {
        {0x0001,
         {"Speed Deviation Error",
          "Diagnostics detected a speed deviation. Monitoring is configured in object 0x2344.",
          0x84F0}},
        {0x0002,
         {"Following Error",
          "Diagnostics detected a following error (deviationbetween position set-point and "
          "position actual value). The following error monitoring is configured via objects "
          "0x6065 "
          "and 0x6066.",
          0x8611}},
        {0x0004,
         {"Over Voltage Error",
          "Overvoltage for one of the supplies. The drive is switched off by this error.",
          0x3210}},
        {0x0008,
         {"Under Voltage Error",
          "At least one of the voltage supplies is reported as too low.",
          0x3220}},
        {0x0010,
         {"Temperature Warning",
          "The current set-points are limited to the set continuous current by the thermal "
          "model.",
          0x2310}},
        {0x0020,
         {"Temperature Error",
          "At least one of the temperature switch-off limits was reached.The drive is switched "
          "off "
          "by this error.",
          0x4310}},
        {0x0040,
         {"Encoder Error",
          "- Analog Hall: The amplitudes of the two or three Hall signals are not sufficiently "
          "equal for a period of time. This results in uneven running. - Digital Hall: Invalid "
          "combination of Hall signals detected. - AES encoder: CRC returns an error.",
          0x7300}},
        {0x0080,
         {"Internal Hardware Error",
          "At least one digital output does not have the expected level and was passively "
          "switched "
          "back.",
          0x5410}},
        {0x0200,
         {"Current Measurement Error",
          "Current measurement indicates an error. The current sum of the three channels is "
          "not "
          "equal to 0. Possible causes : - Fault current via a winding - housing short circuit "
          "- "
          "Motor and controller may not be compatible with respect to current measurement "
          "range "
          "and the rated current of the motor.",
          0x7200}},
        {0x0800, {"Communication Error", "CAN reports", 0x8110}}, // Multiple possible codes
        {0x1000, {"Calculation Error", "Error in the execution of a BASIC script.", 0xFF20}},
        {0x2000,
         {"Dynamic Limit Error",
          "The speed was in excess of the warning threshold configured in object 0x2344.",
          0x84FF}},
        {0x4000,
         {"Safety Monitor Error", "Shutdown of the output stage by the STO function.", 0x5480}},
        {0x0000, {"No Error", "No error present", 0x0000}}};

    QMap<ito::uint16, QString> communicationErrorMap = {
        {0x8110, "CANOverrun: CAN reports an overflow of the receive buffer."},
        {0x8130, "CANGuardingFailed: CAN node guarding or CAN heartbeat failed."},
        {0x8140, "CANRecoveredFromBusOff: The CAN controller recovered after bus-off."},
        {0x8141, "CANBusOff: The CAN controller left the bus after too many error frames."},
        {0x8210, "CANPDOLength: A PDO with an incompatible length was received."},
        {0x8310, "RS232Overrun: The RS232 stack was unable to save a message."},
        {0x8101, "CANInitError: CAN stack could not be initialized."}};

    if (errorMap.contains(0x0000))
    {
        retVal += ito::retOk;
    }
    else if (errorMap.contains(0x0800))
    {
        QString result;
        for (auto it = communicationErrorMap.constBegin(); it != communicationErrorMap.constEnd();
             ++it)
        {
            result += "0x" + QString::number(it.key(), 16).toUpper() + ": " + it.value() + "\n";
        }
        retVal += ito::RetVal(ito::retError, errorCode, result.toLatin1().data());
    }
    else if (errorMap.contains(errorCode))
    {
        retVal += ito::RetVal(
            ito::retError, errorCode, errorMap[errorCode].shortDescription.toLatin1().data());
    }
    else
    {
        retVal += ito::RetVal(
            ito::retError, errorCode, tr("Unknown error code %1").arg(errorCode).toLatin1().data());
    }

    return retVal;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal FaulhaberMCS::interpretCIA402Error(const QByteArray& errorBytes)
{
    ito::RetVal retVal = ito::retOk;
    ito::uint8 length = static_cast<ito::uint8>(errorBytes[1]);
    ito::uint8 node = static_cast<ito::uint8>(errorBytes[2]);
    ito::uint16 index =
        static_cast<ito::uint8>(errorBytes[4]) | (static_cast<ito::uint8>(errorBytes[5]) << 8);
    ito::uint8 subindex = static_cast<ito::uint8>(errorBytes[6]);
    ito::uint8 addCodeLB = static_cast<ito::uint8>(errorBytes[7]);
    ito::uint8 addCodeHB = static_cast<ito::uint8>(errorBytes[8]);
    ito::uint8 errorCode = static_cast<ito::uint8>(errorBytes[9]);
    ito::uint8 errorClass = static_cast<ito::uint8>(errorBytes[10]);
    ito::uint8 additionalCode = addCodeLB | (addCodeHB << 8);

    QMap<uint8_t, QMap<uint8_t, QMap<uint16_t, QString>>> errorMap;
    QString errorMessage;
    // Populate the error map with the data from Tab. 24 (translated to English)
    errorMap[0x05][0x04][0x0001] = "SDO command invalid or unknown";
    errorMap[0x06][0x01][0x0000] = "Access to this object is not supported";
    errorMap[0x06][0x01][0x0001] = "Attempt to read a write-only parameter";
    errorMap[0x06][0x01][0x0002] = "Attempt to write to a read-only parameter";
    errorMap[0x06][0x02][0x0000] = "Object not found in object dictionary";
    errorMap[0x06][0x04][0x0043] = "General parameter incompatibility";
    errorMap[0x06][0x04][0x0047] = "General internal incompatibility error in device";
    errorMap[0x06][0x07][0x0010] = "Data type or parameter length does not match or is unknown";
    errorMap[0x06][0x07][0x0012] = "Data types do not match, parameter length too large";
    errorMap[0x06][0x07][0x0013] = "Data types do not match, parameter length too small";
    errorMap[0x06][0x09][0x0011] = "Subindex not available";
    errorMap[0x06][0x09][0x0030] = "General range error";
    errorMap[0x06][0x09][0x0031] = "Range error: Parameter value too large";
    errorMap[0x06][0x09][0x0032] = "Range error: Parameter value too small";
    errorMap[0x06][0x09][0x0036] = "Range error: Maximum value smaller than minimum value";
    errorMap[0x08][0x00][0x0000] = "General SDO error";
    errorMap[0x08][0x00][0x0020] = "Access not possible";
    errorMap[0x08][0x00][0x0022] = "Access not possible under current device status";


    if (errorMap.contains(errorClass))
    {
        if (errorMap[errorClass].contains(errorCode))
        {
            if (errorMap[errorClass][errorCode].contains(additionalCode))
            {
                errorMessage = errorMap[errorClass][errorCode][additionalCode];
            }
            else
            {
                errorMessage = tr("Unknown additional code %1").arg(additionalCode);
            }
        }
    }
    else
    {
        errorMessage = tr("Unknown error class %1").arg(errorClass);
    }

    retVal += ito::RetVal(
        ito::retError,
        errorCode,
        tr("Error occurred during access of '0x%1.%2' of node '%3' with error message: %4")
            .arg(QString::number(index, 16))
            .arg(QString::number(subindex))
            .arg(QString::number(node))
            .arg(errorMessage.toLatin1().data())
            .toLatin1()
            .data());

    return retVal;
}

//----------------------------------------------------------------------------------------------------------------------------------
QString FaulhaberMCS::convertHexToString(const Register& reg)
{
    return QString("0x") + QString::number(reg.index, 16).rightJustified(4, '0').toUpper() +
        QString(".") + QString::number(reg.subindex, 16).rightJustified(2, '0').toUpper();
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal FaulhaberMCS::waitForDone(const int timeoutMS, const QVector<int> axis, const int flags)
{
    ito::RetVal retVal(ito::retOk);
    bool done = false;
    bool timeout = false;
    int currentPos = 0;
    int targetPos = 0;
    QElapsedTimer timer;
    QMutex waitMutex;
    QWaitCondition waitCondition;

    QVector<int> _axis = axis.isEmpty() ? QVector<int>(m_numOfAxes) : axis;
    if (_axis.isEmpty())
    {
        for (int i = 0; i < m_numOfAxes; i++)
        {
            _axis.append(i);
        }
    }

    timer.start();
    while (!done && !timeout && !retVal.containsWarningOrError())
    {
        if (!done && isInterrupted())
        {
            quickStop();
            replaceStatus(_axis, ito::actuatorMoving, ito::actuatorInterrupted);
            retVal += startupSequence();
            done = true;
            sendStatusUpdate();
            retVal += ito::RetVal(ito::retError, 0, tr("interrupt occurred").toLatin1().data());
            return retVal;
        }

        if (!retVal.containsError())
        {
            waitMutex.lock();
            waitCondition.wait(&waitMutex, m_delayAfterSendCommandMS);
            waitMutex.unlock();
            setAlive();
        }

        for (const auto& i : axis)
        {
            switch (m_params["operationMode"].getVal<int>())
            {
            case -1:
                ito::int16 voltage;
                retVal += getVoltageMCS(voltage);
                currentPos = static_cast<int>(voltage);
                targetPos = currentPos;
                break;
            case 1:
                retVal += getPosMCS(currentPos);
                retVal += getTargetPosMCS(targetPos);
                break;
            case 3:
                retVal += getVelocityMCS(currentPos);
                retVal += getTargetVelocityMCS(targetPos);
                break;
            case 10:
                ito::int16 torque;
                retVal += getTorqueMCS(torque);
                currentPos = static_cast<int>(torque);
                retVal += getTargetTorqueMCS(torque);
                targetPos = static_cast<int>(torque);
                break;
            }

            m_currentPos[i] = static_cast<double>(currentPos);
            m_targetPos[i] = static_cast<double>(targetPos);

            retVal += updateStatus();
            int mode = m_params["operationMode"].getVal<int>();

            if (m_statusWord[10] || m_statusWord[11])  // target flag
            {
                setStatus(
                    m_currentStatus[i],
                    ito::actuatorAtTarget,
                    ito::actSwitchesMask | ito::actStatusMask);
                done = true;
            }
            else if (m_statusWord[13])  // error flag
            {
                setStatus(
                    m_currentStatus[i],
                    ito::actuatorError,
                    ito::actSwitchesMask | ito::actStatusMask);
                done = true;
            }
            else if (mode != 1)  // no target flag for other mode than position control mode
            {
                setStatus(
                    m_currentStatus[i],
                    ito::actuatorError,
                    ito::actSwitchesMask | ito::actStatusMask);
                done = true;
            }
            else
            {
                setStatus(
                    m_currentStatus[i],
                    ito::actuatorMoving,
                    ito::actSwitchesMask | ito::actStatusMask);
                done = false;
            }
        }

        sendStatusUpdate(false);

        if (timer.hasExpired(timeoutMS))
        {
            timeout = true;
            replaceStatus(axis, ito::actuatorMoving, ito::actuatorTimeout);
            retVal += ito::RetVal(
                ito::retError,
                9999,
                "timeout occurred during movement. If necessary increase the parameter "
                "'moveTimeout'.");
            sendStatusUpdate(true);

            quickStop();
            replaceStatus(_axis, ito::actuatorMoving, ito::actuatorInterrupted);
            retVal += startupSequence();
            sendStatusUpdate();
            return retVal;
        }
    }

    return retVal;
}


//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal FaulhaberMCS::updateStatus()
{
    ito::RetVal retVal = updateStatusMCS();

    for (int i = 0; i < m_numOfAxes; i++)
    {
        m_currentStatus[i] = m_currentStatus[i] | ito::actuatorAvailable;

        ito::int32 intPos = 0;
        switch (m_params["operationMode"].getVal<int>())
        {
        case -1:
            ito::int16 voltage;
            retVal += getVoltageMCS(voltage);
            intPos = static_cast<int>(voltage);
            break;
        case 1: // position mode
            retVal += getPosMCS(intPos);
            break;
        case 3: // velocity mode
            retVal += getVelocityMCS(intPos);
            break;
        case 10: // cyclic synch torque mode
            ito::int16 torque;
            retVal += getTorqueMCS(torque);
            intPos = static_cast<int>(torque);
            break;
        }
        m_currentPos[i] = static_cast<double>(intPos);

        if (m_params["targetReached"].getVal<int>())
        {
            replaceStatus(m_currentStatus[i], ito::actuatorMoving, ito::actuatorAtTarget);
        }
        else
        {
            setStatus(
                m_currentStatus[i], ito::actuatorMoving, ito::actSwitchesMask | ito::actStatusMask);
        }
    }

    emit actuatorStatusChanged(m_currentStatus, m_currentPos);
    sendStatusUpdate();

    return retVal;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal FaulhaberMCS::sendCommand(const QByteArray& command)
{
    QSharedPointer<QVector<ito::ParamBase>> _dummy;
    m_pSerialIO->execFunc("clearInputBuffer", _dummy, _dummy, _dummy, nullptr);
    m_pSerialIO->execFunc("clearOutputBuffer", _dummy, _dummy, _dummy, nullptr);

    ito::RetVal retVal = m_pSerialIO->setVal(command.data(), command.length(), nullptr);
    setAlive();
    return retVal;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal FaulhaberMCS::sendCommandAndGetResponse(const QByteArray& command, QByteArray& response)
{
    ito::RetVal retVal = ito::retOk;
    retVal += sendCommand(command);

    if (!retVal.containsError())
    {
        retVal += readResponse(response, command[3]);
    }

    return retVal;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal FaulhaberMCS::readResponse(QByteArray& response, const ito::uint8& command)
{
    ito::RetVal retValue = ito::retOk;
    QElapsedTimer timer;

    *m_serialBufferLength = m_serialBufferSize;
    std::memset(m_serialBuffer.data(), '\0', m_serialBufferSize);

    ito::uint8 length;
    ito::uint8 recievedCommand;

    bool done = false;
    int offset = 0;
    int start = 0;
    int endIndex = 0;

    QMutex waitMutex;
    QWaitCondition waitCondition;

    timer.start();

    while (!done && !retValue.containsError())
    {
        waitMutex.lock();
        waitCondition.wait(&waitMutex, m_delayAfterSendCommandMS);
        waitMutex.unlock();
        setAlive();

        retValue += m_pSerialIO->getVal(m_serialBuffer, m_serialBufferLength, nullptr);
        response += QByteArray(m_serialBuffer.data(), *m_serialBufferLength);
        if (retValue.containsError())
        {
            retValue += ito::RetVal(
                ito::retError,
                0,
                tr("Error occurred during reading the response from the serial port.")
                    .toLatin1()
                    .data());
            break;
        }
        else
        {
            // FOUND
            if ((response.contains(m_S) && response.contains(m_E)))
            {
                done = true;
            }
        }

        if (!done && timer.elapsed() > m_requestTimeOutMS && m_requestTimeOutMS >= 0)
        {
            retValue += ito::RetVal(
                ito::retError,
                m_delayAfterSendCommandMS,
                tr("timeout during read command.").toLatin1().data());
            return retValue;
        }
    }
    if (!(response.contains(m_S) && response.contains(m_E)))
    {
        retValue += ito::RetVal(
            ito::retError,
            0,
            tr("The character 'S' or 'E' was not detected in the received bytearray.")
                .toLatin1()
                .data());
    }

    if (!retValue.containsError())
    {
        int start = response.indexOf(m_S);
        response = response.right(start + response.size() + 1);
        length = static_cast<ito::uint8>(response[1]);
        recievedCommand = static_cast<ito::uint8>(response[3]);

        if (recievedCommand == 0x03) // error
        {
#if QT_VERSION >= QT_VERSION_CHECK(6, 0, 0)
            retValue += interpretCIA402Error(response.sliced(start, length + 1));
#else
            retValue += interpretCIA402Error(response.mid(start, length + 1));
#endif
        }
    }

    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
template <typename T>
inline ito::RetVal FaulhaberMCS::readRegisterWithParsedResponse(
    const ito::uint16& index, const ito::uint8& subindex, T& answer)
{
    QByteArray response;
    ito::RetVal retValue = readRegister(index, subindex, response);
    if (!retValue.containsError())
        retValue += parseResponse<T>(response, answer);
    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
template <typename T>
ito::RetVal FaulhaberMCS::parseResponse(QByteArray& response, T& parsedResponse)
{
    ito::RetVal retValue = ito::retOk;

    qsizetype startIndex = response.indexOf(m_S);

    // Extract basic components from the response
#if QT_VERSION >= QT_VERSION_CHECK(6, 0, 0)
    ito::uint8 length = static_cast<ito::uint8>(response[startIndex + 1]);
    ito::uint8 nodeNumber = static_cast<ito::uint8>(response[startIndex + 2]);
    ito::uint8 command = static_cast<ito::uint8>(response[startIndex + 3]);

#else
    ito::uint8 length = static_cast<ito::uint8>(response.at(startIndex + 1));
    ito::uint8 nodeNumber = static_cast<ito::uint8>(response.at(startIndex + 2));
    ito::uint8 command = static_cast<ito::uint8>(response.at(startIndex + 3));
#endif

    // Verify node number
    if (nodeNumber != m_node)
    {
        return ito::RetVal(
            ito::retError,
            0,
            tr("The node number '%1' does not match the expected node number '%2'.")
                .arg(nodeNumber)
                .arg(m_node)
                .toLatin1()
                .data());
    }

#if QT_VERSION >= QT_VERSION_CHECK(6, 0, 0)
    ito::uint8 receivedCRC = static_cast<ito::uint8>(response[startIndex + length]);
#else
    ito::uint8 receivedCRC = static_cast<ito::uint8>(response.at(startIndex + length));
#endif

    ito::uint8 checkCRC = 0x00;
    QByteArray data = "";

    // Process the response based on command type
    switch (command)
    {
    case 0x01: // SDO read request/response
        if (length > 7)
        {
#if QT_VERSION >= QT_VERSION_CHECK(6, 0, 0)
            checkCRC = calculateChecksum(response.sliced(1, startIndex + length - 1));
            data = response.sliced(7, startIndex + length - 7 + 1);
#else
            checkCRC = calculateChecksum(response.mid(1, startIndex + length - 1));
            data = response.mid(7, startIndex + length - 7 + 1);
#endif
        }
        else
        {
#if QT_VERSION >= QT_VERSION_CHECK(6, 0, 0)
            checkCRC = calculateChecksum(response.sliced(1, startIndex + length));
#else
            checkCRC = calculateChecksum(response.mid(1, startIndex + length));
#endif
        }

        if (receivedCRC != checkCRC)
        {
            return ito::RetVal(
                ito::retError,
                0,
                tr("Checksum mismatch for SDO read request/response (received: '%1', "
                   "calculated: "
                   "'%2').")
                    .arg(receivedCRC)
                    .arg(checkCRC)
                    .toLatin1());
        }

        if constexpr (std::is_same<T, QString>::value) // convert to QString
        {
#if QT_VERSION >= QT_VERSION_CHECK(6, 0, 0)
            parsedResponse = QString::fromUtf8(data.sliced(0, data.size() - 1));
#else
            parsedResponse = QString::fromUtf8(data.mid(0, data.size() - 1));
#endif
        }
        else if constexpr (

            std::is_integral<T>::value || std::is_floating_point<T>::value) // convert to integer
        {
            if (data.size() >= sizeof(T))
            {
                std::memcpy(&parsedResponse, data.constData(), sizeof(T));
            }
            else
            {
                return ito::RetVal(ito::retError, 0, tr("Data size mismatch").toLatin1().data());
            }
        }
        else
        {
            return ito::RetVal(ito::retError, 0, tr("Unsupported type").toLatin1().data());
        }
        break;

    case 0x02: // SDO write request/response
#if QT_VERSION >= QT_VERSION_CHECK(6, 0, 0)
        checkCRC = calculateChecksum(response.sliced(1, length - 1));
#else
        checkCRC = calculateChecksum(response.mid(1, length - 1));
#endif
        if (receivedCRC != checkCRC)
        {
            return ito::RetVal(
                ito::retError,
                0,
                tr("Checksum mismatch for SDO write request/response (received: '%1', "
                   "calculated: "
                   "'%2').")
                    .arg(receivedCRC)
                    .arg(checkCRC)
                    .toLatin1());
        }
        break;
    case 0x05: // SDO write parameter request
#if QT_VERSION >= QT_VERSION_CHECK(6, 0, 0)
        checkCRC = calculateChecksum(response.sliced(1, length - 1));
#else
        checkCRC = calculateChecksum(response.mid(1, length - 1));
#endif
        if (receivedCRC != checkCRC)
        {
            return ito::RetVal(
                ito::retError,
                0,
                tr("Checksum mismatch for SDO write parameter request/response (received: "
                   "'%1', "
                   "calculated: "
                   "'%2').")
                    .arg(receivedCRC)
                    .arg(checkCRC)
                    .toLatin1());
        }
        m_statusWord = (response[4] << 8) | response[5];

        updateStatusBits();
        emit parametersChanged(m_params);
        sendStatusUpdate();
        break;

    case 0x07: // EMCY notification
        retValue += getError();
        break;

    default:
        return ito::RetVal(
            ito::retError,
            0,
            tr("Unknown command received: %1")
                .arg(static_cast<ito::uint8>(command))
                .toLatin1()
                .data());
    }

    return retValue;
}


//----------------------------------------------------------------------------------------------------------------------------------
template <typename T>
ito::RetVal FaulhaberMCS::setRegister(
    const ito::uint16& index,
    const ito::uint8& subindex,
    const ito::uint32& value,
    const ito::uint8& length)
{
    ito::RetVal retVal = ito::retOk;
    QByteArray response;
    std::vector<ito::uint8> command = {
        m_node,
        m_SET,
        static_cast<ito::uint8>(index & 0xFF),
        static_cast<ito::uint8>(index >> 8),
        subindex};

    for (int i = 0; i < length; i++)
    {
        command.push_back((value >> (8 * i)) & 0xFF);
    }

    std::vector<ito::uint8> fullCommand = {static_cast<ito::uint8>(command.size() + 2)};
    fullCommand.insert(fullCommand.end(), command.begin(), command.end());
    QByteArray CRC(reinterpret_cast<const char*>(fullCommand.data()), fullCommand.size());
    fullCommand.push_back(calculateChecksum(CRC));
    fullCommand.insert(fullCommand.begin(), m_S);
    fullCommand.push_back(m_E);

    QByteArray data(reinterpret_cast<char*>(fullCommand.data()), fullCommand.size());

    retVal += sendCommandAndGetResponse(data, response);

    if (!retVal.containsError())
    {
        T parsedResponse;
        retVal += parseResponse<T>(response, parsedResponse);
    }

    return retVal;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::uint8 FaulhaberMCS::calculateChecksum(const QByteArray& message)
{
    ito::uint8 calcCRC = 0xFF;
    int len = message.size(); // Get the length of the QByteArray

    for (int i = 0; i < len; i++)
    {
        calcCRC = calcCRC ^
            static_cast<ito::uint8>(message[i]); // Access QByteArray data and cast to ito::uint8
        for (ito::uint8 j = 0; j < 8; j++)
        {
            if (calcCRC & 0x01)
                calcCRC = (calcCRC >> 1) ^ 0xd5;
            else
                calcCRC = (calcCRC >> 1);
        }
    }

    return calcCRC;
}

//----------------------------------------------------------------------------------------------------------------------------------
bool FaulhaberMCS::verifyChecksum(QByteArray& message, ito::uint8& receivedCRC)
{
    // Calculate the CRC of the message excluding SOF, CRC, and EOF
#if QT_VERSION >= QT_VERSION_CHECK(6, 0, 0)
    ito::uint8 calculatedCRC = calculateChecksum(message.sliced(1, message.size() - 3));
#else
    ito::uint8 calculatedCRC = calculateChecksum(message.mid(1, message.size() - 3));
#endif

    // Compare the calculated CRC with the received CRC
    if (calculatedCRC != receivedCRC)
    {
        return false;
    }
    return true;
}

//----------------------------------------------------------------------------------------------------------------------------------
void FaulhaberMCS::dockWidgetVisibilityChanged(bool visible)
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
            connect(
                this,
                SIGNAL(actuatorStatusChanged(QVector<int>, QVector<double>)),
                widget,
                SLOT(actuatorStatusChanged(QVector<int>, QVector<double>)));
            connect(
                this,
                SIGNAL(targetChanged(QVector<double>)),
                widget,
                SLOT(targetChanged(QVector<double>)));

            emit parametersChanged(m_params);
            sendTargetUpdate();
            sendStatusUpdate(false);
        }
        else
        {
            disconnect(
                this,
                SIGNAL(parametersChanged(QMap<QString, ito::Param>)),
                widget,
                SLOT(parametersChanged(QMap<QString, ito::Param>)));
            disconnect(
                this,
                SIGNAL(actuatorStatusChanged(QVector<int>, QVector<double>)),
                widget,
                SLOT(actuatorStatusChanged(QVector<int>, QVector<double>)));
            disconnect(
                this,
                SIGNAL(targetChanged(QVector<double>)),
                widget,
                SLOT(targetChanged(QVector<double>)));
        }
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
const ito::RetVal FaulhaberMCS::showConfDialog(void)
{
    return apiShowConfigurationDialog(this, new DialogFaulhaberMCS(this));
}

//----------------------------------------------------------------------------------------------------------------------------------
bool FaulhaberMCS::isBitSet(uint32_t value, int bitPosition)
{
    return (value & (1 << bitPosition)) != 0;
}

//----------------------------------------------------------------------------------------------------------------------------------
bool FaulhaberMCS::isBitUnset(uint32_t value, int bitPosition)
{
    return (value & (1 << bitPosition)) == 0;
}
