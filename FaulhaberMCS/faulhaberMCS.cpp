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
#include "iostream"
#include "pluginVersion.h"

#include <qplugin.h>
#include <qstring.h>
#include <qstringlist.h>
#include <qwaitcondition.h>

#include "dockWidgetFaulhaberMCS.h"

QList<ito::uint8> FaulhaberMCS::openedNodes = QList<ito::uint8>();

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
Homing options:\n\
- 'setOrigin' function of the plugin uses the homing method '37' to set the current position to 0.\n\
- 'homing' execFunction can be used for the other homing methods which needs more input parameters.\n\
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
        new ito::IntMeta(1, std::numeric_limits<int>::max(), 1, "Communication"),
        tr("Node ID of device.").toUtf8().data());
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
    m_statusWord(0x00), m_requestTimeOutMS(5000), m_waitForDoneTimeout(60000),
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
        "name", ito::ParamBase::String | ito::ParamBase::Readonly, "FaulhaberMCS", nullptr);
    m_params.insert(paramVal.getName(), paramVal);

    //------------------------------- category general device parameter
    //---------------------------//
    paramVal = ito::Param(
        "serialNumber",
        ito::ParamBase::String | ito::ParamBase::Readonly,
        "",
        tr("Serial number of device.").toUtf8().data());
    paramVal.setMeta(new ito::StringMeta(ito::StringMeta::String, "", "General"), true);
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param(
        "deviceName",
        ito::ParamBase::String | ito::ParamBase::Readonly,
        "",
        tr("Name of device.").toUtf8().data());
    paramVal.setMeta(new ito::StringMeta(ito::StringMeta::String, "", "General"), true);
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param(
        "vendorID",
        ito::ParamBase::String | ito::ParamBase::Readonly,
        "",
        tr("Vendor ID of device.").toUtf8().data());
    paramVal.setMeta(new ito::StringMeta(ito::StringMeta::String, "", "General"), true);
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param(
        "productCode",
        ito::ParamBase::String | ito::ParamBase::Readonly,
        "",
        tr("Product code number.").toUtf8().data());
    paramVal.setMeta(new ito::StringMeta(ito::StringMeta::String, "", "General"), true);
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param(
        "revisionNumber",
        ito::ParamBase::String | ito::ParamBase::Readonly,
        "",
        tr("Revision number.").toUtf8().data());
    paramVal.setMeta(new ito::StringMeta(ito::StringMeta::String, "", "General"), true);
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param(
        "firmware",
        ito::ParamBase::String | ito::ParamBase::Readonly,
        "",
        tr("Firmware version.").toUtf8().data());
    paramVal.setMeta(new ito::StringMeta(ito::StringMeta::String, "", "General"), true);
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param(
        "operationMode",
        ito::ParamBase::Int | ito::ParamBase::Readonly,
        -4,
        10,
        1,
        tr("Operation Mode. -4: ATC, -3: AVC, -2: APC, -1: Voltage mode, 0: Controller not "
           "activated, 1: PP (default), 3: PV, 6: Homing, 8: CSP, 9: CSV, 10: CST.")
            .toUtf8()
            .data());
    paramVal.setMeta(new ito::IntMeta(-4, 10, 1, "General"));
    m_params.insert(paramVal.getName(), paramVal);

    //------------------------------- category communication ---------------------------//
    paramVal = ito::Param("netMode", ito::ParamBase::Int, 0, tr("RS232 net mode.").toUtf8().data());
    paramVal.setMeta(new ito::IntMeta(0, 1, 1, "Communication"));
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param("nodeID", ito::ParamBase::Int, 0, tr("Node number.").toUtf8().data());
    paramVal.setMeta(new ito::IntMeta(
        std::numeric_limits<ito::uint8>::min(),
        std::numeric_limits<ito::uint8>::max(),
        1,
        "Communication"));
    m_params.insert(paramVal.getName(), paramVal);

    //------------------------------- category temperatures ---------------------------//
    paramVal = ito::Param(
        "temperatureCPU",
        ito::ParamBase::Int | ito::ParamBase::Readonly,
        0,
        tr("CPU temperature in [°C].").toUtf8().data());
    paramVal.setMeta(new ito::IntMeta(0, std::numeric_limits<int>::max(), 1, "Temperature"));
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param(
        "temperaturePowerStage",
        ito::ParamBase::Int | ito::ParamBase::Readonly,
        0,
        tr("Power stage temperature in [°C].").toUtf8().data());
    paramVal.setMeta(new ito::IntMeta(0, std::numeric_limits<int>::max(), 1, "Temperature"));
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param(
        "temperatureWinding",
        ito::ParamBase::Int | ito::ParamBase::Readonly,
        0,
        tr("Winding temperature in [°C].").toUtf8().data());
    paramVal.setMeta(new ito::IntMeta(0, std::numeric_limits<int>::max(), 1, "Temperature"));
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
        m_async,
        tr("Enable (1) or Disable (0) operation.").toUtf8().data());
    paramVal.setMeta(new ito::IntMeta(0, 1, 1, "Movement"));
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param(
        "power",
        ito::ParamBase::Int,
        0,
        1,
        m_async,
        tr("Enable (1) or Disable (0) device power.").toUtf8().data());
    paramVal.setMeta(new ito::IntMeta(0, 1, 1, "Movement"));
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param(
        "homed",
        ito::ParamBase::Int,
        0,
        1,
        m_async,
        tr("homed (1) or not homed (0).").toUtf8().data());
    paramVal.setMeta(new ito::IntMeta(0, 1, 1, "Movement"));
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param(
        "torque",
        ito::ParamBase::Int,
        0,
        1,
        m_async,
        tr("Actual value of the torque in relative scaling.").toUtf8().data());
    paramVal.setMeta(new ito::IntMeta(0, 1, 1, "Movement"));
    m_params.insert(paramVal.getName(), paramVal);

    //------------------------------- category Statusword ---------------------------//
    paramVal = ito::Param(
        "statusWord",
        ito::ParamBase::Int | ito::ParamBase::Readonly,
        0,
        65535,
        0,
        tr("16bit statusWord to CiA 402. It indicates the status of the drive unit.")
            .toUtf8()
            .data());
    paramVal.setMeta(new ito::IntMeta(0, 65535, 1, "Status"));
    m_params.insert(paramVal.getName(), paramVal);

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
        ito::ParamBase::Int,
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
        ito::ParamBase::Int,
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
        "maxMotorSpeed", ito::ParamBase::Int, 0, tr("Max motor speed in 1/min.").toUtf8().data());
    paramVal.setMeta(new ito::IntMeta(1, 32767, 1, "Motion control"));
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param(
        "profileVelocity",
        ito::ParamBase::Int,
        0,
        tr("Profile velocity in 1/min.").toUtf8().data());
    paramVal.setMeta(new ito::IntMeta(1, 32767, 1, "Motion control"));
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param(
        "acceleration", ito::ParamBase::Int, 0, tr("Acceleration in 1/s².").toUtf8().data());
    paramVal.setMeta(new ito::IntMeta(1, 30000, 1, "Motion control"));
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param(
        "deceleration", ito::ParamBase::Int, 0, tr("Deceleration in 1/s².").toUtf8().data());
    paramVal.setMeta(new ito::IntMeta(1, 30000, 1, "Motion control"));
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param(
        "quickStopDeceleration",
        ito::ParamBase::Int,
        0,
        tr("Quickstop deceleration in 1/s².").toUtf8().data());
    paramVal.setMeta(new ito::IntMeta(1, 32750, 1, "Motion control"));
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param(
        "maxTorqueLimit", ito::ParamBase::Int, 0, tr("Maximum torque limit.").toUtf8().data());
    paramVal.setMeta(new ito::IntMeta(1, 30000, 1, "Motion control")); // TODO as readonly
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
           "–1…–4: A mechanical limit stop is set as reference.")
            .toUtf8()
            .data());
    pMand.append(paramVal);

    paramVal = ito::Param(
        "offset",
        ito::ParamBase::Int | ito::ParamBase::In,
        std::numeric_limits<int>::min(),
        std::numeric_limits<int>::max(),
        0,
        tr("Offset of the zero position relative to the position of the reference switch in "
           "userdefined units.")
            .toUtf8()
            .data());
    pOpt.append(paramVal);

    paramVal = ito::Param(
        "switchSeekVelocity",
        ito::ParamBase::Int | ito::ParamBase::In,
        1,
        32767,
        400,
        tr("Speed during search for switch.").toUtf8().data());
    pOpt.append(paramVal);

    paramVal = ito::Param(
        "homingSpeed",
        ito::ParamBase::Int | ito::ParamBase::In,
        1,
        32767,
        50,
        tr("Speed during search for zero.").toUtf8().data());
    pOpt.append(paramVal);

    paramVal = ito::Param(
        "acceleration",
        ito::ParamBase::Int | ito::ParamBase::In,
        1,
        30000,
        400,
        tr("Speed during search for zero.").toUtf8().data());
    pOpt.append(paramVal);

    paramVal = ito::Param(
        "limitCheckDelayTime",
        ito::ParamBase::Int | ito::ParamBase::In,
        0,
        32750,
        10,
        tr("Delay time until blockage detection [ms].").toUtf8().data());
    pOpt.append(paramVal);

    int torqueLimits[] = {1000, 1000};
    paramVal = ito::Param(
        "torqueLimits",
        ito::ParamBase::IntArray | ito::ParamBase::In,
        2,
        torqueLimits,
        new ito::IntArrayMeta(0, 1000, 1, "Torque control"),
        tr("Upper/ lower limit values for the reference run in 1/1000 of the rated motor torque.")
            .toUtf8()
            .data());
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
        m_node = (uint8_t)paramsMand->at(1).getVal<int>();
        if (openedNodes.contains(m_node))
        {
            retValue += ito::RetVal(
                ito::retError,
                0,
                tr("An instance of noder number '%1' is already open.")
                    .arg(m_node)
                    .toUtf8()
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
                            .toUtf8()
                            .data());
                }
                else
                {
                    openedNodes.append(m_node);
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
                        .toUtf8()
                        .data());
            }
        }
    }
    else
    {
        retValue += ito::RetVal(
            ito::retError,
            0,
            tr("Input parameter is not a dataIO instance of the SerialIO Plugin!").toUtf8().data());
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
            m_params["serialNumber"].setVal<char*>(answerString.toUtf8().data());
        }
    }

    if (!retValue.containsError())
    {
        QString answerString;
        retValue += getDeviceName(answerString);
        if (!retValue.containsError())
        {
            m_params["deviceName"].setVal<char*>(answerString.toUtf8().data());
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
            m_params["vendorID"].setVal<char*>(answerString.toUtf8().data());
        }
    }

    if (!retValue.containsError())
    {
        QString answerString;
        retValue += getProductCode(answerString);
        if (!retValue.containsError())
        {
            m_params["productCode"].setVal<char*>(answerString.toUtf8().data());
        }
    }

    if (!retValue.containsError())
    {
        QString answerString;
        retValue += getRevisionNumber(answerString);
        if (!retValue.containsError())
        {
            m_params["revisionNumber"].setVal<char*>(answerString.toUtf8().data());
        }
    }

    if (!retValue.containsError())
    {
        QString answerString;
        retValue += getFirmware(answerString);
        if (!retValue.containsError())
        {
            m_params["firmware"].setVal<char*>(answerString.toUtf8().data());
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
        retValue += updateStatus();

        ito::int32 pos;

        for (int i = 0; i < m_numOfAxes; i++)
        {
            retValue += getPosMCS(pos);
            m_currentPos[i] = static_cast<double>(pos);

            retValue += getTargetPosMCS(pos);
            m_targetPos[i] = static_cast<double>(pos);
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
        openedNodes.removeOne(m_node);
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
        else if (key == "statusWord")
        {
            retValue += updateStatus();
            if (!retValue.containsError())
            {
                retValue += it->setVal<int>(static_cast<int>(m_statusWord));
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
            ito::retError, 0, tr("any axis is moving. Parameters cannot be set.").toUtf8().data());
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
            if (!retValue.containsError())
            {
                retValue += it->copyValueFrom(&(*val));
            }
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
                        .toUtf8()
                        .data());
            }

            retValue += it->copyValueFrom(&(*val));
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
                        .toUtf8()
                        .data());
            }
            retValue += updateStatus();
            retValue += it->copyValueFrom(&(*val));
        }
        else if (key == "maxMotorSpeed")
        {
            retValue += setMaxMotorSpeed(static_cast<ito::uint32>(val->getVal<int>()));
            if (!retValue.containsError())
            {
                retValue += it->copyValueFrom(&(*val));
            }
        }
        else if (key == "profileVelocity")
        {
            retValue += setProfileVelocity(static_cast<ito::uint32>(val->getVal<int>()));
            if (!retValue.containsError())
            {
                retValue += it->copyValueFrom(&(*val));
            }
        }
        else if (key == "acceleration")
        {
            retValue += setAcceleration(static_cast<ito::uint32>(val->getVal<int>()));
            if (!retValue.containsError())
            {
                retValue += it->copyValueFrom(&(*val));
            }
        }
        else if (key == "deceleration")
        {
            retValue += setDeceleration(static_cast<ito::uint32>(val->getVal<int>()));
            if (!retValue.containsError())
            {
                retValue += it->copyValueFrom(&(*val));
            }
        }
        else if (key == "quickStopDeceleration")
        {
            retValue += setQuickStopDeceleration(static_cast<ito::uint32>(val->getVal<int>()));
            if (!retValue.containsError())
            {
                retValue += it->copyValueFrom(&(*val));
            }
        }
        else if (key == "maxTorqueLimit")
        {
            retValue += setMaxTorqueLimit(static_cast<ito::uint16>(val->getVal<int>()));
            if (!retValue.containsError())
            {
                retValue += it->copyValueFrom(&(*val));
            }
        }
        else if (key == "moveTimeout")
        {
            int timeout = val->getVal<int>();
            m_waitForDoneTimeout = timeout;
            retValue += it->copyValueFrom(&(*val));
        }
        else if (key == "netMode")
        {
            retValue += setNetMode(static_cast<ito::uint8>(val->getVal<int>()));
            if (!retValue.containsError())
            {
                retValue += it->copyValueFrom(&(*val));
            }
        }
        else if (key == "nodeID")
        {
            ito::uint8 node = static_cast<ito::uint8>(val->getVal<int>());
            retValue += setNodeID(node);
            if (!retValue.containsError())
            {
                retValue += it->copyValueFrom(&(*val));
                openedNodes.replace(openedNodes.indexOf(m_node), node);
                m_node = node;
            }
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
        ito::RetVal(ito::retError, 0, tr("'Calib' function is not implemented.").toUtf8().data());
    if (waitCond)
    {
        waitCond->returnValue = retValue;
        waitCond->release();
    }
    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal FaulhaberMCS::readRegister(
    const ito::uint16& address, const ito::uint8& subindex, QByteArray& response)
{
    ito::RetVal retValue = ito::retOk;

    std::vector<ito::uint8> command = {
        m_node,
        m_GET,
        static_cast<ito::uint8>(address & 0xFF),
        static_cast<ito::uint8>(address >> 8),
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
    setRegister<ito::uint16>(0x6040, 0x00, word, 2);
}

//----------------------------------------------------------------------------------------------------------------------------------
void FaulhaberMCS::shutDown()
{
    setControlWord(0x06);
}

//----------------------------------------------------------------------------------------------------------------------------------
void FaulhaberMCS::enableOperation()
{
    setControlWord(0x0F);
}

//----------------------------------------------------------------------------------------------------------------------------------
void FaulhaberMCS::disable()
{
    setControlWord(0x07);
}

//----------------------------------------------------------------------------------------------------------------------------------
void FaulhaberMCS::disableVoltage()
{
    setControlWord(0x00);
}

//----------------------------------------------------------------------------------------------------------------------------------
void FaulhaberMCS::quickStop()
{
    setControlWord(0x13);
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal FaulhaberMCS::homingCurrentPosToZero(const int& axis)
{
    ito::RetVal retValue = ito::retOk;
    QElapsedTimer timer;
    timer.start();
    QMutex waitMutex;
    QWaitCondition waitCondition;
    retValue += setOperationMode(static_cast<ito::uint8>(6));

    if (retValue.containsError())
    {
        retValue += ito::RetVal(
            ito::retError, 0, tr("Could not set operation mode to Homing (6).").toUtf8().data());
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

            if (m_statusWord && m_params["setPointAcknowledged"].getVal<int>()) // homing successful
            {
                break;
            }
            else if (m_statusWord && m_params["followingError"].getVal<int>()) // error during
                                                                               // homing
            {
                retValue +=
                    ito::RetVal(ito::retError, 0, tr("Error occurs during homing").toUtf8().data());
            }

            // Timeout during movement
            if (timer.hasExpired(m_waitForMCSTimeout))
            {
                retValue += ito::RetVal(
                    ito::retError, 9999, QString("Timeout occurred during homing").toUtf8().data());
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

    retValue += setOperationMode(static_cast<ito::uint8>(1));
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
    ito::uint16& limitCheckDelayTime,
    ito::uint16& negativeLimit,
    ito::uint16& positiveLimit)
{
    ito::RetVal retValue(ito::retOk);

    ito::uint16 torqueLimits[] = {negativeLimit, positiveLimit};

    if (isMotorMoving())
    {
        retValue +=
            ito::RetVal(ito::retError, 0, tr("Any motor axis is already moving").toLatin1().data());
    }
    else
    {
        bool homingComplete = false;
        int setPoint;
        int target;
        bool timeout = false;
        QElapsedTimer timer;
        QMutex waitMutex;
        QWaitCondition waitCondition;

        ito::int8 currentOperation;
        retValue += getOperationMode(currentOperation);

        retValue += setOperationMode(static_cast<ito::uint8>(6)); // change to homing mode

        // set parameters
        retValue += setHomingOffset(offset);
        retValue += setHomingMode(method);
        retValue += setHomingSeekVelocity(switchSeekVelocity);
        retValue += setHomingSpeed(homingSpeed);
        retValue += setHomingAcceleration(acceleration);
        retValue += setHomingTorqueLimits(torqueLimits);
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
                    ito::retError, 0, tr("Interrupt occurred during homing.").toUtf8().data());
                return retValue;
            }

            retValue += updateStatus();
            setPoint = m_params["setPointAcknowledged"].getVal<int>();
            target = m_params["targetReached"].getVal<int>();
            if (setPoint && target)
            {
                homingComplete = true;
                break;
            }
            // short delay of 10ms
            waitMutex.lock();
            waitCondition.wait(&waitMutex, m_delayAfterSendCommandMS);
            waitMutex.unlock();
            setAlive();

            if (timer.hasExpired(m_waitForDoneTimeout)) // timeout during movement
            {
                timeout = true;
                retValue += ito::RetVal(ito::retError, 9999, "Timeout occurred during homing.");
                for (int i = 0; i < m_numOfAxes; i++)
                {
                    replaceStatus(m_currentStatus[i], ito::actuatorMoving, ito::actuatorTimeout);
                }
                sendStatusUpdate(true);
            }
        }

        if (!retValue.containsError())
        {
            ito::int32 pos;
            for (int i = 0; i < m_numOfAxes; i++)
            {
                retValue += getPosMCS(pos);
                m_currentPos[i] = static_cast<double>(pos);

                replaceStatus(m_currentStatus[i], ito::actuatorMoving, ito::actuatorAtTarget);
            }
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
            ito::RetVal(ito::retError, 0, tr("Any motor axis is already moving").toUtf8().data());

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
                ito::retError, 1, tr("axis %i not available").toUtf8().data(), i);
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
                tr("Target position %1 is out of range").arg(pos[i]).toUtf8().data());
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
            tr("motor is running. Additional actions are not possible.").toUtf8().data());
    }
    else
    {
        // check if axis is available TODO for MCS
        foreach (const int i, axis)
        {
            if (i < 0 || i >= m_numOfAxes)
            {
                retValue += ito::RetVal::format(
                    ito::retError, 1, tr("axis %i not available").toUtf8().data(), i);
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
                retValue += setPosAbsMCS(newVal);
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
                tr("Relative position %1 is out of range").arg(pos[i]).toUtf8().data());
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
            tr("motor is running. Additional actions are not possible.").toUtf8().data());
    }
    else
    {
        // check if axis is available TODO for MCS
        foreach (const int i, axis)
        {
            if (i < 0 || i >= m_numOfAxes)
            {
                retValue += ito::RetVal::format(
                    ito::retError, 1, tr("axis %i not available").toUtf8().data(), i);
            }
            else
            {
                m_targetPos[i] = m_currentPos[i] + pos[i];
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
                tr("Operation is not enabled. Please enable operation first.").toUtf8().data());
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

            retValue += performHoming(
                method,
                offset,
                switchSeekVelocity,
                homingSpeed,
                acceleration,
                limitCheckDelayTime,
                negativeLimit,
                positiveLimit);
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
    ito::RetVal retVal = readRegisterWithParsedResponse<ito::uint32>(0x1018, 0x04, c);
    serialNum = QString::number(c);
    return retVal;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal FaulhaberMCS::getDeviceName(QString& name)
{
    return readRegisterWithParsedResponse<QString>(0x1008, 0x00, name);
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal FaulhaberMCS::getVendorID(QString& id)
{
    ito::uint32 c;
    ito::RetVal retVal = readRegisterWithParsedResponse<ito::uint32>(0x1018, 0x01, c);
    id = QString::number(c);
    return retVal;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal FaulhaberMCS::getProductCode(QString& code)
{
    ito::uint32 c;
    ito::RetVal retVal = readRegisterWithParsedResponse<ito::uint32>(0x1018, 0x02, c);
    code = QString::number(c);
    return retVal;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal FaulhaberMCS::getRevisionNumber(QString& num)
{
    ito::uint32 c;
    ito::RetVal retVal = readRegisterWithParsedResponse<ito::uint32>(0x1018, 0x03, c);
    num = QString::number(c);
    return retVal;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal FaulhaberMCS::getFirmware(QString& version)
{
    return readRegisterWithParsedResponse<QString>(0x100A, 0x00, version);
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal FaulhaberMCS::getCPUTemperature(ito::int16& temp)
{
    return readRegisterWithParsedResponse<ito::int16>(0x2326, 0x01, temp);
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal FaulhaberMCS::getPowerStageTemperature(ito::int16& temp)
{
    return readRegisterWithParsedResponse<ito::int16>(0x2326, 0x02, temp);
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal FaulhaberMCS::getWindingTemperature(ito::int16& temp)
{
    return readRegisterWithParsedResponse<ito::int16>(0x2326, 0x03, temp);
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal FaulhaberMCS::getNodeID(ito::uint8& id)
{
    return readRegisterWithParsedResponse<ito::uint8>(0x2400, 0x03, id);
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal FaulhaberMCS::setNodeID(const ito::uint8& id)
{
    return setRegister<ito::uint8>(0x2400, 0x03, id, sizeof(id));
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal FaulhaberMCS::getPosMCS(ito::int32& pos)
{
    return readRegisterWithParsedResponse<ito::int32>(0x6064, 0x00, pos);
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal FaulhaberMCS::getTargetPosMCS(ito::int32& pos)
{
    return readRegisterWithParsedResponse<ito::int32>(0x6062, 0x00, pos);
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal FaulhaberMCS::getMaxMotorSpeed(ito::uint32& speed)
{
    return readRegisterWithParsedResponse<ito::uint32>(0x6080, 0x00, speed);
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal FaulhaberMCS::setMaxMotorSpeed(const ito::uint32& speed)
{
    return setRegister<ito::uint32>(0x6080, 0x00, speed, sizeof(speed));
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal FaulhaberMCS::getAcceleration(ito::uint32& acceleration)
{
    return readRegisterWithParsedResponse<ito::uint32>(0x6083, 0x00, acceleration);
}

ito::RetVal FaulhaberMCS::setAcceleration(const ito::uint32& acceleration)
{
    return setRegister<ito::uint32>(0x6083, 0x00, acceleration, sizeof(acceleration));
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal FaulhaberMCS::getDeceleration(ito::uint32& deceleration)
{
    return readRegisterWithParsedResponse<ito::uint32>(0x6084, 0x00, deceleration);
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal FaulhaberMCS::setDeceleration(const ito::uint32& deceleration)
{
    return setRegister<ito::uint32>(0x6084, 0x00, deceleration, sizeof(deceleration));
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal FaulhaberMCS::getQuickStopDeceleration(ito::uint32& deceleration)
{
    return readRegisterWithParsedResponse<ito::uint32>(0x6085, 0x00, deceleration);
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal FaulhaberMCS::setQuickStopDeceleration(const ito::uint32& deceleration)
{
    return setRegister<ito::uint32>(0x6085, 0x00, deceleration, sizeof(deceleration));
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal FaulhaberMCS::getProfileVelocity(ito::uint32& speed)
{
    return readRegisterWithParsedResponse<ito::uint32>(0x6081, 0x00, speed);
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal FaulhaberMCS::setProfileVelocity(const ito::uint32& speed)
{
    ito::RetVal retVal = ito::retOk;
    ito::uint32 maxSpeed = static_cast<ito::uint32>(m_params["maxMotorSpeed"].getVal<int>());
    if (speed > maxSpeed)
    {
        retVal += setRegister<ito::uint32>(0x6081, 0x00, maxSpeed, sizeof(maxSpeed));
        retVal += ito::RetVal(
            ito::retWarning,
            0,
            tr("Speed is higher than maxMotorSpeed. Speed is set to maxMotorSpeed of value "
               "'%1'.")
                .arg(maxSpeed)
                .toUtf8()
                .data());
    }
    else
    {
        retVal += setRegister<ito::uint32>(0x6081, 0x00, speed, sizeof(speed));
    }
    return retVal;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal FaulhaberMCS::getOperationMode(ito::int8& mode)
{
    return readRegisterWithParsedResponse<ito::int8>(0x6060, 0x00, mode);
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal FaulhaberMCS::setOperationMode(const ito::int8& mode)
{
    return setRegister<ito::int8>(0x6060, 0x00, mode, sizeof(mode));
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal FaulhaberMCS::getMaxTorqueLimit(ito::uint16& limit)
{
    return readRegisterWithParsedResponse<ito::uint16>(0x6072, 0x00, limit);
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal FaulhaberMCS::setMaxTorqueLimit(const ito::uint16 limit)
{
    return setRegister<ito::uint16>(0x6072, 0x00, limit, sizeof(limit));
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal FaulhaberMCS::getNetMode(ito::uint8& mode)
{
    return readRegisterWithParsedResponse<ito::uint8>(0x2400, 0x05, mode);
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal FaulhaberMCS::setNetMode(const ito::uint8& mode)
{
    return setRegister<ito::uint8>(0x2400, 0x05, mode, sizeof(mode));
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal FaulhaberMCS::getTorque(ito::int16& torque)
{
    return readRegisterWithParsedResponse<ito::int16>(0x6077, 0x00, torque);
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal FaulhaberMCS::setHomingOffset(const ito::int32& offset)
{
    return setRegister<ito::int32>(0x607c, 0x00, offset, sizeof(offset));
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal FaulhaberMCS::setHomingSpeed(const ito::uint32& speed)
{
    return setRegister<ito::uint32>(0x6099, 0x02, speed, sizeof(speed));
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal FaulhaberMCS::setHomingSeekVelocity(const ito::uint32& seek)
{
    return setRegister<ito::uint32>(0x6099, 0x01, seek, sizeof(seek));
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal FaulhaberMCS::setHomingAcceleration(const ito::uint32& acceleration)
{
    return setRegister<ito::uint32>(0x609A, 0x00, acceleration, sizeof(acceleration));
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal FaulhaberMCS::setHomingLimitCheckDelayTime(const ito::uint16& time)
{
    return setRegister<ito::uint16>(0x2324, 0x02, time, sizeof(time));
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal FaulhaberMCS::setHomingTorqueLimits(const ito::uint16 limits[])
{
    ito::RetVal retVal = ito::retOk;
    retVal += setRegister<ito::uint16>(0x2350, 0x00, limits[0], sizeof(limits[0]));
    retVal += setRegister<ito::uint16>(0x2351, 0x00, limits[1], sizeof(limits[1]));
    return retVal;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal FaulhaberMCS::updateStatusMCS()
{
    ito::RetVal retVal = readRegisterWithParsedResponse<ito::uint16>(0x6041, 0x00, m_statusWord);

    if (!retVal.containsError())
    {
        QMutex waitMutex;
        QWaitCondition waitCondition;
        qint64 startTime = QDateTime::currentMSecsSinceEpoch();
        qint64 elapsedTime = 0;
        while ((m_statusWord == 0) && (elapsedTime < m_waitForMCSTimeout))
        {
            // TODO delete statudWord == 0
            //  short delay
            waitMutex.lock();
            waitCondition.wait(&waitMutex, m_delayAfterSendCommandMS);
            waitMutex.unlock();
            setAlive();

            retVal += readRegisterWithParsedResponse<ito::uint16>(0x6041, 0x00, m_statusWord);

            if (retVal.containsError())
            {
                retVal += ito::RetVal(
                    ito::retError,
                    0,
                    tr("Error occurred during reading statusword with value %1")
                        .arg(m_statusWord)
                        .toUtf8()
                        .data());
                break;
            }
            elapsedTime = QDateTime::currentMSecsSinceEpoch() - startTime;
        }

        if (!retVal.containsError())
        {
            updateStatusBits(m_statusWord);
            emit parametersChanged(m_params);
        }
        sendStatusUpdate();
    }

    return retVal;
}

//----------------------------------------------------------------------------------------------------------------------------------
void FaulhaberMCS::updateStatusBits(const ito::uint16& statusWord)
{
    m_statusWord = statusWord;
    m_params["statusWord"].setVal<int>(statusWord);
    std::bitset<16> statusBit = static_cast<std::bitset<16>>(m_statusWord);

    // Interpretation of bits
    bool readyToSwitchOn = statusBit[0];
    bool switchedOn = statusBit[1];
    bool operationEnabled = statusBit[2];
    bool fault = statusBit[3];
    bool voltageEnabled = statusBit[4];
    bool quickStop = statusBit[5];
    bool switchOnDisabled = statusBit[6];
    bool warning = statusBit[7];

    bool targetReached = statusBit[10];
    bool internalLimitActive = statusBit[11];
    bool setPointAcknowledged = statusBit[12];
    bool followingError = statusBit[13];

    m_params["readyToSwitchOn"].setVal<int>((readyToSwitchOn ? true : false));
    m_params["switchedOn"].setVal<int>((switchedOn ? true : false));
    m_params["operationEnabled"].setVal<int>((operationEnabled ? true : false));
    m_params["fault"].setVal<int>((fault ? true : false));
    m_params["voltageEnabled"].setVal<int>((voltageEnabled ? true : false));
    m_params["quickStop"].setVal<int>((quickStop ? true : false));
    m_params["switchOnDisabled"].setVal<int>((switchOnDisabled ? true : false));
    m_params["warning"].setVal<int>((warning ? true : false));
    m_params["targetReached"].setVal<int>((targetReached ? true : false));
    m_params["internalLimitActive"].setVal<int>((internalLimitActive ? true : false));
    m_params["setPointAcknowledged"].setVal<int>((setPointAcknowledged ? true : false));
    m_params["followingError"].setVal<int>((followingError ? true : false));
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal FaulhaberMCS::setCommunicationSettings(const ito::uint32& settings)
{
    return setRegister<ito::int8>(0x2400, 0x04, settings, sizeof(settings));
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal FaulhaberMCS::getError()
{
    ito::RetVal retVal = ito::retOk;
    ito::uint16 error;
    retVal += readRegisterWithParsedResponse<ito::uint16>(0x2320, 0x00, error);
    return interpretEMCYError(error);
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal FaulhaberMCS::setPosAbsMCS(const ito::int32& pos)
{
    ito::RetVal retVal = ito::retOk;
    setRegister<ito::int32>(0x607a, 0x00, pos, sizeof(pos));
    setControlWord(0x000f);
    setControlWord(0x003F);
    return updateStatus();
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal FaulhaberMCS::setPosRelMCS(const ito::int32& pos)
{
    ito::RetVal retVal = ito::retOk;
    setRegister<ito::int32>(0x607a, 0x00, pos, sizeof(pos));
    setControlWord(0x000f);
    setControlWord(0x007F);
    return updateStatus();
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal FaulhaberMCS::setHomingMode(const ito::int8& mode)
{
    return setRegister<ito::int8>(0x6098, 0x00, mode, sizeof(mode));
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
    ito::uint16 status = m_statusWord & statusMask;

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

        status = m_statusWord & statusMask; // Cyclically check the status word

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
    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal FaulhaberMCS::shutDownSequence()
{
    ito::RetVal retValue = updateStatus();

    int disableState = 0; // Reset the local step counter

    const ito::uint16 statusMask = 0x6F;
    ito::uint16 status = m_statusWord & statusMask;

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

        status = m_statusWord & statusMask; // Cyclically check the status

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
ito::RetVal FaulhaberMCS::interpretEMCYError(uint16_t errorCode)
{
    ito::RetVal retVal;

    // Use QMap instead of std::unordered_map for error details
    QMap<uint16_t, ErrorInfo> errorMap = {
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

    QMap<uint16_t, QString> communicationErrorMap = {
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
        retVal += ito::RetVal(ito::retError, errorCode, result.toUtf8().data());
    }
    else if (errorMap.contains(errorCode))
    {
        retVal += ito::RetVal(
            ito::retError, errorCode, errorMap[errorCode].shortDescription.toUtf8().data());
    }
    else
    {
        retVal += ito::RetVal(
            ito::retError, errorCode, tr("Unknown error code %1").arg(errorCode).toUtf8().data());
    }

    return retVal;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal FaulhaberMCS::waitForDone(const int timeoutMS, const QVector<int> axis, const int flags)
{
    ito::RetVal retVal(ito::retOk);
    bool done = false;
    bool timeout = false;
    char motor;
    int currentPos = 0;
    int targetPos = 0;
    QElapsedTimer timer;
    QMutex waitMutex;
    QWaitCondition waitCondition;

    QVector<int> _axis = axis;
    if (_axis.size() == 0) // all axis
    {
        for (int i = 0; i < m_numOfAxes; i++)
        {
            _axis.append(i);
        }
    }

    timer.start();
    while (!done && !timeout && !retVal.containsWarningOrError())
    {
        if (!done && isInterrupted()) // movement interrupted
        {
            quickStop();
            replaceStatus(_axis, ito::actuatorMoving, ito::actuatorInterrupted);
            retVal += startupSequence();
            done = true;
            sendStatusUpdate();
            retVal += ito::RetVal(ito::retError, 0, tr("interrupt occurred").toUtf8().data());
            return retVal;
        }

        if (!retVal.containsError()) // short delay to reduce CPU load
        {
            // short delay of 10ms
            waitMutex.lock();
            waitCondition.wait(&waitMutex, m_delayAfterSendCommandMS);
            waitMutex.unlock();
            setAlive();
        }

        foreach (auto i, axis) // Check for completion
        {
            retVal += getPosMCS(currentPos);
            m_currentPos[i] = static_cast<double>(currentPos);

            retVal += getTargetPosMCS(targetPos);
            m_targetPos[i] = static_cast<double>(targetPos);

            retVal += updateStatus();
            if ((m_statusWord && m_params["targetReached"].getVal<int>()))
            {
                setStatus(
                    m_currentStatus[i],
                    ito::actuatorAtTarget,
                    ito::actSwitchesMask | ito::actStatusMask);
                done = true;

                retVal += getPosMCS(currentPos);
                m_currentPos[i] = static_cast<double>(currentPos);

                retVal += getTargetPosMCS(targetPos);
                m_targetPos[i] = static_cast<double>(targetPos);

                break;
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

        if (timer.hasExpired(timeoutMS)) // timeout during movement
        {
            timeout = true;
            // timeout occurred, set the status of all currently moving axes to timeout
            replaceStatus(axis, ito::actuatorMoving, ito::actuatorTimeout);
            retVal += ito::RetVal(ito::retError, 9999, "timeout occurred during movement");
            sendStatusUpdate(true);
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

        ito::int32 intPos;
        retVal += getPosMCS(intPos);

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
                    .toUtf8()
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
                tr("timeout during read command.").toUtf8().data());
            return retValue;
        }
    }
    if (!(response.contains(m_S) && response.contains(m_E)))
    {
        retValue += ito::RetVal(
            ito::retError,
            0,
            tr("The character 'S' or 'E' was not detected in the received bytearray.")
                .toUtf8()
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
            retValue += ito::RetVal(
                ito::retError,
                0,
                tr("Error response received from the drive. Command: '%1'")
                    .arg(command)
                    .toUtf8()
                    .data());
            std::cout << "Response: " << response.toHex().toStdString() << std::endl;
        }
    }

    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
template <typename T>
inline ito::RetVal FaulhaberMCS::readRegisterWithParsedResponse(
    const ito::uint16& address, const ito::uint8& subindex, T& answer)
{
    QByteArray response;
    ito::RetVal retValue = readRegister(address, subindex, response);
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
    ito::uint8 length = static_cast<ito::uint8>(response[startIndex + 1]);
    ito::uint8 nodeNumber = static_cast<ito::uint8>(response[startIndex + 2]);
    ito::uint8 command = static_cast<ito::uint8>(response[startIndex + 3]);

    // Verify node number
    if (nodeNumber != m_node)
    {
        return ito::RetVal(
            ito::retError,
            0,
            tr("The node number '%1' does not match the expected node number '%2'.")
                .arg(nodeNumber)
                .arg(m_node)
                .toUtf8()
                .data());
    }

    ito::uint8 receivedCRC = static_cast<ito::uint8>(response[startIndex + length]);
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
                tr("Checksum mismatch (received: '%1', calculated: '%2').")
                    .arg(receivedCRC)
                    .arg(checkCRC)
                    .toUtf8()
                    .data());
        }

        if constexpr (std::is_same<T, QString>::value)
        {
#if QT_VERSION >= QT_VERSION_CHECK(6, 0, 0)
            parsedResponse = QString::fromUtf8(data.sliced(0, data.size() - 1));
#else
            parsedResponse = QString::fromUtf8(data.mid(0, data.size() - 1));
#endif
        }
        else if constexpr (std::is_integral<T>::value || std::is_floating_point<T>::value)
        {
            if (data.size() >= sizeof(T))
            {
                std::memcpy(&parsedResponse, data.constData(), sizeof(T));
            }
            else
            {
                return ito::RetVal(ito::retError, 0, tr("Data size mismatch").toUtf8().data());
            }
        }
        else
        {
            return ito::RetVal(ito::retError, 0, tr("Unsupported type").toUtf8().data());
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
            return ito::RetVal(ito::retError, 0, tr("Checksum mismatch").toUtf8().data());
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
                tr("Checksum mismatch (received: '%1', calculated: '%2').")
                    .arg(receivedCRC)
                    .arg(checkCRC)
                    .toUtf8()
                    .data());
        }
        m_statusWord = (response[4] << 8) | response[5];
        updateStatusBits(m_statusWord);
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
                .toUtf8()
                .data());
    }

    return retValue;
}


//----------------------------------------------------------------------------------------------------------------------------------
template <typename T>
ito::RetVal FaulhaberMCS::setRegister(
    const ito::uint16& address,
    const ito::uint8& subindex,
    const ito::uint32& value,
    const ito::uint8& length)
{
    ito::RetVal retVal = ito::retOk;
    QByteArray response;
    std::vector<uint8_t> command = {
        m_node,
        m_SET,
        static_cast<uint8_t>(address & 0xFF),
        static_cast<uint8_t>(address >> 8),
        subindex};

    for (int i = 0; i < length; i++)
    {
        command.push_back((value >> (8 * i)) & 0xFF);
    }

    std::vector<uint8_t> fullCommand = {static_cast<uint8_t>(command.size() + 2)};
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
template <typename T>
ito::RetVal FaulhaberMCS::setRegisterWithParsedResponse(
    const ito::uint16& address,
    const ito::uint8& subindex,
    const ito::uint32& value,
    const ito::uint8& length,
    T& answer)
{
    ito::RetVal retVal = ito::retOk;

    QMutex waitMutex;
    QWaitCondition waitCondition;

    retVal += setRegister<T>(address, subindex, value, sizeof(value));
    // retVal += readRegisterWithParsedResponse<T>(address, subindex, answer);

    QElapsedTimer timer;
    timer.start();

    while (answer != value)
    {
        // short delay
        waitMutex.lock();
        waitCondition.wait(&waitMutex, m_delayAfterSendCommandMS);
        waitMutex.unlock();
        setAlive();

        retVal += readRegisterWithParsedResponse<T>(address, subindex, answer);

        if (retVal.containsError() || timer.hasExpired(m_waitForMCSTimeout))
        {
            retVal += ito::RetVal(
                ito::retError,
                0,
                tr("Timeout occurred during setting register %1 to %2")
                    .arg(address)
                    .arg(value)
                    .toUtf8()
                    .data());
            break;
        }
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
            static_cast<ito::uint8>(message[i]); // Access QByteArray data and cast to uint8_t
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
    uint8_t calculatedCRC = calculateChecksum(message.sliced(1, message.size() - 3));
#else
    uint8_t calculatedCRC = calculateChecksum(message.mid(1, message.size() - 3));
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
