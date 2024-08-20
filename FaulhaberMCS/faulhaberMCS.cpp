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

#include <qdatetime.h>
#include <qmessagebox.h>
#include <qplugin.h>
#include <qstring.h>
#include <qstringlist.h>
#include <qwaitcondition.h>

#include <iomanip>
#include <iostream>
#include <sstream>

#include "dockWidgetFaulhaberMCS.h"

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
* Serie MCS 3242: https://www.faulhaber.com/de/produkte/serie/mcs-3242bx4-et/");

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
        tr("An opened serial port of 'SerialIO' plugin instance.").toLatin1().data());
    paramVal.setMeta(new ito::HWMeta("SerialIO"), true);
    m_initParamsMand.append(paramVal);

    paramVal = ito::Param(
        "Node",
        ito::ParamBase::Int | ito::ParamBase::In,
        1,
        new ito::IntMeta(1, std::numeric_limits<int>::max(), 1, "Communication"),
        tr("Node number of device.").toLatin1().data());
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
    m_statusWord(0), m_requestTimeOutMS(5000), m_waitForDoneTimeout(60000)
{
    ito::Param paramVal(
        "name", ito::ParamBase::String | ito::ParamBase::Readonly, "FaulhaberMCS", nullptr);
    m_params.insert(paramVal.getName(), paramVal);

    //------------------------------- category general device parameter
    //---------------------------//
    paramVal = ito::Param(
        "serialNumber",
        ito::ParamBase::String | ito::ParamBase::Readonly,
        "",
        tr("Serial number of device.").toLatin1().data());
    paramVal.setMeta(new ito::StringMeta(ito::StringMeta::String, "", "General"), true);
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param(
        "deviceName",
        ito::ParamBase::String | ito::ParamBase::Readonly,
        "",
        tr("Name of device.").toLatin1().data());
    paramVal.setMeta(new ito::StringMeta(ito::StringMeta::String, "", "General"), true);
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param(
        "vendorID",
        ito::ParamBase::String | ito::ParamBase::Readonly,
        "",
        tr("Vendor ID of device.").toLatin1().data());
    paramVal.setMeta(new ito::StringMeta(ito::StringMeta::String, "", "General"), true);
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param(
        "productCode",
        ito::ParamBase::String | ito::ParamBase::Readonly,
        "",
        tr("Product code number.").toLatin1().data());
    paramVal.setMeta(new ito::StringMeta(ito::StringMeta::String, "", "General"), true);
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param(
        "revisionNumber",
        ito::ParamBase::String | ito::ParamBase::Readonly,
        "",
        tr("Revision number.").toLatin1().data());
    paramVal.setMeta(new ito::StringMeta(ito::StringMeta::String, "", "General"), true);
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param(
        "firmware",
        ito::ParamBase::String | ito::ParamBase::Readonly,
        "",
        tr("Firmware version.").toLatin1().data());
    paramVal.setMeta(new ito::StringMeta(ito::StringMeta::String, "", "General"), true);
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param(
        "softwareVersion",
        ito::ParamBase::String | ito::ParamBase::Readonly,
        "",
        tr("Manufacturer software version.").toLatin1().data());
    paramVal.setMeta(new ito::StringMeta(ito::StringMeta::String, "", "General"), true);
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param(
        "ambientTemperature",
        ito::ParamBase::Int | ito::ParamBase::Readonly,
        0,
        tr("Ambient Temperature.").toLatin1().data());
    paramVal.setMeta(new ito::IntMeta(0, std::numeric_limits<int>::max(), 1, "General"));
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param(
        "operationMode",
        ito::ParamBase::Int,
        -4,
        10,
        1,
        tr("Operation Mode. -4: ATC, -3: AVC, -2: APC, -1: Voltage mode, 0: Controller not "
           "activated, 1: PP, 3: PV, 6: Homing, 8: CSP, 9: CSV, 10: CST")
            .toLatin1()
            .data());
    paramVal.setMeta(new ito::IntMeta(-4, 10, 1, "General"));
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
            .toLatin1()
            .data());
    paramVal.setMeta(new ito::IntMeta(0, 1, 1, "Movement"));
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param(
        "operation",
        ito::ParamBase::Int,
        0,
        1,
        m_async,
        tr("Enable (1) or Disable (0) operation.").toLatin1().data());
    paramVal.setMeta(new ito::IntMeta(0, 1, 1, "Movement"));
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param(
        "power",
        ito::ParamBase::Int,
        0,
        1,
        m_async,
        tr("Enable (1) or Disable (0) device power.").toLatin1().data());
    paramVal.setMeta(new ito::IntMeta(0, 1, 1, "Movement"));
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param(
        "homed",
        ito::ParamBase::Int,
        0,
        1,
        m_async,
        tr("homed (1) or not homed (0).").toLatin1().data());
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
            .toLatin1()
            .data());
    paramVal.setMeta(new ito::IntMeta(0, 65535, 1, "Status"));
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param(
        "readyToSwitchOn",
        ito::ParamBase::Int | ito::ParamBase::Readonly,
        0,
        1,
        0,
        tr("1: Ready to switch ON, 0: Not ready to switch ON (Bit 0).").toLatin1().data());
    paramVal.setMeta(new ito::IntMeta(0, 1, 1, "Status"));
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param(
        "switchedOn",
        ito::ParamBase::Int | ito::ParamBase::Readonly,
        0,
        1,
        0,
        tr("1: Drive is in the 'Switched ON' state, 0: No voltage present (Bit 1).")
            .toLatin1()
            .data());
    paramVal.setMeta(new ito::IntMeta(0, 1, 1, "Status"));
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param(
        "operationEnabled",
        ito::ParamBase::Int | ito::ParamBase::Readonly,
        0,
        1,
        0,
        tr("1: Operation enabled, 0: Operation disabled (Bit 2).").toLatin1().data());
    paramVal.setMeta(new ito::IntMeta(0, 1, 1, "Status"));
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param(
        "fault",
        ito::ParamBase::Int,
        0,
        1,
        0,
        tr("1: Error present, 0: No error present (Bit 3).").toLatin1().data());
    paramVal.setMeta(new ito::IntMeta(0, 1, 1, "Status"));
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param(
        "voltageEnabled",
        ito::ParamBase::Int | ito::ParamBase::Readonly,
        0,
        1,
        0,
        tr("1: Power supply enabled, 0: Power supply disabled (Bit 4).").toLatin1().data());
    paramVal.setMeta(new ito::IntMeta(0, 1, 1, "Status"));
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param(
        "quickStop",
        ito::ParamBase::Int,
        0,
        1,
        0,
        tr("1: Quick stop enabled, Quick stop disabled (Bit 5).").toLatin1().data());
    paramVal.setMeta(new ito::IntMeta(0, 1, 1, "Status"));
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param(
        "switchOnDisabled",
        ito::ParamBase::Int | ito::ParamBase::Readonly,
        0,
        1,
        0,
        tr("1: Switch on disabled, 0: Switch on enabled (Bit 6).").toLatin1().data());
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
            .toLatin1()
            .data());
    paramVal.setMeta(new ito::IntMeta(0, 1, 1, "Status"));
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param(
        "targetReached",
        ito::ParamBase::Int | ito::ParamBase::Readonly,
        0,
        1,
        0,
        tr("1: Target has reached, 0: is moving (Bit 10).").toLatin1().data());
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
            .toLatin1()
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
            .toLatin1()
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
            .toLatin1()
            .data());
    paramVal.setMeta(new ito::IntMeta(0, 1, 1, "Status"));
    m_params.insert(paramVal.getName(), paramVal);


    //------------------------------- category Motion control ---------------------------//
    paramVal = ito::Param(
        "maxMotorSpeed", ito::ParamBase::Int, 0, tr("Max motor speed in 1/min.").toLatin1().data());
    paramVal.setMeta(new ito::IntMeta(1, 32767, 1, "Motion control"));
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param(
        "profileVelocity",
        ito::ParamBase::Int,
        0,
        tr("Profile velocity in 1/min.").toLatin1().data());
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

    int limits[] = {1000, 6000};
    paramVal = ito::Param(
        "torqueLimits",
        ito::ParamBase::IntArray,
        2,
        limits,
        tr("Torque limit values (negative, positive).").toLatin1().data());
    paramVal.setMeta(new ito::IntArrayMeta(0, 6000, 1, "Torque control"), true);
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
            .toLatin1()
            .data());
    pMand.append(paramVal);

    paramVal = ito::Param(
        "offset",
        ito::ParamBase::Int | ito::ParamBase::In,
        std::numeric_limits<int>::min(),
        0,
        std::numeric_limits<int>::max(),
        tr("Offset of the zero position relative to the position of the reference switch in "
           "userdefined units.")
            .toLatin1()
            .data());
    pOpt.append(paramVal);

    paramVal = ito::Param(
        "switchSeekVelocity",
        ito::ParamBase::Int | ito::ParamBase::In,
        1,
        400,
        32767,
        tr("Speed during search for switch.").toLatin1().data());
    pOpt.append(paramVal);

    paramVal = ito::Param(
        "homingSpeed",
        ito::ParamBase::Int | ito::ParamBase::In,
        1,
        50,
        32767,
        tr("Speed during search for zero.").toLatin1().data());
    pOpt.append(paramVal);

    paramVal = ito::Param(
        "acceleration",
        ito::ParamBase::Int | ito::ParamBase::In,
        1,
        400,
        30000,
        tr("Speed during search for zero.").toLatin1().data());
    pOpt.append(paramVal);

    paramVal = ito::Param(
        "limitCheckDelayTime",
        ito::ParamBase::Int | ito::ParamBase::In,
        0,
        10,
        32750,
        tr("Delay time until blockage detection [ms].").toLatin1().data());
    pOpt.append(paramVal);

    int torqueLimits[] = {1000, 1000};
    paramVal = ito::Param(
        "torqueLimits",
        ito::ParamBase::IntArray | ito::ParamBase::In,
        2,
        torqueLimits,
        new ito::IntArrayMeta(0, 300, 1, "Torque control"),
        tr("Homing torque limit values (negative, positive).").toLatin1().data());
    pOpt.append(paramVal);

    registerExecFunc(
        "homing",
        pMand,
        pOpt,
        pOut,
        tr("In most of the cases before position control is to be used, the drive must perform a "
           "reference run to align the position used by the drive to the mechanic setup.")
            .toLatin1()
            .data());
    pMand.clear();
    pOpt.clear();
    pOut.clear();

    // initialize the current position vector, the status vector and the target position vector
    m_currentPos.fill(0.0, m_numOfAxes);
    m_currentStatus.fill(0, m_numOfAxes);
    m_targetPos.fill(0.0, m_numOfAxes);

    // the following lines create and register the plugin's dock widget. Delete these lines if the
    // plugin does not have a dock widget.
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

    QString answerString = "";
    int answerInteger = 0;


    if (reinterpret_cast<ito::AddInBase*>((*paramsMand)[0].getVal<void*>())
            ->getBasePlugin()
            ->getType() &
        (ito::typeDataIO | ito::typeRawIO))
    {
        // Us given SerialIO instance
        m_pSerialIO = (ito::AddInDataIO*)(*paramsMand)[0].getVal<void*>();
        m_node = (uint8_t)paramsMand->at(1).getVal<int>();
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
    }


    // ENABLE
    retValue += updateStatusMCS();
    if (!retValue.containsError())
    {
        resetCommunication();
        startAll();

        retValue += updateStatusMCS();
        if (m_params["switchOnDisabled"].getVal<int>())
        {
            shutDown();
            switchOn();
            enableOperation();
        }
    }

    if (!retValue.containsError())
    {
        retValue += setOperationMode(m_params["operationMode"].getVal<int>(), answerInteger);
        if (!retValue.containsError())
        {
            m_params["operationMode"].setVal<int>(answerInteger);
        }
    }

    if (!retValue.containsError())
    {
        retValue += getSerialNumber(answerString);
        if (!retValue.containsError())
        {
            m_params["serialNumber"].setVal<char*>(answerString.toUtf8().data());
        }
    }

    if (!retValue.containsError())
    {
        retValue += getDeviceName(answerString);
        if (!retValue.containsError())
        {
            m_params["deviceName"].setVal<char*>(answerString.toUtf8().data());
        }
    }

    if (!retValue.containsError())
    {
        retValue += getVendorID(answerString);
        if (!retValue.containsError())
        {
            m_params["vendorID"].setVal<char*>(answerString.toUtf8().data());
        }
    }

    if (!retValue.containsError())
    {
        retValue += getProductCode(answerString);
        if (!retValue.containsError())
        {
            m_params["productCode"].setVal<char*>(answerString.toUtf8().data());
        }
    }

    if (!retValue.containsError())
    {
        retValue += getRevisionNumber(answerString);
        if (!retValue.containsError())
        {
            m_params["revisionNumber"].setVal<char*>(answerString.toUtf8().data());
        }
    }

    if (!retValue.containsError())
    {
        retValue += getSoftwareVersion(answerString);
        if (!retValue.containsError())
        {
            m_params["softwareVersion"].setVal<char*>(answerString.toUtf8().data());
        }
    }

    if (!retValue.containsError())
    {
        retValue += getFirmware(answerString);
        if (!retValue.containsError())
        {
            m_params["firmware"].setVal<char*>(answerString.toUtf8().data());
        }
    }

    if (!retValue.containsError())
    {
        retValue += getAmbientTemperature(answerInteger);
        if (!retValue.containsError())
        {
            m_params["ambientTemperature"].setVal<int>(answerInteger);
        }
    }

    if (!retValue.containsError())
    {
        retValue += getMaxMotorSpeed(answerInteger);
        if (!retValue.containsError())
        {
            m_params["maxMotorSpeed"].setVal<int>(answerInteger);
        }
    }

    if (!retValue.containsError())
    {
        retValue += getAcceleration(answerInteger);
        if (!retValue.containsError())
        {
            m_params["acceleration"].setVal<int>(answerInteger);
        }
    }

    if (!retValue.containsError())
    {
        retValue += getDeceleration(answerInteger);
        if (!retValue.containsError())
        {
            m_params["deceleration"].setVal<int>(answerInteger);
        }
    }

    if (!retValue.containsError())
    {
        retValue += getProfileVelocity(answerInteger);
        if (!retValue.containsError())
        {
            m_params["profileVelocity"].setVal<int>(answerInteger);
        }
    }

    if (!retValue.containsError())
    {
        retValue += getQuickStopDeceleration(answerInteger);
        if (!retValue.containsError())
        {
            m_params["quickStopDeceleration"].setVal<int>(answerInteger);
        }
    }

    if (!retValue.containsError())
    {
        int limits[] = {0, 0};
        retValue += getTorqueLimits(limits);
        if (!retValue.containsError())
        {
            m_params["torqueLimits"].setVal<int*>(limits, 2);
        }
    }

    retValue += updateStatusMCS();

    if (!retValue.containsError())
    {
        int pos;
        retValue += getPosMCS(pos);
        m_currentPos[0] = pos;
        m_targetPos[0] = pos;
        m_currentStatus[0] = ito::actuatorAtTarget | ito::actuatorEnabled | ito::actuatorAvailable;
        retValue += updateStatusMCS();
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

    shutDown();
    disableOperation();
    stop();

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
        // gets the parameter key from m_params map (read-only is allowed, since we only want to get
        // the value).
        retValue += apiGetParamFromMapByKey(m_params, key, it, false);
    }

    if (!retValue.containsError())
    {
        int answerInteger = 0;
        if (key == "operationMode")
        {
            retValue += getOperationMode(answerInteger);
            if (!retValue.containsError())
            {
                retValue += it->setVal<int>(answerInteger);
            }
        }
        else if (key == "ambientTemperature")
        {
            retValue += getAmbientTemperature(answerInteger);
            if (!retValue.containsError())
            {
                retValue += it->setVal<int>(answerInteger);
            }
        }
        else if (key == "maxMotorSpeed")
        {
            retValue += getMaxMotorSpeed(answerInteger);
            if (!retValue.containsError())
            {
                retValue += it->setVal<int>(answerInteger);
            }
        }
        else if (key == "profileVelocity")
        {
            retValue += getProfileVelocity(answerInteger);
            if (!retValue.containsError())
            {
                retValue += it->setVal<int>(answerInteger);
            }
        }
        else if (key == "acceleration")
        {
            retValue += getAcceleration(answerInteger);
            if (!retValue.containsError())
            {
                retValue += it->setVal<int>(answerInteger);
            }
        }
        else if (key == "deceleration")
        {
            retValue += getDeceleration(answerInteger);
            if (!retValue.containsError())
            {
                retValue += it->setVal<int>(answerInteger);
            }
        }
        else if (key == "quickStopDeceleration")
        {
            retValue += getQuickStopDeceleration(answerInteger);
            if (!retValue.containsError())
            {
                retValue += it->setVal<int>(answerInteger);
            }
        }
        else if (key == "statusWord")
        {
            retValue += updateStatusMCS();
            if (!retValue.containsError())
            {
                retValue += it->setVal<int>(m_statusWord);
            }
        }
        else if (key == "torqueLimits")
        {
            int limits[] = {0, 0};
            retValue += getTorqueLimits(limits);
            if (!retValue.containsError())
            {
                retValue += it->setVal<int*>(limits, 2);
            }
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
        if (key == "operationMode")
        {
            int mode;
            retValue += setOperationMode(val->getVal<int>(), mode);
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
                disableOperation();
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

            retValue += it->copyValueFrom(&(*val));
        }
        else if (key == "power")
        {
            int power = val->getVal<int>();
            if (power == 0)
            {
                shutDown();
                updateStatusMCS();
            }
            else if (power == 1)
            {
                switchOn();
                updateStatusMCS();
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
            retValue += it->copyValueFrom(&(*val));
        }
        else if (key == "fault")
        {
            if (val->getVal<int>())
            {
                faultReset();
                updateStatusMCS();
            }
            else
            {
                retValue += ito::RetVal(
                    ito::retError,
                    0,
                    tr("Set the parameter value to 1 for fault reset.").toLatin1().data());
            }
        }
        else if (key == "maxMotorSpeed")
        {
            int speed;
            retValue += setMaxMotorSpeed(val->getVal<int>(), speed);
            if (!retValue.containsError())
            {
                retValue += it->copyValueFrom(&(*val));
            }
        }
        else if (key == "profileVelocity")
        {
            int velocity;
            retValue += setProfileVelocity(val->getVal<int>(), velocity);
            if (!retValue.containsError())
            {
                retValue += it->copyValueFrom(&(*val));
            }
        }
        else if (key == "acceleration")
        {
            int acceleration;
            retValue += setAcceleration(val->getVal<int>(), acceleration);
            if (!retValue.containsError())
            {
                retValue += it->copyValueFrom(&(*val));
            }
        }
        else if (key == "deceleration")
        {
            int deceleration;
            retValue += setDeceleration(val->getVal<int>(), deceleration);
            if (!retValue.containsError())
            {
                retValue += it->copyValueFrom(&(*val));
            }
        }
        else if (key == "quickStopDeceleration")
        {
            int deceleration;
            retValue += setQuickStopDeceleration(val->getVal<int>(), deceleration);
            if (!retValue.containsError())
            {
                retValue += it->copyValueFrom(&(*val));
            }
        }
        else if (key == "torqueLimits")
        {
            int newLimits[] = {0, 0};
            retValue += setTorqueLimits(val->getVal<int*>(), newLimits);

            if (!retValue.containsError())
            {
                retValue += it->copyValueFrom(&(*val));
            }
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
//! calib
/*!
    the given axis should be calibrated (e.g. by moving to a reference switch).
*/
ito::RetVal FaulhaberMCS::calib(const int axis, ItomSharedSemaphore* waitCond)
{
    return calib(QVector<int>(1, axis), waitCond);
}

//----------------------------------------------------------------------------------------------------------------------------------
//! calib
/*!
    the given axes should be calibrated (e.g. by moving to a reference switch).
*/
ito::RetVal FaulhaberMCS::calib(const QVector<int> axis, ItomSharedSemaphore* waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue = ito::retOk;


    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal FaulhaberMCS::readRegister(
    const uint16_t& address, const uint8_t& subindex, std::vector<uint8_t>& response)
{
    ito::RetVal retValue = ito::retOk;

    // Combine command
    std::vector<uint8_t> command = {
        m_node,
        m_GET,
        static_cast<uint8_t>(address & 0xFF),
        static_cast<uint8_t>(address >> 8),
        subindex};
    std::vector<uint8_t> fullCommand = {static_cast<uint8_t>(command.size() + 2)};
    fullCommand.insert(fullCommand.end(), command.begin(), command.end());
    fullCommand.push_back(CRC(fullCommand));
    fullCommand.insert(fullCommand.begin(), m_S);
    fullCommand.push_back(m_E);

    QByteArray data(reinterpret_cast<char*>(fullCommand.data()), fullCommand.size());
    QByteArray answer;
    retValue += sendCommandAndGetResponse(data, answer);

    if (!retValue.containsError())
    {
        retValue += parseResponse(answer, response);
    }

    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
void FaulhaberMCS::setRegister(
    const uint16_t& address, const uint8_t& subindex, const int& value, const uint8_t& length)
{ // combine command
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
    fullCommand.push_back(CRC(fullCommand));
    fullCommand.insert(fullCommand.begin(), m_S);
    fullCommand.push_back(m_E);

    QByteArray data(reinterpret_cast<char*>(fullCommand.data()), fullCommand.size());
    sendCommand(data);
}

//----------------------------------------------------------------------------------------------------------------------------------
void FaulhaberMCS::setControlWord(const uint16_t word)
{
    setRegister(0x6040, 0x00, word, 2);
}

//----------------------------------------------------------------------------------------------------------------------------------
void FaulhaberMCS::start()
{
    setControlWord(0x0A);
}

//----------------------------------------------------------------------------------------------------------------------------------
void FaulhaberMCS::stop()
{
    setControlWord(0x0B);
}

//----------------------------------------------------------------------------------------------------------------------------------
void FaulhaberMCS::resetCommunication()
{
    setControlWord(0x06);
}

//----------------------------------------------------------------------------------------------------------------------------------
void FaulhaberMCS::startAll()
{
    setControlWord(0x0F);
}

//----------------------------------------------------------------------------------------------------------------------------------
void FaulhaberMCS::shutDown()
{
    setControlWord(0x10);
}

//----------------------------------------------------------------------------------------------------------------------------------
void FaulhaberMCS::switchOn()
{
    setControlWord(0x11);
}

//----------------------------------------------------------------------------------------------------------------------------------
void FaulhaberMCS::enableOperation()
{
    setControlWord(0x15);
}

//----------------------------------------------------------------------------------------------------------------------------------
void FaulhaberMCS::disable()
{
    setControlWord(0x12);
}

//----------------------------------------------------------------------------------------------------------------------------------
void FaulhaberMCS::disableOperation()
{
    setControlWord(0x14);
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
void FaulhaberMCS::faultReset()
{
    setControlWord(0x16);
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal FaulhaberMCS::parseResponse(
    const QByteArray& response, std::vector<uint8_t>& parsedResponse)
{
    ito::RetVal retValue = ito::retOk;
    std::vector<uint8_t> ansVector(response.begin(), response.end());

    if (!retValue.containsError() && ansVector.size() >= 7)
    {
        parsedResponse = std::vector<uint8_t>(ansVector.begin() + 7, ansVector.end() - 2);
    }
    else
    {
        retValue += ito::RetVal(
            ito::retError,
            0,
            tr("Response is too short. Expected at least 7 bytes, but got %1 bytes.")
                .arg(ansVector.size())
                .toLatin1()
                .data());
    }

    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal FaulhaberMCS::homingCurrentPosToZero(const int& axis)
{
    ito::RetVal retValue = ito::retOk;
    QElapsedTimer timer;
    timer.start();

    int newMode;
    retValue += setOperationMode(6, newMode);

    retValue += setHomingMode(37);

    // Start homing
    uint8_t mode = 0x000F;
    setControlWord(mode);

    while (!retValue.containsWarningOrError())
    {
        isAlive();
        Sleep(m_delayAfterSendCommandMS);
        retValue += updateStatusMCS();

        if (!(m_statusWord & targetReached) &&
            !(m_statusWord & setPointAcknowledged)) // target still not reached
        {
            retValue += ito::RetVal(
                ito::retError, 0, tr("Target not reached during homing").toLatin1().data());
        }
        else if (m_statusWord & followingError) // error during homing
        {
            retValue +=
                ito::RetVal(ito::retError, 0, tr("Error occurs during homing").toLatin1().data());
        }
        else // target reached
        {
            m_targetPos[axis] = 0.0;
            sendTargetUpdate();
            break;
        }

        // Timeout during movement
        if (timer.hasExpired(10000))
        {
            retValue += ito::RetVal(
                ito::retError, 9999, QString("Timeout occurred during homing").toLatin1().data());
            break;
        }
    }

    setControlWord(0x001F);

    int currentPos;
    retValue += getPosMCS(currentPos);

    m_currentPos[axis] = currentPos;
    setStatus(
        m_currentStatus[axis], ito::actuatorAtTarget, ito::actSwitchesMask | ito::actStatusMask);

    if (retValue.containsError())
        return retValue;

    retValue += setOperationMode(1, newMode);

    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
//! setOrigin
/*!
    the given axis should be set to origin. That means (if possible) its current position should be
    considered to be the new origin (zero-position). If this operation is not possible, return a
    warning.
*/
ito::RetVal FaulhaberMCS::setOrigin(const int axis, ItomSharedSemaphore* waitCond)
{
    return setOrigin(QVector<int>(1, axis), waitCond);
}

//----------------------------------------------------------------------------------------------------------------------------------
//! setOrigin
/*!
    the given axes should be set to origin. That means (if possible) their current position should
   be considered to be the new origin (zero-position). If this operation is not possible, return a
    warning.
*/
ito::RetVal FaulhaberMCS::setOrigin(QVector<int> axis, ItomSharedSemaphore* waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);


    // check axis index
    foreach (const int& i, axis)
    {
        if (i >= m_numOfAxes)
        {
            retValue += ito::RetVal(
                ito::retError, 0, tr("axis number is out of boundary").toLatin1().data());
        }
    }

    // homing routine if motor is not moving
    if (!retValue.containsError())
    {
        if (isMotorMoving())
        {
            retValue += ito::RetVal(
                ito::retError, 0, tr("Any motor axis is already moving").toLatin1().data());

            if (waitCond)
            {
                waitCond->release();
                waitCond->returnValue = retValue;
            }
        }
        else
        {
            setStatus(axis, ito::actuatorMoving, ito::actSwitchesMask | ito::actStatusMask);
            sendStatusUpdate();

            for (const int& i : axis)
            {
                if (auto result = homingCurrentPosToZero(i); result.containsError())
                {
                    retValue += result;
                    break;
                }
            }

            if (!retValue.containsError())
            {
                m_params["homed"].setVal<int>(1);
            }

            if (waitCond)
            {
                waitCond->release();
                waitCond->returnValue = retValue;
            }

            sendStatusUpdate();
        }
    }
    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
//! getStatus
/*!
    re-checks the status (current position, available, end switch reached, moving, at target...) of
   all axes and returns the status of each axis as vector. Each status is an or-combination of the
   enumeration ito::tActuatorStatus.
*/
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
//! getPos
/*!
    returns the current position (in mm or degree) of the given axis
*/
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
//! getPos
/*!
    returns the current position (in mm or degree) of all given axes
*/
ito::RetVal FaulhaberMCS::getPos(
    QVector<int> axis, QSharedPointer<QVector<double>> pos, ItomSharedSemaphore* waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);

    foreach (const int i, axis)
    {
        if (i >= 0 && i < m_numOfAxes)
        {
            int intPos;
            retValue += getPosMCS(intPos);
            if (!retValue.containsError())
            {
                m_currentPos[i] = intPos; // set m_currentPos[i] to the obtained position
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
//! setPosAbs
/*!
    starts moving the given axis to the desired absolute target position

    depending on m_async this method directly returns after starting the movement (async = 1) or
    only returns if the axis reached the given target position (async = 0)

    In some cases only relative movements are possible, then get the current position, determine the
    relative movement and call the method relatively move the axis.
*/
ito::RetVal FaulhaberMCS::setPosAbs(const int axis, const double pos, ItomSharedSemaphore* waitCond)
{
    return setPosAbs(QVector<int>(1, axis), QVector<double>(1, pos), waitCond);
}

//----------------------------------------------------------------------------------------------------------------------------------
//! setPosAbs
/*!
    starts moving all given axes to the desired absolute target positions

    depending on m_async this method directly returns after starting the movement (async = 1) or
    only returns if all axes reached their given target positions (async = 0)

    In some cases only relative movements are possible, then get the current position, determine the
    relative movement and call the method relatively move the axis.
*/
ito::RetVal FaulhaberMCS::setPosAbs(
    QVector<int> axis, QVector<double> pos, ItomSharedSemaphore* waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);
    bool released = false;

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

        if (!retValue.containsError())
        {
            // set status of all given axes to moving and keep all flags related to the status and
            // switches
            for (int naxis = 0; naxis < axis.size(); naxis++)
            {
                setStatus(
                    m_currentStatus[naxis],
                    ito::actuatorMoving,
                    ito::actSwitchesMask | ito::actStatusMask);
                sendStatusUpdate(false);
            }

            // emit the signal targetChanged with m_targetPos as argument, such that all connected
            // slots gets informed about new targets
            sendTargetUpdate();

            foreach (const int i, axis)
            {
                retValue += setPosAbsMCS(m_targetPos[i]);
            }

            // emit the signal sendStatusUpdate such that all connected slots gets informed about
            // changes in m_currentStatus and m_currentPos.
            sendStatusUpdate();
            foreach (const int& a, axis)
            {
                replaceStatus(m_currentStatus[a], ito::actuatorMoving, ito::actuatorInterrupted);
            }
            sendStatusUpdate(false);

            // release the wait condition now, if async is true (itom considers this method to be
            // finished now due to the threaded call)
            if (m_async && waitCond && !released)
            {
                waitCond->returnValue = retValue;
                waitCond->release();
                released = true;
            }

            // call waitForDone in order to wait until all axes reached their target or a given
            // timeout expired the m_currentPos and m_currentStatus vectors are updated within this
            // function
            retValue += waitForDone(m_waitForDoneTimeout, axis);

            // release the wait condition now, if async is false (itom waits until now if async is
            // false, hence in the synchronous mode)
            if (!m_async && waitCond && !released)
            {
                waitCond->returnValue = retValue;
                waitCond->release();
                released = true;
            }
        }
    }

    // if the wait condition has not been released yet, do it now
    if (waitCond && !released)
    {
        waitCond->returnValue = retValue;
        waitCond->release();
    }

    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
//! setPosRel
/*!
    starts moving the given axis by the given relative distance

    depending on m_async this method directly returns after starting the movement (async = 1) or
    only returns if the axis reached the given target position (async = 0)

    In some cases only absolute movements are possible, then get the current position, determine the
    new absolute target position and call setPosAbs with this absolute target position.
*/
ito::RetVal FaulhaberMCS::setPosRel(const int axis, const double pos, ItomSharedSemaphore* waitCond)
{
    return setPosRel(QVector<int>(1, axis), QVector<double>(1, pos), waitCond);
}

//----------------------------------------------------------------------------------------------------------------------------------
//! setPosRel
/*!
    starts moving the given axes by the given relative distances

    depending on m_async this method directly returns after starting the movement (async = 1) or
    only returns if all axes reached the given target positions (async = 0)

    In some cases only absolute movements are possible, then get the current positions, determine
   the new absolute target positions and call setPosAbs with these absolute target positions.
*/
ito::RetVal FaulhaberMCS::setPosRel(
    QVector<int> axis, QVector<double> pos, ItomSharedSemaphore* waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);
    bool released = false;

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

        if (!retValue.containsError())
        {
            // set status of all given axes to moving and keep all flags related to the status and
            // switches
            for (int naxis = 0; naxis < axis.size(); naxis++)
            {
                setStatus(
                    m_currentStatus[naxis],
                    ito::actuatorMoving,
                    ito::actSwitchesMask | ito::actStatusMask);
                sendStatusUpdate(false);
            }

            // emit the signal targetChanged with m_targetPos as argument, such that all connected
            // slots gets informed about new targets
            sendTargetUpdate();

            foreach (const int i, axis)
            {
                retValue += setPosRelMCS(pos[i]);
            }

            // emit the signal sendStatusUpdate such that all connected slots gets informed about
            // changes in m_currentStatus and m_currentPos.
            sendStatusUpdate();
            foreach (const int& a, axis)
            {
                replaceStatus(m_currentStatus[a], ito::actuatorMoving, ito::actuatorInterrupted);
            }
            sendStatusUpdate(false);

            // release the wait condition now, if async is true (itom considers this method to be
            // finished now due to the threaded call)
            if (m_async && waitCond && !released)
            {
                waitCond->returnValue = retValue;
                waitCond->release();
                released = true;
            }

            // call waitForDone in order to wait until all axes reached their target or a given
            // timeout expired the m_currentPos and m_currentStatus vectors are updated within this
            // function
            retValue += waitForDone(m_waitForDoneTimeout, axis);

            // release the wait condition now, if async is false (itom waits until now if async is
            // false, hence in the synchronous mode)
            if (!m_async && waitCond && !released)
            {
                waitCond->returnValue = retValue;
                waitCond->release();
                released = true;
            }
        }
    }

    // if the wait condition has not been released yet, do it now
    if (waitCond && !released)
    {
        waitCond->returnValue = retValue;
        waitCond->release();
    }

    return retValue;
}

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
        int method = (*paramsMand)[0].getVal<int>();
        int offset = (*paramsOpt)[0].getVal<int>();
        int switchSeekVelocity = (*paramsOpt)[1].getVal<int>();
        int homingSpeed = (*paramsOpt)[2].getVal<int>();
        int acceleration = (*paramsOpt)[3].getVal<int>();
        int limitCheckDelayTime = (*paramsOpt)[4].getVal<int>();
        int* torqueLimits = (*paramsOpt)[5].getVal<int*>();
        int newMethod;
        int currentOperation;

        retValue += getOperationMode(currentOperation);
        retValue += setOperationMode(6, newMethod);


        retValue += setRegisterWithAnswerInteger(0x6098, 0x00, method, method);
        retValue += setRegisterWithAnswerInteger(0x607c, 0x00, offset, offset);
        retValue +=
            setRegisterWithAnswerInteger(0x6099, 0x01, switchSeekVelocity, switchSeekVelocity);
        retValue += setRegisterWithAnswerInteger(0x6099, 0x02, homingSpeed, homingSpeed);
        retValue += setRegisterWithAnswerInteger(0x609A, 0x00, acceleration, acceleration);
        retValue +=
            setRegisterWithAnswerInteger(0x2324, 0x02, limitCheckDelayTime, limitCheckDelayTime);
        retValue += setRegisterWithAnswerInteger(0x2350, 0x00, torqueLimits[0], torqueLimits[0]);
        retValue += setRegisterWithAnswerInteger(0x2351, 0x00, torqueLimits[1], torqueLimits[1]);

        setControlWord(0x000F);
        setControlWord(0x001F);

        // Wait for the drive to respond
        // Assuming the statusword is available and can be read
        while ((m_statusWord & (1 << setPointAcknowledged)) == 0 &&
               (m_statusWord & (1 << targetReached)) == 0)
        {
            updateStatusMCS();
        }

        // Check if the reference run was completed successfully
        if ((m_statusWord & (1 << followingError)) != 0)
        {
            retValue +=
                ito::RetVal(ito::retError, 0, tr("Error occurred during homing").toLatin1().data());
        }

        setOperationMode(currentOperation, newMethod);
    }

    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal FaulhaberMCS::getSerialNumber(QString& serialNum)
{
    int serial;
    ito::RetVal retVal = readRegisterWithAnswerInteger(0x1018, 0x04, serial);
    serialNum = QString::number(serial);
    return retVal;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal FaulhaberMCS::getDeviceName(QString& name)
{
    return readRegisterWithAnswerString(0x1008, 0x00, name);
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal FaulhaberMCS::getVendorID(QString& id)
{
    int vedor;
    ito::RetVal retVal = readRegisterWithAnswerInteger(0x1018, 0x01, vedor);
    id = QString::number(vedor);
    return retVal;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal FaulhaberMCS::getProductCode(QString& code)
{
    int product;
    ito::RetVal retVal = readRegisterWithAnswerInteger(0x1018, 0x02, product);
    code = QString::number(product);
    return retVal;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal FaulhaberMCS::getRevisionNumber(QString& num)
{
    int revision;
    ito::RetVal retVal = readRegisterWithAnswerInteger(0x1018, 0x03, revision);
    num = QString::number(revision);
    return retVal;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal FaulhaberMCS::getFirmware(QString& version)
{
    return readRegisterWithAnswerString(0x100A, 0x00, version);
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal FaulhaberMCS::getSoftwareVersion(QString& version)
{
    int software;
    ito::RetVal retVal = readRegisterWithAnswerInteger(0x100A, 0x00, software);
    version = QString::number(software);
    return retVal;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal FaulhaberMCS::getAmbientTemperature(int& temp)
{
    return readRegisterWithAnswerInteger(0x232A, 0x08, temp);
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal FaulhaberMCS::getPosMCS(int& pos)
{
    return readRegisterWithAnswerInteger(0x6064, 0x00, pos);
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal FaulhaberMCS::getTargetPosMCS(int& pos)
{
    return readRegisterWithAnswerInteger(0x6062, 0x00, pos);
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal FaulhaberMCS::getMaxMotorSpeed(int& speed)
{
    return readRegisterWithAnswerInteger(0x6080, 0x00, speed);
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal FaulhaberMCS::setMaxMotorSpeed(const int& speed, int& newSpeed)
{
    return setRegisterWithAnswerInteger(0x6080, 0x00, speed, newSpeed);
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal FaulhaberMCS::getAcceleration(int& acceleration)
{
    return readRegisterWithAnswerInteger(0x6083, 0x00, acceleration);
}

ito::RetVal FaulhaberMCS::setAcceleration(const int& acceleration, int& newAcceleration)
{
    return setRegisterWithAnswerInteger(0x6083, 0x00, acceleration, newAcceleration);
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal FaulhaberMCS::getDeceleration(int& deceleration)
{
    return readRegisterWithAnswerInteger(0x6084, 0x00, deceleration);
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal FaulhaberMCS::setDeceleration(const int& deceleration, int& newDeceleration)
{
    return setRegisterWithAnswerInteger(0x6084, 0x00, deceleration, newDeceleration);
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal FaulhaberMCS::getQuickStopDeceleration(int& deceleration)
{
    return readRegisterWithAnswerInteger(0x6085, 0x00, deceleration);
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal FaulhaberMCS::setQuickStopDeceleration(const int& deceleration, int& newDeceleration)
{
    return setRegisterWithAnswerInteger(0x6085, 0x00, deceleration, newDeceleration);
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal FaulhaberMCS::getProfileVelocity(int& speed)
{
    return readRegisterWithAnswerInteger(0x6081, 0x00, speed);
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal FaulhaberMCS::setProfileVelocity(const int& speed, int& newSpeed)
{
    return setRegisterWithAnswerInteger(0x6081, 0x00, speed, newSpeed);
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal FaulhaberMCS::getOperationMode(int& mode)
{
    return readRegisterWithAnswerInteger(0x6061, 0x00, mode);
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal FaulhaberMCS::setOperationMode(const int& mode, int& newMode)
{
    return setRegister(0x606, 0x00, mode, sizeof(mode));
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal FaulhaberMCS::getTorqueLimits(int limits[])
{
    ito::RetVal retVal = ito::retOk;

    retVal += readRegisterWithAnswerInteger(0x60E0, 0x00, limits[0]);
    retVal += readRegisterWithAnswerInteger(0x60E1, 0x00, limits[1]);
    return retVal;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal FaulhaberMCS::setTorqueLimits(const int limits[], int newLimits[])
{
    ito::RetVal retVal = ito::retOk;

    retVal += setRegisterWithAnswerInteger(0x60E0, 0x00, limits[0], newLimits[0]);
    retVal += setRegisterWithAnswerInteger(0x60E1, 0x00, limits[1], newLimits[1]);
    return retVal;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal FaulhaberMCS::updateStatusMCS()
{
    ito::RetVal retVal(ito::retOk);
    retVal += readRegisterWithAnswerInteger(0x6041, 0x0, m_statusWord);

    if (!retVal.containsError())
    {
        m_params["statusWord"].setVal<int>(m_statusWord);

        m_params["readyToSwitchOn"].setVal<int>(m_statusWord & readyToSwitchOn ? 1 : 0);
        m_params["switchedOn"].setVal<int>(m_statusWord & switchedOn ? 1 : 0);
        m_params["operationEnabled"].setVal<int>(m_statusWord & operationEnabled ? 1 : 0);
        m_params["fault"].setVal<int>(m_statusWord & fault ? 1 : 0);
        m_params["voltageEnabled"].setVal<int>(m_statusWord & voltageEnabled ? 1 : 0);
        m_params["quickStop"].setVal<int>(m_statusWord & quickStopEnable ? 1 : 0);
        m_params["switchOnDisabled"].setVal<int>(m_statusWord & switchOnDisabled ? 1 : 0);
        m_params["warning"].setVal<int>(m_statusWord & warning ? 1 : 0);
        m_params["targetReached"].setVal<int>(m_statusWord & targetReached ? 1 : 0);
        m_params["internalLimitActive"].setVal<int>(m_statusWord & internalLimitActive ? 1 : 0);
        m_params["setPointAcknowledged"].setVal<int>(m_statusWord & setPointAcknowledged ? 1 : 0);
        m_params["followingError"].setVal<int>(m_statusWord & followingError ? 1 : 0);

        emit parametersChanged(m_params);
    }

    sendStatusUpdate();
    return retVal;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal FaulhaberMCS::setPosAbsMCS(const double& pos)
{
    ito::RetVal retVal = ito::retOk;
    int answer;
    retVal += setRegisterWithAnswerInteger(0x607a, 0x00, doubleToInteger(pos), answer);
    setControlWord(0x000f);
    setControlWord(0x003F);
    return ito::RetVal();
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal FaulhaberMCS::setPosRelMCS(const double& pos)
{
    ito::RetVal retVal = ito::retOk;
    int answer;
    retVal += setRegisterWithAnswerInteger(0x607a, 0x00, doubleToInteger(pos), answer);
    setControlWord(0x000f);
    setControlWord(0x007F);
    return ito::RetVal();
}

////----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal FaulhaberMCS::setHomingMode(const int& mode)
{
    ito::RetVal retVal = ito::retOk;
    int answer;
    retVal += setRegisterWithAnswerInteger(0x6098, 0x00, mode, answer);
    return ito::RetVal();
}

//----------------------------------------------------------------------------------------------------------------------------------
int FaulhaberMCS::doubleToInteger(const double& value)
{
    return int(std::round(value * 100) / 100);
}

//----------------------------------------------------------------------------------------------------------------------------------
int FaulhaberMCS::responseVectorToInteger(const std::vector<uint8_t>& response)
{
    int answer = 0;
    for (size_t i = 0; i < response.size(); ++i)
    {
        answer |= static_cast<int>(response[i]) << (8 * i);
    }
    return answer;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal FaulhaberMCS::waitForDone(const int timeoutMS, const QVector<int> axis, const int flags)
{
    ito::RetVal retVal(ito::retOk);
    bool done = false;
    bool timeout = false;
    char motor;
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
            replaceStatus(_axis, ito::actuatorMoving, ito::actuatorInterrupted);
            retVal += ito::RetVal(ito::retError, 0, tr("interrupt occurred").toLatin1().data());
            done = true;
            sendStatusUpdate();
            return retVal;
        }

        if (!retVal.containsError()) // short delay to reduce CPU load
        {
            // short delay of 10ms
            waitMutex.lock();
            waitCondition.wait(&waitMutex, 10);
            waitMutex.unlock();
            setAlive();
        }

        for (int i = 0; i < axis.size(); ++i) // Check for completion
        {
            retVal += updateStatusMCS();
            if ((m_statusWord & targetReached) && (m_statusWord & setPointAcknowledged))
            {
                setStatus(
                    m_currentStatus[i],
                    ito::actuatorAtTarget,
                    ito::actSwitchesMask | ito::actStatusMask);
                done = true;
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

            int currentPos = 0;
            int targetPos = 0;
            retVal += getPosMCS(currentPos);
            m_currentPos[i] = double(currentPos);

            retVal += getTargetPosMCS(targetPos);
            m_targetPos[i] = double(targetPos);
        }

        retVal += updateStatusMCS();
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
//! method obtains the current position, status of all axes
/*!
    This is a helper function, it is not necessary to implement a function like this, but it
   might help.
*/
ito::RetVal FaulhaberMCS::updateStatus()
{
    ito::RetVal retVal = ito::retOk;
    for (int i = 0; i < m_numOfAxes; i++)
    {
        m_currentStatus[i] = m_currentStatus[i] |
            ito::actuatorAvailable; // set this if the axis i is available, else use
        // m_currentStatus[i] = m_currentStatus[i] ^ ito::actuatorAvailable;

        int intPos;
        retVal += getPosMCS(intPos);

        m_currentPos[i] = double(intPos);

        // if you know that the axis i is at its target position, change from moving to
        // target if moving has been set, therefore:
        replaceStatus(m_currentStatus[i], ito::actuatorMoving, ito::actuatorAtTarget);

        // if you know that the axis i is still moving, set this bit (all other moving-related
        // bits are unchecked, but the status bits and switches bits kept unchanged
        setStatus(
            m_currentStatus[i], ito::actuatorMoving, ito::actSwitchesMask | ito::actStatusMask);
    }

    // emit actuatorStatusChanged with m_currentStatus and m_currentPos in order to inform
    // connected slots about the current status and position
    sendStatusUpdate();

    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal FaulhaberMCS::sendCommand(const QByteArray& command)
{
    ito::RetVal retVal;
    retVal += m_pSerialIO->setVal(command.data(), command.length(), nullptr);

    // delay
    if (m_delayAfterSendCommandMS > 0)
    {
        QMutex mutex;
        mutex.lock();
        QWaitCondition waitCondition;
        waitCondition.wait(&mutex, m_delayAfterSendCommandMS);
        mutex.unlock();
    }
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
        retVal += readResponse(response);
    }

    return retVal;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal FaulhaberMCS::readResponse(QByteArray& result)
{
    ito::RetVal retValue = ito::retOk;
    QElapsedTimer timer;

    bool done = false;
    bool timeout = false;

    int buflen = 100;
    QSharedPointer<int> curBufLen(new int);
    QSharedPointer<char> curBuf(new char[buflen]);
    result = "";

    timer.start();
    QThread::msleep(m_delayAfterSendCommandMS);

    while (!done && !retValue.containsError() && !timeout)
    {
        *curBufLen = buflen;
        retValue += m_pSerialIO->getVal(curBuf, curBufLen, nullptr);

        if (!retValue.containsError())
        {
            result += QByteArray(curBuf.data(), *curBufLen);
            done = true;
        }

        if (!done && timer.elapsed() > m_requestTimeOutMS && m_requestTimeOutMS >= 0)
        {
            retValue += ito::RetVal(
                ito::retError,
                m_delayAfterSendCommandMS,
                tr("timeout during read string.").toLatin1().data());
            timeout = true;
        }
        else
        {
            setAlive();
        }
    }
    result.truncate(2 + result[1]);

    std::vector<uint8_t> ansVector(result.begin(), result.end());

    if (CRC(std::vector<uint8_t>(ansVector.begin() + 1, ansVector.end() - 2)) !=
        ansVector[ansVector.size() - 2])
    {
        retValue += ito::RetVal(
            ito::retError,
            0,
            tr("Checksum mismatch for %1").arg(QString::fromUtf8(result)).toLatin1().data());
    }

    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal FaulhaberMCS::readRegisterWithAnswerString(
    const uint16_t& address, const uint8_t& subindex, QString& answer)
{
    ito::RetVal retValue = ito::retOk;
    std::vector<uint8_t> registerValue;

    retValue += readRegister(address, subindex, registerValue);

    if (!retValue.containsError())
    {
        QByteArray byteArray(
            reinterpret_cast<const char*>(registerValue.data()), registerValue.size());
        answer = QString::fromUtf8(byteArray);
    }
    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal FaulhaberMCS::readRegisterWithAnswerInteger(
    const uint16_t& address, const uint8_t& subindex, int& answer)
{
    ito::RetVal retValue = ito::retOk;
    std::vector<uint8_t> registerValue;

    retValue += readRegister(address, subindex, registerValue);

    answer = responseVectorToInteger(registerValue);

    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal FaulhaberMCS::setRegisterWithAnswerInteger(
    const uint16_t& address, const uint8_t& subindex, const int& value, int& answer)
{
    setRegister(address, subindex, value, sizeof(value));
    return readRegisterWithAnswerInteger(address, subindex, answer);
}

//----------------------------------------------------------------------------------------------------------------------------------
uint8_t FaulhaberMCS::CRC(const std::vector<uint8_t>& msg)
{
    uint8_t poly = 0xD5;
    uint8_t crc = 0xFF;

    for (auto byte : msg)
    {
        crc ^= byte;
        for (int i = 0; i < 8; i++)
        {
            if (crc & 0x01)
                crc = (crc >> 1) ^ poly;
            else
                crc >>= 1;
        }
    }

    return crc;
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
